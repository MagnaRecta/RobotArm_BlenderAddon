"""The build file and its status sidecar. BRIDGE_PROTOCOL.md Part A.

**This is the entire integration surface with ROS2** (Option C). There is no
socket, no live link: Blender writes this file, ROS2 reads it, executes it,
and writes status back alongside. Everything either side needs to agree on
lives in the protocol document -- implement against that, not against prose
elsewhere.

Its A.1 design rules, and how they show up here:

1. **Self-contained** -- no references back into the ``.blend``.
2. **Already in robot coordinates** -- metres, in the TARGET robot's own
   root frame (``core_robots.RobotProfile.frame`` -- ``base_link`` for
   so_arm_100, ``base`` for kr10_r900_2; see ``_frame_for_robot`` below,
   not a single fixed name). Blender does the transform
   (``core/transform.py``); ROS2 does none.
3. **Ordered** -- the ``sticks`` array *is* the build order. ROS2 executes it
   as-is and never reorders. One authority, no drift.
4. **Human-readable and diffable** -- pretty-printed, stable key order. Keys
   are emitted in the protocol's own documented order (dicts preserve
   insertion order), *not* sorted: ``sort_keys`` would put ``base`` before
   ``id`` and make the file read nothing like its spec.
5. **Status lives elsewhere** -- the build file is immutable input; progress
   goes to ``<name>.status.json`` so a re-export never destroys it.

Pure Python (``json`` only, no ``bpy``) so the format is testable without
Blender -- the file contract is exactly the thing worth testing hardest.
"""

import datetime
import json
import os

from ..core import robots as core_robots

BUILD_FORMAT = "so100_build"
BUILD_VERSION = 1
STATUS_FORMAT = "so100_build_status"
STATUS_VERSION = 1

# Historical default / fallback only -- see _frame_for_robot() below. A real
# export always carries a real robot id and gets THAT robot's own frame
# (core_robots.RobotProfile.frame), not this constant. Kept only for the
# `robot=""` case a few unrelated tests use deliberately (see
# tests/test_build_file.py).
FRAME = "base_link"
UNITS = "meters"


def _frame_for_robot(robot):
    """The frame this build file's coordinates are expressed in --
    BRIDGE_PROTOCOL.md A.1 rule 2 ("already in robot coordinates... Blender
    does the transform before writing") is a property of WHICH ROBOT, not
    free-form per-export data, so this is intrinsic to the robot
    (``core_robots.RobotProfile.frame``), never a flat constant. Found wrong
    2026-08-23: a real kr10_r900_2 export carried the OLD flat ``FRAME``
    constant's value (``"base_link"``, so_arm_100's own root link) instead
    of kr10_r900_2's real root link (``"base"``) -- the executor
    (``kuka_control/kuka_pick_and_place/build_file.py``) correctly refused
    the file rather than silently using the wrong frame.

    Falls back to the module-level ``FRAME`` default only when ``robot`` is
    falsy -- a handful of tests exercise unrelated behaviour without
    passing a robot id at all; a real export from ``ops/build.py`` always
    passes ``props.robot_id``, a real, registered id.
    """
    if not robot:
        return FRAME
    return core_robots.get_robot(robot).frame

# Coordinates are rounded to the micrometre before writing. Two reasons:
# a diffable file should not churn on float noise, and the protocol's
# `||tip - base|| == length_m` invariant tolerates 1 mm, so 1e-6 m is three
# orders of magnitude inside it (a worst-case 3-axis rounding shifts a
# length by ~1.7e-6 m). Verified by a test rather than assumed.
_COORD_DECIMALS = 6

STATUS_PENDING = "pending"
STATUS_PLACED = "placed"
STATUS_FAILED = "failed"
STATUS_SKIPPED = "skipped"
SIDECAR_STATUSES = (STATUS_PENDING, STATUS_PLACED, STATUS_FAILED, STATUS_SKIPPED)

STATUS_SUFFIX = ".status.json"


class ProtocolError(Exception):
    """A file that does not conform to BRIDGE_PROTOCOL.md Part A."""


def _timestamp(when=None):
    """UTC, ISO 8601, ``Z``-suffixed -- matching the protocol's examples."""
    when = when or datetime.datetime.now(datetime.timezone.utc)
    return when.astimezone(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _round_point(point):
    return [round(float(value), _COORD_DECIMALS) for value in point]


def status_path_for(build_path):
    """``tower_v3.build.json`` -> ``tower_v3.build.status.json``.

    A.3 says the sidecar sits "next to the build file as
    ``<name>.status.json``". A build file conventionally ends ``.json``, so
    that suffix is replaced rather than appended -- otherwise the sidecar
    would be ``tower_v3.build.json.status.json``.
    """
    root, extension = os.path.splitext(build_path)
    if extension.lower() == ".json":
        return root + STATUS_SUFFIX
    return build_path + STATUS_SUFFIX


# --- writing the build file --------------------------------------------------


def stick_entry(ordered, verdict=None):
    """One element of the ``sticks`` array.

    ``ordered`` is a ``core.order.OrderedStick``; ``verdict`` its
    ``core.validate.Verdict`` (or ``None`` when it was never validated, e.g.
    a stick that already failed the geometric checks).
    """
    stick = ordered.stick
    buildable = bool(verdict.buildable) if verdict is not None else False
    reason = None
    if verdict is not None and verdict.reason:
        reason = verdict.reason
    elif ordered.reason:
        reason = ordered.reason

    warnings = list(dict.fromkeys(list(ordered.warnings) + list(stick.warnings)))

    return {
        "id": stick.id,
        "order": ordered.order,
        "base": _round_point(stick.base),
        "tip": _round_point(stick.tip),
        # ⚠ Always 0.0, deliberately. `roll_deg` is spin about the stick's
        # OWN axis, and (a) a wireframe edge carries no such information --
        # the design simply does not say -- and (b) with position fixed, this
        # 5-DOF arm has exactly two orientation DOF, which the stick's
        # DIRECTION already consumes, so the spin is not independently
        # commandable anyway. The wrist roll that ROS2 actually needs is
        # derived from base/tip, not read from here. See the README's Phase D
        # note on where that derivation lives.
        "roll_deg": 0.0,
        "length_m": round(float(stick.length_m), _COORD_DECIMALS),
        "shared_ends": int(stick.shared_ends),
        "supports": list(ordered.supports),
        "validation": {"buildable": buildable, "reason": reason},
        "warnings": warnings,
    }


def build_document(ordered_sticks, verdicts=None, source="", robot="",
                   kinematics_version="", stock=None, build_volume=None,
                   generated=None):
    """Assemble the whole build file as a dict, per A.2.

    ``ordered_sticks`` must already be in build order. Sticks whose
    ``validation.buildable`` is false are **kept** -- A.2 requires them to
    appear so the operator sees the complete picture and can decide to build
    the rest.

    ``robot`` is the BRIDGE_PROTOCOL.md Sec A.1.1 robot id this file was
    validated and exported for (added 2026-08-21, multi-robot support) --
    the executor must refuse to run a file whose ``robot`` doesn't match
    itself, before even checking ``kinematics_version``.
    """
    verdicts = verdicts or {}
    stock = stock or {}
    build_volume = build_volume or {}

    sticks = []
    for position, entry in enumerate(ordered_sticks):
        record = stick_entry(entry, verdicts.get(entry.stick.id))
        # A.2: "order must be dense and match array position". Enforced here
        # rather than trusted, since a mismatch would silently desynchronise
        # ROS2's progress reporting from the array it is iterating.
        record["order"] = position
        sticks.append(record)

    return {
        "format": BUILD_FORMAT,
        "version": BUILD_VERSION,
        "generated": _timestamp(generated),
        "source": source,
        "frame": _frame_for_robot(robot),
        "units": UNITS,
        "robot": robot,
        "kinematics_version": kinematics_version,
        "stock": stock,
        "build_volume": build_volume,
        "sticks": sticks,
    }


def dumps(document):
    """Pretty-printed, newline-terminated, insertion-ordered JSON."""
    return json.dumps(document, indent=2, sort_keys=False) + "\n"


def write_build_file(path, document):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(dumps(document))


# --- reading a build file back ----------------------------------------------


def load_build_file(text):
    """Parse and sanity-check a build file. Raises ``ProtocolError``.

    Blender only needs this to re-open its own output, but checking the
    invariants on the way in is what catches an export bug at the point it
    is cheap to fix rather than on the robot.
    """
    try:
        document = json.loads(text)
    except ValueError as exc:
        raise ProtocolError("not valid JSON: %s" % exc)

    if document.get("format") != BUILD_FORMAT:
        raise ProtocolError(
            "not a build file: format is %r, expected %r"
            % (document.get("format"), BUILD_FORMAT))
    if document.get("version") != BUILD_VERSION:
        raise ProtocolError(
            "build file version %r, this addon writes version %d"
            % (document.get("version"), BUILD_VERSION))

    sticks = document.get("sticks")
    if not isinstance(sticks, list):
        raise ProtocolError("'sticks' must be a list")
    for position, entry in enumerate(sticks):
        if entry.get("order") != position:
            raise ProtocolError(
                "stick %r has order %r at array position %d -- A.2 requires "
                "`order` to be dense and match array position"
                % (entry.get("id"), entry.get("order"), position))
    return document


# --- the status sidecar ------------------------------------------------------


def status_document(build_file_name, statuses, current_index=0, updated=None):
    """Assemble a status sidecar, per A.3.

    ``statuses`` maps stick id -> ``{"status", "at", "reason"}``. Entries
    that are ``pending`` are **omitted**: A.3 says "anything absent is
    pending", so writing them would just be noise in a file whose whole
    point is to be small and diffable.
    """
    entries = {}
    for stick_id, entry in sorted(statuses.items()):
        status = entry.get("status", STATUS_PENDING)
        if status == STATUS_PENDING:
            continue
        record = {"status": status, "at": entry.get("at") or _timestamp(updated)}
        if entry.get("reason"):
            record["reason"] = entry["reason"]
        entries[stick_id] = record

    return {
        "format": STATUS_FORMAT,
        "version": STATUS_VERSION,
        "build_file": build_file_name,
        "updated": _timestamp(updated),
        "current_index": int(current_index),
        "sticks": entries,
    }


def write_status_file(path, document):
    with open(path, "w", encoding="utf-8", newline="\n") as handle:
        handle.write(dumps(document))


def load_status_file(text):
    """Parse a status sidecar. Returns ``(current_index, {id: entry})``.

    Unknown statuses degrade to ``pending`` rather than raising: the
    protocol's additive-evolution rule means a newer ROS2 may write a state
    this addon has never heard of, and refusing to open the file would be a
    worse failure than showing that stick as not-yet-done.
    """
    try:
        document = json.loads(text)
    except ValueError as exc:
        raise ProtocolError("not valid JSON: %s" % exc)

    if document.get("format") != STATUS_FORMAT:
        raise ProtocolError(
            "not a status sidecar: format is %r, expected %r"
            % (document.get("format"), STATUS_FORMAT))

    entries = {}
    for stick_id, entry in (document.get("sticks") or {}).items():
        if not isinstance(entry, dict):
            continue
        status = entry.get("status", STATUS_PENDING)
        if status not in SIDECAR_STATUSES:
            status = STATUS_PENDING
        entries[stick_id] = {
            "status": status,
            "at": entry.get("at", ""),
            "reason": entry.get("reason", ""),
        }
    return document.get("current_index", 0), entries


# --- resuming ----------------------------------------------------------------


def next_stick_to_load(ordered_ids, statuses):
    """A.5 / Sec 10.3: the id the operator should load next.

    The first stick in build order that is not already ``placed``. ``failed``
    and ``skipped`` sticks are **not** skipped over here -- the operator
    decides what to do about them, and silently stepping past a failure would
    hide exactly the thing they need to see. Returns ``None`` when the build
    is finished.
    """
    for stick_id in ordered_ids:
        status = (statuses.get(stick_id) or {}).get("status", STATUS_PENDING)
        if status != STATUS_PLACED:
            return stick_id
    return None
