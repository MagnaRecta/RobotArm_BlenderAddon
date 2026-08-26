"""Per-stick build state. BLENDER_ADDON_PLAN.md Sec 9.2 / 9.3.

QB4 asked for state in **both** the ``.blend`` and a compact JSON sidecar.
This module is the pure-Python half: the state model, the JSON round-trip,
stable-id allocation, and staleness detection. The ``.blend`` half is the
``PropertyGroup`` in ``properties.py`` -- a PropertyGroup *is* .blend
persistence, and it needs ``bpy``, which this module deliberately does not.

Stable ids matter more than they look. **Edge indices are not stable across
mesh edits** (Sec 9.2), so an order computed on Monday would silently point
at different sticks on Tuesday. Ids are allocated once, stored in an integer
attribute layer on the mesh's EDGE domain, and never reused.
"""

import json

STATUS_PENDING = "pending"
STATUS_BUILDABLE = "buildable"
STATUS_IMPOSSIBLE = "impossible"
STATUS_PLACED = "placed"
STATUS_FAILED = "failed"
STATUS_SKIPPED = "skipped"

ALL_STATUSES = (
    STATUS_PENDING, STATUS_BUILDABLE, STATUS_IMPOSSIBLE,
    STATUS_PLACED, STATUS_FAILED, STATUS_SKIPPED,
)

# Statuses written by the ROS2 side into the status sidecar
# (BRIDGE_PROTOCOL.md Sec A.3). Anything absent there is `pending`.
SIDECAR_STATUSES = (STATUS_PENDING, STATUS_PLACED, STATUS_FAILED, STATUS_SKIPPED)

ID_PREFIX = "s_"


def format_id(number):
    """``1 -> 's_001'``. Zero-padded to 3 for readability, but not limited to
    it -- a 1200-stick job gives ``s_1200``, which still sorts sanely next to
    its neighbours."""
    return "%s%03d" % (ID_PREFIX, number)


def parse_id(stick_id):
    """``'s_001' -> 1``, or ``None`` if it is not one of ours."""
    if not isinstance(stick_id, str) or not stick_id.startswith(ID_PREFIX):
        return None
    try:
        return int(stick_id[len(ID_PREFIX):])
    except ValueError:
        return None


def allocate_ids(edge_id_numbers, next_number):
    """Assign stable ids to a mesh's edges.

    ``edge_id_numbers`` is the raw integer attribute layer, one entry per
    edge, where ``0`` means "not yet assigned". Returns
    ``(assigned_numbers, next_number, reassigned_count)``.

    Two cases have to be handled or ids stop being stable:

    * **0** -- a brand new edge. Gets a fresh number.
    * **a duplicate** -- Blender copies attribute values when an edge is
      duplicated (Shift+D, mirror, array), so two edges can carry the same
      id. The first occurrence keeps it; later ones get fresh numbers. Never
      reuse, never renumber the survivor.
    """
    assigned = []
    seen = set()
    reassigned = 0
    for value in edge_id_numbers:
        number = int(value)
        if number <= 0 or number in seen:
            if number > 0:
                reassigned += 1
            number = next_number
            next_number += 1
        seen.add(number)
        assigned.append(number)
    return assigned, next_number, reassigned


def build_order_permutation(orders):
    """``[build_order_per_item] -> [display_position_per_item]``.

    Blender's ``filter_items`` wants a permutation in that direction (the
    new position *of* item i), not a sorted index list -- getting it
    backwards silently scrambles the list rather than erroring, so this is
    split out to be unit-testable without a running UI. Also used directly
    by ``ops/design.py``'s Check By Eye stepping, to walk BUILD order rather
    than the underlying collection's own extraction order (2026-08-23,
    user-reported: stepping followed edge/extraction order, not the order
    the robot actually builds in).

    Unordered sticks (``order == -1``) sort last, keeping their relative
    order, so a partial solve still reads sensibly.
    """
    ranked = sorted(
        range(len(orders)),
        key=lambda i: (orders[i] < 0, orders[i] if orders[i] >= 0 else i),
    )
    permutation = [0] * len(orders)
    for position, original_index in enumerate(ranked):
        permutation[original_index] = position
    return permutation


def topology_signature(edge_ids, stick_lengths_m, places=4):
    """A cheap fingerprint of "the design the order was computed against".

    Sec 12 requires that a mesh edited *after* the order was computed either
    keeps its ids or warns the user that the order is stale. Ids survive
    edits; lengths and membership do not, so both go into the signature.
    """
    parts = [
        "%s:%.*f" % (eid, places, length)
        for eid, length in sorted(zip(edge_ids, stick_lengths_m))
    ]
    return "|".join(parts)


class StickState:
    """One stick's live state (Sec 9.2's table)."""

    __slots__ = ("id", "order", "status", "reason", "stick_length_mm",
                 "expanded_edge_mm", "residual_mm", "flip", "warnings", "at")

    def __init__(self, id, order=-1, status=STATUS_PENDING, reason="",
                 stick_length_mm=0.0, expanded_edge_mm=0.0, residual_mm=0.0,
                 flip=False, warnings=None, at=""):
        self.id = id
        self.order = order
        self.status = status
        self.reason = reason
        self.stick_length_mm = stick_length_mm
        self.expanded_edge_mm = expanded_edge_mm
        self.residual_mm = residual_mm
        self.flip = flip
        self.warnings = list(warnings or ())
        self.at = at

    def to_dict(self):
        return {
            "id": self.id,
            "order": self.order,
            "status": self.status,
            "reason": self.reason,
            "stick_length_mm": round(self.stick_length_mm, 4),
            "expanded_edge_mm": round(self.expanded_edge_mm, 4),
            "residual_mm": round(self.residual_mm, 4),
            "flip": self.flip,
            "warnings": list(self.warnings),
        }

    @classmethod
    def from_dict(cls, data):
        return cls(
            id=data["id"],
            order=data.get("order", -1),
            status=data.get("status", STATUS_PENDING),
            reason=data.get("reason", ""),
            stick_length_mm=data.get("stick_length_mm", 0.0),
            expanded_edge_mm=data.get("expanded_edge_mm", 0.0),
            residual_mm=data.get("residual_mm", 0.0),
            flip=data.get("flip", False),
            warnings=data.get("warnings", ()),
        )


def read_status_sidecar(text):
    """Parse a ``<name>.status.json`` written by ROS2 (BRIDGE_PROTOCOL.md
    Sec A.3). Returns ``(current_index, {stick_id: {status, at, reason}})``.

    Unknown fields are ignored and anything absent is ``pending`` -- both
    per the protocol's additive-evolution rule.
    """
    data = json.loads(text)
    if data.get("format") != "so100_build_status":
        raise ValueError(
            "not a status sidecar: format is %r, expected 'so100_build_status'"
            % data.get("format")
        )
    entries = {}
    for stick_id, entry in (data.get("sticks") or {}).items():
        status = entry.get("status", STATUS_PENDING)
        if status not in SIDECAR_STATUSES:
            status = STATUS_PENDING
        entries[stick_id] = {
            "status": status,
            "at": entry.get("at", ""),
            "reason": entry.get("reason", ""),
        }
    return data.get("current_index", 0), entries


def merge_sidecar(states, sidecar_entries):
    """Apply sidecar statuses onto the in-.blend states.

    Returns ``(updated, conflicts)``. A conflict is a stick whose two sources
    disagree -- Sec 9.3: *on load, if the two disagree, show both and let the
    user choose; never silently pick one.* So this reports them and applies
    nothing for those sticks.
    """
    updated, conflicts = [], []
    by_id = {state.id: state for state in states}
    for stick_id, entry in sidecar_entries.items():
        state = by_id.get(stick_id)
        if state is None:
            conflicts.append((stick_id, "(not in this design)", entry["status"]))
            continue
        if state.status in (STATUS_PLACED, STATUS_FAILED, STATUS_SKIPPED) \
                and state.status != entry["status"]:
            conflicts.append((stick_id, state.status, entry["status"]))
            continue
        if state.status != entry["status"]:
            state.status = entry["status"]
            state.reason = entry["reason"]
            state.at = entry["at"]
            updated.append(stick_id)
    return updated, conflicts
