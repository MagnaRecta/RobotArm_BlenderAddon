"""Build-order solver -- the "slicer" algorithm. BLENDER_ADDON_PLAN.md Sec 6,
Phase C.

Sec 6's three constraints, in priority order:

* **C1 Support.** A stick can only be placed if one of its ends is on the
  base plate or at a vertex of an already-placed stick. Nothing floats. This
  is a *dependency graph* constraint, not a height one -- which is exactly
  what a pure Z sort silently violates.
* **C2 Accessibility.** Build bottom-up, and far-from-robot first within a
  layer (the arm reaches outward horizontally, so a low stick behind a
  finished tall structure is blocked).
* **C3 Jaw clearance.** The jaws sit only ~51 mm above the stick's base --
  i.e. right at the glue joint. The region around the grip point must be
  clear of already-placed sticks.

⚠ **Ordering and buildability are the same problem** (ROS2 plan Sec 10.1):
reachability shrinks as the sculpture grows, so a stick that is trivially
reachable on an empty table can be walled in by the time its turn comes.
That is why ``validate()`` is called *inside* the search rather than as a
separate pass, and why this module exists at all instead of a Z sort.

## "base" means something different here than in Phase B

Sec 5.1's rule (base = the lower end) is a *default*; Sec 6 makes it
**structural** -- the base is the end that seats on the plate or on an
already-placed stick. This solver therefore decides orientation from the
support graph, superseding the reachability-driven auto-flip that
``ops/design.py`` applies at extraction time (README Finding 6). The two
only ever disagree when a stick has just one supported end, and in that
case structure has to win -- flipping it would leave it floating.

When **both** ends are supported (a loop closure, or the horizontal top of
an inverted U once both uprights are up) the solver is free to choose, and
it picks whichever orientation actually validates -- reproducing the
auto-flip benefit without the conflict.

Re-running *Extract Sticks* clears the order (``order`` resets to -1), so
flips fall back to the reachability heuristic until the order is recomputed.
No lasting inconsistency, but it does mean **Compute Build Order must be
re-run after any re-extraction** -- which the staleness check already tells
the user.

## Pure Python

No ``bpy``, no numpy (constraint B4), and chunkable: ``OrderSolver.step()``
does a bounded amount of work and returns, so the operator can drive it
across timer ticks without freezing the UI (constraint B3). This is the
phase where that actually matters -- extraction is sub-10 ms, but ordering
is a search with an IK call inside its inner loop.
"""

import copy
import math

from . import robots as core_robots
from . import validate as core_validate
from .transform import v_add, v_dot, v_length, v_scale, v_sub

# Multi-robot support (docs/STATUS.md 2026-08-21/2026-08-23): the shoulder-
# axis reference point and the jaw-model defaults below now come from
# whichever robot's ``OrderSolver`` is constructed for (its own
# ``CHAIN[0][1]``/``GRASP_OFFSET_M``/``JAW_RADIUS_M``), not so_arm_100's
# unconditionally -- previously EVERY reachability check inside this
# solver silently ran so_arm_100's kinematics regardless of the selected
# robot, so a design correctly validated as buildable for kr10_r900_2
# (``core/validate.py``, robot-aware since 2026-08-22) would still see
# every candidate rejected here, backtrack immediately, and report a
# single opaque "no candidate could be placed" or "floating component"
# error with no obvious cause -- exactly what motivated this fix.
#
# ``jaw_segment``/``jaw_clearance``/``_horizontal_reach`` below are still
# general-purpose geometry helpers usable standalone (as the tests do) --
# their own keyword defaults stay so_arm_100's own constants, unchanged, for
# exactly that reason. ``OrderSolver`` resolves and passes its own robot's
# values explicitly at every call site instead of relying on those defaults.
_SO_ARM_100 = core_robots.get_robot(core_robots.SO_ARM_100_ID).kinematics
# ``.constants`` is reachable even though so_arm_100's own top-level package
# does not re-export CHAIN (kr10_r900_2's does -- packages differ) -- see
# core/mirror.py's own ``_tool_offset`` for why ``pkg.constants`` always
# works regardless: importing a submodule via ``from .constants import X``
# inside ``__init__.py`` always binds it as a real attribute of the package.
_SO_ARM_100_SHOULDER_AXIS_POINT = _SO_ARM_100.constants.CHAIN[0][1]

# --- jaw envelope ------------------------------------------------------------
# ⚠ JAW_LENGTH_M is still a so_arm_100-SHAPED estimate, not (yet) sourced
# per robot. ROS2 plan Phase 0: "Measure the jaw envelope (width, depth, how
# far they protrude past the TCP)". Sec 8.2 gives "~20 mm across" and says
# the clear region runs "out to ~60 mm" from the target vertex, which is
# where 30 mm (centred on so_arm_100's 51 mm grip point, so spanning
# 36..66 mm) comes from. kr10_r900_2's own grip offset (18.8 mm) and real
# jaw envelope are a different scale entirely (its own gripper mesh bbox is
# 23.65 x 15.39 x 36.8 mm -- see that package's README) -- do not assume
# this constant scales sensibly for it without real measurements; it is
# used here as-is (a soft, over-cautious pre-filter, never a hard block)
# rather than inventing a KUKA-specific formula with no evidence behind it.
JAW_LENGTH_M = 0.030

# ROS2 plan Sec 7.2: `place.approach_clearance`, default 0.05 m. The jaws
# sweep down through this before the stick seats, so the swept volume --
# not just the final pose -- has to be clear.
APPROACH_CLEARANCE_M = 0.05
# The sweep is checked at this many sampled heights rather than as a true
# swept solid. Sampling is honest here: this whole check is a conservative
# pre-filter and MoveIt does the real path collision test (Sec 7).
APPROACH_SAMPLES = 5

# Sec 6.2 "Cantilever -- a stick glued at one end only, FAR FROM VERTICAL".
# A one-anchor vertical stick is stable-ish while the glue sets; the risk
# scales with how far the stick leans, since that is what turns the joint
# into a moment arm. 30 deg is a judgement call, not a measurement -- it is
# a warning, never a block.
CANTILEVER_ANGLE_RAD = math.radians(30.0)

# Sec 6.1: "cap the backtrack depth and report honestly if the cap is hit".
DEFAULT_BACKTRACK_LIMIT = 500

# Candidate evaluations per step() call. Tuned so a tick stays well inside
# a frame at 60 Hz even when every evaluation misses the reachability cache.
DEFAULT_STEP_BUDGET = 400

WARN_CANTILEVER = "cantilever"
WARN_LOOP_CLOSURE = "loop_closure"
WARN_JAW_CLEARANCE = "jaw_clearance"
WARN_NO_BUILD_PLATE = "no_build_plate"

ERROR_FLOATING_COMPONENT = "floating_component"
ERROR_FORCED_PLACEMENT = "forced_placement"


# --- geometry ----------------------------------------------------------------


def segment_distance(p1, q1, p2, q2):
    """Minimum distance between two 3D segments.

    The standard clamped-parametric solve (Ericson, *Real-Time Collision
    Detection* Sec 5.1.9), written out because ``core/`` is numpy-free. All
    the degenerate branches matter here: a "segment" can legitimately be a
    point when a stick is shorter than the sampling step.
    """
    d1 = v_sub(q1, p1)
    d2 = v_sub(q2, p2)
    r = v_sub(p1, p2)
    a = v_dot(d1, d1)
    e = v_dot(d2, d2)
    f = v_dot(d2, r)
    eps = 1e-12

    if a <= eps and e <= eps:
        return v_length(r)
    if a <= eps:
        s = 0.0
        t = max(0.0, min(1.0, f / e))
    else:
        c = v_dot(d1, r)
        if e <= eps:
            t = 0.0
            s = max(0.0, min(1.0, -c / a))
        else:
            b = v_dot(d1, d2)
            denom = a * e - b * b
            if denom > eps:
                s = max(0.0, min(1.0, (b * f - c * e) / denom))
            else:
                # Parallel: any s is as good; take the segment start and let
                # the t-clamp below place the other point.
                s = 0.0
            t = (b * s + f) / e
            if t < 0.0:
                t = 0.0
                s = max(0.0, min(1.0, -c / a))
            elif t > 1.0:
                t = 1.0
                s = max(0.0, min(1.0, (b - c) / a))

    c1 = v_add(p1, v_scale(d1, s))
    c2 = v_add(p2, v_scale(d2, t))
    return v_length(v_sub(c1, c2))


def jaw_segment(stick, grasp_offset_m=_SO_ARM_100.GRASP_OFFSET_M, jaw_length_m=JAW_LENGTH_M):
    """The jaw body's own axis segment, centred on the grip point.

    ⚠ Read ``grasp_offset_m`` from the kinematics module, never hardcode it
    (Sec 5.4) -- Phase 0's ruler check propagates from there.
    """
    axis = stick.axis()
    centre = v_add(stick.base, v_scale(axis, grasp_offset_m))
    half = jaw_length_m * 0.5
    return v_sub(centre, v_scale(axis, half)), v_add(centre, v_scale(axis, half))


def jaw_clearance(stick, placed_sticks,
                  grasp_offset_m=_SO_ARM_100.GRASP_OFFSET_M,
                  jaw_width_m=_SO_ARM_100.JAW_RADIUS_M * 2.0,
                  jaw_length_m=JAW_LENGTH_M,
                  section_m=_SO_ARM_100.STICK_SECTION_M,
                  approach_clearance_m=APPROACH_CLEARANCE_M,
                  approach_samples=APPROACH_SAMPLES):
    """Sec 6 C3 / Sec 8.2: is the grip region clear of what is already built?

    Returns ``(ok, worst_gap_m, offender_id)``. The jaws are modelled as a
    **cylinder around the stick's own axis** -- rotationally symmetric,
    which is deliberately conservative: the real jaws are a box whose
    orientation depends on ``stick_roll``, and that roll convention is
    still unverified on hardware (kinematics README caveat), so assuming
    the worst orientation is the honest choice.

    The base plate is deliberately **not** modelled. A horizontal stick
    lying on the plate has its grip point at plate height, so a symmetric
    cylinder would report a clash for every flat first-layer stick -- yet
    physically the jaws come down from above and sit *over* it. The plate
    is a static collision object on the ROS2 side (plan Sec 10) and MoveIt
    checks it properly.
    """
    if not placed_sticks:
        return True, float("inf"), None

    low, high = jaw_segment(stick, grasp_offset_m, jaw_length_m)
    required = jaw_width_m * 0.5 + section_m * 0.5

    # Cheap reject radius: anything further than this from the jaw's centre
    # cannot possibly be within `required` of it at any sweep height.
    centre = v_scale(v_add(low, high), 0.5)
    reject = (jaw_length_m * 0.5 + required + approach_clearance_m
              + 0.5 * max(s.length_m for s in placed_sticks))

    worst = float("inf")
    offender = None
    samples = max(1, approach_samples)
    for index in range(samples):
        lift = approach_clearance_m * (samples - 1 - index) / max(1, samples - 1)
        offset = (0.0, 0.0, lift)
        jaw_low = v_add(low, offset)
        jaw_high = v_add(high, offset)
        for other in placed_sticks:
            other_centre = v_scale(v_add(other.base, other.tip), 0.5)
            if v_length(v_sub(other_centre, centre)) > reject + lift:
                continue
            gap = segment_distance(jaw_low, jaw_high, other.base, other.tip)
            if gap < worst:
                worst = gap
                offender = other.id

    return worst >= required, worst, offender


def _horizontal_reach(point, shoulder_axis_point=_SO_ARM_100_SHOULDER_AXIS_POINT):
    """Horizontal distance from the robot's own first-joint axis -- Sec
    6.1's ``distance_from_robot``. ``OrderSolver`` passes its own robot's
    point explicitly (see module docstring); the default here is
    so_arm_100's, for standalone/backward-compatible callers."""
    return math.hypot(point[0] - shoulder_axis_point[0],
                      point[1] - shoulder_axis_point[1])


def flip_stick(stick):
    """A copy with base/tip (and their vertex indices) swapped.

    The physical segment does not move -- ``core/sticks.py`` computes the
    inset endpoints *before* deciding which is base, so flipping only ever
    relabels which end seats down.
    """
    other = copy.copy(stick)
    other.base, other.tip = stick.tip, stick.base
    other.v_base, other.v_tip = stick.v_tip, stick.v_base
    other.flipped = not stick.flipped
    return other


# --- support graph -----------------------------------------------------------


def grounded_vertices(sticks, ground_epsilon_m=0.0005, ground_height_m=0.0):
    """Vertex indices seating on the base plate.

    ``ground_height_m`` (2026-08-23): the plate's own Z. 0.0 by default, but
    the plate is a real, height-adjustable object -- raising this lets a
    design that sits entirely above Z=0 be treated as resting on it, same
    frame convention as ``core.sticks``'s own ``Topology.ground_height_m``.
    """
    threshold = ground_height_m + ground_epsilon_m
    grounded = set()
    for stick in sticks:
        if stick.base[2] <= threshold:
            grounded.add(stick.v_base)
        if stick.tip[2] <= threshold:
            grounded.add(stick.v_tip)
    return grounded


def components(sticks):
    """Connected components of the stick graph, as lists of sticks."""
    adjacency = {}
    for stick in sticks:
        adjacency.setdefault(stick.v_base, []).append(stick)
        adjacency.setdefault(stick.v_tip, []).append(stick)

    seen_sticks = set()
    out = []
    for stick in sticks:
        if stick.id in seen_sticks:
            continue
        group = []
        stack = [stick]
        seen_sticks.add(stick.id)
        while stack:
            current = stack.pop()
            group.append(current)
            for vertex in (current.v_base, current.v_tip):
                for neighbour in adjacency.get(vertex, ()):
                    if neighbour.id not in seen_sticks:
                        seen_sticks.add(neighbour.id)
                        stack.append(neighbour)
        out.append(group)
    return out


def arbitrary_anchor_vertices(sticks):
    """One vertex per connected component (2026-08-23) -- used by
    ``OrderSolver`` when ``ground_required=False``: there is no plate to
    test against, so each disconnected component still needs ONE anchor to
    seed the build order from. Picks each component's lowest-id stick's own
    base end, deterministically -- not a physically meaningful choice (there
    is no plate height to prefer), just a stable one. Never claims the
    result is physically self-supporting; the caller is trusted to know how
    the assembly is actually held (a jig, a non-flat fixture, ...)."""
    anchors = set()
    for group in components(sticks):
        first = min(group, key=lambda s: s.id)
        anchors.add(first.v_base)
    return anchors


def floating_components(sticks, ground_epsilon_m=0.0005, ground_height_m=0.0):
    """Sec 6.2: sub-graphs with no grounded vertex. Unbuildable -- nothing
    supports them -- AT the plate's CURRENT height (``ground_height_m``);
    see ``grounded_vertices``."""
    grounded = grounded_vertices(sticks, ground_epsilon_m, ground_height_m)
    return [
        group for group in components(sticks)
        if not any(s.v_base in grounded or s.v_tip in grounded for s in group)
    ]


# --- results -----------------------------------------------------------------


class OrderedStick:
    __slots__ = ("stick", "order", "warnings", "reason", "messages", "supports")

    def __init__(self, stick, order, warnings=(), reason=None, messages=(),
                 supports=()):
        self.stick = stick          # the ORIENTED spec (may be a flipped copy)
        self.order = order
        self.warnings = list(warnings)
        self.reason = reason
        # BRIDGE_PROTOCOL.md A.2: ids of already-placed sticks this one's
        # BASE end attaches to; empty means it seats on the base plate. Only
        # the solver knows this -- it depends on the order, not the geometry.
        self.supports = list(supports)
        # (code, message) pairs owned by THIS placement. Kept here rather
        # than pushed straight onto OrderResult because backtracking undoes
        # placements -- anything appended to the result during the search
        # would survive the undo and report warnings for sticks that ended
        # up somewhere else entirely.
        self.messages = list(messages)

    @property
    def id(self):
        return self.stick.id

    def __repr__(self):
        return "OrderedStick(%r, order=%d, warnings=%r)" % (
            self.stick.id, self.order, self.warnings)


class OrderResult:
    __slots__ = ("ordered", "unordered", "warnings", "errors", "backtracks",
                 "complete", "forced")

    def __init__(self):
        self.ordered = []      # list[OrderedStick], in build order
        self.unordered = []    # stick ids that could not be ordered at all
        self.warnings = []     # (stick_id or None, code, message)
        self.errors = []       # (stick_id or None, code, message)
        self.backtracks = 0
        self.complete = False
        self.forced = 0        # placed despite failing validation

    def by_id(self):
        return {entry.id: entry for entry in self.ordered}

    def summary(self):
        return "%d ordered - %d warnings - %d errors%s" % (
            len(self.ordered), len(self.warnings), len(self.errors),
            "" if self.complete else " (INCOMPLETE)",
        )


# --- the solver --------------------------------------------------------------


class _Decision:
    __slots__ = ("chosen", "excluded")

    def __init__(self, chosen, excluded):
        self.chosen = chosen
        self.excluded = excluded


class OrderSolver:
    """Sec 6.1's greedy topological build with backtracking, chunkable.

    Call ``step()`` repeatedly until it returns True, then read ``result``.
    ``step()`` is bounded, so a caller on Blender's main thread can drive it
    from a timer without blocking (constraint B3).
    """

    def __init__(self, sticks,
                 ground_epsilon_m=0.0005,
                 ground_height_m=0.0,
                 ground_required=True,
                 backtrack_limit=DEFAULT_BACKTRACK_LIMIT,
                 robot_id=core_robots.SO_ARM_100_ID,
                 grasp_offset_m=None,
                 jaw_width_m=None,
                 jaw_length_m=JAW_LENGTH_M,
                 section_m=None,
                 approach_clearance_m=APPROACH_CLEARANCE_M,
                 check_jaw_clearance=True):
        self._sticks = {s.id: s for s in sticks}
        self._ground_epsilon_m = ground_epsilon_m
        # The build plate's own Z (2026-08-23) -- 0.0 unless raised; see
        # core.sticks's own ground_height_m docstring for the rationale.
        self._ground_height_m = ground_height_m
        # 2026-08-23, user request: a design held by something this addon
        # does not model (a stick's own base as a jig, a non-flat fixture)
        # rather than by a flat plate at any height -- see core.sticks's own
        # ground_required docstring for the full rationale. False turns off
        # both the floating_component exclusion and grounded_vertices() as
        # the seed for _available, replacing them with
        # arbitrary_anchor_vertices() so the search still has somewhere to
        # start each disconnected component.
        self._ground_required = ground_required
        self._backtrack_limit = backtrack_limit
        # Multi-robot support: everything reachability-related below goes
        # through the SELECTED robot's own kinematics, not so_arm_100's
        # unconditionally (module docstring). `None` for grasp_offset_m/
        # jaw_width_m/section_m means "use this robot's own default" --
        # explicit values (as ops/order.py's build_solver() already passes
        # for section_m/jaw_width_m, from the Design panel's UI fields)
        # still override, unchanged.
        self._robot_id = robot_id
        self._kinematics = core_robots.get_robot(robot_id).kinematics
        self._shoulder_axis_point = self._kinematics.constants.CHAIN[0][1]
        self._grasp_offset_m = (
            self._kinematics.GRASP_OFFSET_M if grasp_offset_m is None else grasp_offset_m)
        self._jaw_width_m = (
            self._kinematics.JAW_RADIUS_M * 2.0 if jaw_width_m is None else jaw_width_m)
        self._jaw_length_m = jaw_length_m
        self._section_m = (
            self._kinematics.STICK_SECTION_M if section_m is None else section_m)
        self._approach_clearance_m = approach_clearance_m
        self._check_jaw_clearance = check_jaw_clearance

        self.result = OrderResult()
        # Not per-placement, so unaffected by backtracking -- see _finalize.
        self._global_warnings = []
        self._global_errors = []

        # Sec 6.2: floating components can never be placed -- detect them up
        # front so the report names them, rather than letting the search
        # thrash and then fail with something vague. Skipped entirely when
        # ground_required=False: there is no plate to be floating relative
        # to, so nothing is ever excluded on that basis.
        self._excluded_ids = set()
        if ground_required:
            for group in floating_components(sticks, ground_epsilon_m, ground_height_m):
                ids = sorted(s.id for s in group)
                for stick_id in ids:
                    self._excluded_ids.add(stick_id)
                lowest_z = min(min(s.base[2], s.tip[2]) for s in group)
                self._global_errors.append((
                    None, ERROR_FLOATING_COMPONENT,
                    "%d stick%s (%s%s) form a component with no vertex within "
                    "%.2f mm of the build plate (currently at Z=%.1f mm) in the "
                    "selected robot's own base frame -- nothing supports it at "
                    "this plate height. Its own lowest point is %.1f mm; raising "
                    "the build plate (ground_height_m) to about there would "
                    "ground it, or uncheck Require Build Plate if it is held by "
                    "something else entirely. If every stick looked buildable at "
                    "extraction, also check that the design mesh actually sits "
                    "on the robot base empty's own ground plane (its position/"
                    "rotation), not just near the world origin"
                    % (len(ids), "" if len(ids) == 1 else "s",
                       ", ".join(ids[:4]), " ..." if len(ids) > 4 else "",
                       ground_epsilon_m * 1000.0, ground_height_m * 1000.0,
                       lowest_z * 1000.0),
                ))
        else:
            self._global_warnings.append((
                None, WARN_NO_BUILD_PLATE,
                "Require Build Plate is off -- no vertex is checked against "
                "the plate, and each disconnected part of the design starts "
                "from an arbitrary anchor point rather than a grounded one. "
                "This does not verify the result is physically self-"
                "supporting; a stick reported as seating on the plate may "
                "instead need external support you are providing yourself",
            ))
        self.result.unordered = sorted(self._excluded_ids)

        self._remaining = {
            s.id for s in sticks if s.id not in self._excluded_ids
        }
        self._total = len(self._remaining)
        self._available = (
            grounded_vertices(sticks, ground_epsilon_m, ground_height_m)
            if ground_required else arbitrary_anchor_vertices(sticks)
        )
        self._placed = []                 # oriented specs, in order
        self._path = []                   # list[_Decision]
        self._excluded_at = [set()]       # per depth
        self._reach_cache = {}
        self._hopeless_cache = {}

        self._queue = None                # pending candidate orientations
        self._queue_index = 0
        self._soft_fallback = None        # (oriented, gap, offender)
        self._best_effort = None          # (oriented, reason)
        self._all_hopeless = False
        self._finished = self._total == 0
        if self._finished:
            self.result.complete = not self._excluded_ids
            self._finalize()

    # --- progress ------------------------------------------------------------

    @property
    def done(self):
        return self._finished

    @property
    def progress(self):
        """Fraction placed, 0..1.

        ⚠ Can move *backwards*: backtracking genuinely un-places sticks, and
        reporting a high-water mark instead would hide a solve that is
        thrashing. A caller driving a progress bar should expect that.
        """
        if self._total == 0:
            return 1.0
        return len(self._placed) / float(self._total)

    # --- evaluation ----------------------------------------------------------

    def _reachability(self, oriented):
        key = (oriented.id, oriented.flipped)
        verdict = self._reach_cache.get(key)
        if verdict is None:
            verdict = core_validate.validate_stick(oriented, self._robot_id)
            self._reach_cache[key] = verdict
        return verdict

    def _is_hopeless(self, stick):
        """True when NO orientation of this stick is reachable.

        Phase B reachability depends only on the stick's own geometry, not
        on what is already placed (only jaw clearance is scene-dependent) --
        so if both orientations fail, **no build order can ever fix it** and
        backtracking over it is pure waste. Without this, one unreachable
        stick burns the entire backtrack budget before the solver gives up.
        Evaluated lazily and cached, so it costs nothing until something
        actually fails.
        """
        hopeless = self._hopeless_cache.get(stick.id)
        if hopeless is None:
            base = self._sticks[stick.id]
            hopeless = not (self._reachability(base).buildable
                            or self._reachability(flip_stick(base)).buildable)
            self._hopeless_cache[stick.id] = hopeless
        return hopeless

    def _orientations(self, stick):
        """Structurally-valid orientations, lower base first.

        C1: the base must be the supported end. With both ends supported the
        choice is free, so both are offered and whichever validates wins.
        """
        options = []
        if stick.v_base in self._available:
            options.append(stick)
        if stick.v_tip in self._available:
            options.append(flip_stick(stick))
        options.sort(key=lambda s: s.base[2])
        return options

    def _cost(self, stick):
        """Sec 6.1's cost, ascending: build upward, far side first, prefer
        better-anchored."""
        max_z = max(stick.base[2], stick.tip[2])
        midpoint = v_scale(v_add(stick.base, stick.tip), 0.5)
        supports = int(stick.v_base in self._available) + int(
            stick.v_tip in self._available)
        return (max_z, -_horizontal_reach(midpoint, self._shoulder_axis_point), -supports)

    def _placement_warnings(self, oriented):
        warnings = []
        base_supported = oriented.v_base in self._available
        tip_supported = oriented.v_tip in self._available

        if base_supported and tip_supported:
            # Sec 6.2: must fit exactly between two already-glued vertices,
            # so it absorbs all accumulated positioning and cutting error.
            warnings.append(WARN_LOOP_CLOSURE)
        elif base_supported:
            axis = oriented.axis()
            lean = math.acos(max(-1.0, min(1.0, abs(axis[2]))))
            if lean > CANTILEVER_ANGLE_RAD:
                # Glued at one end only and leaning: gravity acts on it the
                # moment the robot lets go, before the glue sets.
                warnings.append(WARN_CANTILEVER)
        return warnings

    # --- the search ----------------------------------------------------------

    def _begin_decision(self):
        excluded = self._excluded_at[-1]
        candidates = []
        for stick_id in self._remaining:
            if stick_id in excluded:
                continue
            stick = self._sticks[stick_id]
            if stick.v_base in self._available or stick.v_tip in self._available:
                candidates.append(stick)

        if not candidates:
            self._queue = None
            return False

        candidates.sort(key=self._cost)
        queue = []
        for stick in candidates:
            queue.extend(self._orientations(stick))
        self._queue = queue
        self._queue_index = 0
        self._soft_fallback = None
        self._best_effort = None
        # If every option here is unreachable in principle, backtracking
        # cannot help -- see _is_hopeless.
        self._all_hopeless = all(self._is_hopeless(s) for s in candidates)
        return True

    def _commit(self, oriented, warnings, reason=None, messages=()):
        # Computed BEFORE this stick joins _placed, so it lists only sticks
        # that genuinely precede it.
        supports = [
            placed.id for placed in self._placed
            if oriented.v_base in (placed.v_base, placed.v_tip)
        ]
        self._path.append(_Decision(oriented, set(self._excluded_at[-1])))
        self._excluded_at.append(set())
        self._remaining.discard(oriented.id)
        self._placed.append(oriented)
        self._available.add(oriented.v_base)
        self._available.add(oriented.v_tip)
        self.result.ordered.append(
            OrderedStick(oriented, len(self.result.ordered), warnings, reason,
                         messages, supports))
        self._queue = None

    def _rebuild_available(self):
        sticks = list(self._sticks.values())
        self._available = (
            grounded_vertices(sticks, self._ground_epsilon_m, self._ground_height_m)
            if self._ground_required else arbitrary_anchor_vertices(sticks)
        )
        for oriented in self._placed:
            self._available.add(oriented.v_base)
            self._available.add(oriented.v_tip)

    def _backtrack(self):
        """Undo the last choice and forbid it at that depth. Returns False
        when there is nothing left to undo, or the cap is hit."""
        if not self._path or self.result.backtracks >= self._backtrack_limit:
            return False
        decision = self._path.pop()
        self._excluded_at.pop()
        self._excluded_at[-1].add(decision.chosen.id)
        self._remaining.add(decision.chosen.id)
        self._placed.pop()
        self.result.ordered.pop()
        self._rebuild_available()
        self.result.backtracks += 1
        self._queue = None
        return True

    def _force_place(self):
        """Sec 6.1 says to "report honestly if the cap is hit"; the protocol
        (BRIDGE_PROTOCOL.md A.2) additionally requires that a stick which
        fails validation still appears in the file, in order, so the
        operator sees the whole picture. So rather than abandoning the
        build, place the least-bad candidate and record why."""
        if self._best_effort is not None:
            oriented, reason = self._best_effort
        elif self._soft_fallback is not None:
            oriented, gap, offender = self._soft_fallback
            reason = ("jaws come within %.1f mm of %s and no order avoids it"
                      % (gap * 1000.0, offender))
        else:
            return False
        warnings = self._placement_warnings(oriented)
        self._commit(oriented, warnings, reason,
                     [(ERROR_FORCED_PLACEMENT, reason)])
        return True

    def _finalize(self):
        """Rebuild the per-stick warning/error lists from the order that
        actually survived. Anything appended during the search would still
        be there after a backtrack undid the placement that produced it."""
        warnings = list(self._global_warnings)
        errors = list(self._global_errors)
        forced = 0
        for entry in self.result.ordered:
            for code, message in entry.messages:
                if code == ERROR_FORCED_PLACEMENT:
                    errors.append((entry.id, code, message))
                    forced += 1
                else:
                    warnings.append((entry.id, code, message))
        self.result.warnings = warnings
        self.result.errors = errors
        self.result.forced = forced

    def step(self, budget=DEFAULT_STEP_BUDGET):
        """Advance by at most ``budget`` candidate evaluations. Returns True
        when the solve is finished."""
        if self._finished:
            return True

        evaluations = 0
        while evaluations < budget:
            if not self._remaining:
                self._finished = True
                self.result.complete = not self._excluded_ids
                self._finalize()
                return True

            if self._queue is None:
                if not self._begin_decision():
                    # Nothing here is even supportable -- force-placing
                    # would only put a floating stick in the order, so the
                    # honest options are backtrack or stop.
                    if self._backtrack():
                        continue
                    self._finished = True
                    self._report_stuck()
                    self._finalize()
                    return True
                continue

            if self._queue_index >= len(self._queue):
                # Exhausted this depth's candidates.
                if self._soft_fallback is not None:
                    # Jaw clearance is a soft constraint: the jaw envelope is
                    # an ESTIMATE (Phase 0 measures it), so a clash is worth
                    # reporting but not worth refusing to build over.
                    oriented, gap, offender = self._soft_fallback
                    warnings = self._placement_warnings(oriented)
                    warnings.append(WARN_JAW_CLEARANCE)
                    message = (
                        "jaws come within %.1f mm of %s (need %.1f mm) -- no "
                        "order avoids it, so this is placed anyway; check it by eye"
                        % (gap * 1000.0, offender,
                           (self._jaw_width_m * 0.5 + self._section_m * 0.5) * 1000.0)
                    )
                    self._commit(oriented, warnings, None,
                                 [(WARN_JAW_CLEARANCE, message)])
                    continue
                if not self._all_hopeless and self._backtrack():
                    continue
                if self._force_place():
                    continue
                self._finished = True
                self._report_stuck()
                self._finalize()
                return True

            oriented = self._queue[self._queue_index]
            self._queue_index += 1
            evaluations += 1

            verdict = self._reachability(oriented)
            if not verdict.buildable:
                if self._best_effort is None:
                    self._best_effort = (oriented, verdict.reason)
                continue

            if self._check_jaw_clearance:
                ok, gap, offender = jaw_clearance(
                    oriented, self._placed,
                    grasp_offset_m=self._grasp_offset_m,
                    jaw_width_m=self._jaw_width_m,
                    jaw_length_m=self._jaw_length_m,
                    section_m=self._section_m,
                    approach_clearance_m=self._approach_clearance_m,
                )
                if not ok:
                    if self._soft_fallback is None:
                        self._soft_fallback = (oriented, gap, offender)
                    continue

            warnings = self._placement_warnings(oriented)
            warnings.extend(verdict.warnings)
            self._commit(oriented, warnings)

        return False

    def _report_stuck(self):
        remaining = sorted(self._remaining)
        if not remaining:
            self.result.complete = not self._excluded_ids
            return
        self.result.unordered.extend(remaining)
        self.result.unordered = sorted(set(self.result.unordered))
        self._global_errors.append((
            None, "no_valid_order",
            "%d stick%s could not be placed in any order the solver tried "
            "(%d backtracks, cap %d): %s%s"
            % (len(remaining), "" if len(remaining) == 1 else "s",
               self.result.backtracks, self._backtrack_limit,
               ", ".join(remaining[:4]), " ..." if len(remaining) > 4 else ""),
        ))

    def solve(self, budget=DEFAULT_STEP_BUDGET):
        """Run to completion. Convenience for tests and background runs --
        the interactive path drives ``step()`` from a timer instead."""
        while not self.step(budget):
            pass
        return self.result

    def replay(self, sequence_ids):
        """Places sticks in EXACTLY ``sequence_ids``'s order, never
        searching for a better one -- ``solve()``'s counterpart for
        applying a user's own manual reorder (2026-08-23, "move a step
        before/after its position") without a fresh automatic search
        silently overriding it.

        Each stick's ORIENTATION (which end is ``base``) is taken exactly
        as it already is on the ``StickSpec`` passed to this solver -- from
        the ORIGINAL solve's own flip decision, carried forward through
        ``props.sticks[i].flip`` / ``extract_sticks(flips=...)`` -- and
        never re-decided here. Re-flipping would silently change which end
        glues to what out from under a user who is deliberately curating a
        sequence; if the existing orientation no longer has a supported
        base at its new position, that is reported as an error instead
        (below), not silently fixed by flipping it.

        What IS re-checked, because it genuinely depends on order:

        * **C1 support** -- does ``v_base`` attach to something already
          placed (or the plate/an anchor) at THIS position in the sequence?
        * **C3 jaw clearance** -- depends on what is already built, so a
          move can introduce (or remove) a clash even though the stick's
          own geometry has not changed.

        Reachability itself (Phase B) depends only on the stick's own fixed
        base/tip, never on order, so it is not order-sensitive and is
        simply read via ``_reachability()``'s own cache.

        A stick whose new position breaks C1 or jaw clearance is still
        placed there, flagged with a clear reason (BRIDGE_PROTOCOL.md A.2:
        every stick appears in the file, in order), exactly like ``solve()``
        's own force-place path -- never silently dropped or silently
        reordered again out from under the user.

        Any stick id present in ``self._remaining`` but missing from
        ``sequence_ids`` (typically a stick added since the sequence was
        last stored, so it has no manually-chosen position yet) is placed
        afterward, in a stable id-sorted order, rather than left out of the
        result entirely.
        """
        if self._finished:
            return self.result

        seen = set()
        ids_to_place = []
        for stick_id in sequence_ids:
            if stick_id not in seen:
                seen.add(stick_id)
                ids_to_place.append(stick_id)
        ids_to_place.extend(
            sorted(stick_id for stick_id in self._remaining if stick_id not in seen))

        for stick_id in ids_to_place:
            if stick_id not in self._remaining:
                continue
            oriented = self._sticks[stick_id]
            verdict = self._reachability(oriented)
            base_supported = oriented.v_base in self._available

            reason = None
            messages = []
            warnings = []
            if not base_supported:
                reason = ("base does not attach to anything already built or "
                          "to the plate at this position in the order")
                messages = [(ERROR_FORCED_PLACEMENT, reason)]
            elif not verdict.buildable:
                reason = verdict.reason
                messages = [(ERROR_FORCED_PLACEMENT, reason)]
            else:
                warnings = list(verdict.warnings)
                if self._check_jaw_clearance:
                    ok, gap, offender = jaw_clearance(
                        oriented, self._placed,
                        grasp_offset_m=self._grasp_offset_m,
                        jaw_width_m=self._jaw_width_m,
                        jaw_length_m=self._jaw_length_m,
                        section_m=self._section_m,
                        approach_clearance_m=self._approach_clearance_m,
                    )
                    if not ok:
                        message = (
                            "jaws come within %.1f mm of %s (need %.1f mm) at "
                            "this position in the order; check it by eye"
                            % (gap * 1000.0, offender,
                               (self._jaw_width_m * 0.5 + self._section_m * 0.5)
                               * 1000.0)
                        )
                        warnings.append(WARN_JAW_CLEARANCE)
                        messages = [(WARN_JAW_CLEARANCE, message)]

            warnings.extend(self._placement_warnings(oriented))
            self._commit(oriented, warnings, reason, messages)

        self._finished = True
        self.result.complete = not self._excluded_ids
        self._finalize()
        return self.result


def compute_order(sticks, **kwargs):
    """One-shot ordering. See ``OrderSolver`` for the chunkable form."""
    return OrderSolver(sticks, **kwargs).solve()
