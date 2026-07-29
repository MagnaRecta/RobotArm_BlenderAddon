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

from ..kinematics.constants import CHAIN, GRASP_OFFSET_M, STICK_SECTION_M
from . import validate as core_validate
from .transform import v_add, v_dot, v_length, v_scale, v_sub

_SHOULDER_AXIS_POINT = CHAIN[0][1]

# --- jaw envelope ------------------------------------------------------------
# ⚠ EVERY NUMBER HERE IS AN ESTIMATE, NOT A MEASUREMENT. ROS2 plan Phase 0:
# "Measure the jaw envelope (width, depth, how far they protrude past the
# TCP)". Sec 8.2 gives "~20 mm across" and says the clear region runs "out
# to ~60 mm" from the target vertex, which is where JAW_LENGTH_M's 30 mm
# (centred on the 51 mm grip point, so spanning 36..66 mm) comes from.
# Treat a jaw-clearance verdict as indicative until Phase 0 lands.
JAW_WIDTH_M = 0.020
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


def jaw_segment(stick, grasp_offset_m=GRASP_OFFSET_M, jaw_length_m=JAW_LENGTH_M):
    """The jaw body's own axis segment, centred on the grip point.

    ⚠ Read ``grasp_offset_m`` from the kinematics module, never hardcode it
    (Sec 5.4) -- Phase 0's ruler check propagates from there.
    """
    axis = stick.axis()
    centre = v_add(stick.base, v_scale(axis, grasp_offset_m))
    half = jaw_length_m * 0.5
    return v_sub(centre, v_scale(axis, half)), v_add(centre, v_scale(axis, half))


def jaw_clearance(stick, placed_sticks,
                  grasp_offset_m=GRASP_OFFSET_M,
                  jaw_width_m=JAW_WIDTH_M,
                  jaw_length_m=JAW_LENGTH_M,
                  section_m=STICK_SECTION_M,
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


def _horizontal_reach(point):
    """Horizontal distance from Shoulder_Rotation's own axis -- Sec 6.1's
    ``distance_from_robot``."""
    return math.hypot(point[0] - _SHOULDER_AXIS_POINT[0],
                      point[1] - _SHOULDER_AXIS_POINT[1])


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


def grounded_vertices(sticks, ground_epsilon_m=0.0005):
    """Vertex indices seating on the base plate."""
    grounded = set()
    for stick in sticks:
        if stick.base[2] <= ground_epsilon_m:
            grounded.add(stick.v_base)
        if stick.tip[2] <= ground_epsilon_m:
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


def floating_components(sticks, ground_epsilon_m=0.0005):
    """Sec 6.2: sub-graphs with no grounded vertex. Unbuildable -- nothing
    supports them."""
    grounded = grounded_vertices(sticks, ground_epsilon_m)
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
                 backtrack_limit=DEFAULT_BACKTRACK_LIMIT,
                 grasp_offset_m=GRASP_OFFSET_M,
                 jaw_width_m=JAW_WIDTH_M,
                 jaw_length_m=JAW_LENGTH_M,
                 section_m=STICK_SECTION_M,
                 approach_clearance_m=APPROACH_CLEARANCE_M,
                 check_jaw_clearance=True):
        self._sticks = {s.id: s for s in sticks}
        self._ground_epsilon_m = ground_epsilon_m
        self._backtrack_limit = backtrack_limit
        self._grasp_offset_m = grasp_offset_m
        self._jaw_width_m = jaw_width_m
        self._jaw_length_m = jaw_length_m
        self._section_m = section_m
        self._approach_clearance_m = approach_clearance_m
        self._check_jaw_clearance = check_jaw_clearance

        self.result = OrderResult()
        # Not per-placement, so unaffected by backtracking -- see _finalize.
        self._global_warnings = []
        self._global_errors = []

        # Sec 6.2: floating components can never be placed -- detect them up
        # front so the report names them, rather than letting the search
        # thrash and then fail with something vague.
        self._excluded_ids = set()
        for group in floating_components(sticks, ground_epsilon_m):
            ids = sorted(s.id for s in group)
            for stick_id in ids:
                self._excluded_ids.add(stick_id)
            self._global_errors.append((
                None, ERROR_FLOATING_COMPONENT,
                "%d stick%s (%s%s) form a component with no vertex on the base "
                "plate -- nothing supports it, so it cannot be built"
                % (len(ids), "" if len(ids) == 1 else "s",
                   ", ".join(ids[:4]), " ..." if len(ids) > 4 else ""),
            ))
        self.result.unordered = sorted(self._excluded_ids)

        self._remaining = {
            s.id for s in sticks if s.id not in self._excluded_ids
        }
        self._total = len(self._remaining)
        self._available = grounded_vertices(sticks, ground_epsilon_m)
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
            verdict = core_validate.validate_stick(oriented)
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
        return (max_z, -_horizontal_reach(midpoint), -supports)

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
        self._available = grounded_vertices(
            list(self._sticks.values()), self._ground_epsilon_m)
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


def compute_order(sticks, **kwargs):
    """One-shot ordering. See ``OrderSolver`` for the chunkable form."""
    return OrderSolver(sticks, **kwargs).solve()
