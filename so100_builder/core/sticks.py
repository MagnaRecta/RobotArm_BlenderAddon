"""Wireframe mesh -> physical sticks. BLENDER_ADDON_PLAN.md Sec 5.

**The defining behaviour of this addon (Sec 5.2.1): stick lengths are fixed
and the DESIGN grows to fit them, not the reverse.** The edge lengths the
user draws *are* the physical stick lengths they cut. The addon then moves
mesh vertices apart so sticks of exactly those lengths sit with a joint gap
at every shared vertex::

    required_edge_length = stick_length + joint_allowance * shared_ends

Read that as: nothing here ever shortens a stick to make a design fit.

Pipeline, in the order it must run::

    extract  ->  merge coincident vertices
             ->  snap to stock lengths      (Sec 5.3 -- BEFORE expanding)
             ->  required edge lengths      (Sec 5.2.1, N7 growth mode)
             ->  expansion solve            (Sec 5.2.2)
             ->  inset to physical endpoints (Sec 5.2)
             ->  StickSpec list + cut list  (Sec 5.5)

⚠ Order matters: snapping *after* the solve would re-break every edge length
the solver just satisfied (Sec 5.3).

Pure Python -- no ``bpy``, no ``mathutils``, no numpy (constraint B4), so the
whole pipeline is unit-testable in a bare interpreter. The bpy-facing layer
lives in ``ops/``.
"""

import math

from . import robots as core_robots
from .transform import (
    angle_between,
    v_add,
    v_dist,
    v_normalized,
    v_scale,
    v_sub,
)

# Multi-robot support (docs/STATUS.md 2026-08-21/2026-08-23): every default
# below that reads from a kinematics module now reads from the SELECTED
# robot's own (``robot_id``, resolved via ``core_robots.get_robot``), not
# so_arm_100's unconditionally -- previously the hard length floor, the
# joint-allowance/stock-section defaults, and the tight-clearance geometry
# all silently used so_arm_100's numbers regardless of the Scene's selected
# robot. ``_SO_ARM_100`` below is kept only as the DEFAULT for standalone
# callers (as the tests use directly) of the smaller helper functions --
# ``extract_sticks()`` itself always resolves against whichever robot it is
# actually called for.
_SO_ARM_100 = core_robots.get_robot(core_robots.SO_ARM_100_ID).kinematics

# --- tunables that are NOT in the kinematics module -------------------------

# Sec 5.2.2 / N8, confirmed with the user 2026-07-28: an edge whose solved
# length misses its target by more than this is reported as unbuildable.
DEFAULT_RESIDUAL_TOLERANCE_M = 0.0005  # 0.5 mm

# A vertex at or below this height is treated as seating on the base plate.
DEFAULT_GROUND_EPSILON_M = 0.0005

# Two mesh vertices closer than this are the same structural joint.
DEFAULT_MERGE_TOLERANCE_M = 0.0005

# Below this, two stock lengths are equally good candidates for an edge and
# the tie is broken toward the shorter one. See snap_to_stock().
SNAP_TIE_EPSILON_M = 1e-6

# Sec 5.2.1 / N7, confirmed with the user 2026-07-28: PER_EDGE is the default.
GROWTH_PER_EDGE = "PER_EDGE"
GROWTH_UNIFORM = "UNIFORM"

# How grounded vertices are constrained during the expansion solve.
# Decided 2026-07-28: SLIDE is the default. A grounded vertex is locked to
# z=0 -- the base plate is physical, nothing expands below it -- but is free
# to slide across the plate as the design grows. See solve_expansion().
GROUND_SLIDE = "SLIDE"
GROUND_PIN = "PIN"


def hard_min_stick_length_m(robot_id=core_robots.SO_ARM_100_ID):
    """The floor below which ``min_stick_length`` must never be settable,
    for the given robot.

    Sec 5.4 / D13: this is no longer "grip height + jaw margin below which the
    jaws close past the tip at a FIXED offset" -- the offset itself now
    adapts per stick (``kinematics.grasp.grasp_offset_for_length()``), so the
    floor is instead the length below which *that function itself* refuses
    any offset at all: ``MIN_GRASP_OFFSET_M + JAW_CONTACT_HALF_LENGTH_M``.
    Both constants live in the shared kinematics module, never hardcoded here
    -- Phase 0 confirms the real numbers with a ruler. Read from the
    SELECTED robot's own module, not so_arm_100's unconditionally -- each
    robot's own floor genuinely differs (so_arm_100: 35mm; kr10_r900_2:
    12.64mm as of the 2026-09-09 re-vendor, round 2mm stock -- see that
    package's own constants.py; it was 18mm before that package's gripper
    finger meshes were swapped for shorter ones, so treat any figure quoted
    here as illustrative and the constants as the source of truth).
    """
    kinematics = core_robots.get_robot(robot_id).kinematics
    return kinematics.MIN_GRASP_OFFSET_M + kinematics.JAW_CONTACT_HALF_LENGTH_M


def safe_min_stick_length_bound_m():
    """The lowest hard floor across every REGISTERED robot -- never any one
    robot's own floor specifically.

    Blender's own ``FloatProperty(min=...)`` is fixed at class-registration
    time (``properties.py``), so it cannot depend on whichever robot happens
    to be selected at the moment the user edits the field -- there is no
    per-instance-dynamic bound in the Blender API for this. Using this
    (permissive, safe-for-everyone) value as that STATIC widget bound, and
    leaving the real, robot-SPECIFIC enforcement to ``extract_sticks()``'s
    own runtime check (already robot-aware), is what keeps the widget from
    blocking a value that is genuinely valid for the currently-selected
    robot just because it is below some OTHER robot's own floor.
    """
    return min(hard_min_stick_length_m(robot_id) for robot_id in core_robots.ROBOTS)


# --- data ---------------------------------------------------------------------


class StickSpec:
    """One physical stick. Mirrors BRIDGE_PROTOCOL.md Sec 6 field for field.

    ``base``/``tip`` are the PHYSICAL stick ends, already inset from the ideal
    mesh vertices -- ROS2 places exactly what it is given and does no gap
    arithmetic. ``base`` is the end that seats down (on the plate or on
    another stick).
    """

    __slots__ = (
        "id", "base", "tip", "roll_deg", "length_m", "shared_ends",
        "section_m", "design_edge_m", "required_edge_m", "solved_edge_m",
        "residual_m", "warnings", "flipped", "v_base", "v_tip",
    )

    def __init__(self, id, base, tip, length_m, shared_ends,
                 roll_deg=0.0, section_m=None, design_edge_m=0.0,
                 required_edge_m=0.0, solved_edge_m=0.0, residual_m=0.0,
                 warnings=None, flipped=False, v_base=-1, v_tip=-1):
        self.id = id
        self.base = base
        self.tip = tip
        self.roll_deg = roll_deg
        self.length_m = length_m
        self.shared_ends = shared_ends
        self.section_m = section_m or (_SO_ARM_100.STICK_SECTION_M, _SO_ARM_100.STICK_SECTION_M)
        # Sec 5.2.3: the addon must show, per stick, design edge -> shared
        # ends -> required edge -> residual.
        self.design_edge_m = design_edge_m
        self.required_edge_m = required_edge_m
        self.solved_edge_m = solved_edge_m
        self.residual_m = residual_m
        self.warnings = list(warnings or ())
        self.flipped = flipped
        # Merged-topology vertex indices, for the Phase C order solver.
        self.v_base = v_base
        self.v_tip = v_tip

    @property
    def length_mm(self):
        return self.length_m * 1000.0

    def axis(self):
        return v_normalized(v_sub(self.tip, self.base))

    def __repr__(self):
        return (
            "StickSpec(id=%r, len=%.2fmm, shared=%d, residual=%.3fmm)"
            % (self.id, self.length_mm, self.shared_ends, self.residual_m * 1000.0)
        )


class Topology:
    """Merged vertices + edges, in ``base_link`` metres."""

    def __init__(self, positions, edges, ground_epsilon_m=DEFAULT_GROUND_EPSILON_M,
                ground_height_m=0.0, ground_required=True):
        self.positions = list(positions)
        self.edges = list(edges)  # (edge_id, i0, i1)
        self.ground_epsilon_m = ground_epsilon_m
        # The physical build plate's own Z, in the SELECTED robot's base
        # frame -- 0.0 by default (the plate at the robot's own origin
        # height), but the plate is a real, height-adjustable object, so a
        # component whose lowest point sits above Z=0 is not necessarily
        # unsupported: raising the plate to meet it makes it grounded.
        self.ground_height_m = ground_height_m
        # 2026-08-23, user request: some designs are held by something this
        # addon does not model at all (a stick's own base used as a jig, a
        # non-flat fixture) -- ``ground_required=False`` turns is_grounded()
        # permanently off, so nothing is ever treated as plate-seated. The
        # caller is trusted to know how the assembly is actually supported;
        # this only removes the addon's OWN plate check, never claims the
        # result is physically self-supporting.
        self.ground_required = ground_required
        self._adjacency = None

    @property
    def adjacency(self):
        if self._adjacency is None:
            adj = [[] for _ in self.positions]
            for e_index, (_eid, i0, i1) in enumerate(self.edges):
                adj[i0].append((e_index, i1))
                adj[i1].append((e_index, i0))
            self._adjacency = adj
        return self._adjacency

    def degree(self, i):
        return len(self.adjacency[i])

    def is_grounded(self, i):
        if not self.ground_required:
            return False
        return self.positions[i][2] <= self.ground_height_m + self.ground_epsilon_m

    def components(self):
        """Connected components of the *mesh* graph (ground not counted as a
        connector). Returns a list of vertex-index lists."""
        seen = [False] * len(self.positions)
        out = []
        for start in range(len(self.positions)):
            if seen[start] or not self.adjacency[start]:
                continue
            stack, group = [start], []
            seen[start] = True
            while stack:
                v = stack.pop()
                group.append(v)
                for _e, other in self.adjacency[v]:
                    if not seen[other]:
                        seen[other] = True
                        stack.append(other)
            out.append(group)
        return out


def merge_vertices(points, tolerance_m=DEFAULT_MERGE_TOLERANCE_M):
    """Merge points within ``tolerance_m``. Returns ``(merged, index_map)``.

    A wireframe authored in Blender routinely has visually-coincident but
    numerically distinct vertices; every one of those is a joint the solver
    and the build-order graph must see as a single point (Sec 6.1).

    Uses a hash grid so a few thousand vertices stay linear rather than
    quadratic (constraint B3 -- the UI must never block).
    """
    if tolerance_m <= 0.0:
        return list(points), list(range(len(points)))

    cell = tolerance_m * 2.0
    grid = {}
    merged = []
    index_map = []

    for p in points:
        key = (int(math.floor(p[0] / cell)),
               int(math.floor(p[1] / cell)),
               int(math.floor(p[2] / cell)))
        found = -1
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                for dz in (-1, 0, 1):
                    for candidate in grid.get((key[0] + dx, key[1] + dy, key[2] + dz), ()):
                        if v_dist(merged[candidate], p) <= tolerance_m:
                            found = candidate
                            break
                    if found >= 0:
                        break
                if found >= 0:
                    break
            if found >= 0:
                break
        if found < 0:
            found = len(merged)
            merged.append(p)
            grid.setdefault(key, []).append(found)
        index_map.append(found)

    return merged, index_map


def build_topology(points, edge_pairs, edge_ids=None,
                   merge_tolerance_m=DEFAULT_MERGE_TOLERANCE_M,
                   ground_epsilon_m=DEFAULT_GROUND_EPSILON_M,
                   ground_height_m=0.0, ground_required=True):
    """Merge coincident vertices and drop edges that collapse to a point."""
    merged, index_map = merge_vertices(points, merge_tolerance_m)
    if edge_ids is None:
        edge_ids = ["s_%03d" % (n + 1) for n in range(len(edge_pairs))]

    edges = []
    degenerate = []
    for eid, (a, b) in zip(edge_ids, edge_pairs):
        i0, i1 = index_map[a], index_map[b]
        if i0 == i1:
            degenerate.append(eid)
            continue
        edges.append((eid, i0, i1))

    return (
        Topology(merged, edges, ground_epsilon_m, ground_height_m, ground_required),
        degenerate,
    )


# --- Sec 5.3: length modes (run BEFORE the expansion solve) -------------------


def snap_to_stock(design_lengths_m, stock_lengths_m):
    """Snap each edge length to the nearest available stock length.

    Returns ``(snapped_lengths, errors)`` where ``errors[i]`` is
    ``snapped - design`` -- the amount the design must move to become exact.
    """
    if not stock_lengths_m:
        raise ValueError("fixed-stock-length mode needs at least one stock length")
    stock = sorted(stock_lengths_m)
    snapped, errors = [], []
    for length in design_lengths_m:
        # Ties are broken toward the SHORTER stock, deterministically. An
        # edge drawn at exactly 110 mm against 100/120 mm stock is genuinely
        # ambiguous, and without quantising, the winner is decided by float
        # noise -- so the twelve "identical" edges of one cube snap to
        # different stock lengths.
        #
        # The epsilon has to be coarser than that noise. Blender stores mesh
        # vertex coordinates as **float32**, so a 110 mm edge of a cube sited
        # 370 mm from the base measures anywhere in 0.109999999 .. 0.110000015
        # -- a spread of ~15 nm, which a nanometre-scale epsilon does not
        # close. One micron is comfortably above it and far below anything
        # that means something at a saw.
        best = min(stock, key=lambda s: (round(abs(s - length) / SNAP_TIE_EPSILON_M), s))
        snapped.append(best)
        errors.append(best - length)
    return snapped, errors


# --- Sec 5.2.1: required edge lengths ----------------------------------------


def shared_end_flags(topology):
    """Per edge, ``(end0_shared, end1_shared)``.

    An end is *shared* when another stick meets it. An end that seats on the
    base plate is **not** shared and gets no allowance (Sec 5.2) -- which is
    exactly why the worked inverted-U's uprights have one shared end, not two.
    Note grounding is irrelevant to this test: two sticks meeting *at* the
    plate still interpenetrate, and a free end in mid-air still overhangs.
    """
    return [
        (topology.degree(i0) > 1, topology.degree(i1) > 1)
        for _eid, i0, i1 in topology.edges
    ]


def required_edge_lengths(topology, stick_lengths_m,
                          joint_allowance_m=_SO_ARM_100.JOINT_ALLOWANCE_M,
                          growth_mode=GROWTH_PER_EDGE):
    """``required_edge = stick_length + allowance * shared_ends`` (Sec 5.2.1).

    ``GROWTH_PER_EDGE`` (default, N7) counts only genuinely shared ends, so
    free ends land exactly on the design vertex.

    ``GROWTH_UNIFORM`` (Sec 5.2.2's optional simplification) adds the
    allowance at *every* end, so every edge grows by exactly ``2*allowance``
    and free ends overhang the design vertex harmlessly. A uniform additive
    growth is far better conditioned than a per-edge variable one and removes
    most of the residual error on looped structures.

    Returns ``(required_lengths, shared_counts)``. ``shared_counts`` is always
    the *true* count -- it is reported to the operator and written to the
    build file, and must not be inflated by the uniform mode.
    """
    flags = shared_end_flags(topology)
    required, counts = [], []
    for (a, b), length in zip(flags, stick_lengths_m):
        count = int(a) + int(b)
        grow = 2 if growth_mode == GROWTH_UNIFORM else count
        required.append(length + joint_allowance_m * grow)
        counts.append(count)
    return required, counts


# --- Sec 5.2.2: the constraint solve -----------------------------------------


class ExpansionResult:
    __slots__ = ("positions", "residuals", "iterations", "converged", "method",
                 "max_residual_m")

    def __init__(self, positions, residuals, iterations, converged, method):
        self.positions = positions
        self.residuals = residuals
        self.iterations = iterations
        self.converged = converged
        self.method = method  # per-component: "exact" or "relaxation"
        self.max_residual_m = max((abs(r) for r in residuals), default=0.0)


def _component_is_exactly_solvable(topology, group):
    """True when the component is a tree with at most one ground anchor.

    Sec 5.2.2's "acyclic structures are exact" case, with one addition the
    prose leaves implicit: **pinning closes loops too.** The worked
    inverted-U is acyclic as a mesh (4 vertices, 3 edges) yet the doc treats
    it as the looped case -- because pinning *both* feet to the plate makes
    the base plate behave as a fourth edge. So a second grounded vertex
    disqualifies a component from the exact walk just as a mesh cycle does.
    """
    vertex_set = set(group)
    edge_count = sum(
        1 for _eid, i0, i1 in topology.edges if i0 in vertex_set and i1 in vertex_set
    )
    if edge_count != len(group) - 1:  # a tree has exactly V-1 edges
        return False
    return sum(1 for v in group if topology.is_grounded(v)) <= 1


def _solve_component_exact(topology, group, targets, positions):
    """Walk outward from the anchor, pushing each vertex along its parent
    edge's *original* direction by the required amount. Every edge lands on
    its target exactly and the design's angles are preserved (Sec 5.2.2)."""
    grounded = [v for v in group if topology.is_grounded(v)]
    root = grounded[0] if grounded else min(group)

    seen = {root}
    queue = [root]
    while queue:
        v = queue.pop(0)
        for e_index, other in topology.adjacency[v]:
            if other in seen:
                continue
            direction = v_normalized(v_sub(topology.positions[other], topology.positions[v]))
            positions[other] = v_add(positions[v], v_scale(direction, targets[e_index]))
            seen.add(other)
            queue.append(other)


def solve_expansion(topology, required_lengths_m,
                    ground_mode=GROUND_SLIDE,
                    max_iterations=200,
                    tolerance_m=1e-6):
    """Move vertices apart so every edge reaches its required length.

    Acyclic components with at most one ground anchor are solved **exactly**
    by an outward walk. Everything else -- mesh loops, and trees with two or
    more feet on the plate -- goes to **iterative constraint relaxation**
    (position-based dynamics style): repeatedly, for each edge, move both
    endpoints symmetrically along the edge to correct its length error.

    Growing each edge by a different amount is over-constrained in general,
    so the relaxed solution has a residual per edge. **Report it** -- an edge
    that cannot reach its target is a design the sticks will not physically
    fit, and the user must know which one, not discover it at the glue gun.

    ``ground_mode`` (Sec 5.2.2, decided 2026-07-28):

    * ``GROUND_SLIDE`` (default) -- a grounded vertex is locked to the build
      plate's own Z (``topology.ground_height_m``, 0 unless the plate has
      been raised) but free to move in XY. It cannot rise or sink, because
      the base plate is physical and nothing expands through it, but it may
      slide across the plate as the design grows.
    * ``GROUND_PIN`` -- grounded vertices are fully immobile. Retained as a
      toggle; it was the spec's original instruction.

    The two only diverge when pinning removes *all* freedom. A square drawn
    flat on the plate has four grounded vertices and no free ones, so under
    PIN nothing can move, every edge misses its target by the full
    ``2 * allowance``, and a perfectly buildable design reports as
    impossible. Under SLIDE it solves exactly and simply comes out larger --
    which is what Sec 5.2.1 means by *the design grows to fit the sticks*.

    On a structure that already has free vertices the two agree: the worked
    inverted-U lands all three edges on target either way, with the uprights
    tilted ~1.65 deg.
    """
    positions = list(topology.positions)
    targets = list(required_lengths_m)
    n_edges = len(topology.edges)

    if n_edges == 0:
        return ExpansionResult(positions, [], 0, True, "exact")

    iterations = 0
    converged = True

    groups = topology.components()
    relaxation_vertices = set()
    exact_count = 0
    for group in groups:
        if _component_is_exactly_solvable(topology, group):
            _solve_component_exact(topology, group, targets, positions)
            exact_count += 1
        else:
            relaxation_vertices.update(group)

    if not relaxation_vertices:
        method = "exact"
    elif exact_count == 0:
        method = "relaxation"
    else:
        method = "mixed"

    if relaxation_vertices:
        pinned = set()
        for v in relaxation_vertices:
            if topology.is_grounded(v) and ground_mode == GROUND_PIN:
                pinned.add(v)

        active_edges = [
            (i, i0, i1)
            for i, (_eid, i0, i1) in enumerate(topology.edges)
            if i0 in relaxation_vertices
        ]

        for iteration in range(max_iterations):
            iterations = iteration + 1
            max_error = 0.0
            for e_index, i0, i1 in active_edges:
                w0 = 0.0 if i0 in pinned else 1.0
                w1 = 0.0 if i1 in pinned else 1.0
                total = w0 + w1
                delta = v_sub(positions[i1], positions[i0])
                length = math.sqrt(delta[0] ** 2 + delta[1] ** 2 + delta[2] ** 2)
                if length < 1e-9:
                    # Fully collapsed edge: no direction to push along. Nudge
                    # it back onto its original direction so the solver can
                    # get a grip on it instead of dividing by zero.
                    delta = v_normalized(
                        v_sub(topology.positions[i1], topology.positions[i0])
                    )
                    length = 1e-9
                    if delta == (0.0, 0.0, 0.0):
                        continue
                    unit = delta
                else:
                    unit = (delta[0] / length, delta[1] / length, delta[2] / length)

                error = length - targets[e_index]
                if abs(error) > max_error:
                    max_error = abs(error)
                if total == 0.0:
                    continue  # both ends pinned -- reported as a residual
                correction = v_scale(unit, error)
                positions[i0] = v_add(positions[i0], v_scale(correction, w0 / total))
                positions[i1] = v_sub(positions[i1], v_scale(correction, w1 / total))

            if ground_mode == GROUND_SLIDE:
                for v in relaxation_vertices:
                    if topology.is_grounded(v):
                        positions[v] = (
                            positions[v][0], positions[v][1], topology.ground_height_m)

            if max_error < tolerance_m:
                break
        else:
            converged = False

    residuals = []
    for e_index, (_eid, i0, i1) in enumerate(topology.edges):
        residuals.append(v_dist(positions[i0], positions[i1]) - targets[e_index])

    return ExpansionResult(positions, residuals, iterations, converged, method)


# --- Sec 5.2: inset to the physical stick endpoints --------------------------


def _inset_endpoints(p0, p1, inset0, inset1, length_m, anchor0, anchor1):
    """Place a stick of exactly ``length_m`` inside the solved edge.

    The solved edge may miss its target by the residual, so simply insetting
    both ends would give ``||tip - base|| != length_m`` and violate
    BRIDGE_PROTOCOL.md Sec 6 (the ROS2 side rejects a >1 mm inconsistency).
    Instead the exact-length stick is laid along the edge and the residual is
    absorbed into the joint gaps, which is what physically happens anyway.

    An end seating on the base plate is anchored rather than centred -- the
    stick really does start at z=0 there.
    """
    unit = v_normalized(v_sub(p1, p0))
    if unit == (0.0, 0.0, 0.0):
        return p0, p1

    a = v_add(p0, v_scale(unit, inset0))
    b = v_sub(p1, v_scale(unit, inset1))

    if anchor0 and not anchor1:
        return a, v_add(a, v_scale(unit, length_m))
    if anchor1 and not anchor0:
        return v_sub(b, v_scale(unit, length_m)), b

    span = v_dist(a, b)
    slack = (span - length_m) * 0.5
    a = v_add(a, v_scale(unit, slack))
    return a, v_add(a, v_scale(unit, length_m))


# --- Sec 5.2.3 / 6.2: per-vertex angle warnings ------------------------------


def angle_allowance_m(theta_rad, section_m=_SO_ARM_100.STICK_SECTION_M):
    """``w / (2*tan(theta/2))`` -- how far two sticks of width ``w`` meeting
    at ``theta`` interpenetrate along each axis (Sec 5.2.3). 90 deg gives
    3.2 mm, matching the fixed value; 45 deg needs 7.8 mm; 30 deg needs
    12 mm."""
    half = theta_rad * 0.5
    if half <= 1e-6 or half >= math.pi - 1e-6:
        return float("inf")
    t = math.tan(half)
    if abs(t) < 1e-9:
        return float("inf")
    return abs(section_m / (2.0 * t))


def vertex_clearance_warnings(topology, joint_allowance_m=_SO_ARM_100.JOINT_ALLOWANCE_M,
                              section_m=_SO_ARM_100.STICK_SECTION_M):
    """Per vertex, the worst angle-based allowance and whether it exceeds the
    fixed one. Below ~45 deg the sticks physically clash, which no amount of
    glue fixes -- and under the Sec 5.2.1 model the gaps are what the
    expansion is sized around, so an undersized allowance at a shallow joint
    means the solved mesh puts two sticks in the same place."""
    out = {}
    for v, neighbours in enumerate(topology.adjacency):
        if len(neighbours) < 2:
            continue
        worst_required = 0.0
        worst_theta = math.pi
        for a in range(len(neighbours)):
            for b in range(a + 1, len(neighbours)):
                da = v_sub(topology.positions[neighbours[a][1]], topology.positions[v])
                db = v_sub(topology.positions[neighbours[b][1]], topology.positions[v])
                theta = angle_between(da, db)
                required = angle_allowance_m(theta, section_m)
                if required > worst_required:
                    worst_required = required
                    worst_theta = theta
        out[v] = {
            "theta_rad": worst_theta,
            "required_allowance_m": worst_required,
            "tight": worst_required > joint_allowance_m + 1e-9,
            "valence": len(neighbours),
        }
    return out


# --- the whole pipeline -------------------------------------------------------


class ExtractionResult:
    __slots__ = ("sticks", "topology", "expansion", "warnings", "errors",
                 "design_bounds", "expanded_bounds", "snap_errors_m")

    def __init__(self):
        self.sticks = []
        self.topology = None
        self.expansion = None
        self.warnings = []   # (stick_id or None, code, message)
        self.errors = []     # (stick_id or None, code, message)
        self.design_bounds = None
        self.expanded_bounds = None
        self.snap_errors_m = []

    def counts(self):
        return {
            "sticks": len(self.sticks),
            "warnings": len(self.warnings),
            "errors": len(self.errors),
        }


def _bounds(points):
    if not points:
        return None
    lo = [min(p[i] for p in points) for i in range(3)]
    hi = [max(p[i] for p in points) for i in range(3)]
    return (tuple(lo), tuple(hi))


def extract_sticks(points_m, edge_pairs, edge_ids=None,
                   robot_id=core_robots.SO_ARM_100_ID,
                   joint_allowance_m=None,
                   growth_mode=GROWTH_PER_EDGE,
                   ground_mode=GROUND_SLIDE,
                   stock_lengths_m=None,
                   min_stick_length_m=None,
                   max_stick_length_m=None,
                   section_m=None,
                   merge_tolerance_m=DEFAULT_MERGE_TOLERANCE_M,
                   ground_epsilon_m=DEFAULT_GROUND_EPSILON_M,
                   ground_height_m=0.0,
                   ground_required=True,
                   residual_tolerance_m=DEFAULT_RESIDUAL_TOLERANCE_M,
                   flips=None):
    """Full Sec 5 pipeline. ``points_m`` and the result are in the SELECTED
    robot's own frame (``base_link`` for so_arm_100, ``base`` for
    kr10_r900_2) -- do the Blender transform with ``core.transform`` first.

    ``stock_lengths_m`` selects fixed-stock-length mode (Sec 5.3); ``None``
    means design-driven mode, where the stick length is the edge as drawn.

    ``joint_allowance_m``/``section_m``/``min_stick_length_m``/
    ``max_stick_length_m`` all default (``None``) to the SELECTED robot's
    own kinematics constants, not so_arm_100's unconditionally -- e.g.
    kr10_r900_2's real joint allowance is 1 mm/end (round-stock contact,
    KUKA_IMPLEMENTATION_PLAN.md KQ3), a genuinely different physical model
    from so_arm_100's 3.25 mm square-stock formula, never just a smaller
    number of the same shape. Pass explicit values (as ``ops/design.py``
    does, from the Design panel's own UI fields) to override.

    ``ground_height_m`` (2026-08-23): the build plate's own Z, in the same
    frame as ``points_m``. 0.0 (the default) means the plate sits at the
    robot's own origin height, matching every design so far. The plate is a
    real, height-adjustable object, though -- a component that does not
    reach Z=0 is not necessarily unbuildable, just unbuildable *at the
    plate's current height*. Raising this value lets a design that sits
    entirely above Z=0 (previously reported as ``floating_component``)
    solve and place normally, as if the plate had been physically raised to
    meet it.

    ``ground_required`` (2026-08-23): ``True`` (default) means every design
    is checked against the plate as above. ``False`` turns that check off
    entirely -- no vertex is ever treated as plate-seated, so nothing is
    ever reported as ``floating_component`` or ``below_plate``, and the
    expansion solve treats every component as free-floating (an arbitrary
    vertex anchors each one instead of a grounded one). For a design held
    by something this addon does not model at all -- a stick's own base
    used as a jig, a non-flat fixture -- rather than by a flat plate at any
    height. This does not verify the result is physically self-supporting;
    the caller is trusted to know how it is actually held.
    """
    kinematics = core_robots.get_robot(robot_id).kinematics
    if joint_allowance_m is None:
        joint_allowance_m = kinematics.JOINT_ALLOWANCE_M
    if section_m is None:
        section_m = kinematics.STICK_SECTION_M
    if min_stick_length_m is None:
        min_stick_length_m = kinematics.STICK_LENGTH_RANGE_M[0]
    if max_stick_length_m is None:
        max_stick_length_m = kinematics.STICK_LENGTH_RANGE_M[1]

    floor = hard_min_stick_length_m(robot_id)
    if min_stick_length_m < floor - 1e-9:
        raise ValueError(
            "min_stick_length %.1f mm is below %r's hard physical floor of "
            "%.1f mm (MIN_GRASP_OFFSET_M %.1f mm + JAW_CONTACT_HALF_LENGTH_M "
            "%.1f mm). Below this, grasp_offset_for_length() cannot find any "
            "offset that both fits within the stick and clears the floor-"
            "clearance floor."
            % (min_stick_length_m * 1000.0, robot_id, floor * 1000.0,
               kinematics.MIN_GRASP_OFFSET_M * 1000.0,
               kinematics.JAW_CONTACT_HALF_LENGTH_M * 1000.0)
        )

    result = ExtractionResult()
    topology, degenerate = build_topology(
        points_m, edge_pairs, edge_ids, merge_tolerance_m, ground_epsilon_m,
        ground_height_m, ground_required,
    )
    result.topology = topology
    for eid in degenerate:
        result.errors.append(
            (eid, "degenerate_edge",
             "edge collapses to a point after merging vertices within %.2f mm"
             % (merge_tolerance_m * 1000.0))
        )
    if not topology.edges:
        return result

    # --- Sec 5.3: stick lengths (snap BEFORE expanding) ---------------------
    design_lengths = [
        v_dist(topology.positions[i0], topology.positions[i1])
        for _eid, i0, i1 in topology.edges
    ]
    if stock_lengths_m:
        stick_lengths, snap_errors = snap_to_stock(design_lengths, stock_lengths_m)
        result.snap_errors_m = snap_errors
    else:
        stick_lengths = list(design_lengths)
        result.snap_errors_m = [0.0] * len(design_lengths)

    # --- Sec 5.2.1 -> 5.2.2 -------------------------------------------------
    required, shared_counts = required_edge_lengths(
        topology, stick_lengths, joint_allowance_m, growth_mode
    )
    expansion = solve_expansion(topology, required, ground_mode)
    result.expansion = expansion

    result.design_bounds = _bounds(topology.positions)
    result.expanded_bounds = _bounds(expansion.positions)

    if not expansion.converged:
        result.warnings.append(
            (None, "solve_not_converged",
             "expansion solve hit its iteration cap (max residual %.3f mm) -- "
             "residuals below are the best it reached"
             % (expansion.max_residual_m * 1000.0))
        )

    clearance = vertex_clearance_warnings(topology, joint_allowance_m, section_m)
    for group in (topology.components() if ground_required else []):
        if not any(topology.is_grounded(v) for v in group):
            members = set(group)
            ids = sorted(
                eid for eid, i0, _i1 in topology.edges if i0 in members
            )
            lowest_z = min(topology.positions[v][2] for v in members)
            result.errors.append(
                (None, "floating_component",
                 "%d sticks (%s%s) form a component with no vertex on the base "
                 "plate (currently at Z=%.1f mm) -- nothing supports it. Its "
                 "own lowest point is %.1f mm; raising the build plate to "
                 "about there would ground it"
                 % (len(ids), ", ".join(ids[:4]), " ..." if len(ids) > 4 else "",
                    ground_height_m * 1000.0, lowest_z * 1000.0))
            )

    # --- Sec 5.2 inset + Sec 5.5 output ------------------------------------
    flags = shared_end_flags(topology)
    flips = flips or {}

    for e_index, (eid, i0, i1) in enumerate(topology.edges):
        shared0, shared1 = flags[e_index]
        if growth_mode == GROWTH_UNIFORM:
            inset0 = inset1 = joint_allowance_m
        else:
            inset0 = joint_allowance_m if shared0 else 0.0
            inset1 = joint_allowance_m if shared1 else 0.0

        p0, p1 = expansion.positions[i0], expansion.positions[i1]
        length = stick_lengths[e_index]
        anchor0 = topology.is_grounded(i0) and not shared0
        anchor1 = topology.is_grounded(i1) and not shared1
        a, b = _inset_endpoints(p0, p1, inset0, inset1, length, anchor0, anchor1)

        # Sec 5.1 step 4: base is the end that seats down -- the lower Z,
        # overridable per stick. Build-order-aware base selection (an end
        # meeting an already-placed stick) is applied in Phase C.
        flip = bool(flips.get(eid, False))
        base_is_a = a[2] <= b[2]
        if flip:
            base_is_a = not base_is_a
        base, tip = (a, b) if base_is_a else (b, a)
        v_base, v_tip = (i0, i1) if base_is_a else (i1, i0)

        residual = expansion.residuals[e_index]
        warnings = []

        if clearance.get(i0, {}).get("tight") or clearance.get(i1, {}).get("tight"):
            worst = max(
                clearance.get(i0, {}).get("required_allowance_m", 0.0),
                clearance.get(i1, {}).get("required_allowance_m", 0.0),
            )
            warnings.append("tight_clearance")
            result.warnings.append(
                (eid, "tight_clearance",
                 "joint angle needs %.1f mm allowance but only %.2f mm is set -- "
                 "the sticks will physically clash"
                 % (worst * 1000.0, joint_allowance_m * 1000.0))
            )
        for v in (i0, i1):
            if clearance.get(v, {}).get("valence", 0) >= 4:
                if "high_valence" not in warnings:
                    warnings.append("high_valence")

        stick = StickSpec(
            id=eid, base=base, tip=tip, length_m=length,
            shared_ends=shared_counts[e_index], roll_deg=0.0,
            section_m=(section_m, section_m),
            design_edge_m=design_lengths[e_index],
            required_edge_m=required[e_index],
            solved_edge_m=v_dist(p0, p1),
            residual_m=residual,
            warnings=warnings, flipped=flip, v_base=v_base, v_tip=v_tip,
        )
        result.sticks.append(stick)

        # Sec 5.4: the 80-150 mm range is checked against the PHYSICAL stick
        # length -- the design edge -- never the expanded one, which is
        # longer by the joint gaps and is not what gets cut or gripped.
        if length < min_stick_length_m - 1e-9:
            result.errors.append(
                (eid, "too_short",
                 "stick length %.1f mm is below the %.1f mm minimum"
                 % (length * 1000.0, min_stick_length_m * 1000.0))
            )
        elif length > max_stick_length_m + 1e-9:
            result.errors.append(
                (eid, "too_long",
                 "stick length %.1f mm exceeds the %.1f mm maximum"
                 % (length * 1000.0, max_stick_length_m * 1000.0))
            )

        # The base plate is physical: nothing can sit below its own Z
        # (ground_height_m, 0 unless raised). GROUND_SLIDE holds grounded
        # vertices at exactly that height, but a vertex drawn below the
        # plate -- or pushed there by the expansion -- has to be reported,
        # not silently clamped, since clamping would move the design
        # without saying so. Only applies when there IS a plate to be below
        # (ground_required) -- with no plate this check has nothing to mean.
        lowest = min(base[2], tip[2])
        if ground_required and lowest < ground_height_m - ground_epsilon_m:
            result.errors.append(
                (eid, "below_plate",
                 "stick reaches %.1f mm below the base plate (Z=%.1f mm) -- "
                 "nothing can be built under it"
                 % ((ground_height_m - lowest) * 1000.0, ground_height_m * 1000.0))
            )

        if abs(residual) > residual_tolerance_m:
            result.errors.append(
                (eid, "residual_too_large",
                 "edge misses its required length of %.2f mm by %.3f mm "
                 "(tolerance %.2f mm) -- the sticks will not physically fit here"
                 % (required[e_index] * 1000.0, residual * 1000.0,
                    residual_tolerance_m * 1000.0))
            )

    return result


# --- Sec 5.5: cut list --------------------------------------------------------


def cut_list(sticks, order=None, quantise_mm=0.5):
    """Ordered table: ``# | stick id | stick length (mm) | build order``.

    A stick can only be loaded into the feeder by a human who knows which one
    it is, so this is a deliverable, not a debug dump.
    """
    rows = []
    for n, stick in enumerate(sticks):
        rows.append({
            "index": n,
            "id": stick.id,
            "length_mm": stick.length_mm,
            "order": order.get(stick.id) if order else None,
            "shared_ends": stick.shared_ends,
        })
    return rows


def cut_tally(sticks, quantise_mm=0.5):
    """Collapse the cut list to ``12 x 100 mm, 8 x 120 mm`` -- what you
    actually want at a saw under fixed-stock-length mode (Sec 5.5)."""
    tally = {}
    for stick in sticks:
        key = round(stick.length_mm / quantise_mm) * quantise_mm
        tally[key] = tally.get(key, 0) + 1
    return sorted(tally.items())


def cut_list_csv(sticks, order=None):
    lines = ["index,stick_id,stick_length_mm,build_order,shared_ends"]
    for row in cut_list(sticks, order):
        lines.append(
            "%d,%s,%.2f,%s,%d"
            % (row["index"], row["id"], row["length_mm"],
               "" if row["order"] is None else row["order"], row["shared_ends"])
        )
    return "\n".join(lines) + "\n"
