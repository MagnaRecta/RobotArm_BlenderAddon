# Blender addon — design & implementation plan

**Last updated:** 2026-07-28
**Status:** specification — nothing implemented yet. **Blender is not
installed on this machine.** Target: **Blender 5.2.0 LTS**, Linux + Windows.
**Audience:** a developer or AI agent implementing the addon.
**Companion documents:**
- [`ROS2_IMPLEMENTATION_PLAN.md`](ROS2_IMPLEMENTATION_PLAN.md) — the robot side.
  Its §9 contains the **validated kinematics** this addon depends on.
- [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) — the build-file format and the
  optional live protocol. Implement against that file, not prose here.

---

## 1. What the addon is for

The user models a sculpture in Blender as a **wireframe mesh**: every edge is
one wooden stick (6.45 mm square stock, **50–150 mm** long — §5.4). The addon turns
that mesh into a buildable job:

1. **Extract** sticks from the mesh edges.
2. **Compute the build order** — which stick goes down first, second, … so the
   robot never has to reach into a place it has already walled off (§6).
3. **Validate** every placement against the robot's real kinematics, in build
   order, and show the user exactly which sticks are impossible and why.
4. **Emit a cut list** — the human needs to cut each stick to length, and
   needs to know which one to load into the feeder next.
5. **Drive or export the build**, one stick at a time, with the human gates.

Blender is the design tool, the *slicer*, and the build console.

---

## 2. ✅ Architecture: Option C — "slicer + shared kinematics"

**Decided 2026-07-27.** The user proposed treating Blender as a **slicer** —
export a build file, let ROS2 execute it, no live connection. That instinct
was right, and **Option C below was chosen**: the slicer model, plus the
kinematics module vendored into the addon so validation and preview work
offline.

**Practical consequences — apply these throughout:**
- ❌ **No `net/` layer.** Delete it from §4's tree. No sockets, no worker
  threads, no reconnect logic, no mock server. Constraint B2 becomes moot.
- ✅ **`kinematics/` is vendored and load-bearing.** It must stay pure Python
  (no `numpy` — see B4) so it runs in Blender's interpreter on Linux and
  Windows alike.
- ✅ **Part A of [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md)** (the build file
  and its status sidecar) is the entire integration surface.
- ✅ The `Build` panel (§10.3) shows the **next stick to load** and syncs
  status from the sidecar; there is no live action button.

The full comparison is retained below as the record of why.

---

Three options were considered:

### Option A — Live bridge (originally proposed) ❌ rejected
Blender holds a network socket (TCP, *the protocol* — not to be confused with
Tool Center Point, which is what "TCP" means everywhere else in these docs)
open to a ROS2 bridge node, and commands the robot step by step.

- ➕ Instant validation feedback while designing.
- ➕ Live robot mirroring and live build progress in the viewport.
- ➖ Needs a worker thread, a queue, a main-thread pump, reconnect logic,
  protocol versioning, and a mock server — realistically **~40 % of the
  addon's total complexity, and ~80 % of its bug surface**.
- ➖ Blender must be running, connected, and on the network path to ROS2. The
  user has a **Windows** Blender machine; cross-machine sockets mean firewall
  and address configuration.
- ➖ Nothing is reproducible: no artifact to inspect, diff, version, or replay.

### Option B — Pure slicer (user's proposal) ❌ superseded by C
Blender exports a build file; ROS2 loads and executes it. No connection ever.

- ➕ Radically simpler addon: no networking at all.
- ➕ The build file is a real artifact — inspectable, versionable, replayable,
  emailable, and the exact thing to attach to a bug report.
- ➕ Works across machines and offline by construction.
- ➕ Matches a mental model the user already has (3D printer → G-code).
- ➖ Validation becomes a round trip: export → run a checker on the ROS
  machine → read a report → go back to Blender and fix. Slow, and the fix
  loop is where the user will spend most of their time.
- ➖ No robot mirroring, no live progress (the user asked for mirroring, QB3).

### Option C — Slicer **+ shared kinematics module** ✅ CHOSEN — this is the design
Same as B, **plus** the one insight that removes B's only real drawback:

> **Reachability validation does not need the robot, ROS, or a network — it
> needs the kinematic model.** The ROS2 plan already specifies
> `so_arm_100_kinematics` as **pure Python with zero ROS imports**
> (`ROS2_IMPLEMENTATION_PLAN.md` §9.3, Phase 1). That module can be **vendored
> directly into the addon** and run inside Blender's own interpreter.

So:

| Capability | How it works under Option C |
|---|---|
| Reachability validation | Runs **inside Blender**, instantly, offline, on Windows. Red/green per stick as you model. |
| Reachability volume overlay | Same module, no connection. |
| Robot mirroring (QB3) | Blender runs **FK** from the shared module and poses a rig. Better than mirroring live hardware: it can preview the *whole build* before anything moves, like a printer's preview. |
| Build execution | Export a build file; ROS2 executes it with the human gates in a terminal. |
| Ground truth | ROS2 **re-validates** the file on load with the same module — same code, so no drift, but MoveIt still gets the final say on collisions. |

Option C gets ~95 % of Option A's user experience with ~20 % of its
complexity, and keeps the reproducible artifact. The only thing genuinely lost
is live progress in the Blender viewport during a build — and a terminal on
the robot machine covers that.

**A live link can still be added later** as a strictly *read-only* telemetry
stream (joint states → Blender). Read-only is far simpler than a command
protocol: no state machine, no request/response, no versioning. If it is ever
wanted, it is a small additive feature, not a rewrite.

**Chosen: Option C.** A read-only telemetry link (joint states → Blender) may
be added later; it is additive and needs none of Option A's command
machinery.

---

## 3. Hard constraints (read before writing any code)

| # | Constraint | Consequence |
|---|---|---|
| B1 | **No `rclpy` inside Blender.** Blender bundles its own CPython; ROS2 Humble's extensions are built for CPython 3.10. ABI incompatibility, not a `PYTHONPATH` problem. | Under A/C any robot link is a socket. Under B/C the shared kinematics module works because it is **pure Python with no ROS imports** — that is the whole point of that design rule. |
| B2 | **`bpy` is not thread-safe.** Calling `bpy.*` from a background thread corrupts state or crashes Blender, often not immediately. | *(Option A/live telemetry only)* Network I/O on a worker thread touching **only** a `queue.Queue`; all `bpy` mutation in a `bpy.app.timers` callback on the main thread. |
| B3 | **The UI must never block.** | Long computations (validation of a big mesh) must be chunked across timer ticks or run with a progress modal, not in one call. |
| B4 | **Stdlib only.** Do not require `pip install` into Blender's Python — users will not do it and it breaks on upgrades. | `math`, `json`, `queue`, `threading`, `socket`, `mathutils`, `bpy`, `gpu`. **Note: no `numpy` guarantee** — write the kinematics module in plain Python, or keep a plain-Python fallback path. |
| B5 | Blender quaternions are **(w, x, y, z)**; ROS uses **(x, y, z, w)**. | Convert in exactly one place. The base/tip stick format mostly sidesteps this — deliberately. |
| B6 | Blender's unit scale is configurable (`scene.unit_settings.scale_length`) and objects carry arbitrary parent/scale transforms. | Always read `object.matrix_world` and apply the scene scale. Never read `object.location` directly. |
| B7 | Must be usable with **no robot and no ROS present**, on Windows. | Option C satisfies this by construction. |
| B8 | **Blender 5.x API.** Verify the addon manifest format (`blender_manifest.toml` extensions vs. legacy `bl_info`) against the 5.2 docs before writing the scaffolding — this changed in the 4.2 extensions platform. | Affects packaging and distribution only. |

---

## 4. Addon architecture

```
so100_builder/
├── blender_manifest.toml   (or bl_info — see B8)
├── __init__.py             register()/unregister(), module wiring
├── prefs.py                AddonPreferences
├── properties.py           PropertyGroups on Scene and Object, incl. `robot_id`
├── kinematics/             *** one VENDORED VERBATIM package per robot (§4a) ***
│   ├── so_arm_100/         from so_arm_100_kinematics -- hardware-validated
│   │   ├── __init__.py     exports + __version__
│   │   ├── chain.py        FK + closed-form IK (ROS2 doc §9.3)
│   │   ├── constants.py    URDF-derived geometry, limits, grasp offset
│   │   ├── envelope.py     reachability queries
│   │   ├── grasp.py        grasp-orientation transform (ROS2 doc §9.6) --
│   │   │                   core/validate.py calls this, never reimplements it
│   │   ├── jaw_clearance.py swept-gripper collision pre-filter
│   │   └── VERSION         must match __version__; recorded in every build file
│   │                       (see that package's own README.md for the rules)
│   └── kr10_r900_2/        placeholder -- kr10_r900_2_kinematics doesn't exist yet
├── core/
│   ├── robots.py           *** robot registry + kinematics-package contract (§4a) ***
│   ├── transform.py        Blender world <-> base_link metres (B5, B6)
│   ├── sticks.py           mesh edges -> StickSpec; joint allowance; cut list
│   ├── order.py            *** build-order solver (§6) ***
│   ├── mirror.py           robot-mirror rig geometry (§10.4)
│   ├── validate.py         order-aware validation (§7)
│   └── state.py            build state; .blend + JSON persistence (QB4)
├── io/
│   └── build_file.py       the build file + status sidecar (protocol Part A)
├── net/                    (Option A / live telemetry only)
│   ├── client.py           worker thread + queues + main-thread pump
│   └── protocol.py
├── ops/                    operators
├── ui/
│   ├── panels.py           N-panel "SO-100"
│   ├── lists.py            UIList of sticks with status icons
│   └── overlay.py          GPU overlay: envelope, order, status colours
└── tests/                  run via `blender --background --python tests/run.py`
```

**Each `kinematics/<robot_id>/` folder is a verbatim copy, never a fork.**
Add a CI check (or a `make sync-kinematics`) comparing it against its
ROS2/ROS upstream package, and a `VERSION` file so a mismatch is loud rather
than silent.

### 4a. ✅ Multi-robot support — DONE 2026-08-21; kr10_r900_2 vendored for real 2026-08-22

**Kicked off 2026-08-21** (docs/STATUS.md): the addon is not tied to one
arm. BRIDGE_PROTOCOL.md §A.1.1 defines a `robot` id per supported robot
(`so_arm_100`, `kr10_r900_2`) and requires each to have its own vendored
kinematics package under `kinematics/<robot_id>/`. **Both are now real,
vendored packages** — `kr10_r900_2_kinematics` (a genuine, tested 6-DOF
closed-form solver, `kuka_control` repo) landed 2026-08-22, sooner than
expected; this section documents the addon side of both.

**`core/robots.py`** is the seam: a small `RobotProfile` per registered robot
id (the vendored kinematics module, plus optional `build_volume_min_m` /
`build_volume_max_m` / `stock_section_m` defaults), and `get_robot(robot_id)`
to look one up. Its own module docstring is the canonical, load-bearing
statement of **the kinematics-package interface contract** every robot's
vendored package must satisfy — summarized:

| Name | Contract |
|---|---|
| `__version__` | Written into every build file as `kinematics_version`. |
| `STICK_LENGTH_RANGE_M` | This robot's own stock-length limits (§5.4). |
| `grasp_offset_for_length(length_m)` | The per-stick-length grip offset (D13). |
| `fk(joint_angles_rad)` | `(position, rotation_matrix)`. Loop over `zip(CHAIN, joint_angles_rad)` then unconditionally append ONE fixed final transform — `core/mirror.py`'s preview rig depends on exactly this shape (see below), for any joint count. |
| `check_jaw_clearance(base, tip, placed_sticks, ...)` | The swept-gripper collision pre-filter (§6 C3). |
| `Unreachable` | Exception class, caught by name, never by string-matching. |
| `JOINT_NAMES` | This robot's joint order. |

**"Solve a stick placement" is deliberately NOT in that table.** so_arm_100's
`solve_stick_placement(base, tip)` and kr10_r900_2's own function of the same
name are compatible by coincidence (both take just `base`/`tip` and return
joints, raising `Unreachable`), but kr10_r900_2's round stock has a
genuinely free roll DOF that single-roll call under-reports — its own
`solve_stick_placement_any_roll(base, tip)` (sweeps roll × branch) is what a
real placement needs. `core/validate.py` and `ops/mirror.py` therefore
dispatch **explicitly per robot id** for this one operation
(`_validate_stick_so_arm_100` / `_validate_stick_kr10_r900_2`, and a
matching `_solve_placement()` helper in `ops/mirror.py`) rather than forcing
a fake shared signature across two genuinely different arms.

A new robot means: a new vendored sub-directory under `kinematics/`, a new
`RobotProfile` entry in `core/robots.py`, a new row in BRIDGE_PROTOCOL.md
§A.1.1's table, and — if its own "solve a placement" shape differs from both
existing robots' — its own dispatch branch in `core/validate.py`/
`ops/mirror.py`. Everything else (the registry, `ops/build.py`'s export, the
mirror rig's FK-derivation math) needs no change.

**`core/mirror.py`'s preview rig needs no per-robot EE_OFFSET constant.**
so_arm_100 exports one (`constants.EE_OFFSET`); kr10_r900_2 does not (its
tool transform is a private, composed `(translation, rotation)` pair inside
`chain.py`, never meant to be read from outside). Rather than require every
future robot to export a named constant, `joint_frames()` derives the fixed
offset by calling the robot's own `fk(())` — zero joint angles, so the
`zip(CHAIN, joint_angles_rad)` loop runs zero times and whatever `fk()`
returns *is* the fixed final transform in isolation. Confirmed empirically
to match so_arm_100's own `EE_OFFSET` exactly, and works identically for
kr10_r900_2. Not a re-derivation of either robot's math — a call into their
own already-tested `fk()`, the same "not a second FK implementation"
principle `joint_frames()` already followed for so_arm_100 alone.

**A new Scene-level `robot_id` EnumProperty** (`properties.py`, default
`so_arm_100`) is threaded through:
- `core/validate.py` — so_arm_100 keeps its exact existing behaviour
  (`_validate_stick_so_arm_100`, byte-for-byte unchanged); kr10_r900_2 gets
  its own `_validate_stick_kr10_r900_2` (uses `solve_stick_placement_any_roll`,
  see above); any *other* robot id falls through to a minimal
  `_validate_stick_generic` (contract-only `solve_stick_placement`, any
  failure becomes a clean `Verdict(False, reason)`, never a crash) as a
  safety net until it earns its own richer path.
- `core/mirror.py`'s `joint_frames()` — takes the robot's kinematics module
  explicitly; derives the tool offset via `fk(())` (see above) instead of a
  named constant.
- `ops/mirror.py`'s preview rig — resolves the module via
  `core_robots.get_robot(props.robot_id)`, dispatches to the right "solve a
  placement" call via its own `_solve_placement()`, and reports (rather than
  raises) any failure as the rig's status line. Its point-count/segment
  logic is robot-agnostic (`_segments(point_count)`, not a hardcoded 5).
- `ops/build.py`'s `Export Build File` — stamps `robot`/`kinematics_version`
  from the selected profile, and **refuses to export** (a clean operator
  error, not a crash or an invented number) on either of two independent
  missing facts: `RobotProfile.is_vendored` being false (no kinematics
  package at all), or `has_build_volume` being false (no confirmed
  `build_volume_min_m`/`max_m`) — knowing *where* a robot may build is a
  separate fact from being able to compute *whether* a placement is
  reachable there. Neither gate fires for either registered robot today.
- `ops/design.py`'s `Create Robot Base` operator (added 2026-08-22 — the
  addon previously had no way to create a base empty for anything but
  so_arm_100) — names the created empty per robot (`SO100_Base` /
  `KR10_Base`, `base_empty_name(robot_id)`, the one place that name is
  decided) and sizes its viewport gizmo up for KR10's larger scale (a
  cosmetic choice, not a physical constant). `ui/panels.py`'s Design panel
  shows the button's label dynamically from the same function, so the two
  can never drift apart, and `properties.py`'s `base_empty` pointer
  property is itself already robot-agnostic (marks "the selected robot's
  own URDF root frame", not literally `base_link`).

**`kr10_r900_2`'s build volume is confirmed** (given directly by the user
2026-08-22, corrected 2026-08-23 -- the direction was actually +X, not −Y,
docs/STATUS.md): a 300×300×300 mm cube centred 450 mm from the robot
origin along +X — `X ∈ [300, 600]`, `Y ∈ [−150, +150]`,
`Z ∈ [0, 300]` mm in `base_link`, same convention as so_arm_100's own
`BUILD_VOLUME_MIN_M`/`MAX_M` (one horizontal axis centred on 0, the other
centred on the given distance, Z sitting on the base plate rather than
centred vertically). Set
directly on `core/robots.py`'s `RobotProfile` construction, not read from
the kinematics package — `kr10_r900_2_kinematics` has no build-volume
concept at all (BRIDGE_PROTOCOL.md §A.1.1 treats that as this addon's own
concern). `RobotProfile.stock_section_m`, by contrast, *is* read from the
vendored package now that it's real: `kr10_r900_2_kinematics.STICK_SECTION_M`
= 2 mm (round stock), matching so_arm_100's own pattern.

✅ **`ui/overlay.py`'s build-volume box and `ui/panels.py`'s Reference panel
numbers are robot-aware** (added 2026-08-23, `docs/STATUS.md`): both now
read `core_robots.get_robot(props.robot_id).build_volume_min_m`/`max_m`
instead of always drawing/printing `so_arm_100`'s box — selecting
`kr10_r900_2` shows its real 300×300×300 mm cube in the viewport. The
Reference panel's so_arm_100-specific empirical caveats ("98% reachable
for vertical sticks", the `GRASP_OFFSET_M` grip-height note) stay gated to
`robot_id == "so_arm_100"` rather than being generalized — they are that
robot's own measured/derived facts, not a general truth to project onto a
robot they were never checked against.

✅ **`core/order.py`'s build-order solver is robot-aware** (fixed 2026-08-23,
`docs/STATUS.md`, reported by the user as "Compute Build Order" reporting
`0 in order - 0 warnings - 1 error` for an otherwise-valid kr10_r900_2
design). `OrderSolver` now takes a `robot_id` (default `so_arm_100`,
preserving that robot's exact existing behaviour) and resolves its own
`_reachability()` check, its C2 cost heuristic's shoulder-axis reference
point (`CHAIN[0][1]`), and its C3 jaw-clearance model's `grasp_offset_m`/
`jaw_width_m`/`section_m` defaults from the SELECTED robot's own kinematics
(`GRASP_OFFSET_M`, `JAW_RADIUS_M`, `STICK_SECTION_M`) — previously every one
of these silently used `so_arm_100`'s, regardless of `robot_id`, so a stick
correctly validated as buildable against kr10_r900_2's own kinematics
(`core/validate.py`, robot-aware since 2026-08-22) would still be evaluated
against so_arm_100's much smaller reach *inside the order solver*, forcing
either a bogus rejection or (if nothing in the design even reads as
grounded relative to the selected base empty) a single opaque
`floating_component` error — exactly what was reported, with no indication
which robot's kinematics was actually being checked. `ops/order.py`'s
`build_solver()` now passes `robot_id=props.robot_id`. `jaw_length_m`
(0.030 m, an so_arm_100-shaped estimate — see the inline comment) is the
one piece **not** yet sourced per robot: no equivalent real-world
measurement exists for kr10_r900_2 yet, and it is a soft, over-cautious
pre-filter (a C3 warning, never a hard block), so it is left as-is rather
than inventing an unfounded KUKA-specific formula.
`floating_component`'s own error message now also names the ground
tolerance and suggests checking the design mesh's position against the
selected robot's own base empty, since that remains a likely real cause of
this error independent of the fix above.

✅ **`core/sticks.py`'s mesh-expansion geometry is robot-aware** (fixed
2026-08-23, `docs/STATUS.md`, user request after the C2 fix above didn't
fully resolve a still-recurring build-order error). `extract_sticks()` now
takes `robot_id` (default `so_arm_100`, byte-for-byte unchanged) and
resolves its `joint_allowance_m`/`section_m`/`min_stick_length_m`/
`max_stick_length_m` defaults, and `hard_min_stick_length_m(robot_id)`'s
own floor, from the SELECTED robot's own kinematics
(`JOINT_ALLOWANCE_M`/`STICK_SECTION_M`/`STICK_LENGTH_RANGE_M`/
`MIN_GRASP_OFFSET_M`/`JAW_CONTACT_HALF_LENGTH_M`) instead of so_arm_100's
unconditionally — kr10_r900_2's real joint allowance (1 mm/end, round-stock
contact) is a genuinely different physical model from so_arm_100's 3.25 mm
square-stock formula, not just a smaller number of the same shape.
`ops/design.py`'s `extract_with_autoflip()` now passes `robot_id=
props.robot_id` through (it already passed the OTHER four values explicitly
from the Design panel's own UI fields, so those were never silently wrong,
just defaulted to so_arm_100's numbers when a design started — see below).

**The Design panel's Stock/Stick Length fields still show so_arm_100's own
starting numbers regardless of `robot_id`**, since Blender's
`FloatProperty` defaults are fixed at class-registration time and cannot
depend on another property's runtime value — there is no per-instance-
dynamic default in the Blender API for this, and `properties.py` does not
use `update=` callbacks (existing codebase convention). Rather than leave
that as a trap, a new **`Reset Stock to This Robot's Defaults`** button
(`ops.design.SO100_OT_reset_stock_to_robot_defaults`, in the Design panel's
Stock box) sets Stock Section, Joint Allowance, and Min/Max Stick Length to
the currently-selected robot's own defaults in one explicit, discoverable
action — worth pressing right after switching `robot_id` and before
drawing a new design. For kr10_r900_2, Stock Section and Joint Allowance
default to 2.0 mm / 1.5 mm — a UI-ergonomics starting margin the user
picked (`_STOCK_DEFAULT_OVERRIDES_MM` in `ops/design.py`), not the vendored
kinematics module's own `JOINT_ALLOWANCE_M` (1 mm, a measured hardware
value used in the actual stick-length math) — every field the button sets
stays fully editable afterward. `min_stick_length_mm`'s own static widget
`min=`
bound was also widened from so_arm_100's fixed 35 mm to
`core_sticks.safe_min_stick_length_bound_m()` (the lowest floor across
every registered robot, 18 mm today) so it can never block a value that is
genuinely valid for whichever robot is actually selected — the real,
robot-SPECIFIC floor is still enforced by `extract_sticks()`'s own runtime
check.

**`ops/design.py`'s build mesh is now named per robot too**
(`SO100_BuildMesh` / `KR10_BuildMesh`, `build_mesh_name(robot_id)` — mirrors
`base_empty_name()`'s own pattern exactly) — previously always
`SO100_BuildMesh` regardless of `robot_id`, another user-reported naming
mismatch. Like the base empty, only the object's name at CREATION time is
robot-aware; an existing build mesh from before this fix, or from a design
started under a different `robot_id`, keeps its original name until
cleared and re-extracted.

✅ **The build plate's own height is now adjustable** (added 2026-08-23, user
request: "I would like to be able to print 'floating' sticks. The build
plate can change in height, so I want to make this a possibility"). Every
Sec 5.2.2/6.2 ground check ("is this vertex on the plate", used both to
solve the mesh expansion and to decide which sticks can start a build
order) previously compared a vertex's Z against a fixed 0 — `core/sticks.py`'s
`Topology.ground_height_m` and `core/order.py`'s `grounded_vertices()`/
`floating_components()`/`OrderSolver` all gained a `ground_height_m`
parameter (default 0.0, so every existing design is unaffected byte-for-
byte) that shifts that reference instead. The physical build plate is a
real, height-adjustable object: a component that sits entirely above Z=0 —
previously always rejected outright as `floating_component`, unbuildable no
matter what — is not actually unbuildable, just unbuildable *at the plate's
current height*. Raising `ground_height_m` to that component's own lowest
point (now named directly in the `floating_component`/`below_plate` error
messages, e.g. "raising the build plate to about there would ground it")
makes it solve and order normally, exactly as if the plate had been
physically raised to meet it. Exposed as a new **Build Plate Height**
field (`properties.py`'s `build_plate_height_mm`, Design panel's Mesh
Expansion box, next to Ground mode) threaded through
`ops/design.py::extract_with_autoflip()` and `ops/order.py::build_solver()`
from the same scene property, so extraction's `below_plate` check and the
order solver's grounding check always agree on where the plate currently
is. Not exported in the build file (BRIDGE_PROTOCOL.md unchanged): a
raised plate only changes which components the addon treats as supported
during design/ordering, never any stick's own absolute placement
coordinates, so the ROS2 side needs nothing new to execute the file
correctly.

✅ **A `Require Build Plate` checkbox turns the ground check off entirely**
(added 2026-08-23, same-day follow-up: "sometimes for testing, I would put
a stick's base or other kind or shapes that are not a flat base, so I want
some flexibility for those scenarios"). `ground_height_m` above still
assumes a single FLAT plate at *some* height; some designs are held by
something this addon does not model as a plate at all — a stick's own base
used as a jig, a non-flat fixture. `core/sticks.py`'s
`Topology.ground_required` (default `True`) and `core/order.py`'s
`OrderSolver`/`grounded_vertices()`/`floating_components()` all gained a
`ground_required` parameter with the same default: `False` makes
`Topology.is_grounded()` permanently return `False`, so nothing is ever
plate-seated —

* the `floating_component` and `below_plate` checks are skipped entirely
  (there is no plate to be floating relative to, or below);
* the mesh-expansion solve naturally routes every acyclic component through
  its existing "no grounded vertex" fallback (`_solve_component_exact`'s
  `root = grounded[0] if grounded else min(group)`, already there for a
  transient case, now a real path) and the `GROUND_SLIDE` clamp/`GROUND_PIN`
  pin never fire, so the design solves fully free-floating;
* the order solver seeds `OrderSolver._available` from a new
  `core.order.arbitrary_anchor_vertices()` (one deterministic vertex per
  connected component — the lowest-id stick's own base end) instead of
  `grounded_vertices()`, so the search still has somewhere to start each
  disconnected part, and a `WARN_NO_BUILD_PLATE` global warning is added so
  the result is never silently presented as verified support-valid.

**Topological validity is untouched** — every subsequent stick still has to
attach to an already-placed one (or its own component's arbitrary anchor);
only the PLATE requirement is relaxed, never the "attach to something"
one. Exposed as `properties.py`'s `require_build_plate` (Design panel, Mesh
Expansion box, above Ground/Build Plate Height, which it hides when
unchecked since they have no effect without a plate) threaded through both
`ops/design.py::extract_with_autoflip()` and `ops/order.py::build_solver()`
from the same scene property. Like `ground_height_m`, not exported in the
build file: it only changes which components the addon's OWN checks treat
as supported, never any stick's coordinates.

✅ **"Check By Eye" stepping now follows BUILD order, not extraction/edge
order** (fixed 2026-08-23, user-reported: "the arrow goes to the next stick
maybe by edge number? It does not follow the newly computed order"). Sec
10.3's `props.sticks` deliberately always stays in EXTRACTION order —
`select_stick_in_viewport` needs "build-mesh edge *i* is `sticks[i]`" to
hold so it can index straight into the build mesh — but
`SO100_OT_step_stick` was doing plain `active_stick_index +/- 1` on that
same extraction-order index, which is only ever the build sequence by
coincidence. The Sticks list's own `filter_items` already solved exactly
this for DISPLAY via `build_order_permutation()` (`[build_order_per_item]
-> [display_position_per_item]`, unordered sticks sorting last); `step_stick`
now inverts that same permutation to find "the extraction index whose build
position is current position ± 1", so a repeated click (or hotkey) walks
the actual sequence the robot will place sticks in, falling back to plain
index stepping only when no order has been computed yet (there is no build
sequence to follow in that state). `build_order_permutation()` moved from
`ui/panels.py` to `core/state.py` to make this possible at all — the
function is pure Python (no `bpy`) and was only trapped in a `bpy`-importing
module before; `ops/design.py` importing `ui/panels.py` directly would have
been circular, since `ui/panels.py` already imports from `ops/design.py`.
This also fixed a pre-existing testability gap: the function's own tests
(`TestBuildOrderPermutation`) moved to a new `tests/test_state.py` and now
run in bare CPython, not just under Blender. New
`test_step_stick_follows_build_order_once_one_exists` in
`test_blender_integration.py` exercises the real operator path end to end
against the wireframe-cube fixture (whose build order provably differs from
its extraction order — the top ring can only be placed after the bottom
one) rather than only the pure permutation logic in isolation.

✅ **`kr10_r900_2` gained a robot base box, and its build volume dropped
20mm** (added 2026-08-23, user request: a 320×320×20mm box "below the robot
(origin)", and lowering the build volume box "to account the height of the
base box of the robot above"). `core/robots.py`'s `RobotProfile` gained
`base_box_min_m`/`base_box_max_m` (`None` by default, same "not confirmed /
not applicable" convention as `build_volume_min_m`/`max_m`) and a
`has_base_box` property; `kr10_r900_2`'s profile sets them to a 320×320×20mm
box centred on the robot's own X/Y origin, `Z ∈ [−20, 0]` mm — directly
BELOW the robot's own coordinate origin, representing its physical mounting
pedestal. Since the robot's own origin sits at the TOP of that pedestal,
20mm above the actual table surface, `kr10_r900_2`'s `build_volume_min_m`/
`max_m` Z shifted from `[0, 300]` to `[−20, 280]` mm to match — the build
cube's own floor now sits at the SAME table-level Z as the pedestal's own
floor, not at the robot's coordinate origin. `so_arm_100` has no base box
(`has_base_box` False, nothing drawn) — the user asked for this "just when
the kuka robot is selected". Purely a viewport/Reference-panel reference
aid, like `build_volume_min_m`/`max_m` before it: `ui/overlay.py`'s
box-corner math was factored into a shared `_box_edge_points()` helper so
`_volume_box_points()` and the new `_base_box_points()` share it rather than
duplicating the 12-edge wireframe logic, drawn in a distinct tan/brown
colour (`_BASE_BOX_COLOR`) so it reads as "physical furniture" rather than
"where you may build". `ui/panels.py`'s Reference panel shows its numbers
too, gated the same way. Never exported to the build file (module docstring
note on `base_box_min_m`/`max_m`): the ROS2 side already knows its own
robot's footprint from its URDF, so this stays a Blender-side design aid
only, unlike `build_volume_min_m`/`max_m` which IS exported. New tests in
`test_robots.py`, `test_blender_integration.py` (both the overlay points and
the shifted build-volume/export numbers); existing hardcoded `[0.3, -0.15,
0.0]`-style expectations across those files updated to the new `-0.02`
floor.

✅ **Build order can be manually reordered after computing it** (added
2026-08-23, user request: "After computing a build order, I want to be able
to move the steps before of after its position" — `[s1,s3,s4,s2]` moving
`s3` earlier gives `[s3,s1,s4,s2]`, later gives `[s1,s4,s3,s2]`). New
`core/order.py::OrderSolver.replay(sequence_ids)`, `solve()`'s counterpart:
places sticks in EXACTLY the given order rather than searching for one,
using each stick's EXISTING orientation (never re-deciding which end is
`base` — that stays exactly what the original solve, or `props.sticks[i].
flip`, already chose) and re-checking only what genuinely depends on order:
**C1 support** (does this stick's base attach to something already placed,
or the plate/an anchor, at ITS NEW position?) and **C3 jaw clearance**
(depends on what is already built). Reachability itself does not depend on
order (Phase B validates a stick's own fixed geometry), so it is read from
the solver's own cache, unchanged. A position that breaks C1 or jaw
clearance is still placed there and flagged with a clear reason
(BRIDGE_PROTOCOL.md A.2: every stick appears, in order) — never silently
dropped or silently re-ordered back to "correct" by a hidden second solve.
A stick id missing from the given sequence (new since it was last stored) is
appended afterward in a stable id-sorted order rather than lost.

New `ops.order.SO100_OT_move_build_step` operator (Sticks panel, "Move
Earlier"/"Move Later" buttons next to Check By Eye) swaps the active
stick's `order` with its immediate NEIGHBOUR only — matching the request's
own adjacent-swap examples exactly, not an arbitrary jump to any position —
then immediately re-runs the pipeline through `replay()` (never `solve()`)
so warnings/errors and the build mesh reflect the new order right away, not
only at export time. `ops.build.SO100_OT_export_build_file` now calls
`solver.replay(ordered_ids(props))` instead of `solver.solve()` whenever
`props.has_order` is already True, so an export **respects** a prior manual
reorder instead of silently discarding it with a fresh automatic search —
`solve()` still runs, as before, the first time no order exists yet.
`ordered_ids()` (previously private to `ops/build.py`) moved to
`ops/design.py` so `ops/order.py`'s new operator could read it too without
`ops/order.py` importing `ops/build.py` (which already imports
`build_solver` FROM `ops/order.py` — the other direction would be
circular). New `TestReplay` in `test_order.py` (a deterministic straight
3-stick chain, forced order, no ties for the search to break
arbitrarily — reused from `TestSimpleStack`) covers the algorithm directly;
new tests in `test_blender_integration.py` exercise the real operator path
end to end, including confirming export actually reflects a manual reorder.

⚠ **What is *not* yet robot-parameterized, deliberately (scope boundary, not
an oversight):** `core/order.py`'s C3 jaw-clearance model's `jaw_length_m`
constant (0.030 m, so_arm_100-shaped, no kr10_r900_2 equivalent measurement
yet — see that module's own note) is the one piece of the build pipeline
still not sourced per robot. It is a soft, over-cautious pre-filter (a
warning, never a hard block), so this is a quality nuance, not a
correctness gap like the ones fixed above.

---

## 5. From wireframe mesh to cut list

This is the step where "a mesh" becomes "wooden sticks", and it is where the
geometry gets real.

### 5.1 Extraction

Each **edge** of the designated mesh becomes one stick. For each edge:

1. Take both vertices' world positions via `matrix_world`.
2. Apply `scene.unit_settings.scale_length` → metres.
3. Transform into the robot base empty's frame (`SO100_Base` for so_arm_100,
   `KR10_Base` for kr10_r900_2 — §4a/§9.1) → this *is* that robot's own
   URDF root frame (`base_link` for so_arm_100, `base` for kr10_r900_2).
4. Decide which end is `base` (the end that seats down): the lower Z, unless
   the stick connects to an already-placed stick at the other end — see §6.
   Overridable per stick.

### 5.2 Joint allowance — sticks have volume, mesh vertices do not

⚠ **A wireframe mesh's edges meet at a mathematical point. 6.45 mm square
sticks cannot.** At any shared vertex the stick volumes interpenetrate, which
is physically impossible. Each stick must therefore stop **3.25 mm short** of
the ideal vertex, leaving a gap that a glue blob fills.

✅ **N4 decided: 3.25 mm per shared end.** An end that seats on the base plate
is **not** shared and gets no allowance.

That value is half the 6.45 mm section (6.45/2 = 3.225), i.e. each stick stops
at the *face* of the joint rather than at its centreline — geometrically exact
for a **90°** joint.

⚠ **3.25 mm is per stick end, so the gap between two stick ends is 6.5 mm.**
Both sticks meeting at a vertex stop 3.25 mm short of it, from opposite sides.
Do not read "3.25 mm gap" as the end-to-end distance — that phrasing is what
put the wrong numbers into [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) §A.2's
worked example (corrected 2026-07-28).

### 5.2.1 ⭐ Mesh expansion — preserve stick length, grow the design

**This is the defining behaviour of the addon.** There are two ways to
reconcile a mesh with physical sticks, and the user has chosen the second:

| | Approach | Consequence |
|---|---|---|
| ❌ | **Shorten the sticks** to fit the design. `cut = edge − 3.25×shared_ends` | The design is preserved exactly, but every stick needs a bespoke cut. |
| ✅ | **Grow the design** to fit the sticks. `required_edge = stick_length + 3.25×shared_ends` | Stick lengths stay whatever you drew and cut; the *structure* ends up slightly larger. |

So: **the design mesh's edge lengths are the physical stick lengths** — what
you actually cut. The addon then moves vertices apart so sticks of exactly
those lengths sit with a 3.25 mm clearance from every shared vertex (so a
6.5 mm gap between the two stick ends that meet there).

**The inverted-U example, worked through.** Three sticks, all physically
110 mm:

| Stick | Shared ends | Required edge length |
|---|---|---|
| bottom-left upright | 1 (top only — base sits on the plate) | 110 + 3.25 = **113.25 mm** |
| bottom-right upright | 1 | **113.25 mm** |
| top horizontal | 2 (both ends) | 110 + 2×3.25 = **116.5 mm** |

*(The values 114.25 / 117.5 in the original request are off by 1 mm — the
formula gives 113.25 and 116.5. Worth confirming, since these are the exact
numbers the addon implements.)*

### 5.2.2 The constraint solve

Growing each edge by a *different* amount is **over-constrained in general**.
Two cases:

**Acyclic structures (trees) — exact.** Walk outward from the grounded
vertices and push each vertex along its parent edge's direction by the
required amount. Every edge lands on its target length exactly, and the
design's angles are preserved. No solver needed.

**Structures with closed loops — usually still exact, sometimes not.** In the
inverted U, raising both top vertices by 3.25 mm fixes the uprights but
leaves the top edge at 110 mm; widening it to 116.5 mm then tilts the
uprights by ~1.6°.

*(Corrected 2026-07-28 against the implementation: that tilt does **not**
leave a residual. Tilting shortens the uprights' vertical extent, and the
solver simply raises the top vertices to compensate, landing all three edges
on target — 113.25 / 116.50 / 113.25 mm — to better than 0.001 mm. The
earlier claim here — that the uprights end at 113.30 mm and that "every edge
cannot be satisfied simultaneously" — came from stopping the reasoning one
step early. A 4-vertex U has enough freedom to be exact. Genuinely
over-constrained cases do exist — see the flat-square example below — but
this is not one of them.)*

Solve it as **iterative constraint relaxation** (position-based dynamics
style): repeatedly, for each edge, move both endpoints along the edge to
correct its length error. A few dozen iterations converge to sub-0.1 mm
residuals for structures of this scale.

✅ **Grounded vertices: SLIDE, decided 2026-07-28.** A grounded vertex is
**locked to z = 0 but free to move in XY**. It cannot rise or sink — the base
plate is physical — but it may slide across the plate as the design grows.

This replaces an earlier "pin grounded vertices" instruction, which was
wrong: fully pinning them makes **any closed loop lying on the base plate
unexpandable**. Every vertex of a square drawn flat on the plate would be
immobile, so none of its edges could grow at all and all four would report a
−6.5 mm residual. That contradicts §5.2.1's whole premise that the design
grows to fit the sticks. Under SLIDE the square simply solves exactly, coming
out slightly larger — which is the intended behaviour.

**Where the two modes actually differ.** Not on the inverted U above: both
PIN and SLIDE reach the same answer there (all three edges on target, the
uprights tilted ~1.65°), because two free top vertices already give the
solver enough room. The difference appears only when pinning removes *all*
freedom — a closed loop lying flat on the plate. A square drawn on the floor
has four grounded vertices and no free ones:

| | PIN | SLIDE |
|---|---|---|
| square flat on the plate | no vertex can move; all 4 edges −6.5 mm; unbuildable | solves exactly, square comes out 6.5 mm larger |
| wireframe cube's bottom ring | 4 sticks flagged | solves exactly |
| inverted U | exact, uprights tilt ~1.65° | exact, uprights tilt ~1.65° |

Both remain available — the addon exposes PIN as a toggle — but **SLIDE is
the default**.

⚠ **Note which components this changes.** A component that is a tree with at
most one grounded vertex is still solved exactly by the outward walk of the
acyclic case; the ground mode only matters once a second anchor or a mesh
cycle closes a loop. Pinning is what makes the base plate behave as an extra
edge, so *two* pinned feet turn even an acyclic mesh into the looped case —
which is exactly why the inverted U above is treated as looped despite having
4 vertices and 3 edges.

**Report the residual per edge.** Any edge that cannot reach its target within
tolerance (a genuinely over-constrained case, e.g. the flat square under PIN)
is a design the sticks will not physically fit — the user must know which
one, not discover it at the glue gun.

**Optional simplification: uniform growth.** Add 3.25 mm at *every* end,
jointed or not — so every edge grows by exactly 6.5 mm and free ends simply
overhang the design vertex by 3.25 mm (harmless: it is an exposed stick end).
A uniform additive growth is far better conditioned than a per-edge variable
one, and removes most of the residual error on looped structures. Offer it as
a toggle.

### 5.2.3 ⚠ The built sculpture is larger than the mesh you drew

This follows unavoidably from fixed-length sticks plus physical joints: the
structure grows by ~3.25 mm **per joint along any path through it**. A
10-layer tower ends up ~65 mm taller than designed.

Three consequences the addon must handle:

1. **Validate the expanded mesh, not the design mesh.** A design that fits the
   240 × 160 × 200 mm build volume may not fit after expansion.
2. **Show both.** Display design vs. expanded overall dimensions so the growth
   is never a surprise.
3. **Non-destructive.** Generate a derived *build mesh* as a separate object
   and never modify the user's design mesh. They will iterate.

The addon must show, per stick,
`design edge (= stick length) → shared ends → required edge → residual`.

⚠ **Shallow angles need more.** For two sticks of width `w` meeting at angle
θ, the interpenetration runs about `w / (2·tan(θ/2))` along each axis:

| joint angle θ | required allowance |
|---:|---:|
| 90° | 3.2 mm ✅ matches the fixed value |
| 60° | 5.6 mm |
| 45° | 7.8 mm |
| 30° | 12.0 mm |

The user glues with a glue gun and has accepted gaps and blobs, so the
**fixed 3.25 mm is the default**. But the addon should compute the angle-based
value per vertex and **warn** (`tight_clearance`) when it exceeds the fixed
one — below ~45° the sticks will physically clash, which no amount of glue
fixes. Expose the allowance as a UI field so it can be raised globally.

This warning is now *more* important, not less: under the §5.2.1 model the
gaps are what the expansion is sized around, so an under-sized allowance at a
shallow joint means the solved mesh puts two sticks in the same place.

### 5.3 Two length modes (QB1/QB2 — both required)

Under the §5.2.1 model the design mesh's **edge lengths are the stick
lengths**, so both modes are pre-passes that run *before* the expansion solve:

| Mode | Behaviour |
|---|---|
| **Design-driven** (default) | Stick length = the edge length as drawn. Cut each stick to that. Maximum design freedom, every stick potentially unique. |
| **Fixed stock lengths** | The user defines the available lengths (e.g. 80 / 100 / 120 / 150 mm). Each edge snaps to the nearest **before** expansion, and the addon reports the per-edge error and can move the vertex to make the design exact. Far easier for testing — and it means the whole build uses a handful of pre-cut lengths instead of a bespoke cut list. |

⚠ Order matters: **snap first, then expand.** Snapping after the solve would
re-break every edge length the solver just satisfied.

### 5.4 Length limits

Enforce **50–150 mm** on the **stick length** — which, under §5.2.1, is the
*design* edge length, not the expanded one. The expanded edge is longer by the
joint gaps and is not what gets cut or gripped.

D13 (ROS2_IMPLEMENTATION_PLAN.md §4) replaced the old *fixed* grip offset with
one that adapts per stick (`kinematics.grasp.grasp_offset_for_length()`):
always the largest offset that keeps the jaws' contact fully on the stick,
never less. That is what let the minimum drop from 80 mm to 50 mm.

Two distinct limits (ROS2 doc §8.2):
- **`min_stick_length` = 50 mm** (`STICK_LENGTH_RANGE_M[0]`) — the user's
  stock threshold. A UI field, so it can be lowered further, but only after
  re-examining `MIN_GRASP_OFFSET_M` (see below).
- **Hard floor = 35 mm** — `MIN_GRASP_OFFSET_M` (30 mm) +
  `JAW_CONTACT_HALF_LENGTH_M` (5 mm). Below this, `grasp_offset_for_length()`
  itself raises `Unreachable`: no offset both fits within the stick and clears
  the floor-clearance floor. Never allow the field below it.

⚠ **Read both constants from the shared kinematics module, never hardcode
them here.** `MIN_GRASP_OFFSET_M` is an unmeasured placeholder — same status
as `GRASP_OFFSET_M` elsewhere in this doc — chosen low enough that it does not
bind anywhere in `STICK_LENGTH_RANGE_M` today (the 50 mm case computes to
45 mm), but not yet confirmed on hardware. Confirm with Phase 0 before
trusting either number for a real short stick.

### 5.5 Cut list output

A stick can only be loaded into the feeder by a human who knows which one it
is. Export, and show in the UI, an ordered table:
`# | stick id | stick length (mm) | build order | status`.

Under the fixed-stock-length mode this collapses to a tally
(*"12 × 100 mm, 8 × 120 mm"*), which is what you actually want at a saw.
Provide a printable/CSV export either way.

---

## 6. Build-order generation (the "slicer" algorithm)

The user's instinct — sort by average Z, like a 3D printer's layers — is the
right *starting* point but is not sufficient on its own. Three real
constraints, in priority order:

**C1 — Support.** A stick can only be placed if its base end is either on the
base plate or at a vertex of an already-placed stick. Nothing floats. This is
a **dependency graph** constraint, not a height one — and it is the constraint
that a pure Z sort silently violates.

**C2 — Accessibility.** The robot must be able to reach the placement without
the arm or jaws hitting what is already built. This *is* mostly a height
constraint (hence bottom-up), but not only: a low stick sitting *behind* a
finished tall structure is blocked. Because the arm reaches outward
horizontally (ROS2 doc §9.4), the secondary rule is **build far-from-robot
first, near-to-robot last** within a layer.

⚠ **A design constraint found while implementing this (2026-07-29), worth
knowing before drawing a sculpture: tangential horizontals are cheap, radial
horizontals are expensive.** A horizontal stick pointing *at or away from*
the robot needs the tool vertical, which §9.4 says costs ~150 mm of reach —
and that is enough to matter. A square-ring tower at the nominal build
position came out with **exactly its two radial rungs unreachable at every
height and stick length tried**, while its two tangential rungs were fine.
A "ladder" (stacked inverted-Us, all horizontals tangential) builds cleanly
in the same space. No ordering can fix this: it is a property of the
placement, not the sequence.

**C3 — Jaw clearance.** The jaws sit only ~51 mm above the stick's base — i.e.
right at the glue joint. The cone around each target vertex must be clear.
Placing the sticks that share a vertex in a bad order can make the last one
impossible to insert.

### 6.1 Proposed algorithm — greedy topological build with backtracking

```
vertices = merge mesh vertices within tolerance
grounded = { v : v.z <= epsilon }            # on the base plate
placed   = {}                                 # simulated scene
order    = []

while sticks remain:
    candidates = [ s for s in remaining
                   if s has an end that is grounded or at a placed vertex ]
    if not candidates:
        report "floating component — no stick can be placed next"; stop

    for s in sorted(candidates, key=cost):
        orient s so its supported end is `base`
        if validate(s, placed):               # IK + jaw clearance + collision
            order.append(s); placed.add(s); break
    else:
        backtrack()                           # undo the last choice, try the next best

cost(s) = ( max_z(s),                         # primary: build upward
            -distance_from_robot(s),          # secondary: far side first
            -support_count(s) )               # tertiary: prefer better-anchored
```

Notes:
- The `validate()` call is what makes this a *slicer* rather than a sort: the
  order and the buildability are the same problem (ROS2 doc §10.1).
- Backtracking keeps it correct without an exponential search in practice —
  cap the backtrack depth and report honestly if the cap is hit.
- Run it in chunks across timer ticks for big meshes (B3).

### 6.2 Structural cases to detect and warn about

| Case | Why it matters | Suggested handling |
|---|---|---|
| **Cantilever** — a stick glued at one end only, far from vertical | The robot releases it and gravity acts before the glue sets | Warn; prefer orders that give a stick two anchors where possible; suggest holding time |
| **Loop closure** — the last stick of a triangle/quad | Must fit exactly between two already-glued vertices; absorbs all accumulated error | Flag as high-risk; suggest cutting ~1 mm short (ROS2 doc §10.2) |
| **Floating component** — a sub-graph with no grounded vertex | Unbuildable | Hard error, name the component |
| **High-valence vertex** — many sticks meeting at one point | Jaw clearance collapses | Warn with the computed clearance |
| **Out-of-plane tilt** | ⚠ **Corrected 2026-07-28 (ROS2 doc §9.4) — this IS reachable**, contrary to what this row used to say. Still comparatively untested territory (relative to a full self-collision model), so worth a second look. | Warn (not a hard error), with the offending angle, so the user can "Check By Eye" and decide — see `so100_builder/core/validate.py`'s `WARN_OUT_OF_PLANE_TILT` |

---

## 7. Validation & feedback

`core/validate.py` replays the build order through the vendored kinematics:

```
for stick in order:
    reachable?        -> closed-form IK, exhaustive over both elevation
                          branches and both elbow branches, each candidate
                          verified by round-tripping through fk() (ROS2 doc
                          Sec 9.3/9.6; Sec 9.4's "tool axis must stay in the
                          arm plane" governs the TOOL, not the stick -- an
                          out-of-plane STICK tilt is reachable, see Sec 9.4's
                          correction)
    jaw clearance?    -> swept jaw box vs. already-placed sticks [Phase C]
    within envelope?  -> cached reachability map [viewport overlay only]
    -> verdict (+ a warning, not a hard fail, for an out-of-plane tilt --
       still comparatively untested territory), then add to the scene
```

Per-stick verdicts drive everything the user sees: list icons, viewport
colours, and a summary (`42 sticks · 38 buildable · 4 impossible`). Reasons
must be specific and actionable — `"out of reach: 470 mm, max 442 mm at this
height"` rather than `"unreachable"`.

⚠ **This is an upper bound, not a guarantee.** It does not model self-collision
of the whole arm, the mount platform, or MoveIt's own path planning. ROS2
re-validates on load and MoveIt has the final word.

✅ **Phase B implemented (2026-07-28) as permissive**, decided with the user:
trust the closed-form IK's own success/failure, including for out-of-plane
tilts (§9.4's correction). MoveIt re-validates on load regardless, so an
over-eager Blender-side rejection only costs a design iteration for nothing.
See `so100_builder/README.md`'s Phase B section for the full derivation.

---

## 8. Coordinate transform (`core/transform.py`)

```python
def blender_to_robot(vec_world, base_matrix_world, scale_length):
    """Blender world-space point -> metres in base_link."""
    local = base_matrix_world.inverted() @ vec_world
    return tuple(c * scale_length for c in local)
```

- Blender and ROS are both **right-handed, Z-up** — no axis flip needed. This
  is a common source of bugs when people assume Y-up; do not "fix" it.
- Apply `scale_length` **after** the inverse transform, to translation only.
- Quaternion conversion (w,x,y,z) ↔ (x,y,z,w) lives here and nowhere else.

---

## 9. Scene data model

### 9.1 Conventions
- An empty (scene pointer property, `props.base_empty`) marks the selected
  robot's own URDF root frame — named per robot by `ops/design.py`'s
  `Create Robot Base` operator (§4a): `SO100_Base` for so_arm_100 (its own
  `base_link`), `KR10_Base` for kr10_r900_2 (its own `base`, a different
  name for the same *role* — see that package's own README). Moving it
  repositions the whole design relative to the robot without re-authoring
  anything.
- The wireframe mesh object is designated by a scene pointer property.
- The **build volume** is drawn as a reference box:
  ✅ **240 × 160 × 200 mm centred at Y = −370 mm** — i.e. X ∈ [−120, +120],
  Y ∈ [−290, −450], Z ∈ [0, 200] mm in `base_link` (ROS2 doc §1.2 N2).
  98 % of it is reachable for vertical sticks; the overlay must show the
  actual reachable region inside it, not just the box.

### 9.2 Per-stick state
Stored per edge (an edge attribute layer, or a dict keyed by a stable edge id
— note edge indices are **not** stable across mesh edits, so hash the vertex
positions or maintain an explicit id layer):

| Field | Meaning |
|---|---|
| `id` | Stable identifier, never reused |
| `order` | Build order index |
| `status` | `pending` / `buildable` / `impossible` / `placed` / `failed` / `skipped` |
| `reason` | Specific explanation for `impossible` / `failed` |
| `stick_length_mm` | Physical length to cut — the design edge length (§5.2.1), after any stock snapping |
| `expanded_edge_mm` | The solved edge length after mesh expansion, and its residual |
| `flip` | Manual override of which end is `base` |

### 9.3 Persistence (QB4 — both, as requested)
- **In the `.blend`**: the authoritative live state; travels with the design.
- **A compact JSON sidecar**, rewritten on every status change: survives "don't
  save changes", is diffable, and is what ROS2 reads. Format in
  [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) Part A.
- On load, if the two disagree, show both and let the user choose — never
  silently pick one.

---

## 10. UI specification

3D View sidebar (`N` panel), category **RA130** (the addon's own name --
renamed from "SO-100" 2026-08-23, since multi-robot support means the addon
is no longer specific to that one robot; see `docs/STATUS.md`):

### 10.1 `Design`
Robot picker (§4a) · robot base picker · mesh picker · stock section (6.45 mm) · length mode
(§5.3) · joint allowance · **Extract Sticks** · summary counts.

### 10.2 `Plan`
**Compute Build Order** · **Validate** · summary
(`42 sticks · 38 buildable · 4 impossible`) · warnings list from §6.2 ·
**Export Build File** · **Export Cut List**.

### 10.3 `Build`
`UIList` of sticks in build order: index, id, stick length, status icon, reason
tooltip. The current stick is highlighted in list and viewport.

Under Option C the panel shows the **next stick to load** prominently (id and
stick length in mm — this is what the human needs at the feeder) and offers
**Mark placed / Mark failed**, syncing status from the JSON sidecar written by
the ROS2 side.

*(Option A only)* a single state-dependent action button — **Pick next** →
**Place** → **Glue it, then Release** — mirroring the robot's state machine so
a wrong command is impossible, plus a red **Abort**.

### 10.4 `Preview`
- **Robot mirror** (QB3): a rig posed by the vendored FK. Scrub through the
  build order and watch the arm move to each placement — a printer-style
  preview of the whole job, before anything moves.
- **Reachability volume** overlay, toggleable.
- **Status colours**: grey pending · green buildable · red impossible ·
  blue placed · orange failed.

Keep the GPU overlay in its own module with a hard on/off switch — draw
handlers are the most common cause of addon crashes across Blender versions.

### 10.5 ✅ Localization — Japanese (added 2026-08-24)

User request: "is it possible to add a second language to the addon GUI? or
it would require a reload? ... can you add japanese as a second language."
Uses Blender's own addon-localization mechanism
(`bpy.app.translations.register(__name__, translations_dict)`), not
anything this addon invents — every panel label, button, checkbox, dropdown
option and tooltip is already plain text passed to `bpy.types.UILayout`/
`bpy.props.*` calls, and Blender's own UI-drawing code runs each one
through its own translation lookup automatically once a matching
`(msgctxt, english_text) -> translated_text` entry is registered.

**No reload needed to switch languages.** Confirmed directly against this
exact Blender build (5.2.0 LTS) by registering a probe dict, setting
`bpy.context.preferences.view.language = "ja_JP"`, and reading translated
strings back via `bpy.app.translations.pgettext()` — switching Preferences
> Interface > Translation > Language is instant and live; the addon's own
code only needs to load ONCE (same as any other code change) for its
translations to be registered at all.

New `i18n.py` module: a single `_JA` dict plus `register()`/`unregister()`,
wired into `__init__.py`'s own `_MODULES` tuple. **Scope, deliberately:**
panel titles, section/box headers, button labels, checkbox/dropdown labels,
property names, dropdown option names, and operator/property tooltips are
all translated — the vast majority of what a user actually sees scanning
the UI. **Not translated:** anything built from runtime data (`"%d
stick(s) impossible"`, `self.report(...)` messages, validation "reason"
strings) — Blender's translation lookup is an exact string match on the
msgid, and by the time a `%`-formatted string reaches `layout.label
(text=...)` the runtime value is already baked in, so there is no fixed
string to register a translation against; making these translatable too
would mean wrapping every format call in `ui/panels.py` with an explicit
`pgettext_iface()` on the TEMPLATE before substitution, a larger refactor
not attempted here. Also not translated: anything inside `core/`, which
stays pure Python with no `bpy` import at all (constraint B4) — adding
translation calls there would violate that boundary for a UI concern that
belongs in `ui/`/`ops/` anyway, so `core/validate.py`'s "reason" strings
stay English regardless of language. Everything else in the codebase —
comments, docstrings, identifiers — stays English, the project's own
primary language, per the user's own instruction. Only `ja_JP` exists
today; the dict shape supports adding more locales as additional top-level
keys without touching anything else.

New `TestTranslations` (`test_blender_integration.py`, needs the real,
registered addon — not a copy of the dict) confirms a panel title, a
property name, and an operator label all translate correctly with
`language = "ja_JP"`, that nothing translates with English, and spot-
checks several real operator labels against the dict so a rename without a
matching translation update is caught. A drive-by fix along the way: the
Reference panel's `"98%% reachable..."` label was passed straight to
`layout.label(text=...)` with no `%`-substitution ever applied to it, so
the UI was literally showing a doubled `%%` — found while surveying every
UI string for translation; corrected to a single `%`.

### 10.6 ✅ The build-volume box is movable, and it and grounding always agree (added 2026-08-24)

User request: "There is a box showing the build volume. I want to be able
to move that box from the addon, and then make the bottom of that box the
build plate reference height... I would want a button that 'drops' a mesh
having its lower vertex touch that bottom face or the build plate."

**Found while implementing this:** `ui/overlay.py`'s build-volume box and
`ops/design.py`'s own grounding math (`ground_height_m`, threaded into
`core/sticks.py`/`core/order.py` since §10.5's predecessor entry) were two
INDEPENDENT numbers that only happened to agree for so_arm_100. The box
always drew at the robot profile's own fixed `build_volume_min_m/max_m`;
`props.build_plate_height_mm` (default 0.0, absolute Z) never moved it at
all. For kr10_r900_2 specifically this was a real, silent inconsistency:
the box's own Z was corrected to `[-20, 280]` mm (its mounting-pedestal
floor, §4a's own entry) but the DEFAULT grounding computation was never
updated to match, so a design built at the robot's own coordinate origin
(Z=0, matching every other robot's convention) would silently be treated
as floating 20mm above the plate it looks like it is standing on.

**Fixed by reinterpreting `build_plate_height_mm`'s own meaning**: an
OFFSET above the SELECTED robot's own confirmed build-volume floor, not an
absolute Z. New `ops.design.effective_ground_height_m(profile, props)` =
`profile.build_volume_min_m[2] + build_plate_height_mm/1000` (falling back
to the raw offset for a robot with no confirmed build volume at all, since
there is no floor to offset from). For so_arm_100 (`build_volume_min_m[2]
== 0.0`) this is numerically identical to the old absolute-Z behaviour,
unchanged; for kr10_r900_2 the DEFAULT grounding height is now correctly
-20mm instead of 0mm, closing the inconsistency above with no button press
needed (0.0mm offset is already physically correct for every robot by
construction — properties.py's usual "reset to this robot's own defaults"
workaround for Blender's lack of per-robot property defaults is not needed
here). `ops/design.py::extract_with_autoflip()` and
`ops/order.py::build_solver()` both resolve `ground_height_m` through this
helper now instead of reading `build_plate_height_mm` directly.

**"Move that box"**: `ui/overlay.py`'s `_volume_box_points()` gained a
`plate_offset_m` parameter (`props.build_plate_height_mm` in metres) that
shifts the box's own Z by the same amount `effective_ground_height_m()`
adds — so editing that ONE field (Design panel, Mesh Expansion box) both
moves the visible box AND sets where the design must ground, by
construction, rather than two things that could disagree. (X/Y stay fixed
at the profile's own position; only Z is meaningful here, since nothing
downstream reads an X/Y build-volume offset — moving the box sideways
would be purely cosmetic with no functional effect on validation, so it
was deliberately left out of scope rather than half-implemented.)

**The drop button**: new `so100.drop_to_build_plate` operator (Design
panel, right under the Build Plate Height field) moves the DESIGN MESH
OBJECT (never its mesh data — reversible with a plain undo, like any other
object move) so its lowest vertex touches `effective_ground_height_m()`
exactly. Computes the shift as a single delta along the ROBOT's own Z axis
(`core.transform.robot_to_blender` applied to two points and differenced,
not assumed to be a pure world-Z move — correct even if the base empty is
itself rotated relative to world space), converts every design-mesh vertex
into the robot's own frame the same way extraction already does
(`core.transform.blender_to_robot_batch`), and reports how far it moved.
This directly replaces the "move the cube down ~6.4mm by hand" workaround
from the original `~/KUKABlenderTest.blend` diagnosis (§4a, "the design
cube's lowest vertex sits at Z=6.4mm relative to KR10_Base") with a single
button press.

New `TestEffectiveGroundHeight` and `TestDropToBuildPlate`
(`test_blender_integration.py`) cover the helper directly, confirm the
overlay box and `effective_ground_height_m()` always report the exact same
number for a given `build_plate_height_mm`, and exercise the drop operator
end to end (already-grounded is a no-op, a floating mesh gets grounded, a
mesh below the plate gets raised, a raised kr10_r900_2 plate is honoured,
and only the object moves, never mesh data). Two existing kr10_r900_2
fixtures (`TestMultiRobot`'s export test and its standalone build-order
test) were built grounded at the OLD Z=0 assumption and needed updating to
the new -20mm default floor — not a regression, the intended effect of the
fix; 442/442 in both bare CPython and real Blender.

### 10.7 ✅ Clear Results also deletes the build mesh (added 2026-08-24)

User request: "when clicking on the trashcan icon next to the extract
sticks, can you make it so it also deletes the build mesh? since a new one
has to be generated again?" `ops.design.SO100_OT_clear_results` (the
Design panel's trashcan button, next to Extract Sticks) previously only
cleared `props.sticks` and the summary/dimension strings, leaving
`props.build_mesh` — a real object in the scene — behind, stale, with
nothing pointing at it once `props.sticks` was empty. Since
`rebuild_build_mesh()` always regenerates a fresh build mesh from scratch
on the next extraction (never edits one in place), there was never a
reason to keep the old one around. Now removes the object AND its mesh
datablock (`bpy.data.objects.remove()` / `bpy.data.meshes.remove()`, the
same pattern `ops/mirror.py`'s own cleanup already uses for its rig mesh)
and sets `props.build_mesh = None`, guarded by the same "does it still
exist" check used elsewhere (`rebuild_build_mesh()`'s own). New tests in
`test_blender_integration.py::TestBuildMesh` confirm the object and its
mesh data are both actually gone from `bpy.data` (not just unlinked from
the property), that a later extraction still regenerates a working build
mesh, and that clearing with no build mesh yet is a clean no-op; 445/445
in both bare CPython and real Blender.

### 10.8 ✅ Check By Eye steps the camera back after framing (added 2026-08-24)

User feedback: "can you make it so the camera appears a bit more far
away? It is difficult to grasp where in the mesh is that edge if the
camera is too close." `SO100_OT_select_stick_in_viewport`'s
`view3d.view_selected()` call frames the selected stick's edge edge-to-
edge with no margin — tight enough that the surrounding structure falls
outside the viewport, so there is nothing to judge the stick's position
against. After framing, the operator now multiplies the 3D viewport's own
`region_3d.view_distance` by `CHECK_BY_EYE_ZOOM_MARGIN` (2.0) — a relative
step-back, not an absolute distance, so it scales correctly whether the
design is a 100mm test cube or a metre-scale sculpture, and stays centred
on the stick rather than recentring on the whole design. Applies equally
to `SO100_OT_step_stick` (Sticks panel's arrow buttons), which already
calls this same operator internally.

⚠ **Not verified visually in this sandbox** (same GPU/windowed-mode
limitation as `ui/overlay.py`'s own docstring, and windowed-Blender
smoke-testing attempts earlier in this project's history) — but confirmed
working by the user directly, in their own real Blender: "The camera
looks fine now."

### 10.9 ✅ Check By Eye highlights the selected edge, and reverts on its own (added 2026-08-24)

User request: "can you highlight the selected edge in a different color?
And revert it when the user clicks something else in the viewer?" Edit
Mode's own native selection highlight already shows which edge is
selected, but the user wanted something more distinct.

New `ui/overlay.py::_selected_edge_points(props)`, drawn in `_draw()` on
top of the per-stick status colours in bright yellow
(`_SELECTED_EDGE_COLOR`, width `_SELECTED_EDGE_WIDTH` = 4.0, thicker than
the status lines' own 2.0). **The "revert on its own" half needed no new
machinery at all**: the function reads the build mesh's CURRENT edit-mode
selection live via `bmesh.from_edit_mesh()` on every call, and the draw
handler already re-fires on every viewport redraw — which Blender already
triggers after every click. So selecting a different edge by hand, or
clicking empty space to deselect entirely, is already reflected the very
next redraw with zero event handling, no modal operator, and no extra
scene property tracking "which edge Check By Eye chose" — exactly the
kind of design Sec 10.4's own warning about draw handlers argues for
(minimal state, nothing that can go stale). Returns `[]` (nothing drawn)
whenever the build mesh is not currently in Edit Mode, which
`bmesh.from_edit_mesh` would otherwise raise on — Check By Eye is what
puts it there in the first place, so exiting Edit Mode is itself already
"clicking something else" and clears the highlight for free.

Unlike the GPU drawing itself (module docstring's own note, unverifiable
under `--background`), `_selected_edge_points()` is a plain bmesh read
that works fine in background mode — new tests confirm it returns exactly
the edge Check By Eye selected, follows a DIFFERENT edge selected by hand
(simulating "the user clicks something else in the viewer", since this
sandbox cannot simulate an actual viewport click), clears on deselect, and
is empty outside Edit Mode; 449/449 in both bare CPython and real Blender.
The actual on-screen colour/thickness still could not be checked visually
here.

### 10.10 ✅ "Highlight Previous Sticks" checkbox (added 2026-08-24)

User request: "a checkbox before the check by eye button that makes all
the previous edges highlighted... when it isn't selected, the process
will be the same as it currently is."

New `properties.py`'s `highlight_previous_sticks` (`BoolProperty`, default
`False` — unchecked behaves exactly as §10.9, unchanged), a checkbox in
the Sticks panel right before the Check By Eye row. `ui/overlay.py`
refactored §10.9's own `_selected_edge_points()` to take a pre-computed
list of selected-edge indices (`_selected_edge_indices()`, new, still one
live `bmesh.from_edit_mesh` read) rather than querying the live bmesh
itself, so a new `_previous_edge_points()` can share that SAME query
instead of reading the bmesh a second time per redraw. "Previous" means
BUILD order (`item.order`) once one has been computed, falling back to
extraction order otherwise — the exact same fallback
`SO100_OT_step_stick` already uses (§10.4's own entry), so "previous"
means the same thing here as it does when stepping through the list.
Drawn in a muted gold (`_PREVIOUS_EDGE_COLOR`, same hue family as the
current edge's bright yellow but visibly dimmer and at width 3.0 vs. the
current edge's 4.0 — deliberately NOT reusing `_STATUS_COLOR`'s own
orange for `STATUS_FAILED`, to avoid reading as "these sticks failed"),
drawn before the current edge so the current one stays the most visually
prominent. Only activates for EXACTLY one selected edge — with zero or
several selected, "the current one" is ambiguous, so nothing extra draws
(ordinary selection highlighting still shows whatever IS selected either
way).

Like `_selected_edge_points()`, `_previous_edge_points()` is a plain
mesh/bmesh read with no GPU calls, so — unlike the actual on-screen
drawing — it is directly testable under `--background`. New tests cover
the extraction-order fallback, the build-order case (using a stick that
is neither first nor last, on the wireframe-cube fixture where the two
orders provably differ), the empty-for-the-first-stick edge case, and
that it goes empty with zero or multiple edges selected; 455/455 in both
bare CPython and real Blender. The actual on-screen appearance still
could not be checked visually here.

---

## 11. Implementation phases

### Phase A — Scaffolding & geometry *(no robot, no ROS)*
- [ ] Addon skeleton for Blender 5.2 (verify B8 first), preferences, panels.
- [ ] `core/transform.py` + unit tests via `blender --background`.
- [ ] `core/sticks.py`: edge extraction, joint allowance, both length modes,
      cut list.
- [ ] Per-stick state with **stable ids** (§9.2) and `.blend` persistence.
- **Done when:** a wireframe cube produces 12 sticks with correct metre
  coordinates in `base_link` and correct stick lengths, verified by hand.

### Phase B — Kinematics & validation — ✅ DONE 2026-07-28
- [x] Vendor `so_arm_100_kinematics` — **exists, tested, validated on real
      hardware**, now including `grasp.py` (v1.1.0 — the grasp-orientation
      transform, ROS2 doc §9.6). Follow the vendoring rules in that
      package's own `README.md` (copy the inner module dir + `VERSION`, not
      the ROS packaging files; never fork it).
- [x] Write `kinematics_version` into every exported build file and check it
      on import, per [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) Part A.
- [x] `core/validate.py`; per-stick verdicts with specific reasons. Calls
      `grasp.solve_stick_orientation()` directly rather than reimplementing
      it — see the Phase D note below on why that matters.
- [x] Viewport overlay: build volume, reachability, status colours.
- **Done when:** dragging a vertex outside the envelope turns that stick red
  with a specific reason, live, with no ROS running.

✅ **Decided permissive** (see §7's note below): trust the closed-form IK's
own success/failure, including for out-of-plane tilts — MoveIt re-validates
on load regardless, so an over-eager rejection here only costs a design
iteration for nothing.

### Phase C — Build order — ✅ DONE 2026-07-29
- [x] `core/order.py` per §6, including the §6.2 warnings.
- [x] Chunked execution so large meshes don't freeze the UI
      (`OrderSolver.step()` driven from a modal operator's timer).
- **Done when:** a multi-layer test mesh produces an order that is
  support-valid and fully buildable, and a deliberately-floating component is
  correctly rejected. ✅ `tests/test_order.py`'s `TestMultiLayerTower` and
  `TestFloatingComponent`; support-validity is re-checked structurally by
  replaying the order, not taken from the solver's own bookkeeping.

⚠ **Jaw clearance (C3) is a *soft* constraint.** Its numbers are estimates
(§8.2's "~20 mm across" / "out to ~60 mm"), not measurements — Phase 0 still
owes the real jaw envelope. So a clash steers the search but never refuses
to build: if no order avoids it, the stick is placed with a `jaw_clearance`
warning naming the offender and the gap. Only *reachability* hard-gates a
candidate, matching the permissive policy chosen for Phase B.

⚠ **"base" becomes structural here**, superseding Phase B's
reachability-driven auto-flip. With one end supported the structure wins
(flipping would leave the stick floating); with both supported the solver
picks whichever orientation validates, which reproduces auto-flip's benefit
with no conflict. Re-extracting clears the order, so **Compute Build Order
must be re-run after any re-extraction.**

### Phase D — Export & the build loop — 🟢 SOFTWARE DONE 2026-07-29, awaiting hardware
- [x] Build-file export + cut-list export (`io/build_file.py`, `ops/build.py`).
- [x] JSON status sidecar round-trip; resume a partially-built job.
- [x] `Build` panel with the next-stick-to-load display.
- **Done when:** a design is exported, built on real hardware, and reopening
  the `.blend` shows the correct placed/pending state.
  🟡 **The hardware run is still outstanding** — it cannot be automated here.
  Everything either side of it is covered: the file conforms to
  [`BRIDGE_PROTOCOL.md`](BRIDGE_PROTOCOL.md) Part A (round-tripped through a
  loader that enforces format, version and `order` density), and the
  resume path is tested by writing a sidecar as ROS2 would, wiping the
  in-`.blend` progress, syncing, and checking the right sticks come back
  `placed`.

✅ **Integration gap closed (2026-07-31).** ROS2 re-validates the build file
with the shared kinematics module — the transform that turns `base`/`tip`
into the `(tool_elevation_rad, stick_roll_rad)` pair `ik()` consumes used to
live only in the addon (`core/validate.py`), with ROS2's own Phase 1
checklist marking it "not written yet". That derivation is where three
separate sign/branch bugs were found during Phase B, so an independent
reimplementation on the ROS2 side would have been a realistic way for the
two sides to disagree about where a stick goes — precisely the failure the
shared-module design exists to prevent. It is now promoted into
`so_arm_100_kinematics.grasp` (version 1.1.0), re-vendored into
`so100_builder/kinematics/grasp.py`, and `core/validate.py` calls it rather
than reimplementing it — see the Blender addon workspace's own
`so100_builder/README.md`, "The grasp-orientation transform" section, for
the full derivation and test coverage (that section lives in the addon
repo, not this one — `so_arm_100_kinematics/README.md` here only summarizes
it). **The ROS2 workspace's own copy has been updated to match**
(`so_arm_100_kinematics` 1.1.0, 30/30 tests passing there too) — this is no
longer outstanding.

### Phase E — Preview & polish — ✅ DONE 2026-07-31 (telemetry N/A, Option C has none)
- [x] Robot mirror rig driven by the shared FK; scrub through the build order.
      `core/mirror.py` (pure, testable): ``joint_frames()`` derives all 6
      joint-chain points (base_link origin through the true TCP) entirely
      from the vendored, tested ``chain.fk()`` -- called on successively
      longer joint-angle prefixes, correcting for the one place ``fk()``'s
      own postprocessing (the `EE_OFFSET` add) is only valid for the full
      chain. Not a second FK implementation; see the module's own docstring.
      `ops/mirror.py` reads a stick's base/tip back off the build mesh
      (respecting any `flip`), solves it with
      `kinematics.grasp.solve_stick_placement()` -- the exact function
      `PlaceStick` uses on the ROS2 side -- and poses a preview-only mesh
      object (`SO100_Mirror`, never selectable, never rendered) with the
      resulting 6 points. `SO100_PT_preview` (Sec 10.4) adds the toggle and
      prev/next scrub controls. No telemetry, no live robot link -- Option C
      (Sec 2) has none by design, and none was wanted here.
- [x] *(Option A/telemetry chosen)* — N/A, not applicable under Option C.

**Done when:** tested three ways, per Sec 12's own layering — `core/mirror.py`
against the vendored `fk()` directly (a segment-length invariance property
across 5 named poses + 200 random joint-space samples, so a bug in the
`EE_OFFSET` correction would show up as a length that moves with the pose);
`ops/mirror.py`'s operators executed for real inside Blender (toggle,
wrap-around stepping, object cleanup on unregister, and a geometric check
that the rig's TCP point lands within 1 mm of the stick's own
`grasp_target()` -- not just "drew something"); and the new `Preview`
panel's `draw()` itself smoke-tested in a real **windowed** Blender instance
(background mode cannot fire panel draw callbacks at all), since a typo in a
`layout.prop()`/`operator()` call is invisible to every other test layer.

---

## 12. Testing

| What | How |
|---|---|
| Transform & stick maths | `blender --background --python tests/run.py`, plain `unittest`. No UI, no robot. |
| Kinematics parity | Same test data run against the ROS2 copy of the module; results must match exactly. |
| Build order | Synthetic meshes with known-correct orders; a floating component; a loop closure; a high-valence vertex. |
| Full loop | Real robot, a 3–5 stick design, watched every step. |

Explicitly test: Blender closed and reopened mid-build; a stick marked
`placed` that was actually knocked over (needs a re-sync path); a mesh edited
*after* the order was computed (ids must survive, or the user must be warned
that the order is stale).

---

## 13. Open questions

N1–N4 are **decided** (ROS2 doc §1.2). **Nothing blocks addon work.**

N5 (is there a base plate with pre-drilled sockets?) and N6 (glue set time)
remain open — see [`ROS2_IMPLEMENTATION_PLAN.md`](ROS2_IMPLEMENTATION_PLAN.md)
§13. Neither affects the addon: N5 changes the first-layer strategy on the
robot side, N6 is a hardware concern.
