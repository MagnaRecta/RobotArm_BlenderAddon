# Shared status & open-issue log

**Part of the shared docs triad** (alongside `BLENDER_ADDON_PLAN.md`,
`ROS2_IMPLEMENTATION_PLAN.md`, `BRIDGE_PROTOCOL.md`): this file must stay
**byte-identical** between the two workspaces —

- Blender addon: `RobotArm_UbuntuAddon/docs/STATUS.md`
- ROS2 workspace: `ros2_ws/src/SO-100-arm/docs/STATUS.md`

— reconciled by hand, the same way the other three already are. There is no
automated sync between the two repos on purpose (see `BLENDER_ADDON_PLAN.md`
§2: Option C, the slicer/printer split, is a deliberate scope boundary, not
an oversight).

**What this file is for, and what it is not:**

- It is the first thing either side's agent should read at the start of a
  session, to get "what's the current state, what's open" without reading
  the entire phase plan in both design docs.
- It is **not** a duplicate of the design docs' own inline corrections.
  `BLENDER_ADDON_PLAN.md` and `ROS2_IMPLEMENTATION_PLAN.md` already record
  found-and-fixed findings inline, in the context where they matter (e.g.
  §9.4's out-of-plane tilt correction) — that is the right home for anything
  tied to a specific derivation or spec section. This file is for **current,
  cross-cutting, or in-flight** state: what's open right now, which side
  owns it, and a pointer into the design docs for the detail.
- **Append, don't rewrite history.** Move an entry to "Resolved" with a date
  and a one-line pointer to where the real writeup lives, rather than
  deleting it.

---

## Current state (as of 2026-08-03)

**Shared kinematics — `so_arm_100_kinematics` v1.3.0**, now identical (incl.
`jaw_clearance.py`) across all three copies: the ROS2 package itself, the
Blender workspace's `so_arm_100_kinematics/` staging copy, and
`so100_builder/kinematics/` — the copy the addon's own Blender interpreter
actually imports. Includes the grasp-orientation transform (`grasp.py`,
since 1.1.0) — see `ROS2_IMPLEMENTATION_PLAN.md` §9.6 — the swept-jaw
clearance pre-filter (`jaw_clearance.py`, since 1.2.0) — see §8.2
consequence 2 — and, as of 1.3.0, `grasp_offset_for_length()` (D13, §4) for
short-stick grasp adaptation. Re-vendored per the package README's own
rules (copy verbatim, re-point test imports); all 46/25/6/others vendored
and ROS2-side tests pass in a plain `env -i` Python (no numpy, simulating
Blender's bundled interpreter) — see
`so100_builder/tests/test_vendor_sync.py`. Practical effect: a build file
exported from Blender now correctly stamps `kinematics_version: "1.3.0"`
(`so100_builder/ops/build.py` reads it dynamically from the vendored copy)
— matters because ROS2's `build_file.py` loader (Phase 5) hard-refuses to
execute on any `kinematics_version` mismatch.

**Blender addon (`so100_builder`)** — Phases A–E software-complete, plus a
multi-robot registry (`core/robots.py`, `BLENDER_ADDON_PLAN.md` §4a) with
both `so_arm_100` and `kr10_r900_2` genuinely vendored and wired into
validation/the mirror rig/export as of 2026-08-22. Phase D's hardware run
(build a real design on the real robot, confirm status round-trips) is the
one remaining so_arm_100 "done when" item and cannot be exercised from the
Blender side alone. See `BLENDER_ADDON_PLAN.md` §11.

**ROS2 workspace (`SO-100-arm`)** — Phase 0 (geometry foundation: measure
`mount_platform_joint` origin, jaw envelope, confirm the 51 mm grip offset
with a ruler) is **blocking and physical** — no amount of agent work
substitutes for someone measuring the real hardware. Phase 1 (kinematics &
reachability map) is partly done: FK/IK, the grasp-orientation transform,
and the jaw-clearance test are built, tested (42/42), and (FK/IK only)
hardware-validated; a collision-aware reachability map generator is not
written yet, and the jaw-clearance check has no caller wired up yet (it's a
primitive Phase 4's task server will call per-placement). **Phases 2
(refactor) and 3 (parametric place) are ✅ DONE, hardware-confirmed
2026-08-01** (`pick_and_place_node.py` split into
`motion.py`/`scene.py`/`sequences.py`; new `stick_spec.py` drives the
`place` step from a real stick's base/tip via `so_arm_100_kinematics`, with
the base/tip flip-retry now wired in) — verified by unit tests (48/48), a
MoveIt-only dry run, and on the real robot: the tuned pose reproduced via
`stick_spec`, then `s_004` from the user's own Blender-exported build file
(`SO100BlenderTest.build.build.build.json`) placed at a computed, not
hand-tuned, location — the first time this system has done that. **Phase 4 (task
server) is ✅ DONE, hardware-confirmed 2026-08-02** — new
`stick_task_msgs` interface package, `stick_task_server_node.py`'s
state machine, all three actions, a full `PickStick`→`PlaceStick`→
`ReleaseStick` cycle run end-to-end on real hardware via plain
`ros2 action send_goal` (no Blender, no interactive prompts) with `s_004`.
Getting there took several rounds of real hardware debugging the same
day — the task-server wedge (rclpy bug, two-`Node` fix), a params-file
node-name-scoping bug that left `stick_task_server`/`tune_grasp` silently
running on stale fallback config, a misplaced feeder-stick collision
object, an arm/gripper interface mixup in `ReleaseStick`, and two rounds of
`grasp_verification.gap_threshold` re-tuning once real hardware variance
turned out wider than the first calibration pass suggested — see Resolved
below and `ROS2_IMPLEMENTATION_PLAN.md` §3 findings #12-15 and §4 D8-D12
for the full trail. D13 (grasp height not adapting to stick length) is now
implemented but not yet hardware-confirmed (see Open items), and one
narrower, still-unexplained dry-run-only symptom from the wedge fix
remains open too (see Open items).
**Phase 5 (build execution) is built and dry-run-verified; several real
hardware attempts made 2026-08-03/04, none completed a full clean build
yet** — new `Reset` action (the only way back from `ERROR` to `IDLE`,
discovered during planning to be a hard prerequisite, not optional, for
this phase's own "not fatal to the build" requirement), a `build_file.py`
loader compatible with the Blender addon's own build-file/status-sidecar
format (17 unit tests), and a new `build_runner` node that drives
`stick_task_server` through an entire build stick by stick with the human
gates, persists status after each stick, and resumes correctly after a
restart. The real hardware attempts surfaced two genuine bugs (a
physically-placed-but-unregistered stick after a release failure, and
planning-scene debris surviving a `build_runner` restart) — both diagnosed
and fixed, not yet re-verified on hardware; see Open/Resolved below and
`ROS2_IMPLEMENTATION_PLAN.md` §3 findings #17/#18. `ValidatePlacements` and
Phase 6 (rotating table) have not been started. See
`ROS2_IMPLEMENTATION_PLAN.md` §11 for the full phase plan and each phase's
"done when."

---

## Open items

Format: `[owning side]` short description — pointer.

- `[shared]` **`kr10_r900_2_kinematics` has DRIFTED again (noticed
  2026-09-09), fourth time.** `test_vendor_sync.py` is failing on three
  modules: `constants.py` (~71 changed lines, upstream 2026-09-08),
  `grasp.py` (~65 added lines, upstream 2026-09-07) and `__init__.py` (2 new
  exports). `chain.py` and `jaw_clearance.py` are still byte-identical.
  Noticed while running the suite for an unrelated change — **not** caused by
  it, and deliberately left alone: re-vendoring is a decision about which
  upstream state to freeze, and the diff includes new public functions, not
  just comments. Re-vendor from
  `~/ros2_ws/src/kuka_control/kr10_r900_2_kinematics/kr10_r900_2_kinematics/`
  when the upstream side is ready, reading the diff first as with the three
  previous cycles below.

- `[shared]` **Multi-robot support kicked off 2026-08-21.** The addon and the
  build-file protocol are being generalized to target more than one arm: the
  SO-100 (this repo, testing/dev rig) and a **KUKA KR10 R900-2** (`kuka_control`
  repo, `~/ros2_ws/src/kuka_control` — same colcon workspace as this repo, a
  separate git repo), the production rig. `BRIDGE_PROTOCOL.md` §A.1/§A.1.1 now
  define a `robot` id field on the build file and a robot-id registry; the
  KUKA side owns its own `kr10_r900_2_kinematics` package (mirroring
  `so_arm_100_kinematics`'s structure and vendoring rules) and its own
  `docs/KUKA_IMPLEMENTATION_PLAN.md`. Two known differences from the SO-100
  design worth remembering when reading that repo's docs: the KR10 is 6-DOF
  (no free parameter to sweep the way SO-100's 5-DOF chain has — a KDL/
  numerical or Pieper-style closed-form IK path, not SO-100's exact solver),
  and its gripper is pneumatic/binary (open/closed digital output, no
  continuous position readback) so SO-100's `gap_threshold` grasp-verification
  heuristic does not port as-is. `kuka_control` already has a fake-hardware
  simulation (`kuka_kr10_interface/launch/simulate.launch.py`) and a
  `stick_holder` collision mesh staged in `kuka_control_bringup`, suggesting
  this integration was anticipated there before this session. **The addon
  side of this (registry + interface contract + robot_id plumbing) is now
  done, `kr10_r900_2`'s build volume is confirmed, AND `kr10_r900_2_kinematics`
  itself landed 2026-08-22 (sooner than expected) and is now genuinely
  vendored and wired into validation/the mirror rig/export — see Resolved
  below for all three.** Still open: `core/sticks.py`'s mesh-expansion
  geometry (joint gaps, length limits) and `core/order.py`'s build-order
  solver both stay hardcoded to so_arm_100's own numbers regardless of
  `robot_id` — real, load-bearing work for an actually-usable KUKA design
  workflow, not yet started; see `BLENDER_ADDON_PLAN.md` §4a's own closing
  note.
- `[ROS2, physical]` **Full `PickStick`→`PlaceStick`→`ReleaseStick` cycle
  confirmed working end-to-end on hardware 2026-08-02** (`s_004` picked,
  placed at its computed location, released — all three actions
  `SUCCEEDED` via plain `ros2 action send_goal`, no Blender, no interactive
  prompts). Phase 4's own hardware milestone.
- `[ROS2, physical]` **D13 implemented 2026-08-11, not yet confirmed on
  hardware.** The grasp offset now adapts to short sticks (down to a new
  50mm minimum) instead of always being the fixed 51mm constant — see
  Resolved below and `ROS2_IMPLEMENTATION_PLAN.md` §4 D13 for the full
  design. `MIN_GRASP_OFFSET_M` (the floor-clearance floor used for very
  short sticks) is an unmeasured placeholder, same status as
  `GRASP_OFFSET_M`/`JAW_RADIUS_M` — the next real short-stick `PickStick`
  should be watched via RViz's planned-path preview before trusting it.
- `[ROS2, physical]` One remaining piece of the task-server wedge (see
  Resolved below for the part that's fixed): in the no-controller dry-run
  harness, a `PickStick`'s `home position` planning step itself hung
  indefinitely after the two-node fix (server stayed responsive to other
  goals throughout — this is a different, more localized symptom than the
  fixed total-wedge). Not yet understood; needs a real hardware retest —
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 4.
- `[ROS2]` Reachability map generator is IK-only (`envelope.sweep_envelope()`);
  does not yet model self-collision, the mount platform, the table, or
  already-placed sticks — `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 1.
- `[ROS2, physical]` `GRASP_OFFSET_M = 0.051` is derived, not measured.
  Needs a ruler check against the real feeder — `ROS2_IMPLEMENTATION_PLAN.md`
  §11 Phase 0.
- `[ROS2, physical]` `stick_roll_rad = 0 ⇒ Wrist_Roll = π/2` sign convention
  is untested on hardware — every hardware run so far used `stick_roll = 0`.
  Confirm with a non-zero roll before trusting it for a real build (the
  stock is square, so a wrong sign is visible) — `ROS2_IMPLEMENTATION_PLAN.md`
  §11 Phase 1 caveats.
- `[Blender + ROS2]` Phase D's hardware run itself: export a real design,
  build it, confirm the addon's Build panel reflects the real outcome after
  reopening. Needs both sides working together, not schedulable from either
  workspace alone.
- `[ROS2 + Blender]` `jaw_clearance.check_jaw_clearance()` is built and
  tested (both sides, incl. the now-vendored Blender copy) but has no
  caller on **either** side yet — nothing in this workspace invokes it
  during a real placement, and the addon's `core/validate.py` doesn't call
  it either. Needs wiring into Phase 4's task server (and into
  `ValidatePlacements`, §10.1) on this side once it exists; the Blender
  side's own equivalent wiring is that workspace's call.
- `[ROS2, physical]` **Phase 5's own hardware milestone — several real
  attempts made 2026-08-03/04, none completed a full clean build.** One run
  had a `ReleaseStick` failure right after a successful placement; skipping
  that stick left its collision box missing even though it was physically
  glued in place, and a later placement in the same run also failed. A
  second run had a clean pick, then a `PlaceStick` planning failure
  (`INVALID_MOTION_PLAN`) on a stick that should have been reachable, plus a
  restart not clearing a leftover collision object (only closing
  RViz/`move_group` did). A third run's `INVALID_MOTION_PLAN` was root-
  caused via an RViz screenshot: the resume loop had re-attempted a stick
  already `placed` from an earlier session, driving a second physical stick
  straight at that stick's own already-registered collision box. A fourth
  run (fresh sidecar deleted, clean start) got further — pick, place, and
  glue all succeeded for the first stick — but `ReleaseStick` then failed at
  its own `retreat` step with no other collision objects present besides
  that stick's own about-to-be-registered box. All four bugs diagnosed and
  fixed (see Resolved below, `ROS2_IMPLEMENTATION_PLAN.md` §3 findings
  #17/#18/#19/#20). Not yet re-verified against hardware. Separately, one
  `PlaceStick` run had a visually odd trajectory (dipped toward the floor
  before lifting into place) despite succeeding — not yet root-caused,
  plausibly just OMPL's normal joint-space path variance (finding #11's
  tradeoff); see `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 5's note.

## Resolved

- `[Blender]` **Build order now has real layers, so "far side first" finally
  runs, 2026-09-09** — user report: "when trying to generate a build order, I
  feel like the algorithm prioritizes the outer layer. This is a problem since
  the robot will not be able to reach the inner sticks if the outer layer is
  already built." Correct, and the cause was not the accessibility heuristic
  but the key above it. `_cost`'s primary key was the stick's raw `max_z`, and
  mesh expansion (§5.2) nudges every vertex by a fraction of a millimetre — so
  a physically flat course arrived as dozens of distinct floats that never
  compared equal (measured on the reporter's file: one course spread over ten
  heights spanning 1.1 mm, against a 34 mm gap between courses). The primary
  key never tied, so §6 C2's "far-from-robot first **within a layer**"
  tie-breaker never executed and within-course order was decided by expansion
  noise. New `core/order.py` `layer_boundaries()` groups tops into real layers
  (complete linkage, so a shallow ramp cannot chain into one giant layer);
  `Layer Height Tolerance` in Design ▸ Advanced, default 10 mm. **No new cost
  term** — C2 was already right, it just never got to run. Note the result is
  far → core → near, deliberately *not* the "centre outward" that was asked
  for: centre-out fixes the reported symptom but then forces the arm to reach
  over the finished core to place the far ring at the same height. Measured on
  the reporter's 280-stick lattice: the core of each course used to be placed
  last within its course, now lands mid-course with the whole near-side ring
  last in 6 of 7 courses (course 0 interleaves because C1 will not let a
  ceiling stick precede its uprights — support doing its job). Solve 5.0 s →
  3.9 s. 469/469 tests bar the vendor-drift failures below. **Not yet run on
  hardware** — the claim that this is the order the arm can actually execute
  is geometric reasoning plus the reporter's own observation, not a robot run.

- `[Blender]` **Operators leave Edit Mode by themselves, 2026-09-09** — user
  report: "when trying to change the order of one edge, I get the error
  'Cannot add vertices in edit mode'... it is a hassle having to change
  modes." Check By Eye deliberately leaves the build mesh in Edit Mode, and
  Move Earlier/Later regenerates that mesh — `mesh.from_pydata()` refuses
  while it is in Edit Mode. Reproduced against the reporter's own file and
  found to affect **four** operators (Move Build Step, Compute Build Order,
  Extract Sticks, Export Build File), plus a quieter second failure: an
  attribute layer's `.data` reads back *empty* rather than raising in Edit
  Mode, so `assign_stable_ids()` would have silently renumbered every edge.
  New `object_mode_for_mesh_writes()` context manager drops to Object Mode,
  does the work and restores the exact previous mode, re-applying the Check By
  Eye edge highlight (the mesh is replaced, so the selection cannot survive on
  its own) without re-framing the camera. Leaving Edit Mode is also what makes
  the read *correct*: Blender flushes the BMesh into the Mesh on the way out,
  so a mid-edit design extracts at its on-screen coordinates rather than the
  stale pre-edit ones. `check_design_ready`'s old refusal stays as a guard for
  direct API callers only. ⚠ Multi-object Edit Mode is restored for the active
  object only — same as pressing Tab twice.

- `[Blender]` **"Highlight Previous Sticks" checkbox, 2026-08-24** — user
  request: "a checkbox before the check by eye button that makes all the
  previous edges highlighted... when it isn't selected, the process will
  be the same as it currently is." New `highlight_previous_sticks`
  property (default off, unchanged behaviour when unchecked). "Previous"
  means build order once one exists, falling back to extraction order --
  the same fallback `SO100_OT_step_stick` already uses. Drawn in a muted
  gold, dimmer than the current edge's bright yellow, only when EXACTLY
  one edge is selected (ambiguous otherwise). `ui/overlay.py`'s own
  `_selected_edge_points()` (the prior entry below) was refactored to take
  a pre-computed selected-index list so the new `_previous_edge_points()`
  can share the same live bmesh read rather than querying it twice per
  redraw. Like that prior work, this is a plain mesh/bmesh read with no
  GPU calls, so directly testable under `--background` -- new tests cover
  both the extraction-order and build-order cases (the latter using a
  stick where the two orders provably differ), the first-stick edge case,
  and the zero/multiple-selected ambiguous cases; 455/455 in both bare
  CPython and real Blender. On-screen appearance not verified visually
  here.

- `[Blender]` **Check By Eye steps the camera back after framing,
  2026-08-24** — user feedback: "can you make it so the camera appears a
  bit more far away? It is difficult to grasp where in the mesh is that
  edge if the camera is too close." `view3d.view_selected()` frames the
  selected stick edge-to-edge with no margin; the operator now multiplies
  the viewport's own `region_3d.view_distance` by
  `CHECK_BY_EYE_ZOOM_MARGIN` (2.0, `ops/design.py`) right after framing --
  a relative step-back that scales with the design's own size and keeps
  the stick centred, rather than an absolute distance. Applies to
  `SO100_OT_step_stick` too, which already calls this same operator.
  **Not verified visually in this sandbox** (same GPU/windowed-mode
  limitation as the overlay's own docstring) -- confirmed working by the
  user directly: "The camera looks fine now."

- `[Blender]` **Check By Eye highlights the selected edge, and reverts on
  its own, 2026-08-24** -- user request: "can you highlight the selected
  edge in a different color? And revert it when the user clicks something
  else in the viewer?" New `ui/overlay.py::_selected_edge_points()`, drawn
  in bright yellow on top of the per-stick status colours. The "revert on
  its own" half needed no new machinery: the function reads the build
  mesh's CURRENT edit-mode selection live via `bmesh.from_edit_mesh()` on
  every draw call, and the draw handler already re-fires on every redraw
  (which Blender already triggers after every click) -- so selecting a
  different edge by hand, or deselecting entirely, already shows up the
  very next redraw with zero event handling, no modal operator, and no
  extra scene property. Unlike the GPU drawing itself, this read is plain
  bmesh access that works in background mode -- new tests confirm it
  tracks the selected edge, follows a hand-selected different one,
  clears on deselect, and is empty outside Edit Mode; 449/449 in both
  bare CPython and real Blender. The actual on-screen colour still
  couldn't be checked visually in this sandbox.

- `[Blender]` **Clear Results (the trashcan button) now also deletes the
  build mesh, 2026-08-24** — user: "when clicking on the trashcan icon
  next to the extract sticks, can you make it so it also deletes the
  build mesh? since a new one has to be generated again?" Previously left
  the old build mesh object sitting in the scene, stale, with nothing
  pointing at it once the sticks list was cleared -- `rebuild_build_mesh()`
  always regenerates a fresh one on the next extraction anyway, so there
  was never a reason to keep it. `SO100_OT_clear_results` now removes both
  the object and its mesh datablock (`bpy.data.objects.remove()`/
  `bpy.data.meshes.remove()`, the same pattern `ops/mirror.py`'s own rig
  cleanup already used) and clears `props.build_mesh`. New tests confirm
  both are actually gone from `bpy.data`, a later extraction still
  regenerates cleanly, and clearing with nothing to clear is a no-op;
  445/445 in both bare CPython and real Blender.

- `[shared]` **`kr10_r900_2_kinematics` re-vendored a third time,
  2026-08-24** (user: "Please revendor the kinematics" -- resolving the
  drift flagged as Open the same day). A real, hardware-confirmed
  calibration correction, not just a comment/rename: `GRASP_OFFSET_M`
  moved from 0.0188 to 0.021072 (a ~2.3mm correction) -- "real-hardware
  finding -- every PLACED stick landed ~2mm off, traced to base_xyz_m's
  assumed physical stop being ~2mm short of the real one." Only
  `constants.py` differed from the last re-vendor; re-copied it verbatim
  and confirmed no test in this addon relied on the old value (nothing
  hardcodes `GRASP_OFFSET_M`'s own number, only reads it off the vendored
  module) -- `test_vendor_sync.py::TestVendoredCopyIsVerbatim` passes
  again with no other code changes needed; 442/442 in both bare CPython
  and real Blender.

- `[Blender]` **The build-volume box is movable, and now always agrees
  with grounding, 2026-08-24** — user: "I want to be able to move that
  box from the addon, and then make the bottom of that box the build
  plate reference height... a button that 'drops' a mesh having its lower
  vertex touch that bottom face or the build plate." Found while
  implementing: the viewport box (`ui/overlay.py`) and the actual
  grounding computation (`ground_height_m`) were two INDEPENDENT numbers
  that only happened to agree for so_arm_100 -- for kr10_r900_2, the box's
  own Z was corrected to account for its 20mm mounting pedestal (an
  earlier 2026-08-23 entry) but the DEFAULT grounding height was never
  updated to match, so a design built at Z=0 (every robot's usual
  convention) silently read as floating 20mm above the plate it looks
  like it's standing on. Fixed by reinterpreting `build_plate_height_mm`
  as an OFFSET above the selected robot's own confirmed build-volume
  floor rather than an absolute Z (`ops.design.effective_ground_height_m()`,
  new) -- identical to the old behaviour for so_arm_100 (whose own floor
  is already Z=0), but now correctly -20mm by default for kr10_r900_2,
  with no button press needed since 0.0mm offset is already right for
  every robot by construction. The SAME field now also moves the viewport
  box's own Z (`_volume_box_points()` gained `plate_offset_m`) -- editing
  it moves the box and sets where the design must ground in the same
  action, closing the inconsistency for good. New `so100.
  drop_to_build_plate` operator (Design panel button) moves the design
  mesh OBJECT (never mesh data, undo-safe) so its lowest vertex touches
  that height exactly -- the same fix an earlier session applied by hand
  ("move the cube down ~6.4mm") to `~/KUKABlenderTest.blend`, now one
  button press. Two existing kr10_r900_2 test fixtures needed updating to
  the new -20mm default floor (not a regression, the intended effect).
  New `TestEffectiveGroundHeight`/`TestDropToBuildPlate` in
  `test_blender_integration.py`; 442/442 in both bare CPython and real
  Blender (aside from the unrelated, still-drifting vendor-sync entry
  above).

- `[Blender]` **Japanese added as a second UI language, 2026-08-24** —
  user: "is it possible to add a second language to the addon GUI? ...
  can you add japanese." Uses Blender's own `bpy.app.translations`
  mechanism (`i18n.py`, a `{locale: {(msgctxt, english): translated}}`
  dict wired into `__init__.py`'s module list) -- confirmed working
  directly against this exact Blender build (5.2.0 LTS) by registering a
  probe dict and reading translated strings back via `pgettext()` with
  `preferences.view.language = "ja_JP"`. **No addon reload needed to
  switch languages** -- only the initial code load registers the dict;
  after that, switching Preferences > Interface > Translation > Language
  is instant and live. Scope: panel titles, section headers, button/
  checkbox/dropdown labels, property names, and operator/property
  tooltips -- i.e. static UI text, not anything built from runtime data
  (`self.report()` messages, `"%d stick(s)..."`-style labels, `core/`'s
  own validation "reason" strings) -- see `i18n.py`'s own docstring and
  `BLENDER_ADDON_PLAN.md` §10.5 for the exact scoping reasoning. New
  `TestTranslations` in `test_blender_integration.py` verifies the real,
  registered addon actually translates a panel title/property name/
  operator label, that English stays English, and spot-checks several
  operator labels against the dict. Drive-by fix found while surveying
  every UI string: the Reference panel's `"98%% reachable..."` label had
  no `%`-substitution ever applied to it, so it was literally showing a
  doubled `%%` on screen -- corrected to a single `%`. 430/430 in both
  bare CPython and real Blender.

- `[shared]` **`kr10_r900_2_kinematics` re-vendored a second time,
  2026-08-23** (user: "re-vendor the new kinematics please" -- resolving
  the drift flagged as Open earlier the same day). `kuka_control` gained,
  since the last re-vendor: a real, hardware-confirmed correction to
  joint6's own limit -- "Was +/-350deg (a generic assumption). Confirmed
  against real hardware 2026-08-25: the physical A6 axis does not move past
  +/-180deg" -- in `constants.py` (load-bearing: this directly narrows what
  `chain.ik()`/reachability consider buildable for kr10_r900_2, a real
  behaviour change picked up by this re-vendor, not just a comment/rename);
  and a new `translate_holding_wrist()` in `chain.py` (a singularity-
  avoidance fallback for the ROS2 motion controller's own Cartesian moves,
  re-exported from `__init__.py`) -- purely additive, not called anywhere
  in this addon. Re-copied `kinematics/kr10_r900_2/*.py` verbatim;
  `test_vendor_sync.py::TestVendoredCopyIsVerbatim` passes again with no
  other code changes needed (no test relied on the old +/-350deg limit);
  425/425 in both bare CPython and real Blender.

- `[Blender]` **Manual build-order reordering, 2026-08-23** — user request:
  "After computing a build order, I want to be able to move the steps
  before of after its position." New `OrderSolver.replay(sequence_ids)`
  (`core/order.py`) places sticks in exactly the given order rather than
  searching for one, keeping each stick's existing orientation and
  re-checking only what genuinely depends on order (C1 support, C3 jaw
  clearance) -- a position that breaks either is still placed, flagged with
  a reason, never silently dropped or silently corrected by a hidden second
  solve. New `SO100_OT_move_build_step` operator (Sticks panel, "Move
  Earlier"/"Move Later") does an ADJACENT swap of the active stick's
  `order` -- matching the request's own examples exactly -- then replays
  immediately so the effect is visible right away. `export_build_file` now
  replays the CURRENT `props.sticks[i].order` instead of always re-solving
  from scratch, so a manual reorder actually reaches the exported file
  rather than being silently discarded on export (a real gap found while
  designing this: export previously always called `solve()` fresh).
  `ordered_ids()` moved from `ops/build.py` to `ops/design.py` to avoid a
  circular import. New `TestReplay` (`test_order.py`) plus end-to-end
  operator tests in `test_blender_integration.py`; 425/425 in both bare
  CPython and real Blender (aside from the unrelated, newly-drifted
  vendor-sync failures below).

- `[Blender]` **`kr10_r900_2` robot base box + build volume Z shift,
  2026-08-23** — user request: a 320×320×20mm box "below the robot
  (origin)" shown only for kr10_r900_2, and lowering the build volume box
  "to account the height of the base box of the robot above."
  `core/robots.py`'s `RobotProfile` gained `base_box_min_m`/`base_box_max_m`
  (`None`/"not applicable" by default, same convention as
  `build_volume_min_m`/`max_m`) -- `kr10_r900_2`'s own box sits directly
  below its coordinate origin (`Z ∈ [-20, 0]` mm), representing its
  physical mounting pedestal. Since the robot's own origin is 20mm above
  the actual table surface, `kr10_r900_2`'s `build_volume_min_m`/`max_m` Z
  shifted from `[0, 300]` to `[-20, 280]` mm to match -- both boxes' floors
  now sit at the same table-level Z. Viewport-only (drawn in
  `ui/overlay.py`, listed in `ui/panels.py`'s Reference panel): never
  exported to the build file, since ROS2 already knows its own robot's
  footprint from its URDF. `so_arm_100` has no base box. Updated every
  hardcoded `[0.3, -0.15, 0.0]`-style test expectation across
  `test_robots.py`/`test_blender_integration.py` to the new `-0.02` floor;
  413/413 at the time (before the reordering feature above added more).

- `[shared]` **`kr10_r900_2_kinematics` re-vendored from `kuka_control`,
  2026-08-23** (user: "Re-vendor so both repos are completely in sync. The
  latest kinematics file should be the one in the ROS2 repo" — resolving
  the drift flagged earlier the same day). Read the actual upstream diff
  before touching anything, since the vendor-sync test's own message says
  not to patch around drift blindly: `grasp.py` gained two new generators,
  `iter_stick_placements` and `iter_stick_placements_any_roll`, added
  2026-08-23 upstream after a real hardware finding ("a kinematically valid
  branch put `link_4` into contact with `link_6`" — `chain.ik()`'s branch
  choice has no notion of the robot's own links colliding with each other,
  only MoveIt's collision-aware planner catches that, so a caller doing
  joint-space planning needs every reachable branch to retry against, not
  just the first). `solve_stick_placement`/`solve_stick_placement_any_roll`
  are UNCHANGED in contract -- the latter is now a thin wrapper returning
  the generator's first candidate, same signature, same return shape, same
  `Unreachable` on total failure (only the exception's own message text
  lost its "last failure: ..." clause, asserted nowhere). `__init__.py`
  just exports the two new names alongside the existing ones. Net effect:
  a strict superset, not a breaking rename -- `core/validate.py`,
  `ops/mirror.py`, and `core/robots.py` needed no changes; re-copied
  `kinematics/kr10_r900_2/*.py` verbatim from
  `~/ros2_ws/src/kuka_control/kr10_r900_2_kinematics/kr10_r900_2_kinematics/`
  and confirmed `test_vendor_sync.py::TestVendoredCopyIsVerbatim` passes
  again; 409/409 in both bare CPython and real Blender.

- `[Blender]` **"Check By Eye" stepping now follows build order, not
  extraction/edge order, 2026-08-23** — user report: "the arrow goes to the
  next stick maybe by edge number? It does not follow the newly computed
  order." `props.sticks` deliberately always stays in EXTRACTION order
  (`select_stick_in_viewport` needs "build-mesh edge i is sticks[i]"), but
  `SO100_OT_step_stick` was doing plain `active_stick_index +/- 1` on that
  same index -- only the build sequence by coincidence. Now inverts the
  Sticks list's own display permutation (`core.state.build_order_permutation`)
  to step by BUILD position instead, falling back to plain index stepping
  when no order has been computed yet. Moved `build_order_permutation()`
  from `ui/panels.py` (bpy-only) to `core/state.py` (pure Python) to make
  this callable from `ops/design.py` without a circular import, which also
  fixed a pre-existing testability gap -- its tests now run in bare
  CPython too, in new `tests/test_state.py`. New
  `test_step_stick_follows_build_order_once_one_exists` exercises the real
  operator against the wireframe-cube fixture end to end; 409/409 in both
  bare CPython and real Blender (aside from the unrelated vendor-drift
  failure below).

- `[Blender]` **A `Require Build Plate` checkbox turns the ground check off
  entirely, 2026-08-23** — same-day follow-up to the build-plate-height
  entry below: "I would put a stick's base or other kind or shapes that
  are not a flat base, so I want some flexibility for those scenarios."
  `ground_height_m` still assumes a single flat plate at some height; this
  is for designs held by something the addon does not model as a plate at
  all. New `ground_required` parameter (default `True`) on
  `core/sticks.py`'s `Topology`/`build_topology()`/`extract_sticks()` and
  `core/order.py`'s `OrderSolver`: `False` makes `is_grounded()`
  permanently return `False`, so the `floating_component`/`below_plate`
  checks never fire, the mesh solve treats every component as free-
  floating (already-existing "no grounded vertex" fallback in
  `_solve_component_exact`, previously only reachable transiently), and the
  order solver seeds its search from a new
  `core.order.arbitrary_anchor_vertices()` (one deterministic vertex per
  connected component) instead of `grounded_vertices()`, plus a
  `WARN_NO_BUILD_PLATE` warning so the result is never silently presented
  as support-valid. The topological invariant (every stick attaches to an
  already-placed one) is untouched -- only the PLATE requirement relaxes.
  Exposed as `properties.py`'s `require_build_plate` checkbox (Design
  panel, Mesh Expansion box), which hides Ground/Build Plate Height when
  unchecked since they have no effect without a plate. Threaded through
  both `ops/design.py::extract_with_autoflip()` and
  `ops/order.py::build_solver()`. New tests in `test_sticks.py`,
  `test_order.py` (including a support-valid-order check against the
  actual `arbitrary_anchor_vertices()` seed, not just "no floating
  error"), and `test_blender_integration.py`; 408/408 in both bare CPython
  and real Blender.

- `[Blender]` **KR10's build volume corrected: +X, not -Y, 2026-08-23** —
  the 300x300x300mm cube centred 450mm from the robot origin (given
  directly by the user 2026-08-22) actually faces `+X`, not `-Y` as first
  given. `core/robots.py`'s `KR10_R900_2_ID` profile now reads
  `build_volume_min_m=(0.30, -0.15, 0.0)`,
  `build_volume_max_m=(0.60, 0.15, 0.30)` (was `(-0.15, -0.60, 0.0)` /
  `(0.15, -0.30, 0.30)`). Nothing else depends on which horizontal axis the
  cube sits on -- `ui/overlay.py`'s viewport box and `ui/panels.py`'s
  Reference numbers both already read the profile generically. Updated
  `tests/test_robots.py`, `tests/test_blender_integration.py` (build-file
  export and the overlay box test) to match; 391/391 in both bare CPython
  and real Blender.

- `[Blender]` **The build plate's own height is now adjustable, 2026-08-23**
  — user request: "I would like to be able to print 'floating' sticks. The
  build plate can change in height, so I want to make this a possibility."
  Every ground check in `core/sticks.py` (`Topology.is_grounded`, the
  `GROUND_SLIDE` clamp, the `floating_component`/`below_plate` diagnostics)
  and `core/order.py` (`grounded_vertices()`, `floating_components()`,
  `OrderSolver`) gained a `ground_height_m` parameter (default 0.0, so
  every existing design is unaffected) that shifts the "is this vertex on
  the plate" reference off a hardcoded Z=0. The build plate is a real,
  height-adjustable object: a component sitting entirely above Z=0 is not
  actually unbuildable, just unbuildable at the plate's CURRENT height --
  raising the new **Build Plate Height** field (Design panel, Mesh
  Expansion box; `properties.py`'s `build_plate_height_mm`) to that
  component's own lowest point (now named directly in both diagnostic
  messages) grounds it, as if the plate had physically moved to meet it.
  Threaded through `ops/design.py::extract_with_autoflip()` and
  `ops/order.py::build_solver()` from the same scene property, so
  extraction and order-solving always agree on the plate's height. Not
  added to the exported build file (BRIDGE_PROTOCOL.md unchanged): it only
  changes which components the addon treats as supported during design/
  ordering, never any stick's own absolute coordinates, so the ROS2 side
  needs nothing new to execute the file. New tests in `test_order.py`,
  `test_sticks.py`, and `test_blender_integration.py` (the last exercising
  the full operator path, extraction through Compute Build Order); 399/399
  in both bare CPython and real Blender.

- `[Blender]` **KR10's "Reset Stock" button now offers 2.0mm / 1.5mm as its
  Stock Section / Joint Allowance starting values, 2026-08-23** — the
  button (added in the entry below) previously reproduced
  `kr10_r900_2_kinematics.STICK_SECTION_M`/`JOINT_ALLOWANCE_M` verbatim
  (2mm / **1mm**); the user asked for a 1.5mm joint-allowance default
  specifically, as a more comfortable starting margin. This is a UI
  ergonomics choice, not a change to the vendored kinematics constant
  itself (still 1mm, still used as-is anywhere the addon calls into
  `kr10_r900_2_kinematics` directly) — `ops/design.py` gained a small
  `_STOCK_DEFAULT_OVERRIDES_MM` table the reset button consults before
  falling back to the robot's own kinematics value, so so_arm_100 (and any
  future robot without an entry) is unaffected. Both fields stay fully
  editable after the button is pressed, per the user's own framing ("these
  values can be changed of course, but to make it easier, make them the
  defaults"). Updated `test_blender_integration.py`'s
  `test_reset_stock_to_robot_defaults_uses_kr10s_own_numbers` to assert the
  2.0/1.5 override values instead of the raw kinematics constants; 391/391
  in both bare CPython and real Blender.

- `[Blender]` **`core/sticks.py`'s mesh-expansion geometry made robot-aware;
  `SO100_BuildMesh` naming and a "Reset Stock" convenience added,
  2026-08-23** — follow-up to the `core/order.py` fix above, requested
  directly by the user after re-testing (`~/KUKABlenderTest.blend`) still
  hit the same `floating_component` error, plus separately reported that a
  newly-generated build mesh was still named `SO100_BuildMesh` regardless
  of the selected robot. `extract_sticks()` now takes `robot_id` (default
  `so_arm_100`, unchanged) and resolves `joint_allowance_m`/`section_m`/
  `min_stick_length_m`/`max_stick_length_m` and
  `hard_min_stick_length_m(robot_id)`'s own floor from the SELECTED
  robot's kinematics, not so_arm_100's unconditionally — kr10_r900_2's real
  joint allowance (1mm/end, round-stock contact) is a genuinely different
  physical model from so_arm_100's 3.25mm square-stock formula.
  `ops/design.py` passes `robot_id=props.robot_id` through. Since
  Blender's `FloatProperty` defaults can't depend on another property's
  runtime value (no `update=` callbacks in this codebase), the Design
  panel's Stock/Stick-Length fields still START at so_arm_100's own
  numbers regardless of `robot_id` -- a new **`Reset Stock to This
  Robot's Defaults`** button (Design panel, Stock box) sets them to the
  selected robot's real numbers in one explicit action.
  `min_stick_length_mm`'s static widget floor was widened to
  `core_sticks.safe_min_stick_length_bound_m()` (18mm, the lowest across
  every registered robot) so it can never block a value valid for
  whichever robot is actually selected. Also renamed the build mesh object
  per robot (`SO100_BuildMesh` / `KR10_BuildMesh`,
  `ops.design.build_mesh_name()`, mirrors `base_empty_name()`'s own
  pattern) -- only at creation time, matching the base-empty's own
  behaviour. New `tests/test_sticks.py::TestMultiRobotGeometry` (7 tests)
  and three new `test_blender_integration.py` cases cover the fix; 390/390
  in both bare CPython and real Blender.

  **Root-caused the user's actual recurring build-order error directly
  from their file** (inspected `~/KUKABlenderTest.blend` with a real
  Blender background-mode script): it is a **scene setup issue, not a code
  bug**. The design cube's lowest vertex sits at Z=6.4mm relative to
  `KR10_Base`, not Z=0 -- outside the 0.5mm ground tolerance, so the whole
  design reads as one floating component with nothing on the plate,
  independent of the `core/order.py`/`core/sticks.py` fixes above (both of
  which are confirmed working correctly against this exact file). The
  improved `floating_component` error message (added with the
  `core/order.py` fix) already names this; the fix is to move the cube
  down ~6.4mm (or otherwise get its bottom face to sit exactly at
  `KR10_Base`'s own Z=0) before re-running Compute Build Order. Not
  auto-fixed in the user's file -- that is their own design content.
- `[Blender]` **`core/order.py`'s build-order solver made robot-aware,
  2026-08-23** — user-reported bug: "Compute Build Order" for a kr10_r900_2
  cube reported `0 in order - 0 warnings - 1 error (INCOMPLETE)` with no
  actionable detail, despite every stick showing buildable (green) at
  extraction. Root cause: `OrderSolver`'s own internal reachability check
  (`core_validate.validate_stick(oriented)`) never took a `robot_id` — it
  silently ran `so_arm_100`'s kinematics for every candidate regardless of
  the Scene's selected robot, so a kr10_r900_2 design sitting well within
  THAT robot's own build volume but well beyond so_arm_100's own,
  much smaller documented reach would see every candidate rejected inside
  the solver, either force-placing everything with a bogus so_arm_100-
  flavoured rejection reason, or (if nothing reads as grounded relative to
  the selected base empty at all) failing with a single opaque
  `floating_component` error and nothing placed — matching the report
  exactly. Fixed: `OrderSolver` now takes `robot_id` (default `so_arm_100`,
  so that robot's behaviour is unchanged byte-for-byte) and resolves its
  reachability check, its C2 cost heuristic's shoulder-axis point, and its
  C3 jaw-clearance model's `grasp_offset_m`/`jaw_width_m`/`section_m`
  defaults from the selected robot's own kinematics
  (`GRASP_OFFSET_M`/`JAW_RADIUS_M`/`STICK_SECTION_M`) instead of
  `so_arm_100`'s unconditionally; `ops/order.py`'s `build_solver()` passes
  `robot_id=props.robot_id` through. `jaw_length_m` (an so_arm_100-shaped
  estimate with no kr10_r900_2 equivalent measurement yet) is deliberately
  left as-is — a soft, over-cautious C3 pre-filter, never a hard block, not
  worth inventing an unfounded formula for. Also improved the
  `floating_component` error message to name the ground tolerance and
  suggest checking the design mesh's position against the selected robot's
  own base empty, since that remains a plausible independent cause of the
  same symptom. New `tests/test_order.py::TestKr10BuildOrder` (bare
  CPython) and `test_blender_integration.py`'s
  `test_compute_build_order_works_for_a_kr10_only_design` reproduce the
  exact bug (a design at Y=-550mm: deep in kr10_r900_2's own build zone,
  clearly outside so_arm_100's) and confirm both the fix and that
  so_arm_100's own behaviour is unaffected. **Not yet fixed, separately
  documented as deliberately out of scope:** `core/sticks.py`'s
  mesh-expansion geometry (joint gaps, length limits) still assumes
  so_arm_100's own stock — see `BLENDER_ADDON_PLAN.md` §4a's closing note.
  Verified: 379/379 in both bare CPython and real Blender.
- `[Blender]` **Addon renamed "RA130" in the UI; build-volume overlay/
  Reference panel made robot-aware, 2026-08-23.** The N-panel sidebar
  category (`ui/panels.py`'s `CATEGORY`) and the manifest's display `name`
  were still "SO-100" even after multi-robot support landed — renamed to
  "RA130" ("Robot Arm 130", the addon's own brand, distinct from either
  supported robot's name). The extension `id` (`so100_builder`) and every
  internal Python identifier (module/package name, `SO100_*` class
  prefixes, `so100.*` operator ids, `Scene.so100`) are **unchanged** —
  purely cosmetic, chosen to avoid re-registering the addon under a new
  identity or touching the live-dev extension linkage. Separately,
  `ui/overlay.py`'s 3D-viewport build-volume box and `ui/panels.py`'s
  Reference panel numbers were still always so_arm_100's regardless of
  `robot_id` (a known, documented gap from 2026-08-22's registry work) —
  both now read the selected robot's own `RobotProfile.build_volume_min_m`/
  `max_m`, so selecting `kr10_r900_2` shows its real 300×300×300 mm cube.
  The Reference panel's so_arm_100-only empirical caveats ("98% reachable",
  the `GRASP_OFFSET_M` grip-height note) stay gated to that robot rather
  than generalized. New `TestOverlayBuildVolume` tests the (pure-geometry,
  gpu-free) box-corner math directly for both robots. **Not independently
  verified in a live windowed Blender session this time** (background mode
  cannot run any `Panel.draw()`, and three different attempts to drive it
  through a real window in this environment did not respond) — rely on the
  automated suite (374/374, both environments) plus manual review; a human
  check of the actual 3D viewport is worth doing before trusting this
  fully, same caveat as any other unverified `ui/panels.py`/`ui/overlay.py`
  change in this project's history.
- `[Blender]` **`kr10_r900_2_kinematics` vendored for real, 2026-08-22** —
  it landed in `kuka_control` (Phase 1 done 2026-08-21, per that repo's own
  `docs/STATUS.md`) sooner than this repo's own multi-robot work expected,
  so the `kinematics/kr10_r900_2/` NOT-YET-VENDORED placeholder from earlier
  today was replaced with a verbatim copy of the real package (byte-
  identical, confirmed via `test_vendor_sync`; new
  `test_kinematics_kr10_vendored.py`/`_grasp_vendored.py`/
  `_jaw_clearance_vendored.py` re-run its own 32 tests against the vendored
  copy, all passing). It is a genuine 6-DOF closed-form solver (Pieper
  spherical-wrist decoupling, exact — no round-trip verification needed,
  unlike so_arm_100's own numerically-delicate search) with a **round**
  2 mm stock that has a genuinely free roll DOF, unlike so_arm_100's square
  stock. That last fact broke the assumption baked into the multi-robot
  registry's "one shared `solve_stick_placement` signature" plan from
  earlier today: kr10_r900_2's real placement entry point is
  `solve_stick_placement_any_roll` (sweeps roll × branch), not the
  single-roll `solve_stick_placement` its name coincidentally shares with
  so_arm_100's. Fixed by explicit per-robot dispatch instead of forcing a
  fake shared contract: `core/validate.py` gained
  `_validate_stick_kr10_r900_2` (mirrors the so_arm_100 section's own
  base/tip-flip-retry logic, using the real sweep), and `ops/mirror.py`
  gained a `_solve_placement()` helper that does the same dispatch for the
  preview rig. Separately, `core/mirror.py`'s `joint_frames()` no longer
  needs a per-robot `EE_OFFSET`-style constant at all (kr10_r900_2's own
  tool transform is a private, composed value with no such constant
  exported) — it now derives the fixed tool offset by calling the robot's
  own `fk(())` (zero joint angles), confirmed empirically to reproduce
  so_arm_100's `EE_OFFSET` exactly and to work identically for
  kr10_r900_2's 6-joint chain; `ops/mirror.py`'s segment-count logic was
  also de-hardcoded from "5" to `_segments(point_count)` for the same
  reason. `RobotProfile.stock_section_m` for `kr10_r900_2` now reads the
  vendored package's real `STICK_SECTION_M` (2 mm) instead of staying
  unset. `is_vendored` (added earlier today for the placeholder) now
  correctly reports `True` for both robots with no code change needed there.
  Verified end-to-end in real Blender: switching `robot_id` to
  `kr10_r900_2` runs real validation (no more "not vendored" text), poses
  a real 6-joint/7-point mirror rig, and produces a real exported build
  file stamped `"robot": "kr10_r900_2"` / the real `kinematics_version`
  ("1.0.0"). so_arm_100's own test outcomes are unchanged throughout.
  **Not done by this change** (deliberately, see the Open item above):
  `core/sticks.py`'s mesh-expansion geometry and `core/order.py`'s
  build-order solver still assume so_arm_100's own joint-gap/stock numbers
  regardless of `robot_id`, so a `kr10_r900_2`-targeted design's PHYSICAL
  layout (joint gaps, length limits) is not yet correct for KUKA's round
  2 mm stock even though its REACHABILITY validation now genuinely is.
- `[Blender]` **`kr10_r900_2`'s build volume confirmed, 2026-08-22** —
  given directly by the user, ahead of `kr10_r900_2_kinematics` existing: a
  300×300×300 mm cube centred 450 mm from the robot origin along −Y, same
  X/Y-centring and plate-sitting-Z convention as so_arm_100's own
  `BUILD_VOLUME_MIN_M`/`MAX_M`. Set directly on `core/robots.py`'s
  `RobotProfile` for `kr10_r900_2` (`build_volume_min_m=(-0.15, -0.60, 0.0)`,
  `build_volume_max_m=(0.15, -0.30, 0.30)`) — not inside the placeholder
  kinematics package, since it has nothing to do with whether that package
  is vendored. That distinction is now explicit: `RobotProfile` gained an
  `is_vendored` property (reads the kinematics module's own `VENDORED`
  flag, `kinematics/kr10_r900_2/__init__.py` sets `VENDORED = False`,
  every real package defaults `True`), and `ops/build.py`'s export /
  `ui/panels.py`'s Design-panel warning now check `is_vendored` specifically
  (not `has_build_volume`, which is true for `kr10_r900_2` now) so the
  refusal message stays accurate: "kinematics package not vendored yet",
  not "no build volume". `stock_section_m` stays unset for `kr10_r900_2` --
  not part of what was given. New `tests/test_robots.py` covers the
  registry and both profiles directly. Verified: 339/339 in bare CPython
  (82 skipped) and real Blender.
- `[Blender]` **Addon generalized into a multi-robot registry, 2026-08-22**
  (plumbing only -- no KUKA kinematics math, per the Open item above).
  `kinematics/` renamed `kinematics/so_arm_100/` (mechanical move, byte-
  identical contents, confirmed via `test_vendor_sync`) with a new
  `kinematics/kr10_r900_2/` placeholder package that raises a clear
  `NotVendoredError` the instant anything beyond `__version__` is touched
  (via module `__getattr__`) rather than crashing or silently falling back
  to so_arm_100's geometry. New `core/robots.py`: a `RobotProfile` registry
  (`so_arm_100`, `kr10_r900_2`) plus the documented kinematics-package
  interface contract (`__version__`, `STICK_LENGTH_RANGE_M`,
  `grasp_offset_for_length`, `solve_stick_placement`, `check_jaw_clearance`,
  `Unreachable`, `JOINT_NAMES`) every vendored robot package must satisfy.
  New Scene `robot_id` EnumProperty (default `so_arm_100`), threaded through
  `core/validate.py` (so_arm_100's own logic untouched behind a
  `robot_id == "so_arm_100"` gate; every other robot goes through a new,
  contract-only `_validate_stick_generic`), `core/mirror.py` (`joint_frames()`
  now takes the kinematics module as a parameter instead of importing
  so_arm_100's by name), `ops/mirror.py`'s preview rig, and `ops/build.py`'s
  export (stamps `robot`/`kinematics_version` from the selected profile;
  refuses to export -- a clean operator error -- for a robot with no
  confirmed `build_volume_min_m`/`max_m`, rather than inventing one).
  `io/build_file.py`'s `build_document()` gained the `robot` field
  BRIDGE_PROTOCOL.md §A.2 already specified. `core/sticks.py`/`core/order.py`/
  `ui/panels.py`/`ui/overlay.py` deliberately stay so_arm_100-hardcoded (each
  flagged inline) -- see `BLENDER_ADDON_PLAN.md` §4a for the full writeup and
  what remains robot-specific on purpose. Verified: full suite green in both
  bare CPython (325 tests, 81 skipped) and real Blender (325/325, incl. new
  smoke tests confirming the `kr10_r900_2` stub degrades cleanly through
  validation, the mirror rig and export without ever raising an unhandled
  exception), and the so_arm_100 default path's own test outcomes are
  unchanged from before this refactor.
- `[ROS2]` **D13 implemented 2026-08-11** (feature request: support sticks
  down to 50mm). Previously the grip point along the stick was always
  `GRASP_OFFSET_M` (51mm) from the base, regardless of length — too short a
  stick would have the jaws close past its own tip. Fixed by
  `so_arm_100_kinematics.grasp.grasp_offset_for_length(length_m)` (v1.3.0,
  vendored to both workspaces): returns the LARGEST offset that keeps the
  jaws' 10mm contact width fully on the stick, capped by the stick's own
  length — and since the offset is measured up from the base, a larger
  offset also means MORE clearance above the feeder floor, not less, so
  this maximizes safety margin rather than trading it off. Every
  currently-tested length (>= ~56mm) still gets back exactly the unmodified
  `GRASP_OFFSET_M` — zero behavior change for anything already
  hardware-proven. `run_pick_sequence` only switches away from the hand-tuned
  `steps.lower` joint pose to a computed one when a stick actually needs a
  smaller offset; `run_place_sequence` derives the identical offset from
  `StickSpec.length_m` so pick and place agree on the same grip point.
  `STICK_LENGTH_RANGE_M` lowered to `(0.050, 0.150)`. New constants
  `JAW_CONTACT_HALF_LENGTH_M` (5mm) and `MIN_GRASP_OFFSET_M` (30mm,
  unmeasured placeholder) — see the Open item above for what's still
  unverified. `ROS2_IMPLEMENTATION_PLAN.md` §4 D13 / §8.2.
- `[ROS2]` Two usability gaps raised directly from the Phase 5 hardware
  attempts above, both addressed 2026-08-04: (1) resuming from the status
  sidecar after closing every terminal and rebuilding looked like unwanted
  caching rather than the intended checkpoint feature — added
  `build_runner`'s `fresh_start` param (`-p fresh_start:=true`) as an
  explicit way to discard the sidecar and start over; the resume behavior
  itself needed no fix, it was working as designed. (2) a planning failure
  only logged a bare MoveIt error code, leaving the operator to dig through
  RViz/`get_planning_scene` to find out why — added
  `motion.py`'s `explain_collision()`, which calls MoveIt's
  `check_state_validity` service on the failed goal configuration and logs
  which two collision bodies are actually touching (e.g. `'stick' vs
  'placed_s_005'`) directly in the terminal; best-effort, never raises.
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 5.
- `[ROS2]` Four real bugs found and fixed from actual Phase 5 hardware
  attempts 2026-08-03/04 (see the Open item above for what surfaced them):
  (1) `register_placed_stick` only runs partway through
  `run_release_sequence`, so a `ReleaseStick` failure at its first step
  could leave a physically-glued stick with no collision box and no way to
  recover it — fixed by adding `prompt_stick_physically_placed()` to
  `build_runner`, asked before finalizing any skip/abort; answering yes
  registers the stick's collision box from the build file's own geometry
  and marks it `placed` regardless of which action failed. (2) MoveIt's
  planning scene lives in `move_group`, not in `build_runner`/
  `stick_task_server`, so restarting those two processes doesn't clear
  collision debris a previous crashed run left behind — fixed by having
  `build_runner`'s `main()` deterministically clear the transient `stick`
  id and any `placed_<id>` not currently marked `placed` in the sidecar, by
  id, before rebuilding the scene on resume. (3) The array-position resume
  loop re-attempted `PickStick`/`PlaceStick` on a stick already `placed`
  from an earlier session, driving a second physical stick straight at that
  stick's own already-registered collision box — root-caused via an RViz
  screenshot, fixed by an explicit already-`placed` guard at the top of
  each loop iteration. (4) `run_release_sequence` registered the just-placed
  stick's permanent collision box BEFORE attempting `retreat`, at a pose
  necessarily co-located with the gripper's pre-retreat position — the
  identical "obstacle sitting on the arm's own current pose" bug already
  fixed once for the `"stick"` transient object, resurfacing under the
  permanent box's name — fixed by reordering to retreat first, register
  second (same pose data, applied later; also now registers even if
  retreat itself still fails, since the physical release already happened
  by that point). `ROS2_IMPLEMENTATION_PLAN.md` §3 findings
  #17/#18/#19/#20. Not yet re-verified against hardware.
- `[Blender]` `so_arm_100_kinematics` re-vendored into
  `so100_builder/kinematics/` 2026-08-03 — was stuck at 1.1.0 without
  `jaw_clearance.py` at all, now matches 1.2.0 exactly (copied verbatim per
  the package README's own rules; `chain.py`/`envelope.py`/`grasp.py` were
  byte-identical already, only `__init__.py`/`constants.py` differed, plus
  the missing module). Added the missing
  `test_kinematics_jaw_clearance_vendored.py`; all 42 vendored tests pass
  in a plain `env -i` Python. A build file exported from Blender now
  correctly stamps `kinematics_version: "1.2.0"` instead of a stale
  `"1.1.0"`. See `~/MagnaRecta/RobotArm_UbuntuAddon/KINEMATICS_VENDOR_UPDATE.md`
  for the full writeup.
- `[ROS2]` Phase 5 (build execution) built and dry-run-verified 2026-08-03:
  new `Reset` action (`stick_task_msgs`) as the only way back from
  `ERROR` to `IDLE` — a hard prerequisite discovered during planning, not
  originally requested, since without it a single failure anywhere in a
  multi-stick build would permanently reject every subsequent action, not
  just fail that one stick; `assume_gripper_empty` flag mirrors
  `BRIDGE_PROTOCOL.md` §5.11's own already-designed socket `reset` command.
  New `build_file.py`, a pure-Python build-file/status-sidecar loader
  ported to stay a compatible counterpart to the Blender addon's own
  `so100_builder/io/build_file.py` (same sidecar filename derivation, same
  pending-omission/unknown-status-degrades rules), plus the
  `kinematics_version`/`frame` checks the protocol assigns specifically to
  this side (hard refuse on mismatch, not a warning). New `build_runner`
  node drives `stick_task_server` through an entire build stick by stick
  with the human gates, resumes correctly from a status sidecar (rebuilding
  the scene from already-placed sticks' own base/tip geometry via a new
  `pose_utils.axis_aligned_box_pose()`), and offers retry/skip/abort on any
  failure. Along the way, found and fixed an unrelated bug:
  `stick_task_server.launch.py`'s `name=` parameter was globally remapping
  BOTH of that node's internal `Node`s onto the same displayed name.
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 5, §3 finding #16.
- `[ROS2, physical]` A failed `PickStick` grasp left "stick" sitting in the
  world scene exactly at the robot's current pose (re-added unconditionally
  by `plan_grasp_gripper()` so a success case can attach it, never removed
  again on failure) — blocked planning ANY next move, including a manual
  RViz-driven return to home, until the object was removed by hand.
  Confirmed on hardware 2026-08-02. Fixed: `run_pick_sequence` now removes
  "stick" on all three grasp-failure exits. Also: `PickStick`'s result was
  reporting `gripper_gap: 0.0` on every failure instead of the real
  measured gap — fixed so failures are diagnosable going forward.
  `ROS2_IMPLEMENTATION_PLAN.md` §4 D12.
- `[ROS2]` Motion speed raised for both the programmatic pipeline and RViz's
  own manual-plan defaults, now that the basic sequence is hardware-proven:
  `pick_and_place.yaml`'s `velocity_scaling`/`acceleration_scaling` 0.2 to
  0.4 (affects `pick_and_place_node`/`stick_task_server`/`tune_grasp`, all
  three read this yaml); RViz's MotionPlanning panel defaults
  (`moveit.rviz`/`moveit.xtra.rviz`) 0.1 to 0.5.
- `[ROS2, physical]` The first-pass grasp tuning below (`-12.0°`/`0.0611`
  rad) stopped being valid the moment D9 was fixed: `stick_task_server`
  started using the yaml's real `lower` pose instead of a stale code
  default, which changed the real grasp geometry. `PickStick` then reliably
  stalled at -12° (hard stop ~-7°, 15s `goal_time_tolerance` watchdog,
  `ABORTED`, stick never lifted) — confirmed on hardware. Re-tuned under
  the now-correct pose with the same empty-vs-holding `tune_grasp`
  methodology, at smaller commanded magnitudes (no usable signal below
  ~-8°, since the jaws haven't reached the stick yet there): new values
  `gripper_grasp_position_deg: -9.0`, `gap_threshold: 0.0244` rad (~1.4°) —
  empty ~0.4° gap, holding ~2.1° gap. Set consistently in `pick_and_place.yaml`
  and all three nodes' code fallback defaults, same as before.
  **Grasp tuning is pose-dependent, not just gripper-dependent** — re-run
  `tune_grasp` (both empty and holding) any time `steps.lower` or the stick
  geometry changes. `ROS2_IMPLEMENTATION_PLAN.md` §4 D4.
- `[ROS2, physical]` The `-9.0°`/`0.0244` rad (~1.4°) values above were
  themselves too tight — from only 2 calibration samples per condition.
  Real `PickStick` runs then produced holding gaps as low as 1.09° (real,
  visually-confirmed grasps rejected as "closed on nothing") alongside
  empty gaps consistently under 0.4°. Re-tuned to `gap_threshold: 0.0157`
  rad (~0.9°) from the fuller picture (holding: 1.09/1.35/2.14/2.23°;
  empty: 0.21/0.30/0.39°) — same day, same D4 entry. `gripper_grasp_position_deg`
  stayed at `-9.0`. `ROS2_IMPLEMENTATION_PLAN.md` §4 D4.
- `[ROS2, physical]` `tune_grasp` couldn't close past -10° at all — even
  -10° itself failed ("Planning failed! Error code: FAILURE"), despite -12°
  being already hardware-proven via `PickStick`. Two things, found in order:
  (1) the Gripper joint's URDF `<limit>` was `-0.1792` rad (~-10.27°) — the
  actual enforced bound for real hardware (`so_arm_100_moveit_config`'s own
  `ros2_control.xacro` has no min/max clamp at all) — widened to `-0.24` rad
  (~-13.75°) in `so_arm_100_5dof_arm.urdf.xacro`. That alone didn't fix it:
  (2) the REAL cause was `tune_grasp_node.py`'s close-test never removing
  the "stick" collision object before planning a close, unlike
  `run_pick_sequence`'s proven remove/plan/re-add pattern — masked until
  today by the D9 pose bug (the box used to sit somewhere harmless), now
  that D9 is fixed the box sits exactly where the jaws close and blocked
  every close as a collision. Fixed with a new `_close_gripper()` mirroring
  the proven pattern. Separately: LeRobot's calibration JSONs
  (`so_leader`/`so_follower`) are not read by this ROS2 workspace at all —
  editing them does nothing here. `ROS2_IMPLEMENTATION_PLAN.md` §3 findings
  #13/#14/#15.
- `[ROS2]` `run_release_sequence`'s `open gripper at place` step passed the
  ARM's MoveIt2 interface to `motion.run_step()` instead of the gripper's,
  while still planning a gripper-only trajectory (`motion.plan_gripper()`)
  — so it executed and waited on that trajectory through the wrong
  interface. Confirmed on hardware 2026-08-02: `PickStick`/`PlaceStick`
  succeeded, `ReleaseStick` hung on this exact step for an extended period
  before aborting with "planning/execution failed at 'open gripper at
  place'". An isolated copy/paste slip (the equivalent step in
  `run_pick_sequence` already passed the gripper correctly) — fixed by
  passing `gripper`. `ROS2_IMPLEMENTATION_PLAN.md` §4 D10.
- `[ROS2]` Feature: the fed stick's config was a box-center pose + a fixed
  length, so a different-length stick would land off-position without
  retuning the pose constant. Replaced with `stick.base_xyz_m` (the
  stick's physical base at the feeder hole, same convention as
  `StickSpec.base`) + `stick.section_m` + `stick.default_length_m`;
  `pose_utils.feeder_stick_pose()` computes the box center from base +
  length, so any length lands base-down at the same spot. `stick_task_server`
  now threads `PickStick.length` through per-goal (previously ignored) and
  remembers it for sizing `placed_<stick_id>` at `ReleaseStick`.
  `ROS2_IMPLEMENTATION_PLAN.md` §4 D11.
- `[ROS2]` `config/pick_and_place.yaml` was scoped to node name
  `pick_and_place_node:`, which a ROS2 params-file only applies to a node
  with that exact name. `stick_task_server` and `tune_grasp_node` are
  different node names, so **neither ever received this yaml's tuned
  values** (regardless of whether `--params-file` was passed) — both
  silently ran on their own code-fallback defaults, including a `stick.pose`
  ~13cm off the real feeder location. This, not the attach/detach drift
  below, is the real explanation for the "misplaced stick, visible from the
  very first action" report 2026-08-02. Fixed: yaml rescoped to `/**:`
  (wildcard node-name match), new `stick_task_server.launch.py` wires
  `--params-file` automatically, `tune_grasp_node.py`'s docstring spells out
  the explicit flag it still needs (must stay `ros2 run` for stdin). Code
  fallback defaults synced to the yaml's real values too, as a safety net.
  `ROS2_IMPLEMENTATION_PLAN.md` §4 D9.
- `[ROS2, physical]` `attach_collision_object()` attached the fed stick's
  collision box at whatever pose it currently had in the world scene, and
  `plan_grasp_gripper()` re-added that box at the *static feeder-pose
  constant* before attaching, not the gripper's actual pose. Flagged
  2026-08-02 as cosmetic-only; a second hardware run the same day showed it
  is not: `detach_collision_object()` returns "stick" to the world (rather
  than deleting it) at the pose implied by that wrong offset carried
  through the whole pick→place move, landing it between the feeder and the
  base and blocking `ReleaseStick`'s `retreat` plan. Fixed 2026-08-02:
  `run_pick_sequence` now re-adds "stick" at `motion.get_current_ee_pose()`
  (the same FK technique already proven for `register_placed_stick`)
  instead of the static constant, and `run_release_sequence` now calls
  `scene.remove_stick()` right after detaching, since `placed_<stick_id>`
  is the authoritative record from `ReleaseStick` onward. `ROS2_IMPLEMENTATION_PLAN.md`
  §4 D8.
- `[ROS2 doc]` §9.6 named the per-stick base/tip override `so100_flip`; the
  real field is `flip` (`BLENDER_ADDON_PLAN.md` §9.2, `core/sticks.py`).
  Fixed 2026-07-31.
- `[shared]` Grasp-orientation transform gap — was addon-only, now
  `so_arm_100_kinematics.grasp` v1.1.0, both sides vendor it identically.
  Fixed 2026-07-31 — full writeup in `BLENDER_ADDON_PLAN.md`'s Phase D
  section and `so100_builder/README.md`.
- `[BRIDGE_PROTOCOL.md]` §A.2's worked example had `s_002.shared_ends` wrong
  in one iteration (reasoned from temporary build-order support state
  instead of final topology, which are independent properties). Fixed
  2026-07-31.
- `[ROS2]` Jaw-clearance test (swept-jaw collision check) written —
  `so_arm_100_kinematics.jaw_clearance` v1.2.0. Fixed 2026-07-31 — full
  writeup in `ROS2_IMPLEMENTATION_PLAN.md` §8.2 consequence 2 and §11
  Phase 1. Synced into the Blender workspace's `so_arm_100_kinematics/`
  staging copy (tests re-run and passing there too); still needs
  re-vendoring into `so100_builder/kinematics/` (see the open item above)
  and still has no caller wired up on this side either.
- `[ROS2]` `stick_task_server_node` wedged permanently (every action goal
  on the node hung) after the first goal that involved real motion —
  confirmed on real hardware 2026-08-02, not just the dry-run harness.
  Root cause identified precisely: `rclpy.spin_once()`, which `pymoveit2`
  calls internally, detaches a `Node` from its `Executor` when called from
  within a callback that same `Executor` is dispatching — a known upstream
  bug ([ros2/ros2#1609](https://github.com/ros2/ros2/issues/1609)). Fixed
  2026-08-02 by splitting into two `Node`s (`stick_task_server_motion` for
  `pymoveit2`, `stick_task_server` for the `ActionServer`s), each with its
  own `Executor` — confirmed fixed in the dry-run harness (the server now
  stays responsive to new goals through an in-flight failure). See
  `ROS2_IMPLEMENTATION_PLAN.md` §3 finding #12 and §11 Phase 4. One
  narrower, not-yet-understood symptom remains — see Open items.
- `[ROS2]` `grasp_verification.gap_threshold` was untuned (the old default,
  0.1 rad/~5.7°, was higher than any gap a genuine grasp could produce —
  every real grasp would false-negative). Fixed 2026-08-02: built
  `tune_grasp` (`tune_grasp_node.py`), a new interactive tool that closes
  the real gripper to a commanded position, reads back the actual settled
  position, and reports the verification verdict at several candidate
  thresholds at once. Measured empty-vs-holding on the real gripper/stock;
  new values `gripper_grasp_position_deg: -12.0`,
  `gap_threshold: 0.0611` rad (~3.5°) set consistently in
  `pick_and_place.yaml` and as the code fallback default in all three nodes
  that read it. `ROS2_IMPLEMENTATION_PLAN.md` §4 D4.
- `[ROS2]` Base/tip flip-retry on an unreachable placement — was flagged as
  routine/expected (`Wrist_Roll`'s asymmetric limit, §9.6) but unwired on
  this side, while the Blender side already did it automatically
  (`WARN_AUTO_FLIPPED`). Fixed 2026-08-01: `stick_spec.solve_stick_spec_joints()`
  tries base→tip then tip→base before raising `Unreachable` — see
  `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 3. Not yet reachable from a real
  `PlaceStick` action (Phase 4 doesn't exist yet), but the retry logic
  itself is built, tested, and available to any caller of `stick_spec.py`.
- `[ROS2 doc]` D2/D3 — root `README.md`'s pick-and-place section documented
  the pre-`steps.<name>.mode` config scheme and omitted the §3 #2-4 timing
  watchdog findings. Fixed 2026-08-01, alongside the Phase 2/3 work that
  made the old table even more stale (added the `stick_spec` step mode).
- `[ROS2, physical]` Phases 2 (refactor) and 3 (parametric place) were
  software-verified only (unit tests, MoveIt-only dry run) pending a real
  hardware run. Confirmed on hardware 2026-08-01: the interactive demo
  behaves as before, and `s_004` from the real build file was placed at a
  computed location — `ROS2_IMPLEMENTATION_PLAN.md` §11 Phase 2/3.
