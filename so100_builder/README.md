# so100_builder — Blender addon

Design and slicing tool for the stick-gluing robot. Turns a wireframe mesh
into a validated, ordered build file that a separate ROS2 process executes.
There is deliberately **no live connection** to ROS2 (Option C,
`docs/BLENDER_ADDON_PLAN.md` §2) — the build file is the entire interface.

**Status: Phases A–E complete, plus multi-robot registry support (2026-08-22,
see `docs/BLENDER_ADDON_PLAN.md` §4a).** Phase D's hardware run (building on
the real robot) is the one thing that cannot be automated here — see "What
Phase D delivers" below.

---

## Running the tests

```bash
# In Blender (runs everything, including the bpy integration tests)
/opt/blender/blender --background --python so100_builder/tests/run.py

# Bare CPython (skips the ~80 bpy tests; proves core/ and kinematics/ are pure)
python3 so100_builder/tests/run.py
```

Both must pass. 370 tests; the bare-CPython run is what catches a `bpy`
import leaking into `core/` or `kinematics/`.

## Installing

```bash
/opt/blender/blender --command extension validate so100_builder
/opt/blender/blender --command extension build --source-dir so100_builder --output-dir /tmp
# then in Blender: Preferences > Get Extensions > Install from Disk
```

⚠ Installing copies the source. While developing, prefer adding the repo as
an extension directory instead, or you will debug a stale copy.

## Verified environment

Blender **5.2.0 LTS** (bundled Python 3.13.13) is installed at `/opt/blender`
on this machine — the planning docs say it is not, and that is now out of
date. The manifest, the extension build, and the install/enable path have all
been exercised here.

---

## Decisions taken, and why

**Manifest format (constraint B8).** Verified against the Blender 5.2 manual
before writing scaffolding: 5.2 uses `blender_manifest.toml`
(`schema_version = "1.0.0"`). Legacy `bl_info` still loads via "Install legacy
Add-on" but is deprecated, and Blender rewrites then strips `bl_info` on
Reload Scripts when a manifest is present. This addon ships the manifest only
and targets 5.2+.

**No `net/` layer.** Deleted from §4's tree per the Option C decision.

**N7 growth mode — default `PER_EDGE`** (confirmed with the user 2026-07-28).
`UNIFORM` is a toggle in the Design panel.

**N8 residual tolerance — 0.5 mm** (confirmed 2026-07-28), exposed as a UI
field.

**Ground mode — `SLIDE`** (decided 2026-07-28). See below.

**Nothing is built below z = 0.** The base plate is physical, so a stick
endpoint beneath it raises a `below_plate` error rather than being silently
clamped — clamping would move the design without saying so.

**§5.2.1's worked inverted-U — 113.25 / 113.25 / 116.5 mm** (confirmed
2026-07-28; the original request's 114.25 / 117.5 were off by 1 mm). These are
hardcoded as a test fixture.

---

## ⚠ Things that are assumed, not verified

Nothing load-bearing has been built on these, but they are read from the
kinematics module and will propagate if they are wrong.

1. **`GRASP_OFFSET_M = 0.051` is derived from FK, not measured.** D13
   (`grasp.grasp_offset_for_length()`, kinematics v1.3.0) made the grip
   offset adaptive per stick, but `GRASP_OFFSET_M` remains the ceiling every
   long-enough stick still gets. The addon reads it from
   `kinematics/so_arm_100/constants.py` and never hardcodes it, so a Phase 0
   ruler correction propagates automatically.
2. **The `stick_roll` → `Wrist_Roll` sign convention is untested on hardware.**
   Phase A emits `roll_deg = 0.0` for every stick, so nothing here depends on
   it yet. It becomes load-bearing in Phase B/D.
3. **`MIN_GRASP_OFFSET_M = 0.030` is an unmeasured placeholder**, same status
   as `GRASP_OFFSET_M` above. Together with `JAW_CONTACT_HALF_LENGTH_M`
   (5mm, confirmed by the user's own jaw measurement) it sets the 35 mm hard
   floor on stick length via `hard_min_stick_length_m()`. The addon reads
   both from `kinematics/so_arm_100/constants.py` and never hardcodes them,
   so a Phase 0 correction propagates automatically — re-run the tests,
   since `test_hard_floor_comes_from_the_kinematics_module` asserts the
   35 mm value. Phase 0 measures the jaw envelope directly.
4. **These, and everything else in this section, are so_arm_100-specific.**
   `kr10_r900_2_kinematics` (vendored 2026-08-22) has its own real
   caveats of the same shape — round-2mm-stock `GRASP_OFFSET_M`/`JAW_RADIUS_M`
   estimates, an unconfirmed `MIN_GRASP_OFFSET_M` — documented in *its own*
   package (see `kinematics/kr10_r900_2/constants.py`'s own comments), not
   repeated here. `core/sticks.py`'s mesh-expansion geometry does not read
   any of them yet regardless of `robot_id` — see
   `docs/BLENDER_ADDON_PLAN.md` §4a's closing note.

---

## Two spec problems found while implementing — both now resolved

### 1. Ground mode → `SLIDE` (decided 2026-07-28) ✅

§5.2.2 originally said to *"pin grounded vertices"* during the relaxation
solve. Taken literally that makes **any closed loop lying on the base plate
unexpandable** — every vertex is immobile, so those edges cannot grow at all
and miss their target by the full allowance. A square drawn flat on the plate,
or a wireframe cube's bottom ring, reports as unbuildable with a −6.5 mm
residual.

**Resolved: `SLIDE` is the default.** A grounded vertex is locked to z = 0 —
nothing expands below the physical base plate — but is free to slide across
it as the design grows. `PIN` remains as a toggle. `docs/BLENDER_ADDON_PLAN.md`
§5.2.2 has been updated to match.

| | `SLIDE` (default) | `PIN` |
|---|---|---|
| square flat on the plate | exact, comes out 6.5 mm larger | all 4 edges −6.5 mm, unbuildable |
| cube's bottom ring | exact | 4 sticks flagged |
| inverted U | exact, uprights tilt ~1.65° | exact, uprights tilt ~1.65° |

The two modes only diverge when pinning removes *all* freedom. On the
inverted U they agree, because two free top vertices already give the solver
enough room.

Two related facts the solver encodes: the inverted U is *acyclic as a mesh*
yet behaves as the looped case, because pinning both feet makes the base plate
act as a fourth edge — so a component takes the exact outward walk only if it
is a tree **and** has at most one ground anchor. And the doc's claim that the
U's uprights end at 113.30 mm was wrong: tilting shortens their vertical
extent and the solver raises the tops to compensate, landing all three edges
on target to better than 0.001 mm. That has been corrected in the plan too.

### 2. `BRIDGE_PROTOCOL.md` §A.2's example numbers ✅ corrected

The protocol's worked stacked pair showed `s_001.tip` at z = 0.110 and
`s_002.base` at z = 0.11325 — a **3.25 mm** gap, i.e. only one end inset.
§5.2 says *each* stick stops 3.25 mm short of a shared vertex, giving a
**6.5 mm** gap, and §5.2.1's `required_edge` formula agrees.

Corrected to `s_002.base = 0.1165`, `tip = 0.2265`, verified against the
addon's actual output. The root cause — reading "3.25 mm gap" as the
end-to-end distance rather than the per-end clearance — is now called out
explicitly in §5.2.

---

## Deliberate implementation choices worth knowing

- **Modifiers are not evaluated.** The stable-id attribute layer lives on the
  original mesh; a generative modifier would produce untrackable edges. Apply
  modifiers before extracting.
- **Edit Mode is refused**, not silently mishandled — attribute layers cannot
  be written there and `mesh.vertices` reports pre-edit state.
- **Stock snapping ties break toward the shorter length**, quantised at 1 µm.
  Blender stores vertex coordinates as float32, so a cube's twelve "identical"
  110 mm edges actually measure 0.109999999 – 0.110000015 m. A finer epsilon
  lets that ~15 nm noise decide the snap, and one cube's edges snap to
  *different* stock lengths.
- **Extraction is synchronous.** Measured: 40–150 sticks is under 10 ms;
  2700 edges takes 0.35 s. The timer-chunking that constraint B3 requires
  belongs to Phase C's order + validation solver, where the cost is real.
- **`||tip − base|| == length_m` is exact**, always. The residual is absorbed
  into the joint gaps rather than into the stick, because
  `BRIDGE_PROTOCOL.md` §6 makes ROS2 reject a >1 mm inconsistency. An end
  seating on the plate is anchored rather than centred, so it really does
  start at z = 0.

---

## What Phase A delivers

- `blender_manifest.toml`, `__init__.py`, `prefs.py`, `properties.py`
- `core/transform.py` — Blender world ↔ base_link metres (B5, B6)
- `core/sticks.py` — extraction, joint allowance, **§5.2.1 mesh expansion**,
  both length modes, warnings, cut list
- `core/state.py` — stick state, stable-id allocation, status-sidecar parsing
- `kinematics/` — vendored verbatim, byte-identity enforced by a test
- `ops/design.py`, `ui/panels.py` — Design / Summary / Sticks / Reference panels
- `tests/` — 119 tests

Phase A's "done when" — *a wireframe cube produces 12 sticks with correct
metre coordinates in base_link and correct stick lengths* — is covered by
`TestWireframeCube` (offline) and `TestExtraction` (in Blender).

---

## What Phase B delivers

- `core/validate.py` — per-stick reachability + orientation verdicts against
  the vendored kinematics, wired automatically into **Extract Sticks**
- `ui/overlay.py` — GPU viewport overlay: build volume box + per-stick status
  colours, hard on/off switch (`show_overlay`)
- `ops/design.py` — a "Check By Eye" pair (select + frame a stick's edge in
  the viewport) built ahead of Phase C's order, but designed to need no
  change once a real build order exists
- `tests/test_validate.py` — 27 tests, including a 2000-sample fully-random
  fk()-round-trip sweep

Phase B's "done when" — *dragging a vertex outside the envelope turns that
stick red with a specific reason, live, with no ROS running* — works two
ways: the Sticks list's status icon + reason box (always available), and the
3D viewport overlay (when `show_overlay` is on).

### Policy decided with the user (2026-07-28): permissive validation

Trust the closed-form IK's own success/failure, including for sticks tilted
out of the arm's own vertical plane. ROS2/MoveIt re-validates on load
regardless (Sec 7: this is an upper bound, never a final verdict), so an
over-eager Blender-side rejection only costs a design iteration for nothing.
Where a placement is genuinely unreachable, it's a hard error; where it's
merely *unusual* (see below), it's a warning, not a block — visible via
"Check By Eye" so the user can decide.

### ⚠ A plan-doc claim this module directly contradicts, with evidence

`BLENDER_ADDON_PLAN.md` / `ROS2_IMPLEMENTATION_PLAN.md` Sec 9.4 states a
stick "tilted out of the arm's plane (leaning sideways)" is **categorically
unreachable** with 5 DOF — "would need independent tool yaw." Empirical
testing against the real, hardware-validated `chain.py` directly contradicts
this: such placements **are** reachable. A concrete example (verified by
round-tripping the solved joints back through `fk()` to <0.05°):
`ik((0.30, -0.0452, 0.10), tool_elevation_target_rad=0.0,
stick_roll_rad=math.radians(-45.0))` succeeds and produces a stick tilted
sideways out of the vertical plane. A 2000-sample sweep over fully random 3D
stick directions (not restricted to any plane) found 383 reachable
placements, every one verified to <0.001° error. That table entry in both
planning docs should be corrected; `core/validate.py` does not encode it.

### The grasp-orientation transform — now shared with the ROS2 side

Converting a stick's 3D direction into `chain.ik()`'s
`(tool_elevation_rad, stick_roll_rad)` pair is genuinely non-trivial. It was
first built in `core/validate.py`, then **promoted into
`so_arm_100_kinematics.grasp` (v1.1.0)** so the ROS2 side's `PlaceStick`
action uses the exact same, self-verifying search — closing the integration
gap this section used to flag (see "An integration gap worth flagging
before hardware" below, now resolved). `core/validate.py` calls
`kinematics.grasp.solve_stick_orientation` / `grasp_target` rather than
deriving orientations itself; it built the transform from facts `chain.py`
already states and tests — planarity, perpendicularity, `Wrist_Roll`
TCP-invariance — not from anything new about the arm. Four findings from
building it, all confirmed empirically against `fk()`/`ik()`, not assumed:

1. **The reference stick direction points from the grip toward the stick's
   BASE, not its tip.** `rot(joints) @ (0,0,1)` is a valid "which way does
   the stick point" vector — confirmed against all five tuned poses, which
   all come out *negative* Z. Makes sense once you notice the grip sits
   *above* the base (`GRASP_OFFSET_M` up from it). Get the sign wrong and
   every roll solve is silently off by ~180°.
2. **The elevation equation has two roots 180° apart**, and only one usually
   keeps the needed roll inside `Wrist_Roll`'s limit. `Shoulder_Rotation`
   itself has no branch ambiguity, so trying both elevation roots × both
   elbow branches (4 combinations) is *exhaustive*, not a heuristic.
3. **`stick_roll_rad` rotates clockwise around `tool_axis()`, opposite the
   standard right-hand convention** a naive cross-product formula assumes.
   Invisible for in-plane targets (there `sin(roll)` is always ≈0, so only
   the sign-independent magnitude ever gets exercised) — only surfaces once
   a target has a genuine tangential component. This is exactly the kind of
   bug a "looks right" formula hides; it was caught by the self-verification
   below, not by inspection.
4. **The elevation equation is numerically ill-conditioned right at the
   documented-easy directions** (vertical, horizontal-radial, horizontal-
   tangential) — found 2026-07-29 from the user's own inverted-U test (two
   vertical uprights + a horizontal top, all 110 mm, which they knew for a
   fact was buildable). The top stick's axis, after the mesh-expansion
   solve, was **0.71° off exactly tangential** — ordinary iterative-solver
   noise, not a design intent. That was enough for the exact-root formula to
   swing onto a ~90°-wrong elevation branch demanding a 328 mm reach (the
   sub-chain only spans 19–251 mm), reporting the stick impossible. The
   natural elevation (0°, Sec 9.4's own pose for that direction) reaches it
   fine with <1° of error. Fix: the search now also tries the four cardinal
   elevations (0°, ±90°, 180°) as a second tier, and always prefers
   whichever candidate has the smallest achieved error — an exact root wins
   whenever one succeeds; a cardinal only wins when both exact roots don't.

**Every accepted candidate is verified by feeding the solved joints back
through `fk()` and checking the achieved direction against the target**
(0.05° for the exact roots, `kinematics.grasp.ANCHOR_TOLERANCE_DEG` for the
cardinal fallback — see below) — never trusted from the formula alone. Because the
search is exhaustive over both families and self-verifying, "no candidate
verifies" means the placement is genuinely unreachable, not merely
unvalidated.

⚠ **The cardinal-fallback tolerance needed a second look, same day** (a
different user mesh, not a synthetic test). Their inverted-U's uprights
weren't perfectly vertical — ordinary hand-drawn geometry, not a mistake —
and that propagated a **2.077°** tilt into the top stick, just over
Finding 4's original 2° tolerance. Checked directly rather than just
widening the number to fit: the *exact* mathematical elevation for that
tilt (≈±90°) is genuinely unreachable (reach 0.332 m > the 0.251 m
envelope, confirmed by calling `ik()` on it directly) — so accepting the
near-tangential anchor's small residual really is correct, not a bug to
chase. That made the tolerance itself worth grounding in something
physical instead of another reactive bump: the joint's own glue gap
(3.25–6.5 mm) already absorbs `asin(3.25/110)..asin(6.5/110) ≈ 1.7°–3.4°`
of angular slack over a ~110 mm stick before the gap itself is exceeded.
`kinematics.grasp.ANCHOR_TOLERANCE_DEG = 5.0` sits comfortably above that (covers the
2.08° case with margin) while staying far below where every actual
wrong-branch bug found this session showed up (15–180°, never single
digits). A stick accepted via this fallback — an approximation, not an
exact solve — now carries `WARN_ORIENTATION_APPROXIMATED` so this is never
silent even when it's well within tolerance.

### Why a verified-reachable steep/out-of-plane tilt still gets a warning

A verified "reachable" here is still a single-point IK check — the same
upper-bound Sec 7 already warns about. `WARN_OUT_OF_PLANE_TILT` exists so
the user notices and can eyeball it with "Check By Eye" — not because the
reachability verdict itself is in doubt.

⚠ **Calibration bug found alongside Finding 4, same session:** the warning
originally fired on *alignment* with the tangential direction
(`asin(|dot(axis, tan_hat)|)`), which flags the safe, Sec-9.4-documented
pure-tangential case (the inverted-U's own top stick!) as *maximally*
risky — backwards. What's actually untested territory is a **genuine mix**
of in-plane and tangential lean at once (the disputed 45°/45° example
below), not alignment with either extreme. Fixed to measure angular
distance from *both* extremes (`min(angle_from_in_plane,
angle_from_tangential)`) and warn only when neither is within
`OUT_OF_PLANE_WARN_THRESHOLD_RAD` — so a purely vertical, radial,
tangential, or in-plane-tilted stick never warns, and only a real oblique
lean does. That threshold is now deliberately the *same* value as
`kinematics.grasp.ANCHOR_TOLERANCE_DEG` (5°), not a coincidence: the 2.077° case above
originally tripped both warnings for the same underlying cause (ordinary
hand-drawn imprecision), which was redundant noise, not two distinct
things worth flagging separately.

### Finding 6 — a third real case, same user: `Wrist_Roll`'s asymmetric limit and which end is "base"

Not a numerical-conditioning bug like Findings 4/5 — this one is a genuine
reachability difference between two physically distinct placements, and the
fix is pointing at an existing feature, not widening a tolerance again.

`Wrist_Roll`'s limit is asymmetric (~−157°..+68°, since
`WRIST_ROLL_AT_ZERO_STICK_ROLL_RAD` = +90° eats most of the positive
headroom). Swapping which end of a stick counts as "base" negates the grip
direction, which typically negates the needed roll too — so for a stick
sitting near this asymmetric edge, **one base/tip assignment can be
comfortably reachable while the other needs a roll on the wrong side of the
limit.** For a near-horizontal stick (both ends at nearly the same height —
exactly the inverted-U's top stick again), Sec 5.1's "base = lower Z"
tie-break is then decided by whichever end happens to be a hair lower after
the mesh-expansion solve — noise-scale, and can land either way between
otherwise-identical-looking meshes. The user hit this after *fixing* their
mesh to be more symmetric than Finding 5's case (uprights nearly perfectly
vertical) — ironically, that made the tie-break flip to the unreachable
assignment.

`core/sticks.py` already exposes a per-stick `flip` override for exactly
this (Sec 5.1: *"Overridable per stick"*). `validate_stick` now checks it
automatically on failure — re-running `kinematics.grasp.solve_stick_orientation`
with the negated axis — and if the flip would work, says so directly instead
of a generic reach-based message: *"unreachable with this end as base ...
but the OPPOSITE end works — toggle this stick's 'Flip' setting."* Verified
both ways before shipping: confirmed the reported end is genuinely
unreachable (all four cardinal anchors and both exact roots fail even at a
20° sanity tolerance, not merely `kinematics.grasp.ANCHOR_TOLERANCE_DEG`)
and that flipping it actually builds.

**Automated (2026-07-30, user-requested):** `Extract Sticks` now applies the
suggested flip itself rather than leaving the user to notice the reason text
and toggle a checkbox by hand. Mechanism: extract + validate once, collect
every stick whose `Verdict.suggested_flip` is true (confirmed working, never
guessed), toggle those in the `flips` dict, and — since `core/sticks.py`
applies `flip` per edge independently after the shared expansion solve
already ran, so flip choices for different sticks never interact — run
extraction exactly once more with the updated dict. No loop needed; one
extra pass always converges. An auto-flipped stick carries `WARN_AUTO_FLIPPED`
so it's visible, not silent, and the operator reports how many were flipped.

⚠ This is a Phase-B-only, reachability-driven heuristic. Once Phase C (build
order) exists, "base" gains real *structural* meaning — the end that
attaches to an already-placed stick (Sec 6.1). A stick auto-flipped here for
reachability could disagree with what Phase C's order algorithm later wants
structurally for the same stick; that reconciliation doesn't exist yet and
will need revisiting when Phase C lands. `WARN_AUTO_FLIPPED` exists partly
so that reconciliation has something concrete to search for.

### What Phase B does NOT check (Phase C)

Jaw clearance / collision against already-placed sticks (Sec 6 C3, Sec 8.2)
needs a real build order to know what "already placed" means. Phase A's
angle-based `tight_clearance` warning is the current proxy for that risk.
Self-collision, the mount platform, and MoveIt's own path planning are never
modelled here regardless — Sec 7's upper-bound disclaimer applies in full.

### The GPU overlay could not be tested under `--background`

`blender --background` has no GPU context — even `gpu.shader.from_builtin`
raises `GPU functions ... requires the gpu module to be initialized` there,
so the automated suite cannot exercise `ui/overlay.py` at all (confirmed,
not assumed). It was instead smoke-tested with a real, windowed Blender
instance on this machine's actual display: register through the standard
`addon_enable`/`addon_disable` operators (not called directly, to catch
anything the real lifecycle would), extract a cube, force real redraws with
the overlay on, off, and with no base empty set, disable, re-enable, redraw
again, disable — zero errors throughout. Still worth a look in your own
session (`Ctrl+Alt+R`, toggle **Show Overlay** in the Design panel) since
this class of code (draw handlers) is explicitly called out in the plan as
the most common cause of addon crashes.

---

## What Phase C delivers

- `core/order.py` — Sec 6's greedy topological build with backtracking:
  support (C1), bottom-up + far-side-first (C2), jaw clearance (C3), the
  Sec 6.2 warnings, and a chunkable `OrderSolver.step()`
- `ops/order.py` — **Compute Build Order** / **Clear Build Order**, with a
  chunked modal path for interactive use and a synchronous path for scripts
- `ui/panels.py` — a `Plan` panel (Sec 10.2) and build-order sorting in the
  stick list
- `tests/test_order.py` — 40 tests

Phase C's "done when" — *a multi-layer test mesh produces an order that is
support-valid and fully buildable, and a deliberately-floating component is
correctly rejected* — is `TestMultiLayerTower` and `TestFloatingComponent`.
Support-validity is checked *structurally* (replay the order and assert
every base was grounded or established by an earlier stick), not by trusting
the solver's own bookkeeping.

### "base" means something different in Phase C than in Phase B

Sec 5.1's "base = the lower end" is a default; Sec 6 makes it **structural**
— the end that seats on the plate or on an already-placed stick. So the
order solver decides orientation from the support graph, superseding
extraction's reachability-driven auto-flip (Finding 6). This is the tension
flagged when auto-flip was added, now resolved:

- **One end supported** → structure wins; flipping would leave it floating.
- **Both ends supported** (a loop closure, or the inverted-U's top once both
  uprights are up) → the solver is free, and picks whichever orientation
  actually validates — reproducing auto-flip's benefit with no conflict.

Re-running **Extract Sticks** clears the order, so flips fall back to the
reachability heuristic until it is recomputed. No lasting inconsistency, but
**Compute Build Order must be re-run after any re-extraction** — which the
staleness check already surfaces. (Compute Build Order re-extracts
internally, so it also works as a standalone first action.)

### Jaw clearance is a *soft* constraint, deliberately

C3's numbers are estimates, not measurements: Sec 8.2 gives "~20 mm across"
and "clear out to ~60 mm", and Phase 0 is still due to measure the real jaw
envelope. So a clash steers the search (the solver prefers orders that avoid
it) but never refuses to build: if no order avoids it, the stick is placed
anyway with a `jaw_clearance` warning naming the offender and the gap. Only
*reachability* hard-gates a candidate. This matches the permissive policy
agreed for Phase B — MoveIt does the real collision check on load.

Two modelling choices worth knowing, both deliberately conservative:

- The jaws are modelled as a **cylinder around the stick axis**, not a box.
  The real jaws are a box whose orientation depends on `stick_roll`, and
  that roll convention is still unverified on hardware — so assuming the
  worst orientation is the honest choice.
- The **base plate is not modelled**. A horizontal stick lying on the plate
  has its grip point at plate height, so a symmetric cylinder would report a
  clash for every flat first-layer stick — yet physically the jaws come down
  from above and sit over it. The plate is a static collision object on the
  ROS2 side (Sec 10) and MoveIt checks it properly.

### A found-and-fixed bug: warnings surviving backtracking

Warnings were appended to the result as each stick was placed — but
backtracking un-places sticks, and the appended warnings stayed. A 12-stick
tower reported **109 warnings**. Per-placement messages now live on the
`OrderedStick` and the result's lists are rebuilt from the surviving order in
`_finalize()`, so a backtrack takes its warnings with it. `TestChunkedExecution`
locks this in by asserting a chunked solve produces byte-identical warnings
to a one-shot one.

### A second one: backtracking over a stick that can never be placed

Phase B reachability depends only on a stick's own geometry, not on what is
already placed (only jaw clearance is scene-dependent). So if *both*
orientations of a stick fail, **no order can rescue it** and backtracking is
pure waste — a 4-level ladder whose top is genuinely out of reach burned the
entire 500-backtrack budget before giving up. `OrderSolver._is_hopeless()`
detects that (lazily, cached, so it costs nothing until something fails) and
force-places with a recorded error instead of thrashing: 500 backtracks → 0.

Force-placing rather than abandoning the build is deliberate:
`BRIDGE_PROTOCOL.md` A.2 requires that a stick which fails validation still
appears in the file, in order, so the operator sees the whole picture.

### Workspace reality found while building the test fixtures

A square-ring tower is **not** a valid "fully buildable" fixture: at every
height and stick length tried, exactly its two *radial* rungs (the ones
pointing at/away from the robot) come out unreachable, because a radial
horizontal needs the tool vertical, which Sec 9.4 says costs ~150 mm of
reach. The fixtures use a *ladder* (stacked inverted-Us, all-tangential
horizontals) instead. Worth knowing when designing real sculptures: **tangential
horizontals are cheap, radial ones are expensive.**

### The chunked modal path needs a real Blender, like the overlay

`blender --background` runs no modal operators and no timers, so the
automated suite exercises the **synchronous** path (`bpy.ops.*()` from a
script uses `EXEC_DEFAULT`). The chunked modal path was smoke-tested in a
windowed instance: invoke returns `RUNNING_MODAL`, the timer drives it to
completion, and the stored order is dense and correct.

Panel drawing needed the same treatment, and a first attempt at it was
*wrong in a way worth recording*: toggling the sidebar with
`space.show_region_ui = True` from a timer **segfaults Blender** (harness
bug, not addon code — use `bpy.ops.screen.region_toggle` instead), and even
once it stopped crashing, the panels still never drew, because a 3D-view
region's active tab is read-only and defaulted to a category that is not
ours. Instrumenting the draw methods showed `{'plan': 0, 'sticks': 0}` —
i.e. the "redraw OK" result was meaningless. The working approach registers
a throwaway panel in the always-visible Properties editor that calls the
real panel draw code, which genuinely exercises `template_list` →
`filter_items` (the actual crash risk, since a malformed permutation
scrambles or crashes rather than erroring). Result: 16 draws each of Plan and
Sticks, 16 `filter_items` calls, zero errors, across sorted / unsorted /
cleared. The permutation maths is additionally unit-tested directly.

---

## What Phase D delivers

- `io/build_file.py` — the build file and its status sidecar,
  **BRIDGE_PROTOCOL.md Part A** exactly. Pure Python: the format contract is
  the thing most worth testing without Blender in the way
- `ops/build.py` — **Export Build File**, **Export Cut List** (now with the
  build order and a saw tally), **Sync Status**, **Write Status Sidecar**,
  **Mark Placed/Failed/Skip**, **Reset Build Progress**
- `ui/panels.py` — the `Build` panel (Sec 10.3): next stick to load, its
  length, progress, and the conflict report
- `tests/test_build_file.py` — 44 tests on the format contract alone

Phase D's "done when" ends in *"built on real hardware"*, which cannot be
automated here. What is automated is the half that would make that fail: the
file conforms to Part A (checked by round-tripping through the loader, which
enforces format, version and `order` density), and **reopening after a build
shows the correct placed/pending state** —
`test_sync_restores_progress_from_a_sidecar` writes a sidecar as ROS2 would,
wipes the in-`.blend` progress, syncs, and checks the right sticks come back
`placed`.

### The protocol's own worked example is a test

`TestProtocolWorkedExample` builds a 3-stick vertical stack and asserts the
exported numbers are the ones in A.2: `s_001` at z 0 → 0.110, `s_002` at
0.1165 → 0.2265, a 6.5 mm glue gap between them, `shared_ends` 1 then 2.
Those are the values the protocol document was **corrected to** back in
Phase A, so this test is what keeps the addon and the spec from drifting
apart again. Verified by exporting for real, not just in the unit test.

### Choices worth knowing

- **Key order is the protocol's, not sorted.** `sort_keys=True` would put
  `base` before `id` and make the file read nothing like its spec. Dicts
  preserve insertion order, so the documented order is emitted directly and
  asserted by a test.
- **`order` is renumbered on write, not trusted.** A mismatch between `order`
  and array position would silently desynchronise ROS2's progress reporting
  from the array it iterates.
- **Coordinates are rounded to the micrometre.** A diffable file should not
  churn on float noise, and the `||tip − base|| == length_m` invariant
  tolerates 1 mm — three orders of magnitude of headroom, asserted rather
  than assumed (including for a tilted stick, where all three coordinates
  round at once). The `stock` block needed the same treatment: Blender stores
  it as float32, so 6.45 mm otherwise lands in the file as
  `0.006449999809265136`.
- **Unbuildable sticks are still exported, in order**, per A.2 — so the
  operator sees the complete picture and can decide to build the rest.
- **Pending sticks are omitted from the sidecar.** A.3 says "anything absent
  is pending", so writing them would just be noise in a file whose point is
  to stay small and diffable.
- **A `failed` or `skipped` stick is not skipped over** when computing the
  next stick to load. The operator decides what to do about a failure;
  stepping silently past it would hide exactly what they need to see.
- **An unknown status degrades to `pending` rather than raising.** A newer
  ROS2 may write a state this addon has not heard of, and refusing to open
  the file would be the worse failure.
- **Export re-runs the whole pipeline** (extract → validate → order) rather
  than serialising the PropertyGroup, and then stores what it exported. The
  file must be self-contained and consistent with the settings as they are
  *now*, and the Scene must not be able to disagree with the file about ids,
  order or verdicts.

### `roll_deg` is always 0.0, deliberately

`roll_deg` is spin about the stick's *own* axis. Two independent reasons it
is zero: a wireframe edge carries no such information — the design simply
does not say — and with position fixed this 5-DOF arm has exactly two
orientation DOF, which the stick's *direction* already consumes, so the spin
is not independently commandable anyway. The wrist roll ROS2 actually needs
is derived from `base`/`tip`, not read from this field.

### The grasp-orientation integration gap — closed (2026-07-31)

ROS2 re-validates the build file with the same kinematics module — that
shared module is the entire basis for trusting Blender's verdicts. Turning
`base`/`tip` into the `(tool_elevation_rad, stick_roll_rad)` pair `ik()`
needs used to live only in this addon's `core/validate.py`
(`_solve_orientation`), with the ROS2 side's own Phase 1 checklist marking
that transform "not written yet" — so the two sides shared the *kinematics*
but not the *grasp orientation derivation*, and that derivation is where
three sign/branch bugs were found during Phase B. An independent ROS2-side
reimplementation would have been a realistic way for the two sides to
disagree about where a stick goes — exactly the failure the shared-module
design exists to prevent.

**Resolved:** the transform is now `so_arm_100_kinematics.grasp`
(`grasp_target`, `azimuth_frame`, `solve_stick_orientation`, and a
`solve_stick_placement` convenience for `PlaceStick`'s
target→joints step), version 1.1.0, vendored verbatim into
`so100_builder/kinematics/grasp.py`. `core/validate.py` now calls it instead
of reimplementing it — see "The grasp-orientation transform" above. The
ROS2-side `test/test_grasp.py` reuses the exact numeric regression cases
found during Phase B debugging (the disputed out-of-plane tilt, the 0.71°
and 2.077° near-degenerate elevation cases, the `Wrist_Roll` asymmetric-limit
flip case), and is itself vendored into
`so100_builder/tests/test_kinematics_grasp_vendored.py` so the addon-side
copy is exercised the same way. **Still outstanding: updating the actual
ROS2 workspace's `kinematics/` directory from this new 1.1.0 package** — that
happens outside this repo.

---

## What Phase E delivers

The robot mirror rig (Sec 10.4, QB3): a preview-only rig posed by the shared
FK, scrubbed through the build order — "watch the arm move to each
placement before anything moves." No telemetry, no live robot link; Option C
(Sec 2) never had either, and this preview needs neither.

- `core/mirror.py` — `joint_frames(joint_angles_rad)`, pure Python, no
  `bpy`. Returns the 6 points (`base_link` origin through the true TCP) a
  rig needs to draw the 5-DOF chain as a stick figure.
- `ops/mirror.py` — `SO100_OT_mirror_toggle` / `SO100_OT_mirror_step`, and
  `update_mirror_rig()`, which poses a dedicated, always-non-selectable,
  never-rendered mesh object (`SO100_Mirror`).
- `ui/panels.py` — `SO100_PT_preview` (Sec 10.4's `Preview` panel, new):
  show/hide toggle, prev/next through the build order, a status line.

### `joint_frames()` is not a second FK implementation

Getting the arm's *intermediate* joint positions (elbow, wrist, etc. — not
just the final TCP) sounds like it needs its own forward-kinematics walk
through `CHAIN`, which would mean re-deriving (and risking re-breaking) the
same rotation-composition math `chain.py` already has, exactly the
duplication the vendoring rules exist to prevent.

It doesn't. `fk(joint_angles_rad)`'s own loop is
`for (...), q in zip(CHAIN, joint_angles_rad)` — so calling it with a
**shorter** prefix of the joint angles makes it stop exactly there, and the
position/rotation it has accumulated at that point *is* the correct
partial-chain frame, no reimplementation needed. The one wrinkle:
`fk()`'s only postprocessing step, `pos += rot @ EE_OFFSET`, runs
unconditionally after the loop — correct for the full 5-joint chain (that
offset is real, the gripper's own reach past `Wrist_Roll`), wrong for a
shorter prefix (there is no gripper yet at joint 2's frame). So
`joint_frames()` calls `fk()` once per prefix length and subtracts that
offset back out for every prefix but the last, using nothing but `fk()`'s
own public output and the public `EE_OFFSET` constant.

**Verified, not just argued**: every consecutive pair of points is a rigid
link, so its length must be identical across every pose, reachable or not.
`test_mirror.py` checks that segment-length invariant against 5 named poses
(including the extreme `home`/`lower`/`place` tuned poses) and 200 random
joint-space samples — a bug in the `EE_OFFSET` correction would show up
immediately as a length that moves with the pose, not as a subtle numeric
drift.

### Where a stick's base/tip come from for the rig

`ops/mirror.py` deliberately reads a stick's placement back off the
**build mesh** (converted `blender_to_robot`), not off the design — the
build mesh already reflects any `flip` (Sec 6 makes "base" structural once
an order exists, superseding Phase B's reachability-driven default). The
resulting `(base, tip)` goes straight into
`kinematics.grasp.solve_stick_placement()` — the *exact* function
`PlaceStick` calls on the ROS2 side (Sec 7.2) — so the rig shows the same
joint solution the robot would actually use, not an independently-derived
approximation of it.

### Tested three ways, matching Sec 12's own layering

1. `core/mirror.py` against the vendored `fk()` directly (the invariant
   above) — no Blender needed.
2. `ops/mirror.py`'s operators executed for real inside Blender
   (`test_blender_integration.py::TestMirrorRig`): toggle show/hide, wrap-
   around stepping, object cleanup on `unregister()`, and a geometric check
   that the rig's TCP point lands within 1 mm of the stick's own
   `grasp_target()` — proving the rig is posed *at* the stick, not just
   that something got drawn.
3. The new panel's `draw()` itself, smoke-tested in a real **windowed**
   Blender instance — `blender --background` cannot fire panel draw
   callbacks at all (Phase C's own finding), so a typo in a `layout.prop()`
   or `operator()` call is invisible to every other layer. Registered a
   throwaway Properties-editor panel that calls `SO100_PT_preview.draw()`
   directly (the always-visible-tab trick from Phase C), toggled the mirror
   on and stepped it once first so the fuller draw path (prev/next buttons,
   status line) actually executes, and confirmed one clean call.
