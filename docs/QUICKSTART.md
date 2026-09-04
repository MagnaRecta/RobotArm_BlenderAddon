# RA130 Stick Builder -- Quick Start

Turn a wireframe mesh into a set of cut sticks a robot can glue together.

## 1. Install

1. Blender **5.2 LTS or newer** (uses the Extensions platform, not `bl_info`).
2. Zip the `so100_builder/` folder (or use the `.zip` if you already have one).
3. `Edit > Preferences > Get Extensions > (dropdown) > Install from Disk...` and pick the zip.
4. A new tab called **RA130** appears in the 3D Viewport sidebar (press `N` to open it).

## 2. Set up your scene

In the **Design** panel:

1. **Robot** -- pick the target robot.
2. **Base** -- adds the empty that marks the robot's origin. Move/rotate it to where the robot actually sits.
3. **Mesh** -- pick your wireframe mesh (every edge = one stick).
4. Check **Stock** (stick cross-section) and **Stick Length** settings, or leave the defaults.
5. Under **Mesh Expansion**, if `Require Build Plate` is on, use **Drop to Build Plate** to sit the mesh on the plate before extracting.

## 3. Extract sticks

Click **Extract Sticks**. This grows the design so fixed-length sticks fit with a glue gap at every joint, and lists each stick with its length, status, and any warnings.

- The trash icon next to it clears results (and any generated build mesh) so you can re-extract from scratch.
- Toggle **Show Overlay** to see the build volume / base box in the viewport.


## 4. Compute a build order

In the **Plan** panel, click **Compute Build Order**. This validates every placement against the robot's real kinematics and reports errors/warnings.

- In **Sticks**, once a stick has a build position, use **Move Earlier / Move Later** to manually reorder it.

## 5. Inspect sticks

In the **Sticks** panel, select a row to see its length, residual, and any warnings.

- **Check By Eye** frames the viewport on that stick's edge and highlights it.
- **Highlight Previous Sticks** (checkbox above Check By Eye) also dims-highlights every stick placed before it, so you can see build progression at a glance.
- Left/right arrows step to the previous/next stick.



## 6. Export and build

In the **Build** panel:

1. **Export Build File** writes the file the separate ROS2 process reads.

## Tips

- Full technical details live in `BLENDER_ADDON_PLAN.md` and `STATUS.md` in this folder.
