# isaac_envs

NVIDIA Isaac Sim environments for the smart-factory project. Scope of work is **this folder only** — do not touch siblings like `ros2_ws/` unless explicitly asked.

## Layout

- `assets/` — USD scene assets (robots, parts, mobile base). Binary; do not hand-edit.
  - `m0609_rg2.usd`, `m0609_rg2_d455.usd` — Doosan M0609 arm + OnRobot RG2 gripper (with/without D455 camera).
  - `jetbot.usd` — JetBot mobile base.
  - `iron_cube.usd`, `iron_panel.usd` — task objects.
- `scripts/` — Python scripts that load assets and assemble scenes in Isaac Sim.
  - `test_scene.py` — reference example: spawns ground, JetBot, and Doosan+RG2 with proper articulation roots.
  - `iron_cube2panel.py` — `BehaviorScript` attached to the cube prim. When the cube's world XY enters a configurable zone, hides the cube and spawns `iron_panel.usd` as a sibling at the cube's pose. Reverts on Stop so the demo is replayable.

## Runtime: Isaac Sim Python only

These scripts import `isaacsim.*`, `omni.usd`, `omni.graph.core`, and `pxr`. They **cannot run under system Python**. Run them via:

- VSCode "Isaac Sim VS Code Edition" / integrated extension (preferred — per project README).
- Or Isaac Sim's bundled Python launcher (`python.sh` from the Isaac Sim install).

If a script silently fails on import, the user is almost certainly running stock Python — say so before debugging logic.

## Conventions seen in the codebase

- **Import order matters in Isaac Sim scripts** (project README). Keep `omni.*` and `isaacsim.*` imports in the order shown in `test_scene.py`; do not reorder for style.
- **Asset paths** are built from `os.path.expanduser("~")` joined with `smart_factory_project/isaac_envs/assets/<file>.usd`. Keep this pattern; do not hardcode `/home/rokey/...`.
- **Articulation root rules** (see `test_scene.py:71–86`): exactly one `UsdPhysics.ArticulationRootAPI` per robot, applied to the robot body prim (e.g. `/World/Doosan/m0609`). Strip the API from the parent group prim and from the gripper child prim, otherwise physics simulation breaks.
- **Prim path discovery** uses `find_prim_path_by_name(root_path, link_name)` (walks `Usd.PrimRange`). Reuse this helper rather than hardcoding nested paths — USD hierarchy can shift when assets are re-exported.
- **Gripper control** uses `ParallelGripper` with joints `finger_joint` and `right_inner_knuckle_joint`; closed positions `[0.7, -0.7]`, opened `[0.0, 0.0]`. Mirror these when adding new gripper code unless the user changes the gripper asset.
- Comments and log messages in existing code are in **Korean**. Match the existing language when editing nearby code; don't translate working comments.

## Assets workflow

- `assets.zip` lives on the team Drive (per project README). If `assets/` is missing USDs, ask the user to extract it — do not regenerate from scratch.
- **Isaac Sim's Assembler is broken on this project** (per README). When composing a new robot+gripper, assemble manually inside Isaac Sim and export as a single `.usd`, then load via `add_reference_to_stage`. Don't try to wire it up at runtime from separate URDFs.

## When making changes

- Test scripts by running them in Isaac Sim and watching the GUI — there is no headless test harness here.
- Don't add a `requirements.txt` or `setup.py` to this folder; dependencies come from the Isaac Sim Python env, not pip.
- Keep new scenes/scripts under `scripts/` and new USDs under `assets/`. Don't introduce new top-level folders without asking.

# Reference for scripting
- https://docs.isaacsim.omniverse.nvidia.com/5.1.0/index.html
- the scripting here must follow the version 5.1 of isaac sim

## `omni.kit.scripting` (BehaviorScript) — per-prim Python behaviors

Scripts that live on a prim and react to the timeline. Used by `iron_cube2panel.py`. Verified working on this project's Isaac Sim 5.1 install — note the caveats below, they bit us.

### Class skeleton

```python
from omni.kit.scripting import BehaviorScript

class MyBehavior(BehaviorScript):
    def on_init(self):          ...   # script loaded onto prim
    def on_destroy(self):       ...   # script removed
    def on_play(self):          ...   # ▶ pressed
    def on_pause(self):         ...   # ‖ pressed
    def on_stop(self):          ...   # ■ pressed
    def on_update(self, current_time, delta_time):  ...   # every frame while playing
```

Attributes available on `self`:
- `self.prim_path` — `Sdf.Path` of the prim the script is attached to.
- `self.prim` — the `Usd.Prim`.
- `self.stage` — the `Usd.Stage`.

### GUI workflow to attach a script to a prim

1. Select the prim in the Stage panel.
2. Property panel → **+ Add** → **Python Scripting**.
3. In the new "Python Scripting" section → **+ Add Script** → browse to the `.py` file.
4. **Save the stage** — the attachment is stored as USD metadata on the prim and travels with the scene file.
5. ▶ Play to load and run the script.

### Per-prim parameters: prefer "+ Add → Attribute" over `VARIABLES_TO_EXPOSE`

The documented `VARIABLES_TO_EXPOSE` list (which is supposed to auto-create typed USD attributes on the prim and render editors in the Property panel) **does not work in this build's Isaac Sim 5.1** — the attributes never appear in the Property panel even after Play, and the Console shows no useful error.

Workaround that does work: **create the attributes manually** via Property panel → **+ Add → Attribute**, then read them in the script with `self.prim.GetAttribute(name)` and a fallback default. This is the pattern used in `iron_cube2panel.py` (`_get_vec3`, `_get_asset_path`).

When creating attributes by hand:
- For Vec3 attrs: use type **`vector3d`** (matches `Gf.Vec3d` returned by `attr.Get()`).
- For asset/file attrs: use type **`asset`** — the field then accepts drag-and-drop from the Content browser. `attr.Get()` returns an `Sdf.AssetPath`; use `.resolvedPath` first (handles `omniverse://` and relative paths), fall back to `.path`.
- Attribute names are **case-sensitive** and must match the script's `GetAttribute("...")` strings exactly.

### Reference swap vs. visibility swap — pick visibility

`prim.GetReferences().ClearReferences() + AddReference(other.usd)` looks like the obvious way to "turn one asset into another", but in practice the new asset composes **on top of** the existing prim (often appearing as a child Xform) instead of replacing it — depends on which layer authored the original reference. Don't rely on this.

Robust pattern for "prim A becomes asset B":
1. Snapshot A's world transform with `UsdGeom.Xformable(A).ComputeLocalToWorldTransform(...)`.
2. Hide A with `UsdGeom.Imageable(A).MakeInvisible()`.
3. `stage.DefinePrim("<A's parent>/<name>__Suffix", "Xform")`, add B as a reference on it, and set its transform op to the snapshot.

Reversible on Stop: `MakeVisible()` on A and `stage.RemovePrim(sibling)` in `on_stop` — gives repeatable demos.

### Repeatable testing

Always reset state in **both** `on_play` and `on_stop` (don't rely on only one — users sometimes start a fresh Play without a prior Stop). A latch boolean for "I already did the one-shot action this run" lives on the instance and gets reset in both hooks.

### IDE warnings are noise

Pylance / system Python can't resolve `carb`, `omni.kit.scripting`, or `pxr` — those only exist in Isaac Sim's bundled Python. The "imports cannot be resolved" warnings on BehaviorScript files are expected; don't try to "fix" them by adding pip dependencies. Same for the `current_time` / `delta_time` "unused argument" hints on `on_update` — those args are part of the framework-required signature.
