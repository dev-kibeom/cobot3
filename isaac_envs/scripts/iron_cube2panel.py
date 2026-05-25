"""Iron Cube -> Iron Panel transformation behavior.

Attach this script to the cube prim via the Property panel's
"Python Scripting" component.

When the cube prim's world-space XY position enters the configured zone,
the cube is hidden and a panel prim is spawned at the cube's current
world pose (as a sibling). When the simulation Stop button is pressed,
the panel is removed and the cube is restored, so the demo can be replayed.

The script reads these USD attributes on the prim (create them via the
Property panel's "+ Add -> Attribute" menu if they don't exist):

  - zoneCenter        : vector3d   default (0.5, 0.0, 0.0)
        World-space XY position of the *center* of the trigger zone, in
        meters. Z is ignored; the zone is a 2D footprint, infinite in
        height, so the cube triggers regardless of how high it is.
        Example: (0.5, 0.0, 0.0) puts the zone center 0.5 m in front of
        the world origin along +X.

  - zoneHalfExtents   : vector3d   default (0.1, 0.1, 0.0)
        Half-width (X) and half-depth (Y) of the trigger zone, in meters.
        Z is ignored. The actual zone size is 2 * halfExtents on each
        axis, so the default (0.1, 0.1, _) gives a 0.2 x 0.2 m box.
        The cube triggers when:
            |cube.x - zoneCenter.x| <= zoneHalfExtents.x  AND
            |cube.y - zoneCenter.y| <= zoneHalfExtents.y

  - panelAsset        : asset      default = ../assets/iron_panel.usd
        The USD asset that gets spawned when the cube enters the zone.

Targets Isaac Sim 5.1 (omni.kit.scripting.BehaviorScript).
"""
import os

import carb
from omni.kit.scripting import BehaviorScript
from pxr import Gf, Usd, UsdGeom


_DEFAULT_PANEL_PATH = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "assets", "iron_panel.usd")
)

_PANEL_SUFFIX = "__AsPanel"


class IronCube2Panel(BehaviorScript):

    def on_init(self):
        self._transformed = False
        carb.log_info(
            f"[iron_cube2panel] attached to {self.prim_path}; "
            f"default panel = {_DEFAULT_PANEL_PATH}"
        )

    def on_play(self):
        # Clean slate every Play so testing is repeatable.
        self._remove_panel_sibling()
        self._set_cube_visible(True)
        self._transformed = False

    def on_stop(self):
        # Restore the cube and remove the spawned panel for the next run.
        self._remove_panel_sibling()
        self._set_cube_visible(True)
        self._transformed = False

    def on_update(self, current_time, delta_time):
        if self._transformed:
            return
        if self._in_zone():
            self._swap_to_panel()
            self._transformed = True

    # --- swap logic ------------------------------------------------------
    def _swap_to_panel(self) -> None:
        panel_path = self._get_asset_path("panelAsset", _DEFAULT_PANEL_PATH)
        if not panel_path:
            carb.log_error("[iron_cube2panel] panelAsset is empty")
            return
        if "://" not in panel_path and not os.path.isfile(panel_path):
            carb.log_error(f"[iron_cube2panel] panel USD not found: {panel_path}")
            return

        # Snapshot the cube's current world transform (post-physics).
        world_tf = UsdGeom.Xformable(self.prim).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default()
        )

        # Hide the cube. Visibility is reversible on Stop, so the cube prim
        # (and its physics body) survive for the next run.
        self._set_cube_visible(False)

        # Spawn the panel as a sibling at the cube's world pose.
        sibling_path = self._panel_sibling_path()
        panel_prim = self.stage.DefinePrim(sibling_path, "Xform")
        panel_prim.GetReferences().AddReference(panel_path)

        panel_xform = UsdGeom.Xformable(panel_prim)
        panel_xform.ClearXformOpOrder()
        panel_xform.AddTransformOp().Set(Gf.Matrix4d(world_tf))

        carb.log_warn(
            f"[iron_cube2panel] {self.prim_path} hidden; panel spawned at {sibling_path}"
        )

    def _remove_panel_sibling(self) -> None:
        sibling_path = self._panel_sibling_path()
        if self.stage.GetPrimAtPath(sibling_path).IsValid():
            self.stage.RemovePrim(sibling_path)

    def _set_cube_visible(self, visible: bool) -> None:
        imageable = UsdGeom.Imageable(self.prim)
        if not imageable:
            return
        if visible:
            imageable.MakeVisible()
        else:
            imageable.MakeInvisible()

    def _panel_sibling_path(self) -> str:
        parent = self.prim.GetParent().GetPath()
        return f"{parent}/{self.prim.GetName()}{_PANEL_SUFFIX}"

    # --- zone check ------------------------------------------------------
    def _in_zone(self) -> bool:
        xform = UsdGeom.Xformable(self.prim)
        if not xform:
            return False
        world_tf = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        tx, ty, _ = world_tf.ExtractTranslation()

        center = self._get_vec3("zoneCenter", Gf.Vec3d(0.5, 0.0, 0.0))
        half = self._get_vec3("zoneHalfExtents", Gf.Vec3d(0.1, 0.1, 0.0))
        return (
            (center[0] - half[0]) <= tx <= (center[0] + half[0])
            and (center[1] - half[1]) <= ty <= (center[1] + half[1])
        )

    # --- attribute readers ----------------------------------------------
    def _get_vec3(self, attr_name: str, fallback: Gf.Vec3d) -> Gf.Vec3d:
        attr = self.prim.GetAttribute(attr_name)
        if attr and attr.HasValue():
            return attr.Get()
        return fallback

    def _get_asset_path(self, attr_name: str, fallback: str) -> str:
        attr = self.prim.GetAttribute(attr_name)
        if attr and attr.HasValue():
            v = attr.Get()  # Sdf.AssetPath
            if v:
                if hasattr(v, "resolvedPath"):
                    return v.resolvedPath or v.path or fallback
                return str(v) or fallback
        return fallback
