"""AMR Tools - map2 export and live driving parameter tuning.

Exports map2 straight from the scan log held in memory (builtins._scan_log).
Segment fitting and covariance come from scan_to_map2.py, imported at runtime
so both paths stay in sync.
"""

import builtins
import importlib.util
import math
import os

import omni.ext
import omni.ui as ui
import omni.usd

WINDOW_TITLE = "AMR Tools"

# scan_to_map2.py lives four levels up: exts/amr.tools/amr/tools -> map/
_HERE = os.path.dirname(os.path.abspath(__file__))
_MAP_DIR = os.path.abspath(os.path.join(_HERE, "..", "..", "..", ".."))
_CONVERTER = os.path.join(_MAP_DIR, "scan_to_map2.py")

_OUT_DIR = os.path.join(_MAP_DIR, "output")
DEFAULT_OUT = os.path.join(_OUT_DIR, "scanned.map2")

# Driving parameters, read live by script_node/amr_driving_script_node.py
# through builtins. Keep the defaults in step with DEFAULTS in that file.
TURN_IN_PLACE = ("amr_turn_in_place", True)

# (group, [(builtins key, label, default, min, max), ...])
PARAM_GROUPS = [
    ("Drive", [
        ("amr_max_speed",     "max speed (m/s)",        1.5,  0.1,  4.0),
        ("amr_accel",         "accel / brake (m/s2)",   1.0,  0.1,  5.0),
        ("amr_heading_gain",  "line-follow gain",       1.5,  0.0,  5.0),
    ]),
    ("Turn in place", [
        ("amr_max_turn_rate", "max turn rate (rad/s)",  0.8,  0.1,  3.0),
        ("amr_turn_gain",     "turn gain",              2.0,  0.5,  8.0),
        ("amr_min_turn_rate", "min turn rate (rad/s)",  0.15, 0.0,  1.5),
        ("amr_align_tol_deg", "align tolerance (deg)",  1.5,  0.2, 10.0),
        ("amr_realign_deg",   "re-align above (deg)",   8.0,  2.0, 45.0),
    ]),
    ("Arrival", [
        ("amr_pos_tol",       "position tolerance (m)", 0.05, 0.01, 0.5),
        ("amr_settle_time",   "settle time (s)",        0.3,  0.0,  2.0),
        ("amr_arrive_dist",   "smooth: switch at (m)",  1.2,  0.2,  3.0),
    ]),
    ("Safety", [
        ("amr_obstacle_dist", "obstacle stop (m)",      1.5,  0.3,  5.0),
    ]),
    ("Robot", [
        ("amr_wheel_radius",  "wheel radius (m)",       0.5,  0.05, 1.0),
        ("amr_wheel_base",    "wheel base (m)",         1.25, 0.2,  2.5),
    ]),
]
PARAMS = [row for _, rows in PARAM_GROUPS for row in rows]


def _load_converter():
    """Load scan_to_map2.py as a module."""
    if not os.path.exists(_CONVERTER):
        raise FileNotFoundError(_CONVERTER)
    spec = importlib.util.spec_from_file_location("_scan_to_map2", _CONVERTER)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class AmrToolsExtension(omni.ext.IExt):

    def on_startup(self, ext_id):
        self._window = ui.Window(WINDOW_TITLE, width=460, height=620)
        self._out_model = ui.SimpleStringModel(DEFAULT_OUT)
        self._status = None
        self._param_models = {}

        # Seed defaults so the script node works whether or not this
        # extension is enabled - it reads via getattr(builtins, key, default).
        for key, _, default, _, _ in PARAMS:
            if not hasattr(builtins, key):
                setattr(builtins, key, default)
        key, default = TURN_IN_PLACE
        if not hasattr(builtins, key):
            setattr(builtins, key, 1.0 if default else 0.0)
        self._turn_model = None

        self._build_ui()
        self._dock_to_stage()

    def _dock_to_stage(self):
        """Dock next to the Stage panel as a tab."""
        import asyncio
        import omni.kit.app

        async def dock():
            # Stage window may not exist yet at startup
            for _ in range(20):
                await omni.kit.app.get_app().next_update_async()
                stage_win = ui.Workspace.get_window("Stage")
                if stage_win:
                    self._window.dock_in(stage_win, ui.DockPosition.SAME)
                    self._window.focus()
                    return

        asyncio.ensure_future(dock())

    def on_shutdown(self):
        if self._window:
            self._window.destroy()
            self._window = None

    # ------------------------------------------------------------------ UI

    def _build_ui(self):
        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6, height=0):
                    with ui.CollapsableFrame("Map2 Export", height=0):
                        self._build_export_tab()
                    with ui.CollapsableFrame("AMR Params", height=0):
                        self._build_params_tab()

    def _build_export_tab(self):
        with ui.VStack(spacing=6, height=0):
            ui.Spacer(height=2)
            with ui.HStack(height=24, spacing=6):
                ui.Label("Output", width=60)
                ui.StringField(model=self._out_model)

            ui.Button("Export map2", height=32, clicked_fn=self._on_export)

            self._status = ui.Label("Drive first, then press Export.",
                                    word_wrap=True)
            ui.Spacer(height=2)

    def _build_params_tab(self):
        with ui.VStack(spacing=4, height=0):
            ui.Spacer(height=2)

            key, _ = TURN_IN_PLACE
            with ui.HStack(height=22, spacing=6):
                ui.Label("turn in place", width=150)
                self._turn_model = ui.SimpleBoolModel(
                    getattr(builtins, key, 1.0) >= 0.5)
                ui.CheckBox(model=self._turn_model, width=20)
                ui.Label("off = smooth (cuts corners)",
                         style={"color": 0xFF888888})
                self._turn_model.add_value_changed_fn(
                    lambda m, k=key: setattr(
                        builtins, k, 1.0 if m.get_value_as_bool() else 0.0))

            for group, rows in PARAM_GROUPS:
                ui.Spacer(height=4)
                ui.Label(group, height=18, style={"color": 0xFFB0B0B0})
                for key, label, default, lo, hi in rows:
                    with ui.HStack(height=22, spacing=6):
                        ui.Label(label, width=150)
                        model = ui.SimpleFloatModel(
                            getattr(builtins, key, default))
                        ui.FloatSlider(model=model, min=lo, max=hi)
                        model.add_value_changed_fn(
                            lambda m, k=key: setattr(
                                builtins, k, float(m.get_value_as_float())))
                        self._param_models[key] = model

            ui.Spacer(height=4)
            ui.Button("Reset to defaults", height=26,
                      clicked_fn=self._on_reset)
            ui.Label("Applied live while driving.", word_wrap=True)
            ui.Spacer(height=2)

    # -------------------------------------------------------------- actions

    def _on_reset(self):
        for key, _, default, _, _ in PARAMS:
            setattr(builtins, key, default)
            if key in self._param_models:
                self._param_models[key].set_value(default)
        key, default = TURN_IN_PLACE
        setattr(builtins, key, 1.0 if default else 0.0)
        if self._turn_model is not None:
            self._turn_model.set_value(default)

    def _on_export(self):
        try:
            path = self._out_model.get_value_as_string().strip()
            if not path:
                self._set_status("Enter an output path.")
                return

            scans = getattr(builtins, "_scan_log", None)
            if not scans:
                self._set_status(
                    "No scan data. Press Play and let the lidar run, "
                    "then try again.")
                return

            conv = _load_converter()
            self._set_status(self._export(conv, scans, path))

        except FileNotFoundError as exc:
            self._set_status("scan_to_map2.py not found: " + str(exc))
        except Exception as exc:      # keep the UI alive on any failure
            self._set_status("Failed: " + str(exc))

    def _export(self, conv, scans, out_path):
        import numpy as np

        offset = getattr(builtins, "_lidar_off", (0.0, 0.0))

        # Same transform scan_to_map2.load_scans does for the NPZ, applied to
        # the in-memory log: pose + local points -> world frame.
        world_scans = []
        for rx, ry, yaw, local in scans:
            if len(local) == 0:
                continue
            pts = np.asarray(local, dtype=np.float64).reshape(-1, 2)
            cos_y, sin_y = math.cos(yaw), math.sin(yaw)

            ox = rx + offset[0] * cos_y - offset[1] * sin_y
            oy = ry + offset[0] * sin_y + offset[1] * cos_y

            lx = pts[:, 0] + offset[0]
            ly = pts[:, 1] + offset[1]
            wx = lx * cos_y - ly * sin_y + rx
            wy = lx * sin_y + ly * cos_y + ry

            world = np.column_stack([wx, wy])
            world_scans.append({
                "origin": np.array([ox, oy]),
                "world": world,
                "ranges": np.hypot(world[:, 0] - ox, world[:, 1] - oy),
            })

        if not world_scans:
            return "Scan log holds no usable points."

        points, _, point_scan = conv.flatten(world_scans)
        origins = np.array([s["origin"] for s in world_scans])

        segments = conv.extract_segments(points)
        if not segments:
            return "No segments could be fitted. Drive further and retry."

        weak = 0
        for seg in segments:
            cov, views, incidence = conv.segment_covariance(
                seg, points, origins, point_scan)
            seg["cov"] = cov
            seg["viewpoints"] = views
            seg["incidence"] = incidence
            if views < 3 or incidence < 0.35:
                weak += 1

        nodes = self._waypoints_from_stage()

        parent = os.path.dirname(os.path.abspath(out_path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        conv.write_map2(out_path, segments, nodes)

        msg = ("Saved " + os.path.basename(out_path) + "\n"
               + str(len(world_scans)) + " scans / "
               + str(len(segments)) + " segments / "
               + str(len(nodes)) + " nodes")
        if weak:
            msg += ("\n" + str(weak) + " weakly observed segment(s) - "
                    "re-drive those areas.")
        if not nodes:
            msg += "\nNo waypoints found (/World/Environment/Waypoints)."
        return msg

    def _waypoints_from_stage(self):
        """Read waypoints from the open stage."""
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return []

        scope = None
        for path in ("/World/Environment/Waypoints", "/Environment/Waypoints"):
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                scope = prim
                break
        if scope is None:
            return []

        nodes = []
        for child in scope.GetChildren():
            matrix = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(0)
            t = matrix.ExtractTranslation()
            rot = matrix.ExtractRotationMatrix()
            heading = math.degrees(math.atan2(rot[0][1], rot[0][0]))

            node_id = -1
            name = child.GetName()
            links = []

            attr = child.GetAttribute("ant:nodeId")
            if attr and attr.IsValid() and attr.Get() is not None:
                node_id = int(attr.Get())
            attr = child.GetAttribute("ant:name")
            if attr and attr.IsValid() and attr.Get() is not None:
                name = str(attr.Get())
            attr = child.GetAttribute("ant:linksTo")
            if attr and attr.IsValid() and attr.Get() is not None:
                links = [int(v) for v in attr.Get()]

            nodes.append({"id": node_id, "name": name,
                          "x": float(t[0]), "y": float(t[1]),
                          "heading": heading, "links": links})

        nodes.sort(key=lambda n: n["id"])
        return nodes

    def _set_status(self, text):
        if self._status:
            self._status.text = text
        print("[AMR Tools] " + text.replace("\n", " | "))