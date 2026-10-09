import builtins
import os
import numpy as np
import omni.usd
from isaacsim.util.debug_draw import _debug_draw

try:
    from omni.isaac.dynamic_control import _dynamic_control
except ImportError:
    from isaacsim.core.dynamics import _dynamic_control

PLY_NAME = "amr_route.ply"


def _output_dir():
    """Where scan/route files go.

    AMR_MAP_OUTPUT_DIR is set by dxf_to_usd.py; when the scene is opened by
    hand instead, fall back to the folder of the open USD file.
    """
    out = os.environ.get("AMR_MAP_OUTPUT_DIR")
    if not out:
        layer = omni.usd.get_context().get_stage().GetRootLayer()
        out = os.path.dirname(layer.realPath) if layer.realPath else os.getcwd()
    os.makedirs(out, exist_ok=True)
    return out

def setup(db: og.Database):
    builtins._path = []
    builtins._path_set = set()
    builtins._path_frame = 0
    builtins._dc_route = _dynamic_control.acquire_dynamic_control_interface()
    db.log_warning("ROUTE: tracking started")

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    dc = builtins._dc_route
    draw = _debug_draw.acquire_debug_draw_interface()

    art = dc.get_articulation("/World/simplerobot")
    if art == _dynamic_control.INVALID_HANDLE:
        return True

    root = dc.get_articulation_root_body(art)
    pose = dc.get_rigid_body_pose(root)
    rx, ry = pose.p.x, pose.p.y

    key = (round(rx / 0.1), round(ry / 0.1))
    if key not in builtins._path_set:
        builtins._path_set.add(key)
        builtins._path.append(key)
        draw.draw_points([(key[0]*0.1, key[1]*0.1, 0.3)],
                         [(0.1, 0.4, 1.0, 1.0)], [8.0])

    builtins._path_frame += 1
    if builtins._path_frame % 500 == 0 and len(builtins._path) > 0:
        p = [(k[0]*0.1, k[1]*0.1, 0.3) for k in builtins._path]
        with open(os.path.join(_output_dir(), PLY_NAME), "w") as f:
            f.write("ply\nformat ascii 1.0\nelement vertex " + str(len(p)) + "\nproperty float x\nproperty float y\nproperty float z\nend_header\n")
            for x, y, z in p:
                f.write(str(round(x, 4)) + " " + str(round(y, 4)) + " " + str(round(z, 4)) + "\n")
        db.log_warning("ROUTE saved: " + str(len(p)) + " pts")

    return True