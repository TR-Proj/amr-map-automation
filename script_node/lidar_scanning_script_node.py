import builtins
import os
import numpy as np
import omni.usd
from pxr import UsdGeom
from isaacsim.sensors.physx import _range_sensor
from isaacsim.util.debug_draw import _debug_draw

try:
    from omni.isaac.dynamic_control import _dynamic_control
except ImportError:
    from isaacsim.core.dynamics import _dynamic_control

NPZ_NAME = "scan_log.npz"


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
    builtins._pc_map = set()
    builtins._scan_log = []      # (pose_x, pose_y, yaw, points_local)
    builtins._lidar_frame = 0
    builtins._dc_lidar = _dynamic_control.acquire_dynamic_control_interface()
    _debug_draw.acquire_debug_draw_interface().clear_points()

    stage = omni.usd.get_context().get_stage()
    robot_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/simplerobot"))
    lidar_xf = UsdGeom.Xformable(stage.GetPrimAtPath("/World/simplerobot/front_sensor/Lidar"))
    rel = lidar_xf.ComputeLocalToWorldTransform(0) * robot_xf.ComputeLocalToWorldTransform(0).GetInverse()
    off = rel.ExtractTranslation()
    builtins._lidar_off = (float(off[0]), float(off[1]))
    db.log_warning("LIDAR offset: " + str(builtins._lidar_off))

def cleanup(db: og.Database):
    pass

def compute(db: og.Database):
    lidar_if = _range_sensor.acquire_lidar_sensor_interface()
    draw = _debug_draw.acquire_debug_draw_interface()
    dc = builtins._dc_lidar

    art = dc.get_articulation("/World/simplerobot")
    if art == _dynamic_control.INVALID_HANDLE:
        return True

    root = dc.get_articulation_root_body(art)

    avel = dc.get_rigid_body_angular_velocity(root)
    if abs(float(avel.z)) > 0.3:
        builtins._lidar_frame += 1
        return True

    pose = dc.get_rigid_body_pose(root)
    rx, ry = pose.p.x, pose.p.y
    q = pose.r
    yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    pc = lidar_if.get_point_cloud_data("/World/simplerobot/front_sensor/Lidar")

    if pc is not None and len(pc) > 0:
        pts = np.array(pc, dtype=np.float64).reshape(-1, 3)

        # record raw scan with pose (every 10th frame to limit size)
        if builtins._lidar_frame % 10 == 0:
            builtins._scan_log.append((float(rx), float(ry), float(yaw),
                                       pts[:, :2].astype(np.float32)))

        cos_y = np.cos(yaw)
        sin_y = np.sin(yaw)
        ox, oy = builtins._lidar_off
        lx = pts[:, 0] + ox
        ly = pts[:, 1] + oy
        wx = lx * cos_y - ly * sin_y + rx
        wy = lx * sin_y + ly * cos_y + ry

        new_pts = []
        for i in range(len(wx)):
            key = (round(wx[i] / 0.05), round(wy[i] / 0.05), 0)
            if key not in builtins._pc_map:
                builtins._pc_map.add(key)
                new_pts.append((key[0] * 0.05, key[1] * 0.05, 0.0))

        if new_pts:
            draw.draw_points(new_pts,
                             [(0.9, 0.9, 0.9, 1.0)] * len(new_pts),
                             [3.0] * len(new_pts))

    builtins._lidar_frame += 1
    if builtins._lidar_frame % 500 == 0 and len(builtins._scan_log) > 0:
        poses = np.array([(s[0], s[1], s[2]) for s in builtins._scan_log], dtype=np.float32)
        counts = np.array([len(s[3]) for s in builtins._scan_log], dtype=np.int32)
        allpts = np.concatenate([s[3] for s in builtins._scan_log], axis=0)
        np.savez_compressed(os.path.join(_output_dir(), NPZ_NAME),
                            poses=poses,
                            counts=counts,
                            points=allpts,
                            lidar_offset=np.array(builtins._lidar_off, dtype=np.float32))
        db.log_warning("SCAN LOG saved: " + str(len(poses)) + " scans, " + str(len(allpts)) + " pts")

    return True