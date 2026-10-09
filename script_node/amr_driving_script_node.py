"""amr_driving - drives the waypoint graph.

Default motion is point-to-point: turn in place to face the next node, drive
a straight line to it, stop on the node within a set tolerance, settle, then
turn in place again. The older smooth mode (switch target early and steer
while moving, which cuts corners) is still available as an option.

Follows ant:linksTo rather than cycling the waypoint list in order. A fixed
cycle only ever traverses the edges that happen to sit next to each other in
the list - on this map that leaves the corridor between the two structures
(3 <-> 6) untravelled, which is exactly where the walls that end up with poor
covariance would be seen head-on. Choosing the least-travelled link instead
covers every edge and keeps the visits evenly spread.

Tuning values come from builtins so the AMR Tools sliders apply live; the
defaults below are used when the extension is not loaded.
"""

import builtins
import math

import numpy as np
import omni.timeline
import omni.usd
from pxr import UsdGeom
from isaacsim.sensors.physx import _range_sensor

try:
    from omni.isaac.dynamic_control import _dynamic_control
except ImportError:
    from isaacsim.core.dynamics import _dynamic_control

ROBOT = "/World/simplerobot"
LIDAR = "/World/simplerobot/front_sensor/Lidar"
LEFT_JOINTS = ("front_left_joint", "rear_left_joint")
RIGHT_JOINTS = ("front_right_joint", "rear_right_joint")

# Defaults - keep in step with PARAM_GROUPS in exts/amr.tools extension.py
DEFAULTS = {
    "amr_turn_in_place": 1.0,      # 1 = turn in place + stop on node, 0 = smooth
    "amr_max_speed": 1.5,          # m/s
    "amr_accel": 1.0,              # m/s^2, also used to brake onto the node
    "amr_heading_gain": 1.5,       # steering gain while driving straight
    "amr_max_turn_rate": 0.8,      # rad/s
    "amr_turn_gain": 2.0,          # rotation gain while turning in place
    "amr_min_turn_rate": 0.15,     # rad/s, floor so the last few degrees still turn
    "amr_align_tol_deg": 1.5,      # start driving once heading is within this
    "amr_realign_deg": 8.0,        # stop and turn again if drifting past this
    "amr_pos_tol": 0.05,           # m, counts as arrived
    "amr_settle_time": 0.3,        # s, pause on the node before turning
    "amr_arrive_dist": 1.2,        # m, smooth mode only: switch target early
    "amr_obstacle_dist": 1.5,      # m, stop if something is this close ahead
    "amr_wheel_radius": 0.5,       # m
    "amr_wheel_base": 1.25,        # m
}

MIN_DRIVE_SPEED = 0.05  # m/s, so braking never stalls short of the node
SETTLE_TIMEOUT = 3.0    # s, move on even if the body never reads fully still
ARRIVE_EPS = 0.005      # m, along-track distance treated as on the node
STOP_SPEED = 0.03       # m/s, measured speed that counts as stopped
FINAL_TIMEOUT = 3.0     # s, accept the stop if it keeps hovering in tolerance
REVERSE_SPEED = 0.1     # m/s, cap when backing onto an overshot node
STALL_TIME = 0.4        # s without motion before the command is pushed up
STALL_RAMP = 0.5        # extra rad/s (turning) or m/s x 0.25 (driving) per s


def P(key):
    """Slider value if the extension set one, otherwise the default."""
    return float(getattr(builtins, key, DEFAULTS[key]))


def params():
    return {k: P(k) for k in DEFAULTS}


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# ---------------------------------------------------------------- controller
#
# nav_step is pure: it takes the robot pose, what the lidar sees and the
# current navigation state, and returns (linear, angular, events). Nothing in
# it touches Isaac Sim, so the same code can be exercised outside the app.

def new_nav():
    return {"state": "align", "seg_start": None, "v": 0.0, "settle_t": 0.0,
            "final_t": 0.0, "blocked": False, "stall_t": 0.0, "boost": 0.0,
            "last_err": None}


def _stall_boost(nav, moving, dt):
    """Extra command when the robot is told to move but does not.

    Skid-steer wheels need a real push to break friction, more so when
    turning in place. Small commands near the goal can sit below that and
    the robot just stops short. Instead of a fixed large floor (which would
    overshoot on every turn), the boost grows only while nothing moves and
    is then held for the rest of this turn or approach, so the robot does
    not fall back below the friction threshold and stick-slip. It resets
    when the phase changes.
    """
    if moving:
        nav["stall_t"] = 0.0
    else:
        nav["stall_t"] += dt
        if nav["stall_t"] > STALL_TIME:
            nav["boost"] += STALL_RAMP * dt
    return nav["boost"]


def nav_step(nav, pose, target, front, p, dt, speed=0.0, yaw_rate=0.0):
    """One control tick.

    nav     mutable state dict from new_nav()
    pose    (x, y, yaw) of the robot
    target  (x, y) of the node being driven to
    front   nearest lidar return straight ahead (m)
    p       parameter dict
    speed   measured planar speed, used to decide the robot has settled
    yaw_rate measured yaw rate, used to spot a stalled turn

    Returns (linear m/s, angular rad/s, events). events may contain
    "arrived" (with the miss distance) and "blocked"/"clear".
    """
    x, y, yaw = pose
    tx, ty = target
    dx, dy = tx - x, ty - y
    dist = math.hypot(dx, dy)
    events = {}

    if p["amr_turn_in_place"] < 0.5:
        return _smooth_step(nav, yaw, dx, dy, dist, front, p, events)

    state = nav["state"]

    if state == "settle":
        nav["settle_t"] += dt
        still = speed < 0.02
        if (nav["settle_t"] >= p["amr_settle_time"] and still) \
                or nav["settle_t"] >= SETTLE_TIMEOUT:
            events["next"] = True
        return 0.0, 0.0, events

    if state == "align" and dist <= p["amr_pos_tol"]:
        return _arrive(nav, dist, events)   # already on the node

    if state == "align":
        if nav["seg_start"] is None:
            nav["seg_start"] = (x, y)
        sx, sy = nav["seg_start"]
        want = math.atan2(ty - sy, tx - sx)
        err = wrap(want - yaw)
        tol = math.radians(p["amr_align_tol_deg"])
        # Done when inside tolerance, or when the turn swung through the
        # target and stays close: chasing the last fraction of a degree back
        # and forth wastes time and the straight-line steering absorbs it.
        crossed = (nav["last_err"] is not None
                   and err * nav["last_err"] < 0 and abs(err) < 3 * tol)
        nav["last_err"] = err
        if abs(err) <= tol or crossed:
            nav["state"] = "drive"
            nav["v"] = 0.0
            nav["last_err"] = None
            nav["stall_t"] = nav["boost"] = 0.0
            return 0.0, 0.0, events
        boost = _stall_boost(nav, abs(yaw_rate) > 0.02, dt)
        w_max = p["amr_max_turn_rate"]
        w = max(-w_max, min(w_max, p["amr_turn_gain"] * err))
        w_min = min(w_max, p["amr_min_turn_rate"] + boost)
        if abs(w) < w_min:
            w = math.copysign(w_min, err)
        if w_min >= w_max and nav["stall_t"] > 2.0:
            events["stall"] = True      # cannot turn even at max turn rate
            nav["stall_t"] = 0.0
        return 0.0, w, events

    # state == "drive": follow the straight line seg_start -> target
    sx, sy = nav["seg_start"]
    lx, ly = tx - sx, ty - sy
    seg_len = math.hypot(lx, ly) or 1e-9
    ux, uy = lx / seg_len, ly / seg_len
    along = dx * ux + dy * uy                   # distance left along the line
    cross = ux * (y - sy) - uy * (x - sx)       # + = left of the line

    # Arrived = on the node within tolerance AND actually stopped. Checking
    # the measured speed (not the command) matters: the wheels lag the
    # command, so a robot told to stop on the node still rolls past it.
    if abs(along) <= p["amr_pos_tol"] and dist <= p["amr_pos_tol"]:
        nav["final_t"] = nav.get("final_t", 0.0) + dt
        if speed <= STOP_SPEED or nav["final_t"] >= FINAL_TIMEOUT:
            nav["final_t"] = 0.0
            return _arrive(nav, dist, events)
    else:
        nav["final_t"] = 0.0

    seg_heading = math.atan2(uy, ux)
    if abs(wrap(seg_heading - yaw)) > math.radians(p["amr_realign_deg"]):
        _restart_align(nav, x, y)
        return 0.0, 0.0, events

    # Something ahead that is nearer than the node itself: hold position.
    # Comparing against `along` keeps a wall standing just past the node from
    # stopping the robot before it gets there.
    if along > p["amr_pos_tol"] and front < p["amr_obstacle_dist"] \
            and front < along + 0.3:
        if not nav["blocked"]:
            events["blocked"] = front
        nav["blocked"] = True
        nav["v"] = 0.0
        return 0.0, 0.0, events
    if nav["blocked"]:
        events["clear"] = True
        nav["blocked"] = False

    # Speed toward along = 0: brake on v^2 = 2*a*d, ramp up at accel, and
    # back up slowly if the robot rolled past the node.
    target_v = math.copysign(
        min(p["amr_max_speed"], math.sqrt(2.0 * p["amr_accel"] * abs(along))),
        along)
    if abs(along) <= ARRIVE_EPS:
        target_v = 0.0
    elif abs(target_v) < MIN_DRIVE_SPEED:
        target_v = math.copysign(MIN_DRIVE_SPEED, along)
    if along < 0:
        target_v = max(target_v, -REVERSE_SPEED)
    if target_v != 0.0:
        boost = 0.25 * _stall_boost(nav, speed > 0.01, dt)
        if abs(target_v) < MIN_DRIVE_SPEED + boost:
            target_v = math.copysign(MIN_DRIVE_SPEED + boost, target_v)
    prev = nav["v"]
    if abs(target_v) > abs(prev) and target_v * prev >= 0:
        step = p["amr_accel"] * dt
        v = prev + math.copysign(min(step, abs(target_v - prev)), target_v)
    else:
        v = target_v                          # braking is never rate-limited
    nav["v"] = v

    # Steer back onto the line: aim at a point on it, a little ahead.
    if v >= 0:
        look = max(0.5, min(1.5, along))
        want = seg_heading - math.atan2(cross, look)
    else:
        want = seg_heading                    # short reverse: just hold heading
    err = wrap(want - yaw)
    w = max(-p["amr_max_turn_rate"],
            min(p["amr_max_turn_rate"], p["amr_heading_gain"] * err))
    return v, w, events


def _arrive(nav, dist, events):
    nav["stall_t"] = nav["boost"] = 0.0
    nav["state"] = "settle"
    nav["settle_t"] = 0.0
    nav["v"] = 0.0
    events["arrived"] = dist
    return 0.0, 0.0, events


def _restart_align(nav, x, y):
    nav["last_err"] = None
    nav["stall_t"] = nav["boost"] = 0.0
    nav["state"] = "align"
    nav["seg_start"] = (x, y)
    nav["v"] = 0.0


def start_segment(nav):
    """Called after picking the next node."""
    nav["state"] = "align"
    nav["seg_start"] = None
    nav["settle_t"] = 0.0
    nav["v"] = 0.0
    nav["last_err"] = None
    nav["stall_t"] = nav["boost"] = 0.0


def _smooth_step(nav, yaw, dx, dy, dist, front, p, events):
    """Old behaviour: switch target early and turn while moving."""
    if dist < p["amr_arrive_dist"]:
        events["next"] = True
        events["arrived"] = dist
        return 0.0, 0.0, events
    err = wrap(math.atan2(dy, dx) - yaw)
    w = max(-p["amr_max_turn_rate"],
            min(p["amr_max_turn_rate"], p["amr_turn_gain"] * err))
    if front < p["amr_obstacle_dist"]:
        return 0.0, w, events
    v = p["amr_max_speed"] * max(math.cos(err), 0.0) ** 2
    return v, w, events


# ---------------------------------------------------------------- graph

def pick_next(current):
    """Least-travelled link from here, avoiding an immediate U-turn."""
    nodes = builtins._nodes
    links = nodes[current]["links"]
    if not links:
        return current

    options = [n for n in links if n != builtins._prev_node] or links

    def travelled(n):
        return builtins._edge_visits.get(tuple(sorted((current, n))), 0)

    fewest = min(travelled(n) for n in options)
    tied = [n for n in options if travelled(n) == fewest]
    return tied[builtins._nav_frame % len(tied)]


def nearest_node(x, y):
    nodes = builtins._nodes
    return min(nodes, key=lambda n: (nodes[n]["x"] - x) ** 2
               + (nodes[n]["y"] - y) ** 2)


# ---------------------------------------------------------------- script node

def setup(db: og.Database):
    builtins._nav_frame = 0
    builtins._edge_visits = {}
    builtins._prev_node = None
    builtins._cur_node = None
    builtins._nav = new_nav()
    builtins._nav_last_t = None
    builtins._dc_nav = _dynamic_control.acquire_dynamic_control_interface()
    builtins._lidar_nav = _range_sensor.acquire_lidar_sensor_interface()

    stage = omni.usd.get_context().get_stage()
    nodes = {}
    scope = stage.GetPrimAtPath("/World/Environment/Waypoints")
    if scope and scope.IsValid():
        for child in scope.GetChildren():
            m = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(0)
            t = m.ExtractTranslation()

            nid = None
            attr = child.GetAttribute("ant:nodeId")
            if attr and attr.IsValid() and attr.Get() is not None:
                nid = int(attr.Get())
            if nid is None:
                continue

            links = []
            attr = child.GetAttribute("ant:linksTo")
            if attr and attr.IsValid() and attr.Get() is not None:
                links = [int(v) for v in attr.Get()]

            nodes[nid] = {"x": float(t[0]), "y": float(t[1]), "links": links}

    # Drop links pointing at nodes that do not exist, so the walk cannot
    # steer the robot at a waypoint that was never authored.
    for nid, node in nodes.items():
        node["links"] = [n for n in node["links"] if n in nodes and n != nid]

    builtins._nodes = nodes

    edges = set()
    for nid, node in nodes.items():
        for other in node["links"]:
            edges.add(tuple(sorted((nid, other))))
    db.log_warning("NAV: " + str(len(nodes)) + " nodes, "
                   + str(len(edges)) + " links")


def cleanup(db: og.Database):
    pass


def _front_distance():
    depth = builtins._lidar_nav.get_linear_depth_data(LIDAR)
    if depth is None or len(depth) == 0:
        return 999.0
    d = np.array(depth).flatten()
    d = np.where(d > 0.01, d, 999.0)
    n = len(d)
    f = d[int(n * 0.45):int(n * 0.55)]
    return float(np.min(f)) if len(f) else 999.0


def _set_wheels(dc, art, linear, angular, p):
    r, b = p["amr_wheel_radius"], p["amr_wheel_base"]
    v_left = (linear - angular * b / 2.0) / r
    v_right = (linear + angular * b / 2.0) / r
    for joints, v in ((LEFT_JOINTS, v_left), (RIGHT_JOINTS, v_right)):
        for jn in joints:
            dof = dc.find_articulation_dof(art, jn)
            if dof != _dynamic_control.INVALID_HANDLE:
                dc.set_dof_velocity_target(dof, v)


def _dt():
    now = omni.timeline.get_timeline_interface().get_current_time()
    last = builtins._nav_last_t
    builtins._nav_last_t = now
    if last is None or not (0.0 < now - last < 0.5):
        return 1.0 / 60.0
    return now - last


def compute(db: og.Database):
    dc = builtins._dc_nav
    art = dc.get_articulation(ROBOT)
    nodes = builtins._nodes
    if art == _dynamic_control.INVALID_HANDLE or not nodes:
        return True

    root = dc.get_articulation_root_body(art)
    pose = dc.get_rigid_body_pose(root)
    rx, ry = pose.p.x, pose.p.y
    q = pose.r
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    vel = dc.get_rigid_body_linear_velocity(root)
    speed = math.hypot(vel.x, vel.y)
    yaw_rate = float(dc.get_rigid_body_angular_velocity(root).z)

    p = params()
    dt = _dt()
    nav = builtins._nav

    if builtins._cur_node is None:
        # Start from wherever the robot was placed.
        builtins._cur_node = nearest_node(rx, ry)
        start_segment(nav)

    node = nodes[builtins._cur_node]
    front = _front_distance()
    linear, angular, ev = nav_step(nav, (rx, ry, yaw),
                                   (node["x"], node["y"]), front, p, dt,
                                   speed, yaw_rate)

    if "arrived" in ev:
        db.log_warning("NAV: arrived node " + str(builtins._cur_node)
                       + " miss=" + str(round(ev["arrived"] * 100, 1)) + "cm")
    if "blocked" in ev:
        db.log_warning("NAV: obstacle " + str(round(ev["blocked"], 2))
                       + "m ahead - waiting")
    if "clear" in ev:
        db.log_warning("NAV: path clear - resuming")
    if "stall" in ev:
        db.log_warning("NAV: not turning even at max turn rate - "
                       "raise 'max turn rate' in AMR Tools")

    if ev.get("next"):
        cur = builtins._cur_node
        nxt = pick_next(cur)
        edge = tuple(sorted((cur, nxt)))
        builtins._edge_visits[edge] = builtins._edge_visits.get(edge, 0) + 1
        builtins._prev_node = cur
        builtins._cur_node = nxt
        start_segment(nav)
        db.log_warning("NAV: -> node " + str(nxt))

    _set_wheels(dc, art, linear, angular, p)

    builtins._nav_frame += 1
    if builtins._nav_frame % 120 == 0:
        dist = math.hypot(node["x"] - rx, node["y"] - ry)
        db.log_warning(
            "NAV: (" + str(round(rx, 2)) + "," + str(round(ry, 2)) + ") "
            + nav["state"] + " -> node " + str(builtins._cur_node)
            + " d=" + str(round(dist, 2))
            + " v=" + str(round(linear, 2))
            + " w=" + str(round(angular, 2))
            + " F=" + str(round(front, 1))
            + " edges=" + str(len(builtins._edge_visits)))

    return True
