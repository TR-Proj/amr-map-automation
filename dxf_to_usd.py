"""DXF/DWG 도면 -> Isaac Sim USD 씬 (+로봇 씬 합치기).

    ./python.bat dxf_to_usd.py --dxf simple.dxf --robot mobile.usd

Isaac Sim 의 python.bat / python.sh 로 실행. pip install ezdxf 필요.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from collections import Counter, defaultdict

import ezdxf

CONFIG = {
    "dxf": "simple.dxf",
    "out": None,                  # None: simple.dxf -> <out_dir>/simple.usda
    "out_dir": "output",          # 생성물 폴더 (스크립트 폴더 기준, 자동 생성)

    # 씬 합치기 (None 이면 맵만 생성)
    "robot": "mobile.usd",        # 로봇 씬 USD (OmniGraph script node 포함)
    "script_dir": "script_node",  # script node 스크립트 폴더 (씬 생성 시 경로 재연결)
    "scene": None,                # None: <out>_scene.usd
    "robot_path": "/World/simplerobot",
    "env_path": "/World/Environment",
    "start_wp": 1,                # 로봇을 놓을 웨이포인트 번호 (nodeId)

    # 도면 읽기
    "layers": ["WALL"],           # None 이면 전 레이어 (--stats 로 먼저 확인)
    "unit": "mm",
    "collapse": 0.25,             # 이중선 벽 -> 중심선. 벽두께보다 약간 크게. 0=끔
    "min_len": 0.10,
    "gap_tol": 0.05,
    "recenter": False,
    "offset_x": 0.0,
    "offset_y": 0.0,

    # 벽 생성
    "wall_height": 4.0,
    "wall_thickness": 0.20,
    "ground": False,              # 바닥은 로봇 씬에 있음
    "lights": True,

    # 주행 노드: (id, 이름, x, y, heading(도), 연결 노드들)
    "waypoints": [
        (1, "1",  3.0,  3.0,  90, [2, 6]),
        (2, "2",  3.0, 21.0,   0, [1, 3]),
        (3, "3", 20.0, 21.0,   0, [2, 4, 6]),
        (4, "4", 37.0, 21.0, 270, [3, 5]),
        (5, "5", 37.0,  3.0, 180, [4, 6]),
        (6, "6", 20.0,  3.0, 180, [5, 1, 3]),
    ],

    "stats": False,
    "top_view": True,
    "robot_cam_view": True,       # 두 번째 뷰포트에 로봇 카메라 띄우기
    "robot_cam": "/World/simplerobot/front_sensor/sensor_mount/Camera",
    "enable_ext": True,           # AMR Tools 확장 자동 등록·활성화
    "ext_dir": "exts",            # 확장 검색 경로 (스크립트 폴더 기준)
    "ext_name": "amr.tools",
}

UNIT_SCALE = {"mm": 0.001, "cm": 0.01, "m": 1.0, "in": 0.0254, "ft": 0.3048}

# pxr 은 SimulationApp 생성 후에만 import 가능 -> 지연 로드
Gf = Sdf = Usd = UsdGeom = UsdLux = UsdPhysics = None
_SIM_APP = None


def _init_usd():
    global Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, _SIM_APP
    if UsdGeom is not None:
        return _SIM_APP

    SimulationApp = None
    try:
        from isaacsim import SimulationApp
    except ImportError:
        try:
            from omni.isaac.kit import SimulationApp
        except ImportError:
            pass
    if SimulationApp is not None:

        _SIM_APP = SimulationApp({"headless": False})

    from pxr import Gf as _Gf, Sdf as _Sdf, Usd as _Usd
    from pxr import UsdGeom as _UsdGeom, UsdLux as _UsdLux, UsdPhysics as _UsdPhysics
    Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics = (
        _Gf, _Sdf, _Usd, _UsdGeom, _UsdLux, _UsdPhysics)
    return _SIM_APP


def _enable_extension(ext_dir, ext_name):
    """확장 검색 경로에 등록하고 활성화. 이미 켜져 있으면 그대로 둠."""
    try:
        import omni.kit.app
        manager = omni.kit.app.get_app().get_extension_manager()

        ext_dir = os.path.abspath(ext_dir)
        if not os.path.isdir(ext_dir):
            print(f"  ! 확장 폴더 없음: {ext_dir}")
            return False

        folders = list(manager.get_folders())
        if ext_dir not in folders:
            manager.add_path(ext_dir)

        if manager.is_extension_enabled(ext_name):
            return True

        manager.set_extension_enabled_immediate(ext_name, True)
        return True

    except Exception as exc:
        print(f"  ! 확장 실패: {exc}")
        return False


# ---------------------------------------------------------------- 선분 정리

def _unit(t):
    return math.cos(t), math.sin(t)


def _angle_diff(a, b):
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def _collinear(si, sj, ti, tj, ang_tol, lat_tol, gap_tol):
    if _angle_diff(ti, tj) > ang_tol:
        return False

    for (a, t), b in (((si, ti), sj), ((sj, tj), si)):
        dx, dy = _unit(t)
        nx, ny = -dy, dx
        rho = a[0] * nx + a[1] * ny
        if abs(b[0] * nx + b[1] * ny - rho) > lat_tol:
            return False
        if abs(b[2] * nx + b[3] * ny - rho) > lat_tol:
            return False

    dx, dy = _unit(ti)
    pi1, pi2 = si[0] * dx + si[1] * dy, si[2] * dx + si[3] * dy
    pj1, pj2 = sj[0] * dx + sj[1] * dy, sj[2] * dx + sj[3] * dy
    gap = max(min(pj1, pj2) - max(pi1, pi2),
              min(pi1, pi2) - max(pj1, pj2), 0.0)
    return gap <= gap_tol


def _grid_candidates(segs, cell):
    cells = defaultdict(list)
    for i, s in enumerate(segs):
        L = math.dist(s[:2], s[2:])
        n = max(2, int(L / (cell * 0.5)) + 2)
        for k in range(n):
            u = k / (n - 1)
            x = s[0] + (s[2] - s[0]) * u
            y = s[1] + (s[3] - s[1]) * u
            cells[(int(x // cell), int(y // cell))].append(i)
    return cells


def merge_collinear(segs, angle_tol_deg=1.5, gap_tol=0.05, min_len=0.10,
                    lat_tol=0.02):
    """같은 직선 위 조각들을 긴 벽 선분으로 병합 (쌍별 판정 + union-find)."""
    if not segs:
        return []

    n = len(segs)
    ang_tol = math.radians(angle_tol_deg)
    thetas = [math.atan2(s[3] - s[1], s[2] - s[0]) % math.pi for s in segs]
    lengths = [math.dist(s[:2], s[2:]) for s in segs]

    parent = list(range(n))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    cell = max(gap_tol * 4, lat_tol * 4, 0.5)
    for idxs in _grid_candidates(segs, cell).values():
        uniq = sorted(set(idxs))
        for x in range(len(uniq)):
            i = uniq[x]
            for y in range(x + 1, len(uniq)):
                j = uniq[y]
                if find(i) != find(j) and _collinear(
                        segs[i], segs[j], thetas[i], thetas[j],
                        ang_tol, lat_tol, gap_tol):
                    parent[find(j)] = find(i)

    comps = defaultdict(list)
    for i in range(n):
        comps[find(i)].append(i)

    out = []
    for members in comps.values():
        # 길이 가중 평균 방향 (mod pi -> 2배각 평균)
        sx = sum(lengths[i] * math.cos(2 * thetas[i]) for i in members)
        sy = sum(lengths[i] * math.sin(2 * thetas[i]) for i in members)
        t = math.atan2(sy, sx) / 2.0
        dx, dy = _unit(t)
        nx, ny = -dy, dx

        wsum = sum(lengths[i] for i in members) or 1.0
        rho = sum(lengths[i] * ((segs[i][0] + segs[i][2]) / 2 * nx
                                + (segs[i][1] + segs[i][3]) / 2 * ny)
                  for i in members) / wsum
        ox, oy = rho * nx, rho * ny

        spans = sorted(
            (min(s[0] * dx + s[1] * dy, s[2] * dx + s[3] * dy),
             max(s[0] * dx + s[1] * dy, s[2] * dx + s[3] * dy))
            for s in (segs[i] for i in members))

        cur_lo, cur_hi = spans[0]
        merged = []
        for lo, hi in spans[1:]:
            if lo <= cur_hi + gap_tol:
                cur_hi = max(cur_hi, hi)
            else:
                merged.append((cur_lo, cur_hi))
                cur_lo, cur_hi = lo, hi
        merged.append((cur_lo, cur_hi))

        for lo, hi in merged:
            if hi - lo >= min_len:
                out.append([ox + lo * dx, oy + lo * dy,
                            ox + hi * dx, oy + hi * dy])
    return out


def collapse_parallel(segs, max_thickness=0.25, angle_tol_deg=2.0,
                      min_overlap=0.20):
    """마주보는 벽면 두 선분 -> 중심선 하나. 더 두꺼운 구조물은 그대로."""
    if not segs:
        return []

    ang_tol = math.radians(angle_tol_deg)
    items = []
    for s in segs:
        t = math.atan2(s[3] - s[1], s[2] - s[0]) % math.pi
        dx, dy = _unit(t)
        nx, ny = -dy, dx
        p1 = s[0] * dx + s[1] * dy
        p2 = s[2] * dx + s[3] * dy
        items.append({"seg": s, "t": t, "d": (dx, dy), "n": (nx, ny),
                      "rho": s[0] * nx + s[1] * ny,
                      "lo": min(p1, p2), "hi": max(p1, p2)})

    used = [False] * len(items)
    out = []
    for i, a in enumerate(items):
        if used[i]:
            continue

        best, best_key = None, None
        for j in range(i + 1, len(items)):
            if used[j]:
                continue
            b = items[j]
            if _angle_diff(a["t"], b["t"]) > ang_tol:
                continue
            gap = abs(a["rho"] - b["rho"])
            if gap < 1e-6 or gap > max_thickness:
                continue
            overlap = min(a["hi"], b["hi"]) - max(a["lo"], b["lo"])
            if overlap < min_overlap:
                continue
            # 겹침 긴 쪽 우선 (마구리면 오매칭 방지)
            key = (-overlap, gap)
            if best_key is None or key < best_key:
                best, best_key = j, key

        if best is None:
            out.append(a["seg"])
            used[i] = True
            continue

        b = items[best]
        used[i] = used[best] = True
        dx, dy = a["d"]
        nx, ny = a["n"]
        rho = (a["rho"] + b["rho"]) / 2.0
        lo, hi = min(a["lo"], b["lo"]), max(a["hi"], b["hi"])
        out.append([rho * nx + lo * dx, rho * ny + lo * dy,
                    rho * nx + hi * dx, rho * ny + hi * dy])
    return out


# ---------------------------------------------------------------- DXF 읽기

def flatten_entity(e, arc_seg_len=0.10):
    t = e.dxftype()
    out = []

    if t == "LINE":
        s, p = e.dxf.start, e.dxf.end
        out.append([s.x, s.y, p.x, p.y])

    elif t in ("LWPOLYLINE", "POLYLINE"):
        pts = ([(p[0], p[1]) for p in e.get_points("xy")]
               if t == "LWPOLYLINE" else
               [(v.dxf.location.x, v.dxf.location.y) for v in e.vertices])
        closed = bool(getattr(e, "closed", False) or e.dxf.get("flags", 0) & 1)
        for i in range(len(pts) - 1):
            out.append([*pts[i], *pts[i + 1]])
        if closed and len(pts) > 2:
            out.append([*pts[-1], *pts[0]])

    elif t in ("SPLINE", "ELLIPSE"):
        try:
            pts = [(p[0], p[1]) for p in e.flattening(arc_seg_len)]
            for i in range(len(pts) - 1):
                out.append([*pts[i], *pts[i + 1]])
        except Exception:
            pass

    elif t in ("ARC", "CIRCLE"):
        c, r = e.dxf.center, e.dxf.radius
        if t == "CIRCLE":
            a0, a1 = 0.0, 2 * math.pi
        else:
            a0 = math.radians(e.dxf.start_angle)
            a1 = math.radians(e.dxf.end_angle)
            if a1 <= a0:
                a1 += 2 * math.pi
        steps = max(2, int(r * (a1 - a0) / max(arc_seg_len, 1e-3)))
        prev = None
        for k in range(steps + 1):
            a = a0 + (a1 - a0) * k / steps
            p = (c.x + r * math.cos(a), c.y + r * math.sin(a))
            if prev is not None:
                out.append([*prev, *p])
            prev = p

    return out


def open_drawing(path):
    if path.lower().endswith(".dwg"):
        try:
            from ezdxf.addons import odafc
            return odafc.readfile(path)
        except Exception as exc:
            raise SystemExit(f"DWG 읽기 실패: {exc}")
    return ezdxf.readfile(path)


def extract(dxf_path, layers=None, scale=1.0, explode_blocks=True):
    doc = open_drawing(dxf_path)
    segs = []
    for e in doc.modelspace().query("*"):
        if e.dxftype() == "INSERT" and explode_blocks:
            try:
                for sub in e.virtual_entities():
                    if layers and sub.dxf.layer not in layers:
                        continue
                    segs.extend(flatten_entity(sub))
            except Exception:
                pass
            continue
        if layers and e.dxf.get("layer") not in layers:
            continue
        segs.extend(flatten_entity(e))
    return [[c * scale for c in s] for s in segs]


# ---------------------------------------------------------------- USD 생성

def add_wall(stage, path, seg, height, thickness, z_base):
    x1, y1, x2, y2 = seg
    length = math.dist((x1, y1), (x2, y2))
    if length < 1e-6:
        return None

    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d((x1 + x2) / 2, (y1 + y2) / 2,
                                     z_base + height / 2))
    xf.AddRotateZOp().Set(math.degrees(math.atan2(y2 - y1, x2 - x1)))
    xf.AddScaleOp().Set(Gf.Vec3f(length, thickness, height))
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


def add_waypoint(stage, nid, name, x, y, heading_deg, links=(), one_way=(),
                 size=0.5):
    """주행 노드. 마커는 purpose=guide 라 벽 추출에서 제외됨."""
    path = f"/Environment/Waypoints/WP{nid}"
    xform = UsdGeom.Xform.Define(stage, path)
    xf = UsdGeom.Xformable(xform)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.0))
    xf.AddRotateZOp().Set(float(heading_deg))

    prim = xform.GetPrim()
    prim.CreateAttribute("ant:nodeId", Sdf.ValueTypeNames.Int).Set(int(nid))
    prim.CreateAttribute("ant:name", Sdf.ValueTypeNames.String).Set(str(name))
    if links:
        prim.CreateAttribute("ant:linksTo", Sdf.ValueTypeNames.IntArray).Set(
            [int(v) for v in links])
    if one_way:
        prim.CreateAttribute("ant:linksOneWay",
                             Sdf.ValueTypeNames.IntArray).Set(
            [int(v) for v in one_way])

    body = UsdGeom.Cylinder.Define(stage, path + "/Marker")
    body.GetRadiusAttr().Set(size * 0.25)
    body.GetHeightAttr().Set(0.05)
    body.GetAxisAttr().Set("Z")
    UsdGeom.Xformable(body).AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.03))
    UsdGeom.Imageable(body).CreatePurposeAttr().Set(UsdGeom.Tokens.guide)

    tip = UsdGeom.Cone.Define(stage, path + "/Heading")
    tip.GetRadiusAttr().Set(size * 0.18)
    tip.GetHeightAttr().Set(size * 0.6)
    tip.GetAxisAttr().Set("X")
    UsdGeom.Xformable(tip).AddTranslateOp().Set(Gf.Vec3d(size * 0.7, 0, 0.03))
    UsdGeom.Imageable(tip).CreatePurposeAttr().Set(UsdGeom.Tokens.guide)

    return xform


def build_stage(out_path, segments, wall_height=3.0, wall_thickness=0.10,
                z_base=0.0, ground=True, ground_margin=5.0, lights=True,
                waypoints=None):
    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    env = UsdGeom.Xform.Define(stage, "/Environment")
    stage.SetDefaultPrim(env.GetPrim())
    UsdGeom.Scope.Define(stage, "/Environment/Walls")

    for i, seg in enumerate(segments):
        add_wall(stage, f"/Environment/Walls/Wall_{i:04d}",
                 seg, wall_height, wall_thickness, z_base)

    UsdGeom.Scope.Define(stage, "/Environment/Waypoints")
    UsdGeom.Scope.Define(stage, "/Environment/VirtualWalls")
    UsdGeom.Scope.Define(stage, "/Environment/Reflectors")

    for nid, name, x, y, deg, links in (waypoints or []):
        add_waypoint(stage, nid, name, x, y, deg, links)

    if lights:
        UsdGeom.Scope.Define(stage, "/Environment/Lights")
        dome = UsdLux.DomeLight.Define(stage, "/Environment/Lights/Dome")
        dome.CreateIntensityAttr(600.0)
        sun = UsdLux.DistantLight.Define(stage, "/Environment/Lights/Sun")
        sun.CreateIntensityAttr(1500.0)
        sun.CreateAngleAttr(0.53)
        UsdGeom.Xformable(sun).AddRotateXYZOp().Set(Gf.Vec3f(-45, 0, 30))

    stage.GetRootLayer().Save()
    return stage


# ---------------------------------------------------------------- 씬 합치기

def _world_xy(prim):
    t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
    return float(t[0]), float(t[1])


def _world_yaw(prim):
    r = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractRotationMatrix()
    return math.atan2(r[0][1], r[0][0])


def _collect_waypoints(stage, env_path):
    scope = stage.GetPrimAtPath(env_path + "/Waypoints")
    if not scope or not scope.IsValid():
        return []

    found = []
    for child in scope.GetChildren():
        x, y = _world_xy(child)
        nid = None
        a = child.GetAttribute("ant:nodeId")
        if a and a.IsValid() and a.Get() is not None:
            nid = int(a.Get())
        found.append({"name": child.GetName(), "id": nid,
                      "x": x, "y": y, "yaw": _world_yaw(child)})

    key = (lambda w: w["id"]) if all(w["id"] is not None for w in found) \
        else (lambda w: w["name"])
    found.sort(key=key)
    return found


def _wall_clearance(stage, env_path, x, y):
    scope = stage.GetPrimAtPath(env_path + "/Walls")
    if not scope or not scope.IsValid():
        return float("inf")

    cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
    best = float("inf")
    for child in scope.GetChildren():
        rng = cache.ComputeWorldBound(child).ComputeAlignedRange()
        if rng.IsEmpty():
            continue
        lo, hi = rng.GetMin(), rng.GetMax()
        dx = max(lo[0] - x, 0.0, x - hi[0])
        dy = max(lo[1] - y, 0.0, y - hi[1])
        best = min(best, math.hypot(dx, dy))
    return best


def _relink_script_nodes(stage, script_dir):
    """OmniGraph script node 의 scriptPath 를 이 저장소의 script_node 폴더로 재연결.

    로봇 씬에 저장된 경로는 씬을 만든 PC 기준이라, 그대로 두면 다른 PC 에서는
    스크립트를 찾지 못함. 파일 이름만 보고 script_dir 아래 같은 이름으로 바꾼다.
    """
    script_dir = os.path.abspath(script_dir)
    linked, missing = [], []
    for prim in stage.Traverse():
        attr = prim.GetAttribute("inputs:scriptPath")
        if not attr or not attr.HasAuthoredValue():
            continue
        old = attr.Get()
        if not old:
            continue
        name = os.path.basename(str(old).replace("\\", "/"))
        new = os.path.join(script_dir, name)
        if os.path.exists(new):
            attr.Set(new.replace(os.sep, "/"))
            linked.append(name)
        else:
            missing.append(name)
    return linked, missing


def compose_scene(map_path, robot_path, scene_path,
                  robot_prim="/World/simplerobot",
                  env_path="/World/Environment", start_wp=1,
                  script_dir=None):
    """로봇 씬 복사본에 맵을 reference 로 얹고 로봇을 웨이포인트에 배치.

    로봇 씬을 바탕으로 하는 이유: PhysicsScene, articulation, OmniGraph 가
    전부 절대 경로를 참조하므로 그쪽을 유지해야 함.
    """
    import shutil

    shutil.copyfile(robot_path, scene_path)
    stage = Usd.Stage.Open(scene_path)
    if stage is None:
        raise SystemExit(f"로봇 씬을 열 수 없습니다: {scene_path}")

    notes = []

    # 맵 reference (Clear 로 재실행시 중복 방지. Reload() 금지 - 편집이 날아감)
    env = stage.GetPrimAtPath(env_path)
    if not env or not env.IsValid():
        env = UsdGeom.Xform.Define(stage, env_path).GetPrim()
        notes.append(f"{env_path} 생성")

    # 맵은 도면 좌표 그대로 놓아야 함. 로봇 씬에 남은 이동/회전이 있으면
    # 벽·웨이포인트가 함께 밀려서, 만들어지는 map2 가 실제 공장 좌표와 어긋남.
    env_xf = UsdGeom.Xformable(env)
    if env_xf.GetOrderedXformOps():
        moved = env_xf.ComputeLocalToWorldTransform(0).ExtractTranslation()
        env_xf.ClearXformOpOrder()
        for attr in env.GetAttributes():
            if attr.GetName().startswith("xformOp:"):
                env.RemoveProperty(attr.GetName())
        if any(abs(v) > 1e-6 for v in moved):
            notes.append(f"{env_path} 이동값 ({moved[0]:.2f}, {moved[1]:.2f}) "
                         "제거 - 맵을 도면 좌표에 맞춤")

    rel = os.path.relpath(os.path.abspath(map_path),
                          os.path.dirname(os.path.abspath(scene_path)))
    rel = rel.replace(os.sep, "/")
    if not rel.startswith("."):
        rel = "./" + rel

    refs = env.GetReferences()
    refs.ClearReferences()
    refs.AddReference(assetPath=rel)

    if not stage.GetPrimAtPath(env_path + "/Walls").IsValid():
        notes.append("Walls 없음 - 맵 defaultPrim 확인")

    # PhysicsScene 은 /World 직속이어야 함 (아니면 라이다·구동 등록 실패)
    world = stage.GetPrimAtPath("/World")
    if not any(c.IsA(UsdPhysics.Scene) for c in world.GetChildren()):
        stray = [p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
        if stray:
            notes.append(f"PhysicsScene 이 {stray[0].GetPath()} - "
                         "/World 직속으로 옮기세요 (라이다·구동 실패 원인)")
        else:
            UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
            notes.append("PhysicsScene 생성")

    # 벽 collider (raycast 라이다는 collider 만 감지)
    walls = stage.GetPrimAtPath(env_path + "/Walls")
    if walls and walls.IsValid():
        kids = list(walls.GetChildren())
        bare = sum(1 for c in kids if not c.HasAPI(UsdPhysics.CollisionAPI))
        if bare:
            notes.append(f"collider 없는 벽 {bare}/{len(kids)} - 라이다 미감지")

    # 바닥이 도면을 덮는지
    ground = stage.GetPrimAtPath("/World/GroundPlane")
    if ground and ground.IsValid() and walls and walls.IsValid():
        cache = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_])
        g = cache.ComputeWorldBound(ground).ComputeAlignedRange()
        w = cache.ComputeWorldBound(walls).ComputeAlignedRange()
        if not g.IsEmpty() and not w.IsEmpty():
            glo, ghi = g.GetMin(), g.GetMax()
            wlo, whi = w.GetMin(), w.GetMax()
            if (wlo[0] < glo[0] or wlo[1] < glo[1]
                    or whi[0] > ghi[0] or whi[1] > ghi[1]):
                notes.append(
                    f"도면 {whi[0]-wlo[0]:.1f}x{whi[1]-wlo[1]:.1f}m > "
                    f"바닥 {ghi[0]-glo[0]:.1f}x{ghi[1]-glo[1]:.1f}m - "
                    "GroundPlane 을 키우세요")

    # 로봇 배치 (원래 z 유지 -> 바퀴가 바닥에 붙음)
    start = None
    waypoints = _collect_waypoints(stage, env_path)
    robot = stage.GetPrimAtPath(robot_prim)

    if not robot or not robot.IsValid():
        notes.append(f"로봇 prim 없음: {robot_prim}")
    elif not waypoints:
        notes.append("웨이포인트 없음 - 로봇 위치 유지")
    else:
        start = next((w for w in waypoints if w["id"] == start_wp), None)
        if start is None:
            start = waypoints[0]
            notes.append(f"WP{start_wp} 없음 -> {start['name']} 사용")

        z = float(UsdGeom.Xformable(robot)
                  .ComputeLocalToWorldTransform(0).ExtractTranslation()[2])
        xf = UsdGeom.Xformable(robot)
        xf.ClearXformOpOrder()      # 기존 translate 위에 쌓이지 않게
        xf.AddTranslateOp().Set(Gf.Vec3d(start["x"], start["y"], z))
        xf.AddRotateZOp().Set(math.degrees(start["yaw"]))

        clear = _wall_clearance(stage, env_path, start["x"], start["y"])
        if clear < 0.6:
            notes.append(f"{start['name']} 가 벽에서 {clear:.2f}m ")

    if script_dir:
        linked, missing = _relink_script_nodes(stage, script_dir)
        if missing:
            notes.append(f"script node 파일 없음 ({script_dir}): "
                         + ", ".join(missing))
        elif not linked:
            notes.append("script node 없음 - 주행·스캔이 동작하지 않음")

    stage.GetRootLayer().Save()
    return scene_path, waypoints, start, notes


# ---------------------------------------------------------------- Isaac Sim

def open_in_isaac_sim(usd_path, top_view=True, robot_cam_view=True,
                      robot_cam=None, ext_dir=None, ext_name=None):
    app = _SIM_APP
    if app is None:
        raise SystemExit("Isaac Sim 을 찾을 수 없습니다. ")

    abs_path = os.path.abspath(usd_path)
    print(f"실행: {os.path.basename(abs_path)}")

    import omni.usd
    import carb.settings
    st = carb.settings.get_settings()
    st.set_bool("/app/omni.graph.scriptnode/opt_in", True)
    st.set_bool("/app/omni.graph.scriptnode/enable_opt_in", False)
    omni.usd.get_context().open_stage(abs_path)

    for _ in range(120):
        app.update()

    if ext_dir and ext_name:
        _enable_extension(ext_dir, ext_name)
        for _ in range(30):
            app.update()

    if top_view:
        try:
            _set_top_view()
        except Exception as exc:
            print(f"  ! 탑뷰 실패: {exc}")

    if robot_cam_view and robot_cam:
        try:
            _open_second_viewport(app, robot_cam)
        except Exception as exc:
            print(f"  ! 뷰포트2 실패: {exc}")

    import omni.timeline
    for _ in range(30):
        app.update()
    omni.timeline.get_timeline_interface().play()

    while app.is_running():
        app.update()
    app.close()


def _open_second_viewport(app, camera_path):
    """두 번째 뷰포트를 띄워 로봇 카메라를 물린다."""
    import omni.usd
    from omni.kit.viewport.utility import create_viewport_window

    stage = omni.usd.get_context().get_stage()
    if not stage.GetPrimAtPath(camera_path).IsValid():
        print(f"  ! 로봇 카메라 없음: {camera_path}")
        return

    win = create_viewport_window("Robot Camera", width=520, height=380)
    for _ in range(10):
        app.update()

    win.viewport_api.camera_path = camera_path
    # 탑뷰 옆에 붙여 두 화면을 나란히
    try:
        import omni.ui as ui
        win.dock_in(ui.Workspace.get_window("Viewport"),
                    ui.DockPosition.RIGHT, 0.5)
    except Exception:
        pass



def _set_top_view():
    import omni.usd
    from omni.kit.viewport.utility import get_active_viewport

    stage = omni.usd.get_context().get_stage()
    bbox = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])

    # 합쳐진 씬(/World/Environment)과 맵 단독(/Environment) 둘 다 지원
    root = None
    for p in ("/World/Environment", "/Environment"):
        prim = stage.GetPrimAtPath(p)
        if prim and prim.IsValid():
            root = prim
            break
    if root is None:
        return

    rng = bbox.ComputeWorldBound(root).ComputeAlignedRange()
    if rng.IsEmpty():
        return

    lo, hi = rng.GetMin(), rng.GetMax()
    cx, cy = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    span = max(hi[0] - lo[0], hi[1] - lo[1])

    cam_path = str(root.GetPath()) + "/TopCamera"
    cam = UsdGeom.Camera.Define(stage, cam_path)
    xf = UsdGeom.Xformable(cam)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(Gf.Vec3d(cx, cy, span * 1.2))
    cam.CreateFocalLengthAttr(24.0)
    cam.CreateClippingRangeAttr(Gf.Vec2f(0.1, span * 10))

    vp = get_active_viewport()
    if vp:
        vp.camera_path = cam_path


# ---------------------------------------------------------------- CLI

def print_stats(path):
    doc = open_drawing(path)
    per = defaultdict(Counter)
    for e in doc.modelspace():
        per[e.dxf.get("layer", "?")][e.dxftype()] += 1

    width = max((len(k) for k in per), default=10)
    for lname in sorted(per):
        kinds = ", ".join(f"{k} {v}" for k, v in per[lname].most_common())
        print(f"{lname:<{width}}  {kinds}")


def _resolve(path):
    if not path or os.path.isabs(path):
        return path
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), path)


def main():
    C = CONFIG
    ap = argparse.ArgumentParser(description="DXF/DWG -> Isaac Sim USD")
    ap.add_argument("--dxf", default=C["dxf"])
    ap.add_argument("--out", default=None)
    ap.add_argument("--out-dir", default=C["out_dir"],
                    help="생성물 폴더 (기본: output)")
    ap.add_argument("--stats", action="store_true", default=C["stats"])
    ap.add_argument("--layers", nargs="*", default=C["layers"])
    ap.add_argument("--unit", default=C["unit"], choices=list(UNIT_SCALE))
    ap.add_argument("--collapse", type=float, default=C["collapse"])
    ap.add_argument("--min-len", type=float, default=C["min_len"])
    ap.add_argument("--gap-tol", type=float, default=C["gap_tol"])
    ap.add_argument("--recenter", action="store_true", default=C["recenter"])
    ap.add_argument("--offset-x", type=float, default=C["offset_x"])
    ap.add_argument("--offset-y", type=float, default=C["offset_y"])
    ap.add_argument("--wall-height", type=float, default=C["wall_height"])
    ap.add_argument("--wall-thickness", type=float,
                    default=C["wall_thickness"])
    ap.add_argument("--z-base", type=float, default=0.0)
    ap.add_argument("--no-ground", action="store_true",
                    default=not C["ground"])
    ap.add_argument("--no-lights", action="store_true",
                    default=not C["lights"])
    ap.add_argument("--no-top-view", action="store_true",
                    default=not C["top_view"])
    ap.add_argument("--save-segments", default=None)
    ap.add_argument("--robot", default=C["robot"],
                    help="로봇 씬 USD. 지정하면 주행 가능한 씬까지 생성")
    ap.add_argument("--scene", default=C["scene"],
                    help="합쳐진 씬 출력 경로 (기본: <out>_scene.usd)")
    ap.add_argument("--script-dir", default=C["script_dir"],
                    help="OmniGraph script node 스크립트 폴더")
    ap.add_argument("--robot-path", default=C["robot_path"])
    ap.add_argument("--env-path", default=C["env_path"])
    ap.add_argument("--start-wp", type=int, default=C["start_wp"],
                    help="시작 웨이포인트 번호 (nodeId)")
    ap.add_argument("--no-robot-cam", action="store_true",
                    default=not C["robot_cam_view"])
    ap.add_argument("--robot-cam", default=C["robot_cam"])
    ap.add_argument("--no-ext", action="store_true",
                    default=not C["enable_ext"])
    ap.add_argument("--ext-dir", default=C["ext_dir"])
    ap.add_argument("--ext-name", default=C["ext_name"])
    args = ap.parse_args()

    args.dxf = _resolve(args.dxf)

    out_dir = _resolve(args.out_dir)
    os.makedirs(out_dir, exist_ok=True)

    if args.out is None:
        name = os.path.splitext(os.path.basename(args.dxf))[0] + ".usda"
        args.out = C["out"] or os.path.join(out_dir, name)
    args.out = _resolve(args.out)

    if args.stats:
        print_stats(args.dxf)
        return

    steps = 3 if args.robot else 2

    # 1. DXF -> 선분
    raw = extract(args.dxf, args.layers, UNIT_SCALE[args.unit])
    segs = merge_collinear(raw, gap_tol=args.gap_tol, min_len=args.min_len)
    if args.collapse > 0:
        segs = collapse_parallel(segs, max_thickness=args.collapse)
        segs = merge_collinear(segs, gap_tol=args.gap_tol,
                               min_len=args.min_len)

    dx, dy = args.offset_x, args.offset_y
    if args.recenter and segs:
        xs = [c for s in segs for c in (s[0], s[2])]
        ys = [c for s in segs for c in (s[1], s[3])]
        dx -= min(xs)
        dy -= min(ys)
    if dx or dy:
        segs = [[s[0] + dx, s[1] + dy, s[2] + dx, s[3] + dy] for s in segs]

    if not segs:
        raise SystemExit("선분 없음")

    xs = [c for s in segs for c in (s[0], s[2])]
    ys = [c for s in segs for c in (s[1], s[3])]
    print(f"[1/{steps}] {os.path.basename(args.dxf)}: "
          f"{len(raw)} -> {len(segs)} 선분, "
          f"{max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f} m")

    if args.save_segments:
        with open(args.save_segments, "w") as f:
            json.dump(segs, f)

    # 2. 선분 -> 맵 USD
    _init_usd()
    build_stage(args.out, segs, args.wall_height, args.wall_thickness,
                args.z_base, not args.no_ground, lights=not args.no_lights,
                waypoints=C["waypoints"])
    print(f"[2/{steps}] {os.path.basename(args.out)}: "
          f"벽 {len(segs)} / 노드 {len(C['waypoints'] or [])}")

    to_open = args.out

    # 3. 맵 + 로봇 씬 -> 주행 가능한 씬
    if args.robot:
        robot = _resolve(args.robot)
        if not os.path.exists(robot):
            raise SystemExit(f"로봇 씬을 찾을 수 없습니다: {robot}")

        scene = _resolve(args.scene
                         or os.path.splitext(args.out)[0] + "_scene.usd")

        scene, wps, start, notes = compose_scene(
            args.out, robot, scene,
            robot_prim=args.robot_path, env_path=args.env_path,
            start_wp=args.start_wp,
            script_dir=_resolve(args.script_dir))

        where = (f", {start['name']} 시작" if start else "")
        print(f"[3/{steps}] {os.path.basename(scene)}: "
              f"+ {os.path.basename(robot)}{where}")
        for n in notes:
            print(f"  ! {n}")

        to_open = scene

    # script node 가 스캔 로그·주행 궤적을 저장할 폴더 (같은 프로세스에서 읽음)
    os.environ["AMR_MAP_OUTPUT_DIR"] = out_dir

    open_in_isaac_sim(
        to_open,
        top_view=not args.no_top_view,
        robot_cam_view=not args.no_robot_cam,
        robot_cam=args.robot_cam,
        ext_dir=None if args.no_ext else _resolve(args.ext_dir),
        ext_name=None if args.no_ext else args.ext_name)


if __name__ == "__main__":
    main()