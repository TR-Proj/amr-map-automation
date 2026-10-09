"""
scan_to_map2.py - Lidar scan log + USD waypoints -> map2 (BlueBotics ANT)

Standalone script. Run with Isaac Sim's python (needs pxr for USD reading):

    ./python.bat scan_to_map2.py --scan output/scan_log.npz --usd output/simple_scene.usd --out output/scanned.map2

Inputs
------
  --scan : NPZ written by the lidar_scanning script node. Contains
             poses        (N, 3)  robot x, y, yaw per scan
             counts       (N,)    number of points in each scan
             points       (M, 2)  lidar-local x, y for all scans concatenated
             lidar_offset (2,)    lidar mounting offset from robot centre
  --usd  : USD stage holding /World/Environment/Waypoints with ant:* attributes

Outputs
-------
  map2 file with
    [Localization.Segments]  fitted from the lidar scans, covariance derived
                             from how each wall was actually observed
    [Navigation.Nodes]       read from the USD waypoint graph
    [Navigation.Home]        first node

Why the scan log rather than a PLY: a PLY keeps only xyz, so the observation
geometry is lost. Keeping pose + local points lets each segment be weighted by
how it was actually seen (range, incidence angle, number of viewpoints), which
is what the covariance in map2 is meant to express.
"""

import argparse
import math
import os
import sys

import numpy as np


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

RANSAC_ITERATIONS = 400
RANSAC_INLIER_DIST = 0.10       # m, point-to-line distance to count as inlier
RANSAC_MIN_INLIERS = 25
MIN_SEGMENT_LENGTH = 0.5        # m, shorter fits are discarded
MAX_POINT_GAP = 0.6             # m, split a fitted line where points thin out

MERGE_ANGLE_DEG = 6.0           # merge fits whose directions agree within this
MERGE_OFFSET = 0.12             # m, and that lie this close to the same line
MERGE_GAP = 1.0                 # m, and whose spans touch or nearly touch

FALLBACK_COV = 5.625e-03        # used when a segment has too few observations
MIN_COV = 1.0e-06               # floor so ANT never sees a zero covariance
MAX_COV = 2.5e-01               # ceiling for very poorly observed segments
ALONG_COV = 1.0e-06             # third covariance term, as in reference maps

RANGE_MAX = 100.0               # m, ignore returns beyond this


# ---------------------------------------------------------------------------
# Scan log
# ---------------------------------------------------------------------------

def load_scans(path):
    """Return (list_of_scans, lidar_offset).

    Each scan carries the sensor origin in world coords, the world-frame
    points, and the range of every point measured from that origin.
    """
    data = np.load(path)
    poses = data["poses"]
    counts = data["counts"]
    points = data["points"]
    offset = data["lidar_offset"] if "lidar_offset" in data else np.zeros(2)

    scans = []
    cursor = 0
    for i in range(len(poses)):
        n = int(counts[i])
        local = points[cursor:cursor + n].astype(np.float64)
        cursor += n
        if n == 0:
            continue

        rx, ry, yaw = float(poses[i][0]), float(poses[i][1]), float(poses[i][2])
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)

        # lidar origin in world coords
        ox = rx + offset[0] * cos_y - offset[1] * sin_y
        oy = ry + offset[0] * sin_y + offset[1] * cos_y

        # local points -> world, through the same mounting offset
        lx = local[:, 0] + offset[0]
        ly = local[:, 1] + offset[1]
        wx = lx * cos_y - ly * sin_y + rx
        wy = lx * sin_y + ly * cos_y + ry

        world = np.column_stack([wx, wy])
        ranges = np.hypot(world[:, 0] - ox, world[:, 1] - oy)

        keep = ranges < RANGE_MAX
        if not np.any(keep):
            continue

        scans.append({
            "origin": np.array([ox, oy]),
            "world": world[keep],
            "ranges": ranges[keep],
        })

    return scans, np.asarray(offset, dtype=np.float64)


def flatten(scans):
    """Concatenate all scans, tracking which scan each point came from."""
    pts = np.concatenate([s["world"] for s in scans], axis=0)
    rng = np.concatenate([s["ranges"] for s in scans], axis=0)
    src = np.concatenate([np.full(len(s["world"]), i, dtype=np.int32)
                          for i, s in enumerate(scans)], axis=0)
    return pts, rng, src


# ---------------------------------------------------------------------------
# Line fitting
# ---------------------------------------------------------------------------

def fit_once(points, rng):
    """One RANSAC pass. Returns (inlier_mask, unit_direction, centroid) or None."""
    n = len(points)
    if n < RANSAC_MIN_INLIERS:
        return None

    best_mask = None
    best_count = 0

    for _ in range(RANSAC_ITERATIONS):
        i, j = rng.choice(n, 2, replace=False)
        a, b = points[i], points[j]
        ab = b - a
        length = np.linalg.norm(ab)
        if length < 1e-6:
            continue
        normal = np.array([-ab[1], ab[0]]) / length
        dist = np.abs((points - a) @ normal)
        mask = dist < RANSAC_INLIER_DIST
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask

    if best_mask is None or best_count < RANSAC_MIN_INLIERS:
        return None

    inliers = points[best_mask]
    centroid = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - centroid, full_matrices=False)
    return best_mask, vh[0], centroid


def split_on_gaps(indices, projections):
    """Break one collinear cluster into runs separated by gaps > MAX_POINT_GAP."""
    order = np.argsort(projections)
    ordered_idx = indices[order]
    ordered_proj = projections[order]

    runs = []
    start = 0
    for k in range(1, len(ordered_proj)):
        if ordered_proj[k] - ordered_proj[k - 1] > MAX_POINT_GAP:
            runs.append((ordered_idx[start:k], ordered_proj[start:k]))
            start = k
    runs.append((ordered_idx[start:], ordered_proj[start:]))
    return runs


def extract_segments(points, seed=0):
    """Greedy RANSAC: pull out lines until nothing substantial is left.

    Each result keeps the indices of the points that produced it, so covariance
    can be computed from the same observations.
    """
    rng = np.random.default_rng(seed)
    remaining = np.arange(len(points))
    segments = []

    while len(remaining) >= RANSAC_MIN_INLIERS:
        subset = points[remaining]
        result = fit_once(subset, rng)
        if result is None:
            break

        mask, direction, centroid = result
        chosen = remaining[mask]
        projections = (points[chosen] - centroid) @ direction

        for run_idx, run_proj in split_on_gaps(chosen, projections):
            if len(run_idx) < RANSAC_MIN_INLIERS:
                continue
            p1 = centroid + direction * run_proj.min()
            p2 = centroid + direction * run_proj.max()
            if np.linalg.norm(p2 - p1) < MIN_SEGMENT_LENGTH:
                continue
            segments.append({
                "p1": p1,
                "p2": p2,
                "direction": direction,
                "indices": run_idx,
            })

        remaining = remaining[~mask]

    return merge_collinear(segments, points)


def merge_collinear(segments, points):
    """Fuse fits that describe the same wall.

    Greedy RANSAC peels a thick wall in layers: it takes the inliers within
    RANSAC_INLIER_DIST, and the returns just outside that band survive to be
    fitted again on the next pass. Left alone, one wall becomes several nearly
    identical segments. Two fits are the same wall when they point the same
    way, sit on the same line, and their spans overlap or nearly touch.
    """
    if not segments:
        return []

    cos_limit = math.cos(math.radians(MERGE_ANGLE_DEG))
    merged = []

    for seg in sorted(segments,
                      key=lambda s: -np.linalg.norm(s["p2"] - s["p1"])):
        target = None

        for candidate in merged:
            direction = candidate["direction"]

            # same orientation? (either sign, a wall has no preferred end)
            if abs(float(direction @ seg["direction"])) < cos_limit:
                continue

            # same infinite line?
            normal = np.array([-direction[1], direction[0]])
            anchor = candidate["p1"]
            offsets = [abs(float((seg["p1"] - anchor) @ normal)),
                       abs(float((seg["p2"] - anchor) @ normal))]
            if max(offsets) > MERGE_OFFSET:
                continue

            # overlapping or adjacent spans?
            own = sorted([float((candidate["p1"] - anchor) @ direction),
                          float((candidate["p2"] - anchor) @ direction)])
            other = sorted([float((seg["p1"] - anchor) @ direction),
                            float((seg["p2"] - anchor) @ direction)])
            if other[0] > own[1] + MERGE_GAP or other[1] < own[0] - MERGE_GAP:
                continue

            target = candidate
            break

        if target is None:
            merged.append({
                "direction": seg["direction"],
                "indices": seg["indices"],
            })
        else:
            target["indices"] = np.concatenate([target["indices"],
                                                seg["indices"]])

        # refit whichever entry just changed, using every point it now owns
        entry = merged[-1] if target is None else target
        owned = points[entry["indices"]]
        centroid = owned.mean(axis=0)
        _, _, vh = np.linalg.svd(owned - centroid, full_matrices=False)
        direction = vh[0]
        projections = (owned - centroid) @ direction
        entry["direction"] = direction
        entry["p1"] = centroid + direction * projections.min()
        entry["p2"] = centroid + direction * projections.max()

    return [s for s in merged
            if np.linalg.norm(s["p2"] - s["p1"]) >= MIN_SEGMENT_LENGTH]


# ---------------------------------------------------------------------------
# Covariance from observations
# ---------------------------------------------------------------------------

def orient_segment(segment, points, origins, point_scan):
    """Flip the endpoints so the segment's normal faces the sensor.

    ANT derives a wall's facing from p1 -> p2: rotate that vector 90 degrees
    and you get the normal it draws as a small triangle. The fit itself has no
    preferred direction - SVD returns an axis whose sign is arbitrary - so
    without this step the triangles point inward on some walls and outward on
    others.

    The scan log settles it: a lidar can only strike the face turned towards
    it, so the correct normal is the one pointing back at the sensor.

    Each viewpoint votes rather than averaging the directions to them. A long
    wall gets seen from positions spread along its whole length, and those
    directions cancel out to something nearly parallel to the wall - an average
    whose sign is then decided by noise. Every individual viewpoint still sits
    firmly on one side, so counting sides is stable where averaging is not.
    """
    idx = segment["indices"]
    centroid = points[idx].mean(axis=0)

    direction = segment["p2"] - segment["p1"]
    length = np.linalg.norm(direction)
    if length < 1e-9:
        return
    direction = direction / length
    normal = np.array([-direction[1], direction[0]])

    seen_from = origins[np.unique(point_scan[idx])] - centroid
    side = seen_from @ normal
    side = side[np.abs(side) > 1e-9]
    if side.size == 0:
        return

    if int((side > 0).sum()) * 2 < side.size:
        segment["p1"], segment["p2"] = segment["p2"].copy(), segment["p1"].copy()


def segment_covariance(segment, points, origins, point_scan):
    """Estimate endpoint covariance for one segment.

    Three things degrade a segment's reliability, and all three are visible in
    the scan log:
      - residual spread of the inliers about the fitted line
      - how obliquely the surface was hit (grazing hits are less trustworthy)
      - how many distinct viewpoints contributed
    """
    idx = segment["indices"]
    pts = points[idx]

    centroid = pts.mean(axis=0)
    direction = segment["direction"]
    normal = np.array([-direction[1], direction[0]])

    residual = (pts - centroid) @ normal
    spread = float(np.var(residual)) if len(residual) > 1 else FALLBACK_COV

    scan_ids = np.unique(point_scan[idx])
    viewpoints = len(scan_ids)

    # incidence: 1.0 when hit head-on, towards 0 when grazing
    incidence = []
    for sid in scan_ids:
        to_wall = centroid - origins[sid]
        norm = np.linalg.norm(to_wall)
        if norm < 1e-6:
            continue
        incidence.append(abs(float((to_wall / norm) @ normal)))
    mean_incidence = float(np.mean(incidence)) if incidence else 0.5

    cov = spread
    cov /= max(mean_incidence, 0.15)            # grazing hits inflate uncertainty
    # more viewpoints tighten it, but only mildly - a wall seen 60 times is not
    # 60x more certain than one seen 10 times, the residual spread already says
    # most of it
    cov *= (1.0 + 4.0 / math.sqrt(max(viewpoints, 1)))

    if not np.isfinite(cov) or cov <= 0.0:
        cov = FALLBACK_COV

    return float(np.clip(cov, MIN_COV, MAX_COV)), viewpoints, mean_incidence


# ---------------------------------------------------------------------------
# USD waypoints
# ---------------------------------------------------------------------------

def load_waypoints(usd_path, scope_path):
    try:
        from pxr import Usd, UsdGeom
    except ImportError:
        # Isaac Sim's python refuses pxr until Kit has started. Booting a
        # headless app costs a minute but is the only way to read the stage.
        try:
            from isaacsim import SimulationApp
            print("Starting Isaac Sim (headless) to read the stage...")
            SimulationApp({"headless": True})
            from pxr import Usd, UsdGeom
        except Exception as exc:
            print("pxr unavailable, waypoints skipped: " + str(exc))
            return []

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        print("Could not open USD: " + usd_path)
        return []

    scope = stage.GetPrimAtPath(scope_path)
    if not scope or not scope.IsValid():
        print("Waypoint scope not found: " + scope_path)
        return []

    nodes = []
    for child in scope.GetChildren():
        xform = UsdGeom.Xformable(child)
        matrix = xform.ComputeLocalToWorldTransform(0)
        translation = matrix.ExtractTranslation()
        rotation = matrix.ExtractRotationMatrix()
        heading = math.degrees(math.atan2(rotation[0][1], rotation[0][0]))

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

        nodes.append({
            "id": node_id,
            "name": name,
            "x": float(translation[0]),
            "y": float(translation[1]),
            "heading": heading,
            "links": links,
        })

    nodes.sort(key=lambda n: n["id"])
    return nodes


# ---------------------------------------------------------------------------
# map2 output
# ---------------------------------------------------------------------------

def num(value):
    """Format a coordinate the way ANT writes them: 6 significant digits, no
    trailing zeros, plain '0' rather than '0.000000'."""
    return "{0:g}".format(round(float(value), 6))


def write_map2(path, segments, nodes, map_id=1, level=1, version=1):
    lines = []
    lines.append('Description "" ~')
    lines.append("Id {0} ~".format(map_id))
    lines.append("Level {0} ~".format(level))
    lines.append("Version {0} ~".format(version))

    lines.append("Bin Localization.Segments")
    for i, seg in enumerate(segments, start=1):
        p1, p2, cov = seg["p1"], seg["p2"], seg["cov"]
        # ANT stores three covariance components per endpoint; the third is the
        # along-segment term, held at a small constant as in reference maps.
        cov_text = "{0:.3e} {0:.3e} {1:.3e}".format(cov, ALONG_COV)
        lines.append(
            "    Segment id={i} p1={x1} {y1} p2={x2} {y2} "
            "cov1={c} cov2={c} ~".format(
                i=i, x1=num(p1[0]), y1=num(p1[1]),
                x2=num(p2[0]), y2=num(p2[1]), c=cov_text))
    lines.append("~")

    lines.append("Bin Navigation.Nodes")
    lines.append("    defaultRadius=1")
    lines.append("    defaultMaxRadius=1")
    lines.append("    curveType=Spline4")
    for node in nodes:
        links = " ".join(str(v) for v in node["links"])
        heading = math.radians(node["heading"])
        lines.append(
            '    Node id={nid} name="{name}" pose={x} {y} {h} rt=1 '
            "links={links} ~".format(
                nid=node["id"], name=node["name"],
                x=num(node["x"]), y=num(node["y"]), h=num(heading),
                links=links))
    if nodes:
        lines.append("    Home node={0} ~".format(nodes[0]["id"]))
    lines.append("~")

    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Build a map2 from a lidar scan log and USD waypoints.")
    parser.add_argument("--scan", default="output/scan_log.npz",
                        help="NPZ scan log written by the lidar_scanning node")
    parser.add_argument("--usd", default="output/simple_scene.usd",
                        help="USD stage containing the waypoint graph")
    parser.add_argument("--out", default="output/scanned.map2",
                        help="output map2 path")
    parser.add_argument("--waypoints", default="/World/Environment/Waypoints",
                        help="prim path of the waypoint scope")
    parser.add_argument("--seed", type=int, default=0,
                        help="RANSAC seed, for reproducible fits")
    parser.add_argument("--map-id", type=int, default=1, help="Id field in the map2 header")
    parser.add_argument("--report", action="store_true",
                        help="print per-segment observation quality")
    args = parser.parse_args()

    # relative paths follow this script, not whatever folder you ran it from
    base = os.path.dirname(os.path.abspath(__file__))

    def resolve(path):
        return path if os.path.isabs(path) else os.path.join(base, path)

    args.scan = resolve(args.scan)
    args.usd = resolve(args.usd)
    args.out = resolve(args.out)

    out_parent = os.path.dirname(args.out)
    if out_parent:
        os.makedirs(out_parent, exist_ok=True)

    if not os.path.exists(args.scan):
        print("Scan log not found: " + args.scan)
        return 1

    scans, offset = load_scans(args.scan)
    if not scans:
        print("Scan log contains no usable returns.")
        return 1

    points, _, point_scan = flatten(scans)
    origins = np.array([s["origin"] for s in scans])
    print("Scans: {0}   points: {1}   lidar offset: ({2:.3f}, {3:.3f})".format(
        len(scans), len(points), offset[0], offset[1]))

    segments = extract_segments(points, seed=args.seed)
    if not segments:
        print("No segments could be fitted. Try lowering RANSAC_MIN_INLIERS "
              "or scanning for longer.")
        return 1

    weak = 0
    for seg in segments:
        orient_segment(seg, points, origins, point_scan)
        cov, viewpoints, incidence = segment_covariance(
            seg, points, origins, point_scan)
        seg["cov"] = cov
        seg["viewpoints"] = viewpoints
        seg["incidence"] = incidence
        if viewpoints < 3 or incidence < 0.35:
            weak += 1

    print("Segments: {0}   weakly observed: {1}".format(len(segments), weak))

    if args.report:
        print("")
        print(" idx   length   cov         views  incidence")
        for i, seg in enumerate(segments):
            length = float(np.linalg.norm(seg["p2"] - seg["p1"]))
            flag = "  <- weak" if (seg["viewpoints"] < 3
                                   or seg["incidence"] < 0.35) else ""
            print("  {0:3d}  {1:6.2f}m  {2:.3e}  {3:4d}   {4:.2f}{5}".format(
                i, length, seg["cov"], seg["viewpoints"],
                seg["incidence"], flag))
        print("")

    nodes = load_waypoints(args.usd, args.waypoints)
    print("Waypoints: " + str(len(nodes)))

    write_map2(args.out, segments, nodes, map_id=args.map_id)
    print("Wrote " + args.out)

    if weak:
        print("")
        print("{0} segment(s) rest on few viewpoints or grazing hits. Those "
              "carry the largest covariance and are where ANT localisation is "
              "most likely to struggle - worth re-driving that area or adding "
              "a physical reflector.".format(weak))

    return 0


if __name__ == "__main__":
    sys.exit(main())