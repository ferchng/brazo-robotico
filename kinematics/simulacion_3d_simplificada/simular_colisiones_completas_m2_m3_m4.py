import math
from dataclasses import dataclass


@dataclass
class Geometry:
    h_base: float
    w_base: float
    l1: float
    w1: float
    offset_m3: float
    l2: float
    w2: float
    l3: float
    w3: float
    gripper_len: float
    gripper_width: float


GEOM = Geometry(
    h_base=100.0,
    w_base=35.0,
    l1=113.0,
    w1=25.0,
    offset_m3=27.0,
    l2=137.0,
    w2=25.0,
    l3=60.0,
    w3=25.0,
    gripper_len=100.0,
    gripper_width=65.0,
)

OFFSET_SIGN = 1.0
MIRROR_X = True
M2_SIGN = 1.0
M3_SIGN = 1.0
M4_SIGN = 1.0

FLOOR_Y = 0.0
FLOOR_TOUCH_THRESHOLD = 5.0
FLOOR_RISK_CLEARANCE = 30.0


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def scale(v, factor: float):
    return (v[0] * factor, v[1] * factor)


def length(v):
    return math.hypot(v[0], v[1])


def vec(length_value: float, angle_rad: float):
    return (length_value * math.cos(angle_rad), length_value * math.sin(angle_rad))


def perpendicular_offset(length_value: float, angle_rad: float):
    return (
        length_value * math.cos(angle_rad + OFFSET_SIGN * math.pi / 2.0),
        length_value * math.sin(angle_rad + OFFSET_SIGN * math.pi / 2.0),
    )


def point_segment_distance(p, a, b):
    ab = sub(b, a)
    ap = sub(p, a)
    ab_len2 = dot(ab, ab)
    if ab_len2 <= 1e-9:
        return length(sub(p, a))
    t = max(0.0, min(1.0, dot(ap, ab) / ab_len2))
    proj = add(a, scale(ab, t))
    return length(sub(p, proj))


def orient(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def on_segment(a, b, p):
    return (
        min(a[0], b[0]) <= p[0] <= max(a[0], b[0]) and
        min(a[1], b[1]) <= p[1] <= max(a[1], b[1])
    )


def segments_intersect(a, b, c, d):
    o1 = orient(a, b, c)
    o2 = orient(a, b, d)
    o3 = orient(c, d, a)
    o4 = orient(c, d, b)

    if o1 == 0 and on_segment(a, b, c):
        return True
    if o2 == 0 and on_segment(a, b, d):
        return True
    if o3 == 0 and on_segment(c, d, a):
        return True
    if o4 == 0 and on_segment(c, d, b):
        return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def point_in_rect(p, x_min, x_max, y_min, y_max):
    return x_min <= p[0] <= x_max and y_min <= p[1] <= y_max


def compute_points(m2_deg: float, m3_deg: float, m4_deg: float):
    a1 = deg_to_rad((m2_deg - 180.0) * M2_SIGN + 90.0)
    a2 = deg_to_rad((m3_deg - 90.0) * M3_SIGN)
    a3 = deg_to_rad((m4_deg - 180.0) * M4_SIGN)

    p0 = (0.0, GEOM.h_base)
    p1 = add(p0, vec(GEOM.l1, a1))
    p1_offset = add(p1, perpendicular_offset(GEOM.offset_m3, a1))
    p2 = add(p1_offset, vec(GEOM.l2, a1 + a2))
    p3 = add(p2, vec(GEOM.l3, a1 + a2 + a3))
    points = [p0, p1, p1_offset, p2, p3]
    if MIRROR_X:
        points = [(-x, y) for x, y in points]
    return points


def compute_gripper(points):
    _, _, _, p2, p3 = points
    dir_l3 = sub(p3, p2)
    dir_len = length(dir_l3)
    if dir_len <= 1e-6:
        u = (1.0, 0.0)
    else:
        u = (dir_l3[0] / dir_len, dir_l3[1] / dir_len)
    n = (-u[1], u[0])
    half_w = GEOM.gripper_width / 2.0

    g0 = add(p3, scale(n, -half_w))
    g1 = add(p3, scale(n, half_w))
    g2 = add(g1, scale(u, GEOM.gripper_len))
    g3 = add(g0, scale(u, GEOM.gripper_len))
    return [g0, g1, g2, g3]


def classify_floor(gripper):
    min_y = min(p[1] for p in gripper)
    if min_y <= FLOOR_Y + FLOOR_TOUCH_THRESHOLD:
        return "PISO", min_y
    if min_y <= FLOOR_Y + FLOOR_RISK_CLEARANCE:
        return "RIESGO_PISO", min_y
    return "OK", min_y


def collides_with_base(gripper):
    x_min = -GEOM.w_base / 2.0
    x_max = GEOM.w_base / 2.0
    y_min = 0.0
    y_max = GEOM.h_base
    for p in gripper:
        if point_in_rect(p, x_min, x_max, y_min, y_max):
            return True
    edges = [(gripper[i], gripper[(i + 1) % 4]) for i in range(4)]
    rect_edges = [
        ((x_min, y_min), (x_max, y_min)),
        ((x_max, y_min), (x_max, y_max)),
        ((x_max, y_max), (x_min, y_max)),
        ((x_min, y_max), (x_min, y_min)),
    ]
    for e1 in edges:
        for e2 in rect_edges:
            if segments_intersect(e1[0], e1[1], e2[0], e2[1]):
                return True
    return False


def collides_with_link(gripper, a, b, thickness):
    radius = thickness / 2.0
    edges = [(gripper[i], gripper[(i + 1) % 4]) for i in range(4)]
    for p in gripper:
        if point_segment_distance(p, a, b) <= radius:
            return True
    for edge_a, edge_b in edges:
        if segments_intersect(edge_a, edge_b, a, b):
            return True
        # also sample edge endpoints against link thickness
        if point_segment_distance(edge_a, a, b) <= radius:
            return True
        if point_segment_distance(edge_b, a, b) <= radius:
            return True
    return False


def classify_pose(m2, m3, m4):
    points = compute_points(m2, m3, m4)
    gripper = compute_gripper(points)

    floor_state, clearance = classify_floor(gripper)
    if floor_state == "PISO":
        return floor_state, clearance

    if collides_with_base(gripper):
        return "COLISION_BASE", clearance

    p0, p1, p1o, p2, p3 = points
    if collides_with_link(gripper, p0, p1, GEOM.w1):
        return "COLISION_L1", clearance
    if collides_with_link(gripper, p1o, p2, GEOM.w2):
        return "COLISION_L2", clearance

    if floor_state == "RIESGO_PISO":
        return floor_state, clearance

    return "OK", clearance


def sweep():
    results = []
    for m2 in range(90, 271, 5):
        for m3 in range(90, 271, 5):
            for m4 in range(75, 261, 5):
                state, clearance = classify_pose(m2, m3, m4)
                if state != "OK":
                    results.append((m2, m3, m4, state, clearance))
    return results


def summarize(results):
    categories = {}
    for item in results:
        categories.setdefault(item[3], []).append(item)

    print(f"Total casos no OK: {len(results)}")
    for key in sorted(categories.keys()):
        print(f"{key}: {len(categories[key])}")
    print()

    for key in sorted(categories.keys()):
        print(f"Primeros casos {key}:")
        for m2, m3, m4, state, clearance in categories[key][:20]:
            print(f"  M2={m2} M3={m3} M4={m4} -> {state} ({clearance:.1f} mm)")
        print()

    print("Resumen por M2/M3:")
    seen = set()
    for m2 in range(90, 271, 10):
        for m3 in range(90, 271, 10):
            subset = [r for r in results if r[0] == m2 and r[1] == m3]
            if not subset:
                continue
            min_m4 = min(r[2] for r in subset)
            max_m4 = max(r[2] for r in subset)
            states = sorted(set(r[3] for r in subset))
            key = (m2, m3)
            if key in seen:
                continue
            seen.add(key)
            print(f"  M2={m2}, M3={m3} -> M4 problemático {min_m4}..{max_m4} | {', '.join(states)}")


if __name__ == "__main__":
    print("Barrido simplificado completo de M2/M3/M4")
    print("Criterios: piso, base, colision con L1/L2")
    results = sweep()
    summarize(results)
