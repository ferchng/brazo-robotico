import math
from dataclasses import dataclass


@dataclass
class Geometry:
    h_base: float
    w_base: float
    l1: float
    offset_m3: float
    l2: float
    l3: float
    gripper_len: float
    gripper_width: float


GEOM = Geometry(
    h_base=100.0,
    w_base=35.0,
    l1=113.0,
    offset_m3=27.0,
    l2=137.0,
    l3=60.0,
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


def scale(v, factor: float):
    return (v[0] * factor, v[1] * factor)


def vec(length: float, angle_rad: float):
    return (length * math.cos(angle_rad), length * math.sin(angle_rad))


def perpendicular_offset(length: float, angle_rad: float):
    return (
        length * math.cos(angle_rad + OFFSET_SIGN * math.pi / 2.0),
        length * math.sin(angle_rad + OFFSET_SIGN * math.pi / 2.0),
    )


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


def gripper_corners(m2_deg: float, m3_deg: float, m4_deg: float):
    p0, p1, p1o, p2, p3 = compute_points(m2_deg, m3_deg, m4_deg)
    dir_l3 = (p3[0] - p2[0], p3[1] - p2[1])
    dir_len = math.hypot(dir_l3[0], dir_l3[1])
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


def classify_floor(m2_deg: float, m3_deg: float, m4_deg: float):
    corners = gripper_corners(m2_deg, m3_deg, m4_deg)
    min_y = min(p[1] for p in corners)
    if min_y <= FLOOR_Y + FLOOR_TOUCH_THRESHOLD:
        return "PISO", min_y
    if min_y <= FLOOR_Y + FLOOR_RISK_CLEARANCE:
        return "RIESGO", min_y
    return "OK", min_y


def sweep():
    results = []
    for m2 in range(90, 271, 5):
        for m3 in range(90, 271, 5):
            for m4 in range(75, 261, 5):
                state, min_y = classify_floor(m2, m3, m4)
                if state != "OK":
                    results.append((m2, m3, m4, state, min_y))
    return results


def summarize(results):
    piso = [r for r in results if r[3] == "PISO"]
    riesgo = [r for r in results if r[3] == "RIESGO"]
    print(f"Total casos no OK: {len(results)}")
    print(f"Casos PISO: {len(piso)}")
    print(f"Casos RIESGO: {len(riesgo)}")
    print()

    for title, subset in [("PISO", piso[:30]), ("RIESGO", riesgo[:30])]:
        print(f"Primeros casos {title}:")
        for m2, m3, m4, state, min_y in subset:
          print(f"  M2={m2} M3={m3} M4={m4} -> {state} ({min_y:.1f} mm)")
        print()

    # Reglas aproximadas por barrido
    print("Resumen por M2/M3:")
    m2_values = range(90, 271, 10)
    m3_values = range(90, 271, 10)
    for m2 in m2_values:
        for m3 in m3_values:
            bad_m4 = [m4 for mm2, mm3, m4, _, _ in results if mm2 == m2 and mm3 == m3]
            if bad_m4:
                print(f"  M2={m2}, M3={m3} -> M4 problemático desde {min(bad_m4)} a {max(bad_m4)}")


if __name__ == "__main__":
    print("Barrido simplificado de M2/M3/M4")
    print("Criterio: bloque garra respecto al piso")
    results = sweep()
    summarize(results)
