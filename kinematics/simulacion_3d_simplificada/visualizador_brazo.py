import math
from dataclasses import dataclass

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.widgets import Slider


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

FLOOR_Y = 0.0
FLOOR_TOUCH_THRESHOLD = 5.0
FLOOR_RISK_CLEARANCE = 30.0

# Ajustes visuales pedidos:
# - offset espejado respecto de la version anterior
# - espejo horizontal del modelo
OFFSET_SIGN = 1.0
MIRROR_X = True

# Calibracion visual:
# postura erecta real del brazo:
# - M2 = 180
# - M3 = 90
# - M4 = 180
#
# Estos signos permiten invertir cada eje si visualmente gira al reves.
M2_SIGN = 1.0
M3_SIGN = 1.0
M4_SIGN = 1.0


def deg_to_rad(deg: float) -> float:
    return deg * math.pi / 180.0


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def vec(length: float, angle_rad: float):
    return (length * math.cos(angle_rad), length * math.sin(angle_rad))


def scale(v, factor: float):
    return (v[0] * factor, v[1] * factor)


def perpendicular_offset(length: float, angle_rad: float):
    return (
        length * math.cos(angle_rad + OFFSET_SIGN * math.pi / 2.0),
        length * math.sin(angle_rad + OFFSET_SIGN * math.pi / 2.0),
    )


def compute_points(m2_deg: float, m3_deg: float, m4_deg: float, geom: Geometry):
    # Mapeo desde grados reales del motor a angulos geometricos del visualizador.
    # Queremos que:
    # - M2=180 deje L1 vertical
    # - M3=90 deje L2 alineado con L1
    # - M4=180 deje L3 alineado con L2
    a1 = deg_to_rad((m2_deg - 180.0) * M2_SIGN + 90.0)
    a2 = deg_to_rad((m3_deg - 90.0) * M3_SIGN)
    a3 = deg_to_rad((m4_deg - 180.0) * M4_SIGN)

    p0 = (0.0, geom.h_base)
    p1 = add(p0, vec(geom.l1, a1))
    p1_offset = add(p1, perpendicular_offset(geom.offset_m3, a1))
    p2 = add(p1_offset, vec(geom.l2, a1 + a2))
    p3 = add(p2, vec(geom.l3, a1 + a2 + a3))
    points = [p0, p1, p1_offset, p2, p3]

    if MIRROR_X:
        points = [(-x, y) for x, y in points]

    return points


def draw_arm(ax, m2_deg: float, m3_deg: float, m4_deg: float, geom: Geometry):
    ax.clear()
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.25)
    ax.set_title("Visualizador 2D simplificado del brazo")

    base_rect = Rectangle(
        (-geom.w_base / 2.0, 0.0),
        geom.w_base,
        geom.h_base,
        facecolor="#475569",
        edgecolor="#cbd5e1",
        alpha=0.8,
    )
    ax.add_patch(base_rect)
    ax.axhline(FLOOR_Y, color="#94a3b8", linewidth=1.5, linestyle="--")
    ax.axhline(FLOOR_Y + FLOOR_TOUCH_THRESHOLD, color="#ef4444", linewidth=1.5, linestyle="-.")
    ax.axhline(FLOOR_Y + FLOOR_RISK_CLEARANCE, color="#fbbf24", linewidth=1.5, linestyle=":")

    p0, p1, p1o, p2, p3 = compute_points(m2_deg, m3_deg, m4_deg, geom)

    xs = [p0[0], p1[0], p1o[0], p2[0], p3[0]]
    ys = [p0[1], p1[1], p1o[1], p2[1], p3[1]]

    ax.plot([p0[0], p1[0]], [p0[1], p1[1]], linewidth=8, color="#22c55e", label="L1")
    ax.plot([p1[0], p1o[0]], [p1[1], p1o[1]], linewidth=5, color="#f59e0b", label="offset M3")
    ax.plot([p1o[0], p2[0]], [p1o[1], p2[1]], linewidth=8, color="#38bdf8", label="L2")
    ax.plot([p2[0], p3[0]], [p2[1], p3[1]], linewidth=8, color="#f43f5e", label="L3")

    # Bloque simplificado de la garra:
    # mantenemos la geometria general, pero anclado en el otro vertice
    # del extremo de L3 para evitar superposicion con la barra.
    dir_l3 = (p3[0] - p2[0], p3[1] - p2[1])
    dir_len = math.hypot(dir_l3[0], dir_l3[1])
    if dir_len > 1e-6:
        u = (dir_l3[0] / dir_len, dir_l3[1] / dir_len)
    else:
        u = (1.0, 0.0)
    n = (-u[1], u[0])
    half_w = geom.gripper_width / 2.0

    # El eje M5 debe coincidir con el medio del lado de union del bloque.
    # Usamos p3 como centro de ese lado, no como esquina.
    g0 = add(p3, scale(n, -half_w))
    g1 = add(p3, scale(n, half_w))
    g2 = add(g1, scale(u, geom.gripper_len))
    g3 = add(g0, scale(u, geom.gripper_len))

    gx = [g0[0], g1[0], g2[0], g3[0], g0[0]]
    gy = [g0[1], g1[1], g2[1], g3[1], g0[1]]
    ax.fill(gx, gy, color="#a78bfa", alpha=0.25, label="bloque garra 100x65")
    ax.plot(gx, gy, color="#c4b5fd", linewidth=2)

    min_gripper_y = min(gy[:-1])
    clearance_mm = min_gripper_y - FLOOR_Y
    touches_floor = min_gripper_y <= FLOOR_Y + FLOOR_TOUCH_THRESHOLD
    risk_floor = min_gripper_y <= FLOOR_Y + FLOOR_RISK_CLEARANCE

    if touches_floor:
        clearance_text = f"PISO = {clearance_mm:.1f} mm"
        clearance_color = "#ef4444"
        clearance_state = "PISO"
    elif risk_floor:
        clearance_text = f"RIESGO PISO = {clearance_mm:.1f} mm"
        clearance_color = "#f59e0b"
        clearance_state = "RIESGO PISO"
    else:
        clearance_text = f"OK = {clearance_mm:.1f} mm"
        clearance_color = "#22c55e"
        clearance_state = "OK"

    xs.extend([g0[0], g1[0], g2[0], g3[0]])
    ys.extend([g0[1], g1[1], g2[1], g3[1]])

    m5 = p3
    xs.append(m5[0])
    ys.append(m5[1])

    ax.scatter(xs, ys, s=50, color="white", zorder=5)
    ax.text(p0[0], p0[1] + 8, "M2", color="white")
    ax.text(p1[0], p1[1] + 8, "fin L1", color="white")
    ax.text(p1o[0], p1o[1] + 8, "offset M3", color="white")
    ax.text(p2[0], p2[1] + 8, "M4", color="white")
    ax.text(p3[0], p3[1] + 8, "pinza", color="white")
    ax.text(m5[0], m5[1] + 8, "M5", color="white")
    ax.text(g2[0], g2[1] + 8, "bloque garra", color="white")
    ax.text(g0[0], FLOOR_Y + FLOOR_TOUCH_THRESHOLD + 6, "piso +5mm", color="#ef4444")
    ax.text(g0[0], FLOOR_Y + FLOOR_RISK_CLEARANCE + 6, "riesgo piso +30mm", color="#fbbf24")

    ax.legend(loc="upper right")

    margin = 80
    ax.set_xlim(min(xs) - margin, max(xs) + margin)
    ax.set_ylim(min(0.0, min(ys) - margin), max(ys) + margin)

    info = (
        f"M2={m2_deg:.1f} deg  "
        f"M3={m3_deg:.1f} deg  "
        f"M4={m4_deg:.1f} deg\n"
        f"erecto real: M2=180  M3=90  M4=180\n"
        f"signos: M2={M2_SIGN:+.0f}  M3={M3_SIGN:+.0f}  M4={M4_SIGN:+.0f}\n"
        f"offset_sign={OFFSET_SIGN:+.0f}  mirror_x={MIRROR_X}\n"
        f"Hbase={geom.h_base}  Wbase={geom.w_base}  "
        f"L1={geom.l1}  offsetM3={geom.offset_m3}  "
        f"L2={geom.l2}  L3={geom.l3}\n"
        f"garra={geom.gripper_len} x {geom.gripper_width}\n"
        f"{clearance_text}"
    )
    ax.text(
        0.02,
        0.02,
        info,
        transform=ax.transAxes,
        color="white",
        fontsize=10,
        bbox=dict(facecolor="black", alpha=0.35, edgecolor="none"),
    )
    ax.text(
        0.98,
        0.02,
        clearance_state,
        transform=ax.transAxes,
        color=clearance_color,
        fontsize=12,
        ha="right",
        bbox=dict(facecolor="black", alpha=0.35, edgecolor="none"),
    )


def main():
    fig, ax = plt.subplots(figsize=(11, 8))
    plt.subplots_adjust(left=0.1, bottom=0.22)

    init_m2 = 180.0
    init_m3 = 180.0
    init_m4 = 120.0

    draw_arm(ax, init_m2, init_m3, init_m4, GEOM)

    ax_m2 = plt.axes((0.15, 0.12, 0.7, 0.03))
    ax_m3 = plt.axes((0.15, 0.08, 0.7, 0.03))
    ax_m4 = plt.axes((0.15, 0.04, 0.7, 0.03))

    slider_m2 = Slider(ax_m2, "M2", 90.0, 270.0, valinit=init_m2)
    slider_m3 = Slider(ax_m3, "M3", 90.0, 270.0, valinit=init_m3)
    slider_m4 = Slider(ax_m4, "M4", 75.0, 260.0, valinit=init_m4)

    def update(_):
        draw_arm(ax, slider_m2.val, slider_m3.val, slider_m4.val, GEOM)
        fig.canvas.draw_idle()

    slider_m2.on_changed(update)
    slider_m3.on_changed(update)
    slider_m4.on_changed(update)

    plt.show()


if __name__ == "__main__":
    main()
