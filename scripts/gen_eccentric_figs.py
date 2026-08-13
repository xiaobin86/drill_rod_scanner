#!/usr/bin/env python3
"""生成偏心补偿原理示意图，保存到 docs/figures/。"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 中文字体配置
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "AR PL UMing CN", "WenQuanYi Zen Hei", "DejaVu Sans"
]
plt.rcParams["axes.unicode_minus"] = False


def rotz(th: float) -> np.ndarray:
    c, s = np.cos(np.deg2rad(th)), np.sin(np.deg2rad(th))
    return np.array([[c, -s], [s, c]])


d = 0.055  # 光心偏心 5.5cm
R_axis = 2.0  # 世界点 P 距轴心水平距离

# ============ 图1: 偏心几何（俯视图） ============
fig, ax = plt.subplots(figsize=(7, 7))

# 光心圆弧轨迹
theta = np.linspace(0, 360, 200)
ax.plot(d * np.cos(np.deg2rad(theta)), d * np.sin(np.deg2rad(theta)),
        "b--", lw=1, label=f"光心圆弧轨迹 (半径 {d*100:.1f}cm)")

# 几个转盘角度的光心位置
for th, marker in [(0, "o"), (90, "s"), (180, "^"), (270, "D")]:
    c = rotz(th) @ np.array([d, 0.0])
    ax.plot(c[0], c[1], marker, color="b", ms=8)
    ax.annotate(f"θ={th}°", (c[0], c[1]), textcoords="offset points",
                xytext=(8, 8), fontsize=9, color="b")

# 轴心
ax.plot(0, 0, "k*", ms=18, label="转盘轴心 O")
ax.annotate("O(转盘轴心)", (0, 0), textcoords="offset points",
            xytext=(-60, -20), fontsize=11, color="k")

# 世界固定点 P（如墙上的点）
p_angles = [0, 45, 90]
for th in p_angles:
    p = rotz(th) @ np.array([R_axis, 0.0])
    c = rotz(th) @ np.array([d, 0.0])
    ax.plot(p[0], p[1], "r+", ms=12, mew=2)
    ax.plot([c[0], p[0]], [c[1], p[1]], "r-", lw=1, alpha=0.6)
    if th == 0:
        ax.annotate("世界固定点 P\n(雷达应测到同一位置)", (p[0], p[1]),
                    textcoords="offset points", xytext=(15, -25),
                    fontsize=10, color="r")

# 光心偏移标注
ax.annotate("", xy=(d, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="b", lw=1.5))
ax.text(d / 2, -0.09, f"偏心 d = {d*100:.1f}cm",
        ha="center", fontsize=10, color="b")

ax.set_aspect("equal")
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_xlabel("世界 X (m)")
ax.set_ylabel("世界 Y (m)")
ax.set_title("图1 偏心几何：光心绕转盘轴做圆弧运动\n(俯视图)")
ax.legend(loc="upper left", fontsize=9)
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("docs/figures/eccentric_geometry.png", dpi=150)
plt.close()

# ============ 图2: 无补偿的拼接误差 ============
fig, ax = plt.subplots(figsize=(7, 7))

# 无补偿: 每帧直接绕轴旋转雷达系点 → 光心被当成轴心 → 误差随角度转
# 模拟"墙在 x=2"被无补偿扫描
angles = np.arange(0, 360, 10)
for th in angles:
    r = rotz(th)
    # 无补偿: 世界点 = Rz(θ)·p (p 是光心系测量, 不含 +d)
    # 实际 p = Rz(-θ)·(P - C), 无补偿重建 = Rz(θ)·p = P - C
    p_true = np.array([R_axis, 0.0])
    c = rotz(th) @ np.array([d, 0.0])
    p_err = p_true - c  # 这就是无补偿的结果
    ax.plot(p_err[0], p_err[1], "b.", ms=5, alpha=0.7)

# 真值位置
ax.plot(R_axis, 0, "r+", ms=20, mew=3, label="真值 P (x=2m)")
ax.plot(0, 0, "k*", ms=16, label="转盘轴心")

ax.set_aspect("equal")
ax.set_xlim(-2.5, 2.5)
ax.set_ylim(-2.5, 2.5)
ax.set_xlabel("世界 X (m)")
ax.set_ylabel("世界 Y (m)")
ax.set_title("图2 无偏心补偿：同一世界点被拼成\"各圆柱面\"\n(蓝色=每帧重建位置, 绕真值散开成圆环)")
ax.legend(loc="upper left", fontsize=9)
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("docs/figures/eccentric_error.png", dpi=150)
plt.close()

# ============ 图3: 补偿 vs 无补偿 3D 对比 ============
fig = plt.figure(figsize=(12, 5.5))

# 模拟一面竖直平面墙在 x=2, 扫描 360°
wall = np.array([[2.0, y, z] for y in np.linspace(-1, 1, 7)
                 for z in np.linspace(-1, 1, 7)])
angles = np.arange(0, 360, 8)

for idx, (title, compensate, sub) in enumerate([
    ("无补偿（错误）", False, 121),
    ("有补偿（正确）", True, 122),
]):
    ax = fig.add_subplot(sub, projection="3d")
    all_pts = []
    for th in angles:
        r3 = np.eye(3)
        c3, s3 = np.cos(np.deg2rad(th)), np.sin(np.deg2rad(th))
        r3[:2, :2] = [[c3, -s3], [s3, c3]]
        d3 = np.array([d, 0.0, 0.0])
        C = r3 @ d3  # 光心圆弧
        for P in wall:
            p_radar = r3.T @ (P - C)  # 光心系测量
            if compensate:
                recon = r3 @ (p_radar + d3)  # 先加 d 再旋转
            else:
                recon = r3 @ p_radar          # 不加 d（错误）
            all_pts.append(recon)
    pts = np.array(all_pts)
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], s=1, c="g", alpha=0.6)
    ax.set_xlabel("X"); ax.set_ylabel("Y"); ax.set_zlabel("Z")
    ax.set_title(title, fontsize=13)
    ax.view_init(elev=25, azim=-60)

plt.suptitle("图3 平面墙扫描拼接：偏心补偿前后对比", fontsize=14)
plt.tight_layout()
plt.savefig("docs/figures/eccentric_compare.png", dpi=150)
plt.close()

print("已生成:")
print("  docs/figures/eccentric_geometry.png  (图1 偏心几何)")
print("  docs/figures/eccentric_error.png     (图2 无补偿误差)")
print("  docs/figures/eccentric_compare.png   (图3 补偿对比)")
