#!/usr/bin/env python3
"""生成双偏心（offset-x -0.055, offset-z -0.025）示意图，保存到 docs/figures/。"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 中文字体配置
plt.rcParams["font.sans-serif"] = [
    "Noto Sans CJK SC", "AR PL UMing CN", "WenQuanYi Zen Hei", "DejaVu Sans"
]
plt.rcParams["axes.unicode_minus"] = False

# 双偏心参数（雷达系 x-z 平面，单位米）
OFF_X = -0.055   # x 方向偏移（负 = 后方）
OFF_Z = -0.025   # z 方向偏移（负 = 左方）
R = np.hypot(OFF_X, OFF_Z)  # 合成偏移半径

fig, ax = plt.subplots(figsize=(8, 8))

# 光心圆弧轨迹（绕转盘轴心，半径 R）
theta = np.linspace(0, 360, 300)
ax.plot(R * np.cos(np.deg2rad(theta)), R * np.sin(np.deg2rad(theta)),
        "b--", lw=1.2, label=f"光心圆弧轨迹 (合成半径 {R*100:.1f}cm)")

# 转盘轴心 O（原点）
ax.plot(0, 0, "k*", ms=20)
ax.annotate("O (转盘轴心)", (0, 0), textcoords="offset points",
            xytext=(-70, -22), fontsize=12, color="k")

# 光心 C（偏移位置）
ax.plot(OFF_X, OFF_Z, "b^", ms=14)
ax.annotate(f"C (光心)\n({OFF_X*100:.1f}cm, {OFF_Z*100:.1f}cm)",
            (OFF_X, OFF_Z), textcoords="offset points",
            xytext=(15, 12), fontsize=11, color="b")

# 偏移分量箭头
ax.annotate("", xy=(OFF_X, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="r", lw=2))
ax.annotate("", xy=(0, OFF_Z), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="g", lw=2))
ax.text(OFF_X / 2, 0.012, f"offset-x = {OFF_X*100:.1f}cm (后方)",
        ha="center", fontsize=11, color="r")
ax.text(0.012, OFF_Z / 2, f"offset-z = {OFF_Z*100:.1f}cm (左方)",
        va="center", fontsize=11, color="g", rotation=90)

# 合成偏移箭头
ax.annotate("", xy=(OFF_X, OFF_Z), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="b", lw=1.5))

# 雷达系坐标轴（x 向前, z 向右 —— 俯视图）
ax.annotate("", xy=(0.12, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="k", lw=1.2))
ax.annotate("", xy=(0, 0.12), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="k", lw=1.2))
ax.text(0.125, 0.005, "雷达系 +x (向前)", fontsize=10, color="k")
ax.text(0.005, 0.125, "雷达系 +z (向右)", fontsize=10, color="k", rotation=90)

# 几个转盘角度位置示意
for th, marker in [(0, "o"), (90, "s"), (180, "^"), (270, "D")]:
    c = np.array([R * np.cos(np.deg2rad(th)), R * np.sin(np.deg2rad(th))])
    ax.plot(c[0], c[1], marker, color="b", ms=6, alpha=0.6)

ax.set_aspect("equal")
ax.set_xlim(-0.12, 0.12)
ax.set_ylim(-0.12, 0.12)
ax.set_xlabel("x (米)")
ax.set_ylabel("z (米)")
ax.set_title("图 光心双偏心示意（俯视图，x-z 平面）\n"
             "转盘旋转时光心绕轴心做圆弧运动")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.3)
plt.tight_layout()
plt.savefig("docs/figures/eccentric_dual_offset.png", dpi=150)
plt.close()
print("已生成 docs/figures/eccentric_dual_offset.png")
print(f"合成偏移半径 = {R*100:.1f}cm")
