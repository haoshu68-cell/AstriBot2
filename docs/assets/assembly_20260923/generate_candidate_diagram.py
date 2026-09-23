#!/usr/bin/env python3
"""Reproduce a review illustration, not a CAD or contact-validation artifact.

All lengths are in millimetres. SVG text is emitted as paths for portability.
Requires only numpy and matplotlib. Writes beside this script.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Rectangle
import numpy as np


OUT = Path(__file__).resolve().parent
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
font_manager.fontManager.addfont(FONT)
plt.rcParams.update({
    "font.family": font_manager.FontProperties(fname=FONT).get_name(),
    "font.size": 12,
    "axes.unicode_minus": False,
    "svg.fonttype": "path",
    "figure.facecolor": "#F6F8FC",
    "savefig.facecolor": "#F6F8FC",
})

INK = "#18283C"
MUTED = "#576B80"
EDGE = "#64788F"
ACCENT = "#147C92"
ORANGE = "#AA5B0E"
CLEARANCE = 3.0


def ring(n, radius, phase=0):
    angle = np.arange(n) * 2 * np.pi / n + phase
    return np.c_[np.cos(angle), np.sin(angle)] * radius


def normal_offset_polygon(points, distance):
    """Illustrative mitered offset of a convex CCW polygon.

    Circles are sampled for display only. This does not define corner radii,
    tolerances, chamfer manufacture, or a production collision mesh.
    """
    points = np.asarray(points, dtype=float)
    edge = np.roll(points, -1, axis=0) - points
    outward = np.c_[edge[:, 1], -edge[:, 0]]
    outward /= np.linalg.norm(outward, axis=1)[:, None]
    support = np.sum(outward * points, axis=1) + distance
    return np.array([
        np.linalg.solve(np.array([outward[i - 1], outward[i]]),
                        [support[i - 1], support[i]])
        for i in range(len(points))
    ])


def txt(ax, x, y, text, size=12, color=INK, **kw):
    return ax.text(x, y, text, fontsize=size, color=color, **kw)


def dim_h(ax, x0, x1, y, text, extension=None, color=MUTED):
    ax.annotate("", (x1, y), (x0, y),
                arrowprops={"arrowstyle": "<->", "lw": 1.1, "color": color})
    if extension is not None:
        for x in (x0, x1):
            ax.plot([x, x], [extension, y + 3], color=color, lw=.8)
    txt(ax, (x0+x1)/2, y+3, text, size=12, color=color,
        ha="center", va="bottom", bbox={"fc": "#F6F8FC", "ec": "none", "pad": 1})


def dim_v(ax, x, y0, y1, text, color=MUTED, rotation=90):
    ax.annotate("", (x, y1), (x, y0),
                arrowprops={"arrowstyle": "<->", "lw": 1.1, "color": color})
    txt(ax, x-2, (y0+y1)/2, text, size=11, color=color,
        ha="right", va="center", rotation=rotation,
        bbox={"fc": "#F6F8FC", "ec": "none", "pad": 1})


def callout(ax, target, anchor, text, color=MUTED, size=11, align="left"):
    ax.annotate(text, xy=target, xytext=anchor, ha=align, va="center",
                fontsize=size, color=color,
                arrowprops={"arrowstyle": "-", "lw": 1, "color": color,
                            "connectionstyle": "angle,angleA=0,angleB=90,rad=4"})


fig = plt.figure(figsize=(20, 13))
fig.text(.045, .955, "多形状盲槽装配 · 保留顶部抓持段", fontsize=27,
         color=INK, weight="bold")
fig.text(.045, .918, "候选设计 / 非加工图 / 未做接触验证", fontsize=21,
         color="#9A4906", weight="bold")
fig.text(.955, .923, "AstriBot  ·  2026-09-23\n尺寸单位：mm", fontsize=12,
         color=MUTED, ha="right", va="center")

fig.text(.045, .863, "01   水平板俯视", fontsize=18, color=INK, weight="bold")
fig.text(.61, .863, "02   名义插入状态剖面", fontsize=18, color=INK, weight="bold")
fig.text(.61, .838, "选矩形工件的 30 mm 方向示意；夹爪未绘入", fontsize=11, color=MUTED)

plan = fig.add_axes([.035, .335, .535, .495])
plan.set_aspect("equal")
plan.set_xlim(-194, 184)
plan.set_ylim(-155, 155)
plan.axis("off")
plan.add_patch(Rectangle((-160, -120), 320, 240, fc="#DDE5EF", ec=EDGE, lw=1.8))
plan.plot([-160, 160], [0, 0], color="#A6B4C4", lw=.8, ls=(0, (7, 5)))
plan.plot([0, 0], [-120, 120], color="#A6B4C4", lw=.8, ls=(0, (7, 5)))

arc = np.linspace(np.pi/3, 5*np.pi/3, 150)
d_shape = np.c_[np.cos(arc), np.sin(arc)] * 20
shapes = [
    ("圆", "Ø 36", ring(180, 18), (-100, 55), "#247E98"),
    ("矩形", "42 × 30", np.array([[-21,-15],[21,-15],[21,15],[-21,15]]), (0,55), "#BA7622"),
    ("等边三角", "外接半径 24", ring(3,24,np.pi/2), (100,55), "#647FA8"),
    ("正六边形", "对边 36", ring(6,36/np.sqrt(3)), (-100,-55), "#34827A"),
    ("D 形", "R20，保留 x ≤ 10", d_shape, (0,-55), "#85649D"),
    ("单角削切键形", "40 × 30，削切 8 × 8", np.array([[-20,-15],[20,-15],[20,7],[12,15],[-20,15]]), (100,-55), "#AC6461"),
]
for name, dimension, pts, center, color in shapes:
    center = np.array(center)
    plan.add_patch(Rectangle(center-40,80,80,fc="#FFFFFF",ec="#A8B6C6",lw=1.3))
    outer = normal_offset_polygon(pts, CLEARANCE)
    plan.add_patch(Polygon(outer+center,closed=True,fc=color,alpha=.18,ec="none"))
    plan.add_patch(Polygon(outer+center,closed=True,fill=False,ec=color,lw=1.7))
    plan.add_patch(Polygon(pts+center,closed=True,fill=False,ec=INK,lw=1.2,ls=(0,(4,3))))
    txt(plan,center[0],center[1]+35,name,11,ha="center",va="center")
    txt(plan,center[0],center[1]-33,dimension,10,ha="center",va="center",color=MUTED)

dim_h(plan,-160,160,139,"基板 320",extension=120)
dim_v(plan,-179,-120,120,"基板 240")
dim_h(plan,-100,0,-138,"中心距 100")
dim_h(plan,0,100,-138,"中心距 100")
plan.annotate("+x",(155,0),(133,0),ha="right",va="bottom",fontsize=10,color=MUTED,
              arrowprops={"arrowstyle":"->","lw":1,"color":MUTED})
txt(plan,4,113,"+y",10,color=MUTED)

fig.text(.057, .31, "6 个可更换孔座：80 × 80；中心 x = −100 / 0 / 100，y = ±55", fontsize=12, color=INK)
fig.text(.057, .281, "实线：孔壁轮廓示意     虚线：名义插入截面     颜色：仅辅助区分", fontsize=11, color=MUTED)
fig.text(.057, .255, "孔形按单边法向间隙 c = 3 示意偏置；孔口倒角未在俯视图展开。", fontsize=11, color=MUTED)

side = fig.add_axes([.594,.334,.365,.483])
side.set_aspect("equal")
side.set_xlim(-72,90)
side.set_ylim(-48,93)
side.axis("off")
# Section through a nominal 80 mm replaceable socket. Chamfer slants are
# illustrative; their angles and corner details are not manufacturing data.
socket = np.array([[-40,-35],[40,-35],[40,0],[20,0],[18,-2],
                   [18,-20],[-18,-20],[-18,-2],[-20,0],[-40,0]])
side.add_patch(Polygon(socket,closed=True,fc="#DFE6F0",ec=EDGE,lw=1.6,hatch="///"))
part = np.array([[-13,-20],[13,-20],[15,-18],[15,5],[10,5],
                 [10,80],[-10,80],[-10,5],[-15,5],[-15,-18]])
side.add_patch(Polygon(part,closed=True,fc="#90C9D3",ec=ACCENT,lw=1.8))
side.plot([-10,10],[5,5],lw=1,ls=(0,(3,3)),color=ACCENT)
# Highlight the proposed contact/grasp centre, without asserting finger bounds.
side.plot([-26,28],[50,50],color=ORANGE,lw=1.5,ls=(0,(5,3)))
side.scatter([0],[50],s=42,color=ORANGE,zorder=5)
side.plot([-45,46],[0,0],color=INK,lw=.8,ls=(0,(6,4)))
txt(side,44,-2,"板面 z = 0",10,ha="left",va="top")
dim_v(side,-55,-20,80,"工件总高 100")
dim_v(side,-38,5,80,"抓持柱高 75")
dim_v(side,32,-20,0,"孔深 20")
dim_v(side,-46,-35,0,"板厚 35")
dim_v(side,50,-35,-20,"底厚 15")
callout(side,(10,67),(30,77),"统一抓持柱\n截面 26 × 20",size=12,color=INK)
callout(side,(2,50),(30,51),"候选抓持中心\n距 tip 70\n高于板面 50",size=12,color=ORANGE)
callout(side,(15,5),(31,24),"截面转换处\n高于板面 5",size=11)
callout(side,(19,-1),(48,10),"孔口倒角 2",size=10)
callout(side,(-14,-19),(-38,-43),"tip 导向倒角 2",size=10)
side.plot([-18,-15],[-8,-8],color=ORANGE,lw=3)
callout(side,(-16.5,-8),(-37,16),"单边间隙\nc = 3",size=11,color=ORANGE,align="center")
txt(side,30,88,"工件顶面：板上 80",11,color=MUTED)
txt(side,-1,-23,"tip：板下 20",10,ha="center",va="top",color=ACCENT)

fig.text(.61, .31, "插入段：tip 起 0–25；抓持柱：tip 起 25–100。", fontsize=12, color=INK)
fig.text(.61, .281, "图中名义到底位置只用于尺寸关系；不代表已通过插入或抓持。", fontsize=11, color=MUTED)
fig.text(.61, .255, "倒角斜线为示意，角度、圆角和加工公差尚未定义。", fontsize=11, color=MUTED)

fig.add_artist(plt.Line2D([.045,.955],[.226,.226],transform=fig.transFigure,
                        color="#C7D2DF",lw=1.2))
fig.text(.045,.189,"间隙分级",fontsize=15,color=INK,weight="bold")
fig.text(.045,.16,"首级 c = 3  →  后续 c = 1 / 0.5",fontsize=13,color=ACCENT)
fig.text(.045,.13,"圆孔总直径间隙 = 2c；矩形两对边尺寸差各为 2c。",fontsize=11,color=MUTED)
fig.text(.045,.104,"三角、六角和键形按边法向偏置；不按外接半径等量放大。",fontsize=11,color=MUTED)

fig.text(.50,.189,"验证边界与后续阶段",fontsize=15,color=INK,weight="bold")
fig.text(.50,.16,"当前阶段保留顶部抓持段；齐平嵌入另设后续工件。",fontsize=13,color=INK)
fig.text(.50,.13,"完整夹爪最低点、扫掠包络、释放空间与接触稳定性均待验证。",fontsize=11,color=MUTED)
fig.text(.50,.104,"本图不是精密 CAD、加工图、碰撞模型或视觉身份检测的实现证明。",fontsize=11,color=MUTED)
fig.text(.045,.05,"图纸状态：设计评审示意  ·  六孔轮廓为二维候选几何  ·  固定结构与紧固方式另行设计",fontsize=10,color=MUTED)

for extension in ("png","svg"):
    path = OUT / f"assembly_candidate_top_grasp.{extension}"
    fig.savefig(path,dpi=200)
    if extension == "svg":
        path.write_text("\n".join(line.rstrip() for line in path.read_text().splitlines()) + "\n")
    print(path)
plt.close(fig)
