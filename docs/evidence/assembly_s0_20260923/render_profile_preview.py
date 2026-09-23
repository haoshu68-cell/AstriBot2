#!/usr/bin/env python3
"""Preview verified DXF geometry and its centered-pose screening matrix."""
from pathlib import Path
import json, math
import numpy as np
import ezdxf
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Polygon, Circle, Rectangle

root=Path(__file__).resolve().parents[2]/'assets/assembly_20260923/profiles_v2'
report=json.loads((root/'report.json').read_text())
font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
font_manager.fontManager.addfont(font)
plt.rcParams.update({'font.family':font_manager.FontProperties(fname=font).get_name(),'axes.unicode_minus':False,'font.size':11,'figure.facecolor':'#f6f8fc','savefig.facecolor':'#f6f8fc','svg.fonttype':'path'})
ids=list(report['profiles']); labels=['圆形','矩形','三角形','六边形','D形','切角形']
centers=[(-100,55),(0,55),(100,55),(-100,-55),(0,-55),(100,-55)]

def draw(ax,path,center=(0,0),fill='#edb868',edge='#846124'):
    entities=list(ezdxf.readfile(path).modelspace())
    p=np.array(center)
    if entities[0].dxftype()=='CIRCLE':
        ax.add_patch(Circle(p,entities[0].dxf.radius,facecolor=fill,edgecolor=edge,lw=.9))
    else:
        if entities[0].dxftype()=='LWPOLYLINE':
            points=np.array(list(entities[0].get_points('xy')))
        else:
            # Sampling is for raster preview only; validated DXF retains its exact ARC.
            a=entities[0]; angles=np.linspace(math.radians(a.dxf.start_angle),math.radians(a.dxf.end_angle),240)
            points=np.c_[np.cos(angles),np.sin(angles)]*a.dxf.radius
        ax.add_patch(Polygon(points+p,closed=True,facecolor=fill,edgecolor=edge,lw=.9))

fig=plt.figure(figsize=(15,7.8)); gs=fig.add_gridspec(1,2,width_ratios=[1.18,1.],wspace=.23,left=.045,right=.96,top=.81,bottom=.19)
ax=fig.add_subplot(gs[0]);ax.set_aspect('equal');ax.set_facecolor('#f6f8fc')
ax.add_patch(Rectangle((-160,-120),320,240,facecolor='#dce5eb',edgecolor='#526a80',lw=1.3))
for name,label,center in zip(ids,labels,centers):
    x,y=center
    ax.add_patch(Rectangle((x-40,y-40),80,80,fill=False,edgecolor='#9babba',linestyle='--',lw=.7))
    draw(ax,root/f'hole_{name}_c3mm.dxf',center,fill='white',edge='#326e94')
    draw(ax,root/f'part_{name}.dxf',center)
    ax.text(x,y-46,label,ha='center',va='top',fontsize=11,color='#23394d')
ax.set_xlim(-173,173);ax.set_ylim(-130,130);ax.set_xlabel('board x / mm');ax.set_ylabel('board y / mm')
ax.set_title('已读回的 CAD 轮廓 · 单边间隙 3 mm',fontsize=14,pad=14,color='#1f344a')
ax.spines[['top','right']].set_visible(False)

mx=fig.add_subplot(gs[1]);mx.set_xlim(-.5,5.5);mx.set_ylim(5.5,-.5);mx.set_aspect('equal')
for r in report['cross_fit']:
    if abs(r['clearance_level_m']-.003)>1e-9:continue
    i,j=ids.index(r['part_id']),ids.index(r['slot_id'])
    cls=r['classification']
    color=('#d5e9f3' if i==j else '#f7d7a1' if cls=='POSITIVE_CLEARANCE_WITNESS' else '#e8eaf0' if cls=='TANGENCY_ONLY_WITNESS' else '#ffffff')
    mx.add_patch(Rectangle((j-.5,i-.5),1,1,facecolor=color,edgecolor='#d5dce5',lw=1))
    text=(f"{r['best_sampled_clearance_m']*1000:.2f}" if cls=='POSITIVE_CLEARANCE_WITNESS' else '接触' if cls=='TANGENCY_ONLY_WITNESS' else '—')
    mx.text(j,i,text,ha='center',va='center',color='#243d53',fontsize=11)
mx.set_xticks(range(6),labels,fontsize=10);mx.set_yticks(range(6),labels,fontsize=10)
mx.set_xlabel('目标槽位');mx.set_ylabel('工件截面')
mx.set_title('可容纳筛查 · 表内为最优采样净空 / mm',fontsize=14,pad=14,color='#1f344a')
for s in mx.spines.values():s.set_visible(False)
fig.text(.045,.95,'六形状槽板：截面已验证，任务身份仍必须绑定',fontsize=23,weight='bold',color='#182b40')
fig.text(.045,.892,'27 份 DXF 独立读回通过  ·  18 组正确配对满足 3 / 1 / 0.5 mm 标称间隙',fontsize=13,color='#536a80')
fig.text(.045,.103,'橙色单元格是错槽可放入的正例；即使收紧到 0.5 mm，圆形→六边孔、切角形→矩形孔仍成立。',fontsize=12,color='#704d18')
fig.text(.045,.055,'仅二维直壁截面；中心对齐、yaw 步长 0.5°。未发现≠任意位姿都放不入。未验证三维抓持柱、倒角、夹爪扫掠或物理接触。',fontsize=10.5,color='#667b8e')
fig.savefig(root.parent/'profile_screening_v2.png',dpi=150)
fig.savefig(root.parent/'profile_screening_v2.svg')
svg_path = root.parent/'profile_screening_v2.svg'
svg_path.write_text('\n'.join(line.rstrip() for line in svg_path.read_text().splitlines()) + '\n')
plt.close(fig)
