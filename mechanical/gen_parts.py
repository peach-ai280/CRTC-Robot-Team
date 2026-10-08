# -*- coding: utf-8 -*-
"""
gen_parts.py —— CRTC2026 机器人总动员战队 3D 打印件生成器

【用法】
    python gen_parts.py
产出：
    mechanical/stl/*.stl        可直接导入切片软件的模型
    mechanical/图纸集.html       带尺寸标注的图纸集（浏览器打开）

【改尺寸的正确姿势】
    下面 PARAMS 区里所有数值都是毫米，改完直接重跑脚本就出新的 STL。
    不要去改 STL 本身 —— 3D 打印件的迭代成本几乎为零，改模型才是正道。

【打印参数建议（PLA）】
    层高 0.2mm / 壁厚 1.2mm（3 圈）/ 填充 25% / 底面试温度 60℃
    受力件（马达座、大臂、夹爪、铲）填充提到 45%，壁厚 1.6mm
"""

import os, math, html
import numpy as np
from shapely.geometry import Polygon, Point, box
from shapely.ops import triangulate, orient, unary_union
from stl import mesh

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(OUT, "stl")
os.makedirs(STL_DIR, exist_ok=True)

# ============================================================================
# PARAMS —— 全车关键尺寸（mm）。改这里。
# ============================================================================
TRACK       = 136.0    # 左右轮中心距  => 半轮距 68
WHEELBASE   = 180.0    # 前后轮中心距  => 半轴距 90
WHEEL_D     = 60.0     # 麦轮外径
WHEEL_W     = 26.0     # 麦轮宽度

DECK_W      = 162.0    # 甲板宽（车宽 = 162 < 195 规则上限）
DECK_T      = 4.0      # 甲板厚
DECK_SEG_L  = 128.0    # 单段甲板长（两段 + 20 搭接 = 236 ≈ 车长）
SPLICE_L    = 60.0

# ★ 2026-10-04 由 15 改 22：把「电控托盘 + STM32 + 驱动」下移到夹层
#   原因（血账，别再改回去）：甲板总长只有 236mm，而
#       机械臂底座 ~52 + 电控托盘 110 + 储仓 138 = 300mm  →  超 64mm，纵向上放不下。
#   解法：电子件全部进夹层（放在金属板面上，托盘是绝缘的 3mm 打印件），
#         甲板上面只留「机械臂（前）+ 储仓（后）」，两者各占一半，互不干涉。
#   夹层净高 = COL_H - 托盘厚 3 = 19mm，够放 STM32 + DRV8833（含排针约 12~14mm）。
#   代价：整车总高 +7mm（甲板顶面 30+22+4 = 56mm，离 210 红线还远）。
#   ⚠ 螺杆要跟着换：22 + 4 = 26mm → 用 M3x30，不是 M3x25。
COL_H       = 22.0     # 甲板立柱高（金属底盘面 -> 甲板下表面）
COL_D       = 12.0     # 立柱直径

# ★★★ 甲板 -> 金属底盘的固定方式（2026-10-04 整个改掉，别再改回圆孔）★★★
#
#   【为什么改】原来的做法是「甲板打 4 个圆孔，去对金属板上的孔」。
#   这条路从一开始就错：那块板是手工折弯件，卖家自己注明 ±0.5~1cm 误差，
#   它上面现成的孔还是给通用 TT 马达留的，跟我们的甲板根本不是一套。
#   结果就是用户打出来发现孔对不上——这是我的设计失误，不是打错。
#
#   【现在怎么做】把圆孔换成**沿车宽方向的长圆槽**：
#     - 螺钉可以在槽里自由滑动，(left/right) 位置现场定
#     - 就算金属板上一个能用孔的没有，扎带也能穿过这条槽把甲板绑到板上
#     - 槽宽 4.0mm：M3 螺钉能穿，标准 2.5mm 扎带也能穿
#
#   【覆盖范围】槽的 y 范围是 ±(MOUNT_Y ∓ MOUNT_SLOT_H) = ±45 ~ ±75，
#   也就是说金属板实测宽度只要在 **100 ~ 160mm** 之间，都能找到可用位置，
#   **一毫米都不用量**。低于 100mm 的板不存在（四个 TT 马达都塞不下）。
#
#   出问题时的现象与处理：
#     螺钉滑到槽里最靠外的位置还是探不到板 → 板太窄，改用扎带绕过板边缘。
#     螺钉拧不紧（槽是通的，螺母没地方吃力）→ 加一片 M3 平垫/加大垫片。
# ★★ 用户 2026-10-04 实测的金属底盘 / 电机数据（cm 换算，尺子量的，按 ±1mm 误差处理）
#   底板： 长 25.5cm=255mm（括号 18.7cm=187mm）｜ 宽 15cm=150mm（14.3 / 9.4）
#          高 2.7cm=27mm ｜ 板厚 0.15cm=1.5mm ｜ 板上现有螺丝孔 Φ4mm 和 Φ5mm 两种
#   电机： 外径 Φ2.0cm=20mm ｜ 输出轴离地 2.5cm=25mm ｜ 法兰安装孔孔距 1.65cm=16.5mm
#
#   ⚠ 两个数我无法自洽，设计上用「可调」绕开，不猜：
#     ① 麦轮半径 30mm 要求轮轴离地 30mm，但实测"电机/输出轴离地 25mm" → 差 5mm。
#        可能是尺子量到的是电机外壳底边，也可能买到的麦轮是 Φ50 不是 Φ60。
#        → 影响的是"铲唇离地"，已改成现场贴地调（见 05_scoop_floor 的说明）。
#        若确认麦轮是 Φ50，改 board.h 的 WHEEL_RADIUS = 25.0 即可，其它不用动。
#     ② 括号里的 187 / 143 / 9.4 我按"轴距 187、轮距 143"处理但也可能是平面段尺寸。
#        → 甲板安装孔已升级成**十字槽**（见下），纵向也能调 ±12mm，两种解读都装得上。
#        轮距若真是 143，把 board.h 的 ROBOT_HALF_TRACK 从 68.0 改成 71.5 再烧录。
#
#   ★ 板上现成的孔是 Φ4 / Φ5：M3 螺钉（Φ3）能穿 Φ4 的孔（余 1mm，够调）；
#     Φ5 的孔穿 M3 会晃 → 加一片 M3 大垫片（外径 8~10mm）再拧，或者直接用扎带。
MOUNT_Y        = 60.0   # 槽中心（半宽）→ 比原来的 70 更靠内，给窄板留余量
MOUNT_SLOT_X   = 12.0   # ★ 新增：槽在**车长方向**也能调 ±12mm（原来是死位置）
MOUNT_SLOT_H   = 15.0   # 半槽长 -> 槽总长 30mm，可调范围 ±15mm
MOUNT_SLOT_W   = 4.0    # 槽宽：M3 螺钉 & 标准扎带都能过
MOUNT_X0       = 14.0   # 前/后段甲板安装槽，距该段前缘
MOUNT_X1       = 114.0  # 同一段上的另一组（纵向孔距 = 100）

# ----------------------------------------------------------------------------
# ★★★ 道具方块尺寸 —— 全车所有拾取/储存类零件的唯一源头 ★★★
#   ⚠⚠ 手册 V1.4（2026-10-04 收到）把方块从 40mm 改小到 30mm！
#      p.17 原文：「外部立方体规格 30mm*30mm*30mm，12 条棱边均有倒角处理；
#                内部镂空立方体规格 15mm*15mm*15mm；倒角 45 度、斜外立面 2.121mm」
#      V1.3 是 40 / 20 / 3.0。改成 30 以后：
#        - 壁厚 (30-15)/2 = 7.5mm，比原来薄 2.5mm
#        - 转 45° 时的投影宽只有 42.4mm（原来 56.6）→ 喉口、V 型口都要跟着缩
#        - 单块质量 11.29g（原来没给），比想象中轻得多 → 储仓可以考虑不带隔板多装
#      下面所有跟方块有关的尺寸都从这里推导，**改这一个数就够了**。
# ----------------------------------------------------------------------------
CUBE        = 30.0     # 能量单元 / 能量核心外部边长（两者同规格）
CUBE_HOLLOW = 15.0     # 内部镂空边长 → 壁厚 = (30-15)/2 = 7.5mm
CUBE_DIAG   = CUBE * 1.4142   # 水平转 45° 时的最大投影宽 42.4mm
CUBE_CLEAR  = 2.0      # 单边配合余量（打印误差 + 方块倒角）
CUBE_MASS   = 11.29    # g，手册给的组委会打印工艺下的单块质量
# ★ 2026-10-04 第二次重算：储仓原本只有 3 格 = 一次带 3 块，**这是设计事故**。
#   规则是「比赛结束时车上还剩几个道具」算分（见 docs/01 第一节第 1 条），
#   方块上了车就再也不用卸下来 → **储仓越大分数越高**，不存在"能带走就行了"。
#   旧尺寸 BIN_W=56 / BIN_L=138 分 3 格：每格 42mm 只能摆 1 块 → 总共 3 块 = 4.5 分。
#   保守战术要拿 8~12 块，所以必须扩容。
#
#   新尺寸（去掉隔板，当一个大兜用）：
#     外 132 × 116 → 内 126 × 110（壁厚 3）
#     长向：126 / 30 = 4.2 → **4 列**
#     宽向：110 / 30 = 3.6 → **3 排**
#     => 4 × 3 = **12 块**（12 × 11.29g = 135g，车总重稳够）
#   为什么不再做大：一场一共才 23 块（18 白 + 5 黄），12 块已经是战术上限的两倍，
#   再大只会让重心升高、打印费料、臂最后一排放不进去。
#
#   ⚠ 隔板（14_bin_divider）从「必装」改成「选装」：装了会压到 9 块（3 区 × 3 排），
#     但方块不会前后串位。默认不装，追求最大载量。
BIN_N       = 12       # 储仓设计容量（块）—— 旧值是 3，已作废
BIN_W       = 116.0    # 储仓外宽（旧 56）→ 内侧净宽 110 = 3 排 × 30 + 20 余量
BIN_L       = 132.0    # 储仓外长（旧 138）→ 内侧净长 126 = 4 列 × 30 + 6 余量
BIN_H       = 52.0     # 储仓外高（旧 50）→ 方块 30 + 22 上沿，急停不蹦出去
BIN_T       = 3.0      # 储仓壁厚

# ★ 2026-10-04 重算：收集铲的进深被 290mm 车长红线卡死，原来是 90mm，装上去超长
#   板长 255mm（实测）→ 车头前伸件最多只能有 290 - 255 = 35mm。
#   原来 SCOOP_L=90 意味着整车长 255+90 = 345mm，**直接违规**。
#   改成 28mm：整车长 255 + 28 = 283mm，留 7mm 余量。
#   功能不受影响：方块 30mm，进深 28mm 刚好能把一整块兜进来（对中是靠两侧壁 +
#   后方挡板限制，不是靠"铲得很深"）。唇贴地、车慢推，方块进兜即停。
SCOOP_W     = 96.0     # 收集铲外宽（原 150，跟着进深一起收，且不超板宽 150）
SCOOP_L     = 28.0     # 收集铲进深（原 90 —— 会超 290 红线，见上）
SCOOP_LIP   = 6.0      # 铲唇厚度（贴地那一段）

M3          = 3.2      # M3 过孔
M25         = 2.6      # M2.5 过孔（SG90 自攻）
M2          = 2.2

# ============================================================================
# 几何工具
# ============================================================================
def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]

def rrect(x0, y0, x1, y1, r, qs=4):
    """圆角矩形（用作腰形槽 / 过孔）"""
    return box(min(x0, x1) + r, min(y0, y1) + r, max(x0, x1) - r, max(y0, y1) - r).buffer(r, quad_segs=qs)

def circ(cx, cy, r, qs=16):
    return Point(cx, cy).buffer(r, quad_segs=qs)

def slot_cross(cx, cy, hx, hy, w=4.0):
    """十字槽 —— 车长方向和车宽方向都能调的安装孔

    为什么升级成十字：单向槽只能吸收一个方向的位置误差。用户给的数里
    "板长 255（187）"有两种可能的解读，纵向孔位到底能落在哪并不确定。
    十字槽让螺钉在 (2*hx) x (2*hy) 的窗口里任意选位置，
    配上扎带兜底 → 板上孔在哪都无所谓，真正做到了不用量。
    """
    a = rrect(cx - hx, cy - w / 2, cx + hx, cy + w / 2, w / 2)   # 沿车长
    b = rrect(cx - w / 2, cy - hy, cx + w / 2, cy + hy, w / 2)   # 沿车宽
    return unary_union([a, b])

def slot_y(cx, y_lo, y_hi, w=4.0):
    """沿车宽方向（y）的长圆槽 —— 全车吸收容差的关键件

    为什么不用圆孔：金属底盘是手工折弯件，孔位不可信（卖家自己注 ±0.5~1cm）。
    与其猜它的孔在哪，不如让甲板上的孔变成一条槽，螺钉/扎带能在里面自由选位置。
    """
    return rrect(cx - w / 2, min(y_lo, y_hi), cx + w / 2, max(y_lo, y_hi), w / 2)

def build(outline, holes=()):
    """外轮廓减去孔，返回带洞 Polygon"""
    poly = Polygon(outline)
    if not poly.is_valid:
        poly = poly.buffer(0)
    for h in holes:
        g = h if isinstance(h, Polygon) else Polygon(h)
        poly = poly.difference(g)
    if not poly.is_valid:
        poly = poly.buffer(0)
    return orient(poly, sign=1.0)

def polys(poly):
    return list(poly.geoms) if poly.geom_type == "MultiPolygon" else [poly]

# ---------------------------------------------------------------- 网格生成
def _faces_from_polygon(poly, z0, z1):
    """把带洞多边形挤出成三角面片（含上下盖 + 侧壁）"""
    tris = []
    for g in polys(poly):
        # ⚠ 不要改成 triangulate(g, edges=True)：shapely 2.x 里 edges=True 返回的是
        #   **约束边的 LineString 列表，不是三角形**（2026-10-04 试过，网格直接全烂，
        #   非配对边从 286 飙到 21686）。默认 edges=False 返回三角形是对的。
        #
        #   残留问题：edges=False 拿到的是「顶点集 Delaunay 剖分里落在多边形内的三角形」，
        #   带孔/凹多边形会在贴近边界处漏极窄的一圈 -> 上盖与侧壁不完全贴合，
        #   实测仍有少量非配对边（25 件合计 286 条，集中在甲板/电池卡箍/摄像头支架）。
        #   数量很小，主流切片软件（含 Bambu Studio）都能自动修复，暂时接受。
        for t in triangulate(g):
            c = t.representative_point()
            if not g.contains(c):
                continue
            t = orient(t, sign=1.0)          # 外环 CCW
            xs, ys = t.exterior.coords.xy
            p = [(xs[i], ys[i]) for i in range(3)]
            tris.append(p)
    faces = []
    z0, z1 = float(z0), float(z1)
    for p in tris:
        a, b, c = p
        # 下盖（法向 -z）
        faces.append([(a[0], a[1], z0), (c[0], c[1], z0), (b[0], b[1], z0)])
        # 上盖（法向 +z）
        faces.append([(a[0], a[1], z1), (b[0], b[1], z1), (c[0], c[1], z1)])
    # 侧壁：沿多边形真实的边界环走。
    #   ⚠ 以前是「沿每个三角形的三条边」生成，结果内部边大量重复 -> 网格不闭合，
    #   切片软件会弹"模型有错误 / 自动修复"（PrusaSlicer 尤其明显），
    #   修复过程还可能改动形状。改成只沿外环+内环走，网格才真正水密。
    for g in polys(poly):
        # ⚠ 外环要 CCW、内环（洞）要 CW —— 两者旋向相反！
        #   早先一律统一成 CCW，结果孔的侧壁法向朝反了，
        #   算体积时孔被"加"进去而不是减掉（误差 = 2 倍孔体积，立柱实测偏差 15%）。
        rings = [(list(g.exterior.coords), True)]
        rings += [(list(r.coords), False) for r in g.interiors]
        for ring, is_outer in rings:
            if len(ring) < 3:
                continue
            s = 0.0
            for i in range(len(ring) - 1):
                s += ring[i][0] * ring[i + 1][1] - ring[i + 1][0] * ring[i][1]
            if is_outer and s < 0:
                ring = ring[::-1]
            if (not is_outer) and s > 0:
                ring = ring[::-1]
            n = len(ring) - 1 if ring[0] == ring[-1] else len(ring)
            for i in range(n):
                a = ring[i]
                b = ring[(i + 1) % n]
                faces.append([(a[0], a[1], z0), (b[0], b[1], z0), (b[0], b[1], z1)])
                faces.append([(a[0], a[1], z0), (b[0], b[1], z1), (a[0], a[1], z1)])
    return faces

def _resample(outline, n):
    """把闭合轮廓重采样成 n 个点（用于放样）"""
    g = Polygon(outline) if not isinstance(outline, Polygon) else outline
    if not g.is_valid:
        g = g.buffer(0)
    ring = orient(g, sign=1.0).exterior
    length = ring.length
    return [(ring.interpolate(length * i / n).x, ring.interpolate(length * i / n).y) for i in range(n)]

def _faces_from_loft(layers, zs, n=96):
    """layers: 从下到上的轮廓列表；zs: 对应高度"""
    rs = [_resample(l, n) for l in layers]
    faces = []
    for k in range(len(zs) - 1):
        z0, z1 = float(zs[k]), float(zs[k + 1])
        for i in range(n):
            a = rs[k][i];       b = rs[k][(i + 1) % n]
            c = rs[k + 1][(i + 1) % n]; d = rs[k + 1][i]
            faces.append([(a[0], a[1], z0), (b[0], b[1], z0), (c[0], c[1], z1)])
            faces.append([(a[0], a[1], z0), (c[0], c[1], z1), (d[0], d[1], z1)])
    # 上下盖
    faces += _cap(rs[-1], zs[-1], up=True)
    faces += _cap(rs[0],  zs[0],  up=False)
    return faces

def _cap(pts, z, up=True):
    g = orient(Polygon(pts), sign=1.0)
    faces = []
    for t in triangulate(g):
        if not g.contains(t.representative_point()):
            continue
        t = orient(t, sign=1.0)
        xs, ys = t.exterior.coords.xy
        a, b, c = (xs[0], ys[0]), (xs[1], ys[1]), (xs[2], ys[2])
        z = float(z)
        if up:
            faces.append([(a[0], a[1], z), (b[0], b[1], z), (c[0], c[1], z)])
        else:
            faces.append([(a[0], a[1], z), (c[0], c[1], z), (b[0], b[1], z)])
    return faces

def signed_volume_of(faces):
    """三角面对原点的有向体积之和（绝对值即实体体积 mm^3）。

    自己算而不用 numpy-stl 的 get_mass_properties，是因为后者对非水密网格
    会直接报 "mesh is not closed" 并算错。

    注：2026-09-30 修好了网格（侧壁改为沿真实边界环生成），现在模型是水密的，
    这个自算体积和 get_mass_properties 的结果应当一致 —— 可以用这一点做校验。"""
    v = 0.0
    for a, b, c in faces:
        v += (a[0] * (b[1] * c[2] - b[2] * c[1])
              - a[1] * (b[0] * c[2] - b[2] * c[0])
              + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0
    return abs(v)


# PLA 密度 1.24 g/cm3；切片后实际用料系数（外壳实心 + 内部 25~30% 填充）
# ⚠ 这个 0.45 是**全仓库统一口径**：gen_parts.py / gen_shell.py / tools/print_bill.py
#   三处必须一致，否则用户把几个数加起来会对不上。宁可估高一点（超重比超预算严重）。
PLA_DENSITY = 1.24e-3      # g/mm^3
SOLID_RATIO = 0.45


def save_stl(name, faces):
    if not faces:
        print("  !! 空模型:", name); return 0.0
    data = np.zeros(len(faces), dtype=mesh.Mesh.dtype)
    for i, f in enumerate(faces):
        data["vectors"][i] = np.array(f, dtype=np.float64)
    m = mesh.Mesh(data)
    m.save(os.path.join(STL_DIR, name + ".stl"))
    return signed_volume_of(faces)


# ============================================================================
# DXF 导出 —— 给会用 SolidWorks / Fusion / Creo 的队友改图用
#   本工程所有零件都是「2D 轮廓 + 拉伸」，所以一张 DXF 就是这个件的完整定义：
#   队友导入 → 选轮廓 → 拉伸到厚度 → 打设计时用的是哪些孔（这些孔在 DXF 里是内环）。
#   纯标准库手写，不依赖 ezdxf / cadquery（用户机器上没有也照样跑）。
# ============================================================================
DXF_DIR = os.path.join(OUT, "dxf")
os.makedirs(DXF_DIR, exist_ok=True)

def _num(v):
    s = "%.4f" % float(v)
    return s.rstrip("0").rstrip(".") if "." in s else s

def _rings_of(poly):
    """取出多边形的外环 + 所有内环（孔），供 DXF 画闭合多段线"""
    out = []
    for g in polys(poly):
        out.append([(float(p[0]), float(p[1])) for p in g.exterior.coords])
        for r in g.interiors:
            out.append([(float(p[0]), float(p[1])) for p in r.coords])
    return out

def write_dxf(path, items):
    """items: [(图层名, [ring, ring, ...]), ...]"""
    L = ["0", "SECTION", "2", "HEADER", "9", "$ACADVER", "1", "AC1015",
         "9", "$INSUNITS", "70", "4", "0", "ENDSEC",
         "0", "SECTION", "2", "ENTITIES"]
    n = 0
    for layer, rings in items:
        for ring in rings:
            pts = list(ring)
            if pts and pts[0] != pts[-1]:
                pts.append(pts[0])
            if len(pts) < 3:
                continue
            L += ["0", "LWPOLYLINE", "8", layer, "100", "AcDbEntity",
                  "100", "AcDbPolyline", "90", str(len(pts)), "70", "1", "43", "0.0"]
            for x, y in pts:
                L += ["10", _num(x), "20", _num(y)]
            n += 1
    L += ["0", "ENDSEC", "0", "EOF"]
    with open(path, "w", encoding="ascii", newline="\n") as f:
        f.write("\n".join(L) + "\n")
    return n

DXF_ALL = []      # 汇总到一个大文件的图层列表

def save_dxf(name, poly, thickness):
    rings = _rings_of(poly)
    layer = "%s_t%.1f" % (name, thickness)
    write_dxf(os.path.join(DXF_DIR, name + ".dxf"), [(layer, rings)])
    DXF_ALL.append((layer, rings))
    return len(rings)


# ============================================================================
# 零件清单
# ============================================================================
PARTS = []   # (文件名, 中文名, 数量, poly或loft信息, 工艺备注)

# 打印批次：1=底盘（先打，装完车能跑） 2=收集与储仓 3=机械臂与机构
# 顺序不能乱——底盘不出来，后面全卡住
BATCH = {
    "01_deck_front": 1, "02_deck_rear": 1, "03_deck_splice": 1, "04_column": 1,
    "18_electronics_tray": 1, "19_battery_clip": 1, "20_bumper_front": 1,
    "21_anti_tip": 1,
    "05_scoop_floor": 2, "05b_scoop_wall_L": 2, "05c_scoop_wall_R": 2,
    "05d_scoop_back": 2,
    "12_bin_floor": 2, "13_bin_side": 2, "14_bin_divider": 2, "15_bin_flap": 2,
    "22_ramp_anchor": 2,
    "06_limit_lever": 3, "07_grip_finger_L": 3, "08_grip_finger_R": 3,
    "09_arm_upper": 3, "10_arm_fore": 3, "11_servo_horn_plate": 3,
    "11b_claw_adapter": 3,
    "16_rack_hook": 3, "17_cave_probe": 3, "23_cam_mast": 3,
}
# 易损/易丢，建议各多打一份（放在 05_备件多打一份 文件夹里）
SPARE = {"04_column", "06_limit_lever", "07_grip_finger_L",
         "08_grip_finger_R", "11_servo_horn_plate", "11b_claw_adapter"}

BATCH_NAME = {1: "① 底盘必打", 2: "② 收集与储仓", 3: "③ 机械臂与机构"}


def slice_param(p):
    """按件的厚度/尺寸给建议切片参数：层高 / 填充 / 支撑 / 理由"""
    minx, miny, maxx, maxy = p["poly"].bounds
    longest = max(maxx - minx, maxy - miny)
    t = p["t"]
    big = longest > 120.0
    # 注意判定顺序：先看厚度再看面积。薄板件的内部空间本来就被外壳占满了，
    # 填充分辨率对它意义不大，真正要防的是翘边，所以大面积的一律加 Brim。
    if t <= 2.5:
        return ("0.20", "40%", "Brim 8mm" if big else "否",
                "超薄件（≤2.5mm），填充给满才不容易掰断")
    if t <= 4.5:
        return ("0.20", "30%", "Brim 8mm" if big else "否",
                "薄板件，切片时基本会被外壳填满；大面积要防翘边")
    if big:
        return ("0.20", "25%", "Brim 8mm", "大件怕翘边，加一圈裙边；别用整面支撑")
    return ("0.20", "30%", "否", "结构件受力，填充别低于 25%")


def add_extrude(fname, cn, qty, outline, holes, t, note, z0=0.0):
    poly = build(outline, holes)
    vol = save_stl(fname, _faces_from_polygon(poly, z0, z0 + t))
    save_dxf(fname, poly, t)
    PARTS.append(dict(f=fname, cn=cn, qty=qty, poly=poly, t=t, zs=(z0, z0 + t),
                      note=note, vol=vol))

def add_loft(fname, cn, qty, layers, zs, note):
    vol = save_stl(fname, _faces_from_loft(layers, zs))
    top = orient(Polygon(layers[-1]), sign=1.0)
    save_dxf(fname, top, zs[-1] - zs[0])   # 放样件只导顶层轮廓，并在文件名注明厚度
    PARTS.append(dict(f=fname, cn=cn, qty=qty, poly=top, t=(zs[-1] - zs[0]),
                      zs=(zs[0], zs[-1]), note=note, vol=vol))

# ---------------------------------------------------------------- 1. 甲板
_h_deck_front = [
    # ↓ 金属底盘配合：长圆槽（不是圆孔！见 PARAMS 里 MOUNT_Y 的说明）
    slot_cross(MOUNT_X0,  MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    slot_cross(MOUNT_X0, -MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    slot_cross(MOUNT_X1,  MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    slot_cross(MOUNT_X1, -MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    # 机构安装腰形槽（SG90 孔距 27.5 / MG995 孔距 49.5 都能对上）
    rrect(36, -30, 52, 30, 2.0),      # 舵机位 A（纵向槽）
    rrect(72, -26, 88, 26, 2.0),      # 舵机位 B
    # 减重孔阵 ⚠ x 必须避开 14/114 两条安装槽（槽宽 4，孔 r=5 -> |x-槽中心|>7）
    #           2026-10-04 把网格从 24..112 改成 28..100，否则会和长圆槽撞在一起
    *[circ(x, y, 5) for x in range(28, 124, 24) for y in (-52, -34, -16, 16, 34, 52)],
    # 走线孔
    circ(64, 0, 9),
]
add_extrude("01_deck_front", "前段主甲板", 1,
            rect(0, -DECK_W / 2, DECK_SEG_L, DECK_W / 2), _h_deck_front, DECK_T,
            "填充 30%，壁厚 1.2mm。两块甲板用搭接板 + M3x8 连接，中间夹金属底盘。")

_h_deck_rear = [
    # ↓ 同前段：金属底盘配合长圆槽（不是圆孔）
    slot_cross(MOUNT_X0,  MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    slot_cross(MOUNT_X0, -MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    slot_cross(MOUNT_X1,  MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    slot_cross(MOUNT_X1, -MOUNT_Y, MOUNT_SLOT_X, MOUNT_SLOT_H, MOUNT_SLOT_W),
    # 储仓定位槽
    rrect(20, -BIN_W / 2 - 1, 20 + 4, BIN_W / 2 + 1, 1.0),
    # ⚠ 下面这组的 x 是 BIN_L-4+20 = 154，超出本段 x 范围（0..128），实际不生效。
    #   BIN_L(138) > DECK_SEG_L(128)，储仓必然跨接缝，所以只有上面一组槽能定位。
    #   先不动它：留着说明"这里本该有第二组"，等储仓实际位置定了再回填。
    rrect(BIN_L - 4 + 20, -BIN_W / 2 - 1, BIN_L + 20, BIN_W / 2 + 1, 1.0),
    # 电控托盘安装孔
    circ(50, -40, M3 / 2), circ(50, 40, M3 / 2),
    circ(100, -40, M3 / 2), circ(100, 40, M3 / 2),
    *[circ(x, y, 5) for x in range(28, 124, 24) for y in (-52, -34, -16, 16, 34, 52)],
]
add_extrude("02_deck_rear", "后段主甲板", 1,
            rect(0, -DECK_W / 2, DECK_SEG_L, DECK_W / 2), _h_deck_rear, DECK_T,
            "填充 30%。电池盒建议吊在这块板下方，重心越低越防滑。")

add_extrude("03_deck_splice", "甲板搭接板", 1,
            rect(0, -30, SPLICE_L, 30),
            [circ(10, -18, M3 / 2), circ(10, 18, M3 / 2),
             circ(30, -18, M3 / 2), circ(30, 18, M3 / 2),
             circ(50, -18, M3 / 2), circ(50, 18, M3 / 2)], 3.0,
            "垫在两段甲板接缝下方，M3x8 从下往上穿。")

# ---------------------------------------------------------------- 2. 立柱
add_extrude("04_column", "甲板立柱", 4,
            list(Point(0, 0).buffer(COL_D / 2, quad_segs=16).exterior.coords),
            [circ(0, 0, M3 / 2)], COL_H,
            "填充 45%。中间 M3 通孔，配 M3x30 螺杆（22 立柱 + 4 甲板 = 26mm，"
            "M3x25 会短一截）把甲板锁在金属底盘上。")

# ---------------------------------------------------------------- 3. 收集铲（V 型喇叭口）
# 做成「底板 + 两块侧板」的平板件组合：既能放进 220mm 打印床，
# 又比一体放样件轻一半以上，坏了单换一片就行。
# 喉口：V 型喇叭口最窄处。必须 ≥ CUBE_DIAG（42.4），否则方块转成 45° 会卡死。
# 再加 4mm 余量给打印飞边和地胶摩擦。V1.3 时代是 52（当时 CUBE_DIAG=56.6，其实是欠的），
# 现在按「对角线 + 4」算，反而比老版更保险。
THROAT = 50.0                          # 喉口：方块转45°投影42.4 + 余量7.6（原46.4偏紧）
_L = SCOOP_L / 2          # 45
_W = SCOOP_W / 2          # 75

add_extrude("05_scoop_floor", "收集铲底板（V 型）", 1,
            [(_L, -_W), (_L, _W), (-_L, THROAT / 2), (-_L, -THROAT / 2)],
            [rrect(-6, 27, 6, 31, 2.0), rrect(-6, -31, 6, -27, 2.0)],
            SCOOP_LIP,
            "填充 45%、壁厚 1.6mm。**打印完必须用锉刀把前缘（宽边）倒成 30° 斜面**，"
            "否则 6mm 厚的直边铲不进方块，只会把方块推走。这是整个收集机构成败的关键一步。"
            "★ 装车不用量高度：把车放在地上，松开这两条槽里的螺丝，让铲唇自己坐到地面，"
            "再垫 1~2 片 M3 垫圈填满缝隙后拧紧。板面离地到底是 25 还是 32mm 都不影响。")

add_extrude("05b_scoop_wall_L", "收集铲侧板（左）", 1,
            [(_L, -_W), (_L, -_W + 6), (-_L, -THROAT / 2 + 6), (-_L, -THROAT / 2)],
            [circ(-10, -25.3, M25 / 2), circ(6, -38.4, M25 / 2)],
            26.0,
            "填充 45%。竖直立在底板左侧斜边上，用 PLA 胶 + M2.5 螺丝固定。"
            "两块侧板把方块从 96mm 引导收窄到喉口 50mm，靠的是斜壁的分力，不是靠铲得深。")
add_extrude("05c_scoop_wall_R", "收集铲侧板（右）", 1,
            [(_L, _W), (_L, _W - 6), (-_L, THROAT / 2 - 6), (-_L, THROAT / 2)],
            [circ(-10, 25.3, M25 / 2), circ(6, 38.4, M25 / 2)],
            26.0,
            "左件的镜像。")

# ★ 2026-10-04 新增：铲是"兜"不是"通"，必须有一块后壁让方块停下来。
#   没有它，方块会被斜壁一路引导着滑进车底（那里是马达和线束），永远等不到夹爪。
#   方块顶到这块板 = 到位，也是后面做"到位检测"微动开关的安装位。
add_extrude("05d_scoop_back", "收集铲后壁（对中挡板）", 1,
            rect(-4, -32, 4, 32),
            [circ(0, -22, M3 / 2), circ(0, 22, M3 / 2)],
            26.0,
            "★ 装在铲的最后端（喉口后方）。方块被两侧斜壁收窄后顶到这块板上停住，"
            "横向被限制在 50mm 内 → 夹爪从正上方下伸就能夹到，容差 ±10mm。"
            "这块板同时也是微动开关的安装位（顶到 = 到位信号）。")

add_extrude("06_limit_lever", "对中限位摆杆", 2,
            rect(0, -6, 45, 6),
            [circ(5, 0, M25 / 2), circ(38, 0, M25 / 2)], 4.0,
            "装在铲两侧的槽里，用 M2.5 轴 + 拉簧复位。方块同时压到左右两根 => 判定对中。")

# ---------------------------------------------------------------- 4. 夹爪
# ⚠ 这一对指是「自己建模版」的兜底方案。原设计是按 40mm 方块做的，
#   V1.4 改成 30mm 后按 CUBE/40 整体等比缩小（孔位跟着缩，但孔径不变 ——
#   M2.5/M3 螺钉是实物尺寸，不能跟着缩）。
#
# ★ 比"指长"重要得多的一个数：**两指闭合后，两块防滑垫之间的净间距**。
#   它必须 < CUBE（30mm），最好在 26~28mm。
#   - 如果闭到底还有 32mm 以上 → 爪子在方块两侧干跑，根本夹不住（最常见的翻车点）
#   - 如果 < 26mm → 只能夹薄片，方块反而被"顶出去"
#   这个数由「舵机转动量 + 连杆/齿条位置」决定，属于装完必须实测的项，
#   在固件里对应 config.h 的 SERVO_GRIP_HOLD_CUBE（不要在建模阶段猜死）。
_GRIP_REF = [(0, -8), (46, -8), (58, -14), (62, -6), (56, 2), (46, 8), (0, 8)]  # 原 40mm 版轮廓
_k      = CUBE / 40.0
_grip   = [(round(x * _k, 2), round(y * _k, 2)) for (x, y) in _GRIP_REF]
_grip_h = [circ(8 * _k, 0, M25 / 2), circ(24 * _k, 0, M3 / 2), circ(40 * _k, 0, 2.0)]

add_extrude("07_grip_finger_L", "夹爪指（左）", 1, _grip, _grip_h, 6.0,
            "填充 45%。这是「自制钩形指」方案；若改用网上的齿轮齿条爪（MG996R 款），"
            "这对指不用装。内侧贴 1mm EVA/硅胶垫增大摩擦。镜像打印得到右手件。")
add_extrude("08_grip_finger_R", "夹爪指（右）", 1,
            [(-x, y) for (x, y) in _grip],
            [circ(-8 * _k, 0, M25 / 2), circ(-24 * _k, 0, M3 / 2), circ(-40 * _k, 0, 2.0)], 6.0,
            "左件的镜像。")

# ---------------------------------------------------------------- 5. 机械臂
add_extrude("09_arm_upper", "大臂（MG995 驱动）", 1,
            [(0, -12), (86, -12), (92, -4), (92, 4), (86, 12), (0, 12)],
            [circ(8, 0, M3 / 2), circ(20, 0, M3 / 2),   # 舵盘孔（间距按 16mm 舵盘）
             circ(34, 0, 5), circ(52, 0, 5), circ(70, 0, 5),   # 减重
             circ(86, 0, M3 / 2)],                       # 小臂关节
            8.0,
            "填充 45%，壁厚 1.6mm。MG995 扭矩大，这根臂是受力件，别用 25% 填充。")

add_extrude("10_arm_fore", "小臂 / 取块叉", 1,
            [(0, -9), (58, -9), (66, -14), (70, -6), (64, 6), (58, 9), (0, 9)],
            [circ(7, 0, M25 / 2), circ(24, 0, 4), circ(44, 0, 4)], 6.0,
            "前端叉口 26mm（< 方块 30mm，方块能横架在两叉上）。用来把台阶/焦点平台上"
            "的方块叉下来、或把方块从矮墙槽里托出来。")

# ---------------------------------------------------------------- 5.5 外购机械爪转接板
# ★ 2026-10-04 新增：用户买了市售的「MG996R 齿轮齿条平行爪」，要装到我们的臂上。
#
#   【为什么做成一片全是槽的板】
#   市售爪子各厂家的安装孔距五花八门（25 / 29 / 35 / 40 / 45 / 48mm 都有），
#   用户手上没有卡尺，也来不及量。按本项目的铁律——
#   **凡是要和外购件配合的尺寸，一律做「槽」不做「孔」**——
#   这里把整块板做成 4 竖 2 横的井字槽：
#       竖槽中心 x = 8 / 26 / 44 / 62（每条长 40mm，能吞下 y 方向 0~40mm 的任意孔距）
#       横槽中心 y = ±14（每条长 64mm，能吞下 x 方向 0~64mm 的任意孔距）
#   → 只要买的爪子安装孔构成的两条边落在 64 x 40 的范围内，直接就能装，
#     **不用量任何一个数**。剩下实在装不上的，用扎带穿过槽绑，一样锁得死。
#
#   【竖槽 x 位置为什么是 8 / 26 / 44】
#   这三个位置是专门对着小臂 10_arm_fore 末端已有的三个孔（x=7 / 24 / 44）排的：
#   槽宽 4mm，窗口分别是 6~10 / 24~28 / 42~46 → 三个孔各有一条槽能吃到。
#   也就是说这块板叠在小臂末端时，**不用重新打孔**。
#
#   【装在哪】两种装法，二选一：
#     A（推荐）装在小臂 10_arm_fore 末端 → 替代自制的 07/08 钩形指。
#       优点：够得远（小臂还能摆）。缺点：MG996R 约 55g 挂在末端，SG90 小臂吃力。
#     B（更稳）装在大臂 09_arm_upper 末端（x=86 那个孔）→ 整个替换掉小臂。
#       优点：大臂是 MG995 驱动的，带得动。缺点：整体变短，探洞/上架够不到那么深。
#   建议先按 A 装，发现小臂抖/抬不起来立刻改 B（同一块板，换个位置拧螺丝而已）。
_ADA_W, _ADA_L = 52.0, 76.0
_ada_holes = []
for _x in (8.0, 26.0, 44.0, 62.0):                       # 4 条竖槽（沿车宽 y）
    _ada_holes.append(rrect(_x - 2, -20, _x + 2, 20, 2))
for _y in (-14.0, 14.0):                                  # 2 条横槽（沿车长 x）
    _ada_holes.append(rrect(6, _y - 2, 70, _y + 2, 2))

add_extrude("11b_claw_adapter", "外购机械爪通用转接板", 1,
            rect(0, -_ADA_W / 2, _ADA_L, _ADA_W / 2), _ada_holes, 5.0,
            "★ 装你买的那款 MG996R 平行爪用的。整块板是井字槽，安装孔距 64x40 以内"
            "都装得上，不用量尺寸。装不上就用扎带穿槽绑。填充 45%，这是受力件。"
            "装法 A：叠在小臂末端替代自制夹爪指；装法 B：装在末端替代小臂（更稳）。")

# ---------------------------------------------------------------- 5.55 机械臂增高座
# ★ 2026-10-04 补：这是**功能缺口补丁**，不是可选件。
#
#   【为什么必须有这个件】
#   物资架上的黄块质心距地 250mm，而车是站在地面去够它的。
#   原来的肩关节（大臂舵机轴）只有：
#       板面 30 + 立柱 22 + 甲板 4 + 舵机轴高 20 ≈ 76mm
#   臂展 = 大臂 86 + 小臂到夹爪中心 50 = 136mm
#   → 最大可达 76 + 136 = 212mm < 250mm   ❌ 够不到，物资架的黄块拿不到。
#
#   加了这个 60mm 的增高座后：肩高 76 + 60 = 136mm
#   → 最大可达 136 + 136 = 272mm ≥ 250mm  ✅ 能夹到（还余 22mm）
#
#   【收起时会不会超 210mm 红线？不会。】
#   收起姿态是大臂**平放**（不是竖着），最高点 = 肩高 136 + 大臂半宽 12 ≈ 148mm，
#   离 210 的红线还有 62mm 余量。规则允许变形，只有"初始状态"才卡 210。
#
#   【为什么做成"框"而不是实心块】
#   实心 50x54x60 要 217g，太重；框型（壁厚 6）只有约 82g（25% 填充更轻）。
#   框中间是通的，**扎带直接从内腔穿过去**就能把舵机绑在顶面、把座绑在甲板上，
#   一颗螺丝都不用，也不用对孔。
_RIS_L, _RIS_W, _RIS_T = 50.0, 54.0, 6.0      # 长(x) / 宽(y) / 壁厚

# ★ 2026-10-04：这个件**可能要被切短**，但不在 STL 上开刻线
#   （原因：生成器只支持「2D 轮廓 + 整体拉伸」，没法在中间某个高度上开槽；
#     在底面开槽又会被一起切掉，等于没做。）
#   改成施工说明：拿尺子从**底脚**往上量，用记号笔画一圈，沿线下锯。
#        底板面高度 = 轮外径 + 2.5（板装在马达上方）
#        增高座高度 = 90 − 板面高   →   Φ60: 27.5 ｜ Φ50: 38.5 ｜ Φ40: 48.5
#   板若保持 30mm（解法 B，给板开缺口）就一刀不动、直接用 60mm。
add_extrude("25_arm_riser", "机械臂增高座（★ 必打，可能要切短）", 1,
            rect(0, -_RIS_W / 2, _RIS_L, _RIS_W / 2),
            [rect(_RIS_T, -_RIS_W / 2 + _RIS_T,
                  _RIS_L - _RIS_T, _RIS_W / 2 - _RIS_T)],   # 中间掏空成框
            60.0,
            "★ 必打。把大臂 MG995 抬高，夹爪才够得到 250mm 高的物资架黄块。"
            "框型，扎带从中间穿过去绑舵机+绑甲板，不用对孔。填充 45%，这是受力件。"
            "★★ 装车前先看《04_轮组与底板干涉的三条解法.png》：底板是装在马达**上方**的，"
            "板面高度 = 轮外径 + 2.5，而肩高要锁死 136mm，所以增高座高度 = 90 − 板面高。"
            "拿尺子从**底脚**往上量、画一圈线，沿线下锯（只锯四壁、别碰顶面舵机区）："
            "Φ60 轮锯到 27.5 ｜ Φ50 轮锯到 38.5 ｜ Φ40 轮锯到 48.5；"
            "板若保持 30mm 就一刀不动。")

add_extrude("11_servo_horn_plate", "舵盘转接板", 3,
            list(Point(0, 0).buffer(16, quad_segs=16).exterior.coords),
            [circ(-8, -8, M2 / 2), circ(8, -8, M2 / 2),
             circ(-8, 8, M2 / 2), circ(8, 8, M2 / 2), circ(0, 0, 6)], 3.0,
            "用 M2 自攻拧到舵机自带塑料舵盘上，再把臂锁到这块板上。"
            "直接把臂拧在塑料舵盘上会滑牙，这层转接板很值。")

# ============================================================================
# 5.6 平行夹爪（★ 用户点名要的「市售那种平行爪」，自己打印自己装）
# ============================================================================
# 【他要的是什么】
#   用户发了一张市售「MG996R 齿轮齿条平行爪」的照片，本意是：
#   「我想要这种两片爪子平行开合的爪子，你帮我建模出来，我打印了自己装」。
#   我一开始理解成「他已经买了，需要一块转接板」，于是只给了 11b 转接板 —— 理解反了。
#   这一节才是他真正要的东西：**整套平行夹爪的建模**。
#
# 【为什么不用「齿轮齿条」】
#   市售那款是齿轮 + 两根齿条。齿条啮合对打印精度很敏感：
#   FDM 0.2mm 层高打出来的齿形有毛刺，齿轮一转就卡；而且齿是薄壁悬空结构，容易断。
#   我们只能打一次，赌不起。
#
# 【改用：导轨滑块 + 交叉连杆】
#   两片爪指各固定在一个滑块上，滑块在底板的直导槽里平移
#   → **爪面永远平行**（这是平行爪的核心价值：夹方块时是面接触，不会把方块挤歪斜）。
#   舵机摇臂用两根连杆分别推左右滑块，两根连杆在 z 方向错开一层，互不干涉。
#   全机构只有「销钉铰接」，没有任何啮合/螺纹配合 → 打印精度要求低，失败率极低。
#
# 【关键几何推导（改尺寸前先看这段，别瞎改）】
#   摇臂半径 r=13.5，连杆孔距 L=28.5。滑块铰点在 y=0 线上，坐标 x。
#   摇臂端点 A = (r·cosθ, r·sinθ)，连杆约束 |A - B| = L，B=(x, 0)：
#       θ=0°   ->  x = r - L = -15.0   （两爪闭合）
#       θ=90°  ->  x = -√(L²-r²) = -25.1（两爪张开）
#   单侧行程 10.1mm。爪指厚 6，所以：
#       闭合净距 = 2×15.0 - 6 = 24.0 mm  < 方块 30  ->  夹得住
#       张开净距 = 2×25.1 - 6 = 44.2 mm  > 方块 30  ->  方块进得去（余 14mm）
#   ⚠ 这两个数和 r、L 是一一绑定的，改任意一个都要重算，别单独改爪指长度。
#
# 【z 方向分层（装配时靠不同长度的销钉实现，件本身都是等厚平板）】
#   底板      z ∈ [ 0,  4]
#   摇臂      z ∈ [-4, -1]   ← 舵机在底板上方、轴朝下，摇臂装在轴末端
#   （空）
#   连杆      z ∈ [-11,-8]   ← 销钉 M3×16 从上方穿过摇臂再穿过连杆
#   滑块      z ∈ [-8,  4]   ← 穿过导槽，销钉 M3×20 从滑块顶面穿到连杆
#   爪指      z ∈ [-42,-8]   ← 两颗 M2.5 自攻从滑块底面拧进爪指
#
# 【和 07/08 钩形指的关系】
#   07/08 是剪刀式（两片绕同一轴反向转），件少但夹持是「点接触」，方块容易歪。
#   这一套是平行式，夹持是「面接触」，方块自动居中 —— 推荐用这套。
#   两套都打了，**装车时二选一**；07/08 留着当兜底。
CLAW_L, CLAW_W, CLAW_T = 74.0, 56.0, 4.0      # 爪架底板
CLAW_SLOT_HH = 8.2                            # 导槽半宽（滑块 y=16 + 0.4 间隙）
CLAW_SLI_L, CLAW_SLI_W, CLAW_SLI_T = 18.0, 16.0, 12.0   # 滑块
CLAW_FIN_T, CLAW_FIN_W, CLAW_FIN_H = 6.0, 26.0, 34.0    # 爪指（厚 6：要打 Φ2.7 孔）
CLAW_R = 13.5                                 # 摇臂半径
CLAW_LINK = 28.5                              # 连杆孔距

_cb = [unary_union([Polygon(rect(-35, -CLAW_SLOT_HH, 35, CLAW_SLOT_HH)), circ(0, 0, 5)])]
# 舵机固定：扎带槽（不用螺丝孔 —— SG90 / MG996R 孔位不一样，扎带通吃）
for _x in (10.0, 22.0, -10.0, -22.0):
    for _y in (11.0, -11.0):
        _cb.append(rrect(_x - 4, _y - 1.75, _x + 4, _y + 1.75, 1.75))
# 装到机械臂末端的井字槽（分段避开导槽，滑块经过时不会失去导向）
for _x in (10.0, 26.0, -10.0, -26.0):
    _cb.append(rrect(_x - 2, 15, _x + 2, 26, 2))
    _cb.append(rrect(_x - 2, -26, _x + 2, -15, 2))
for _y in (20.0, -20.0):
    _cb.append(rrect(-30, _y - 2, 30, _y + 2, 2))

add_extrude("24a_claw_base", "平行夹爪·爪架底板", 1,
            rect(-CLAW_L / 2, -CLAW_W / 2, CLAW_L / 2, CLAW_W / 2), _cb, CLAW_T,
            "填充 45%。中间的贯通槽是滑块导轨（半宽 8.2mm = 滑块 16 + 0.4 间隙）。"
            "中心的 Φ10 孔是舵机轴穿出来的。舵机用**扎带**绑在上表面的 8 条槽上"
            "（SG90 / MG996R 都能绑，不用管孔距）。四角的井字槽用来装到机械臂末端。")

add_extrude("24b_claw_slider", "平行夹爪·滑块", 2,
            rect(-CLAW_SLI_L / 2, -CLAW_SLI_W / 2, CLAW_SLI_L / 2, CLAW_SLI_W / 2),
            [circ(0, 0, 3.4 / 2),                       # 连杆销孔（贯穿，M3）
             circ(0, 5, 2.7 / 2), circ(0, -5, 2.7 / 2)],  # 爪指安装孔（M2.5 自攻）
            CLAW_SLI_T,
            "填充 45%。打 2 件（左右各一，同一件翻面装即可）。"
            "它穿过底板导槽，上下靠爪指和销钉夹住，不会掉出来。"
            "打印完用砂纸把 16mm 那两面磨一下，在槽里要能用手推动但不能晃。")

add_extrude("24c_claw_finger", "平行夹爪·爪指", 2,
            rect(-CLAW_FIN_T / 2, -CLAW_FIN_W / 2, CLAW_FIN_T / 2, CLAW_FIN_W / 2),
            [circ(0, 5, 2.7 / 2), circ(0, -5, 2.7 / 2)],
            CLAW_FIN_H,
            "填充 45%。打 2 件。**内侧贴 1mm EVA 或鼠标垫增大摩擦**，不然 PLA 太滑夹不住。"
            "厚 6mm 是为了能打 Φ2.7 的安装孔（4mm 厚的版本孔壁只剩 0.65mm，一拧就裂）。"
            "闭合净距 24mm / 张开 44mm（方块是 30mm）。")

_horn = unary_union([Polygon(rect(-CLAW_R, -3, CLAW_R, 3)),
                     circ(-CLAW_R, 0, 3), circ(CLAW_R, 0, 3)])
add_extrude("24d_claw_horn", "平行夹爪·舵机摇臂", 1, _horn,
            [circ(0, 0, 3.0 / 2),                        # 中心：舵机轴 / 舵盘螺丝
             circ(-CLAW_R, 0, 3.4 / 2), circ(CLAW_R, 0, 3.4 / 2)], 3.0,
            "填充 45%。双臂摇臂，两端各推一根连杆。中心孔拧在舵机舵盘上"
            "（建议先用 11_servo_horn_plate 那套 M2 自攻固定到塑料舵盘上，别直接拧）。")

_link = unary_union([Polygon(rect(-CLAW_LINK / 2, -3, CLAW_LINK / 2, 3)),
                     circ(-CLAW_LINK / 2, 0, 3), circ(CLAW_LINK / 2, 0, 3)])
add_extrude("24e_claw_link", "平行夹爪·连杆", 2, _link,
            [circ(-CLAW_LINK / 2, 0, 3.4 / 2), circ(CLAW_LINK / 2, 0, 3.4 / 2)], 3.0,
            "填充 45%。打 2 件。孔距 28.5mm 是算出来的，不要改（改了行程就不对了）。"
            "两根连杆在高度上差一层，所以它们交叉时不会打架。")

# ------------------------------------------------ 5b. 齿轮齿条平行夹爪（可选，与连杆版二选一）
#
# 【为什么要有第二套】你给的那款市售平行爪就是「齿轮 + 双齿条」结构。
#   05 批次那套是连杆驱动（失败率低），这一套才是齿轮驱动（样子和你发的那款一样）。
#   **两套都打，装车时二选一** —— 反正只能打一次，多备一套最稳。
#
# 【布局（俯视）】
#   舵机在底板上方，轴朝下穿过中心 Φ10 孔 → 齿轮在底板下方 z ∈ [-6, 0]
#   两条齿条分列齿轮的 y 两侧（分度线 y = ±6），沿 x 延伸
#   → 齿轮一转，两条齿条沿 x 反向移动 → 两块爪指在 x 方向相对 → 平行开合
#
# 【行程】齿轮分度圆半径 R = 6mm，舵机转 θ → 单侧行程 s = R·θ
#   舵机 90° → s = 9.4mm；闭合净距 26mm → 张开 44.8mm（方块 30mm，余 14.8mm）
#
# 【装配高度】
#   底板      z ∈ [  0,   4]
#   齿条      z ∈ [ -8,   4]  （件本身厚 12，装配时中心放在 z = -2）
#   齿轮      z ∈ [ -6,   0]  （套在 M3×25 螺丝上，螺丝从舵盘中心往下伸）
#   爪指      z ∈ [-42,  -8]  （从齿条底面向下 34mm）
#
# ⚠ 齿条打 2 件，**第二件要在桌面上转 180° 再装**（不是翻面！）
#   绕竖直轴转 180° → 齿跑到另一侧、爪指也跑到 x = -13，两块爪指才相对。

GEAR_M, GEAR_Z = 1.5, 8                        # 模数 / 齿数
GEAR_R = GEAR_M * GEAR_Z / 2                   # 6.0   分度圆半径
GEAR_RA = GEAR_R + GEAR_M                      # 7.5   齿顶圆
GEAR_RF = GEAR_R - 1.25 * GEAR_M               # 4.125 齿根圆
GEAR_P = math.pi * GEAR_M                      # 4.712 齿距
GEAR_T = 6.0                                   # 齿轮厚度

# 齿轮俯视轮廓：8 个梯形齿（齿顶弧 11.2°、齿根弧 22.5°，齿根处补中点让圆弧更圆）
_gp = []
for _i in range(GEAR_Z):
    _c = math.radians(_i * 360.0 / GEAR_Z)
    for _r, _da in ((GEAR_RF, -11.25), (GEAR_RA, -5.6),
                    (GEAR_RA, 5.6), (GEAR_RF, 11.25), (GEAR_RF, 22.5)):
        _a = _c + math.radians(_da)
        _gp.append((_r * math.cos(_a), _r * math.sin(_a)))
_gear_poly = Polygon(_gp)

add_extrude("24f_claw_gear", "齿轮夹爪·齿轮", 2, _gear_poly,
            [circ(0, 0, 3.4 / 2)], GEAR_T,
            "填充 45%。打 2 件（1 用 1 备）。**打印完必须用锉刀把每个齿的毛刺修一遍**，"
            "齿侧面要光滑，否则一转就卡。装在底板下方，套在 M3×25 螺丝上、"
            "上下各一个螺母夹紧（舵机轴不够长，用螺丝当延长轴）。")

RACK_L = 44.0            # 齿条长度（x 方向）
RACK_Y0, RACK_Y1 = 7.8, 19.8   # 齿条本体两条边（齿根线 / 背面）
RACK_T = 12.0            # 齿条厚度
RACK_TIP = 4.5           # 齿顶面（= 分度线 6.0 − 齿顶高 1.5）
_rw, _tw = GEAR_P * 0.255, GEAR_P * 0.16   # 齿根半宽 1.20 / 齿顶半宽 0.75

_rp = [(-RACK_L / 2, RACK_Y1), (RACK_L / 2, RACK_Y1), (RACK_L / 2, RACK_Y0)]
for _k in range(4, -5, -1):            # 从 +x 往 −x 走，依次画出 9 个齿
    _x = _k * GEAR_P
    _rp += [(_x + _rw, RACK_Y0), (_x + _tw, RACK_TIP),
            (_x - _tw, RACK_TIP), (_x - _rw, RACK_Y0)]
_rp.append((-RACK_L / 2, RACK_Y0))
_rack_poly = Polygon(_rp)

add_extrude("24g_claw_rack", "齿轮夹爪·齿条滑块", 2, _rack_poly,
            [circ(13, 9.0, 2.7 / 2), circ(13, 16.5, 2.7 / 2)],   # 爪指安装孔（M2.5 自攻）
            RACK_T,
            "填充 45%。打 2 件。它就是「齿条 + 滑块」合一体：带齿的那边朝齿轮，"
            "本体在底板的导槽里滑动。**第二件要在桌面上转 180° 装**（绕竖直轴），"
            "这样齿跑到另一侧、爪指跑到 x = −13，两块爪指才面对面。"
            "打印完用砂纸把 12mm 那两面磨一下，在槽里能用手推动但不能晃。")

add_extrude("24i_claw_finger_gear", "齿轮夹爪·爪指", 3,
            rect(-3, -18, 3, 18),
            [circ(0, 9.0, 2.7 / 2), circ(0, 16.5, 2.7 / 2)],
            34.0,
            "填充 45%。打 3 件（2 用 1 备）。和连杆版的 24c 爪指不一样："
            "这件是沿 y 方向的长条（36mm），才能同时搭到齿条上、又盖住 30mm 方块。"
            "**内侧贴 1mm EVA 或鼠标垫**，不然 PLA 太滑夹不住。")

# 齿轮版爪架底板（导槽是两条，和连杆版的一条不一样，所以必须单独一件）
_gh = [Polygon(rect(-33, 7.6, 33, 20.0)),                   # 上导槽
       Polygon(rect(-33, -20.0, 33, -7.6)),                 # 下导槽
       circ(0, 0, 5)]                                        # 中心舵机轴孔
# ⚠ 槽的数量刻意压到最少（4 条扎带槽 + 2 横 4 竖的井字槽）。
#   2026-10-04 自检：槽开多了，4mm 薄板的截面几乎全被 3 圈壁厚填满，
#   "挤出量 / 几何体积" 到 1.51（上限 1.45）—— 打印出来槽会被挤窄甚至糊死。
#   所以这里只留刚好够用的槽，别再加。
for _x in (14.0, -14.0):                                     # 舵机扎带槽（中心区内）
    for _y in (4.0, -4.0):
        _gh.append(rrect(_x - 4, _y - 1.75, _x + 4, _y + 1.75, 1.75))
for _y in (24.0, -24.0):                                     # 装到机械臂末端的井字槽
    _gh.append(rrect(-30, _y - 2, 30, _y + 2, 2))
for _x in (14.0, -14.0):
    _gh.append(rrect(_x - 2, 20.5, _x + 2, 26.5, 2))
    _gh.append(rrect(_x - 2, -26.5, _x + 2, -20.5, 2))

add_extrude("24h_claw_base_gear", "齿轮夹爪·爪架底板", 1,
            rect(-37, -28, 37, 28), _gh, 4.0,
            "填充 45%。**只有用齿轮版夹爪时才需要这一件** —— 它的导槽是分开的两条"
            "（y = ±7.6~±20），中间那块实心区是留给齿轮和舵机轴的；"
            "连杆版 24a 的导槽是一整条，两套不通用。舵机同样用扎带绑在中心区的 8 条小槽上。")

# ---------------------------------------------------------------- 5c. 能量块道具（拍视频 / 练夹取用）
# 规则 V1.4：30×30×30，内镂空 15 → 壁厚 7.5，外棱 45° 倒角 2.121，六面实心无开口。
# 组委会发下来的块只有一份、丢了就练不了，所以自己按同规格打 4 个当道具：
#   ① 拍中期考核视频（夹取过程必须有实物）
#   ② 标定夹爪闭合角度（外形必须严格 30.00，否则标定白做）
#   ③ 反复练夹取，夹坏了不心疼
# ⚠ 内腔是封闭空腔：z=22.5 处要架桥封顶，跨度只有 15mm，PLA 完全架得住。
#   打印时不用加支撑，腔体朝上即可（切片器按层实心填充会自动铺顶）。
_CUBE_H   = CUBE / 2.0          # 15.0
_CUBE_CH  = 3.0                 # 45° 倒角 2.121 → 沿边方向切掉 3.0mm
_cube_out = [(-12.0, -15.0), (12.0, -15.0), (15.0, -12.0), (15.0, 12.0),
             (12.0, 15.0), (-12.0, 15.0), (-15.0, 12.0), (-15.0, -12.0)]
_cube_solid = Polygon(_cube_out)
_cube_ring  = build(_cube_out, [rect(-CUBE_HOLLOW / 2, -CUBE_HOLLOW / 2,
                                     CUBE_HOLLOW / 2,  CUBE_HOLLOW / 2)])
_cube_faces = (_faces_from_polygon(_cube_solid, 0.0,  7.5) +    # 底盖 7.5 实心
               _faces_from_polygon(_cube_ring,  7.5, 22.5) +    # 中段环形（壁厚 7.5）
               _faces_from_polygon(_cube_solid, 22.5, 30.0))    # 顶盖 7.5 实心
_cube_vol = save_stl("26_cube_mock", _cube_faces)
save_dxf("26_cube_mock", _cube_solid, CUBE)
PARTS.append(dict(f="26_cube_mock", cn="能量块道具 30×30×30", qty=4,
                  poly=_cube_solid, t=CUBE, zs=(0.0, CUBE), vol=_cube_vol,
                  note="★ 拍视频 / 练夹取 / 标定夹爪用，不是装车件。"
                       "外形严格 30.00mm（45° 倒角 2.121），内腔 15³、壁厚 7.5 —— "
                       "与组委会发的块同规格同样式。打 4 个：建议 3 白 + 1 黄"
                       "（黄色要换料；没黄料就全打白的再贴黄胶带）。"
                       "⚠ 我们的 PLA 块比官方块重（官方 11.29g），"
                       "夹爪能夹起我们的，就一定能夹起官方的 —— 只会更稳。"))

# ---------------------------------------------------------------- 6. 储仓（斜槽重力自锁）
add_extrude("12_bin_floor", "储仓底板", 1,
            rect(0, -BIN_W / 2, BIN_L, BIN_W / 2),
            [circ(10, -BIN_W / 2 + 12, M3 / 2), circ(10, BIN_W / 2 - 12, M3 / 2),
             circ(BIN_L - 10, -BIN_W / 2 + 12, M3 / 2),
             circ(BIN_L - 10, BIN_W / 2 - 12, M3 / 2),
             *[circ(x, y, 4) for x in range(24, int(BIN_L) - 16, 22)
               for y in (-BIN_W / 2 + 20, 0, BIN_W / 2 - 20)]],
            BIN_T,
            "底板做成前低后高（整体抬高 5°），方块靠重力滑到最里侧，急停也掉不出来。"
            "⚠ 132×116 的大底板是最容易翘边的一件：**必须打 Brim 8mm、首层慢速**，"
            "翘了就用热风枪烤平，别硬掰。")

add_extrude("13_bin_side", "储仓侧板", 2,
            rect(0, 0, BIN_L, BIN_H),
            [circ(10, 8, M3 / 2), circ(BIN_L - 10, 8, M3 / 2),
             *[circ(x, y, 4) for x in range(24, int(BIN_L) - 16, 24) for y in (20, 38)],
             # 隔板榫槽（选装隔板用，默认不插）
             *[rrect(x - 1.6, 0, x + 1.6, 10, 0.4) for x in (44, 88)]],
            BIN_T,
            "下部两个榫槽是给「选装隔板」留的，**默认不插**。"
            "侧板高 52mm，比 30mm 方块高 22mm，急刹车方块也蹦不出去。")

add_extrude("14_bin_divider", "储仓隔板", 2,
            rect(0, 0, BIN_W - 2 * BIN_T, BIN_H - 6),
            [rrect(0, 0, BIN_W - 2 * BIN_T, 10, 0.4),
             circ((BIN_W - 2 * BIN_T) / 2, 22, 5), circ((BIN_W - 2 * BIN_T) / 2, 34, 5)],
            BIN_T,
            "【选装】插进侧板榫槽，用 PLA 胶或热熔胶固定。"
            "⚠ 装了会把容量从 12 块压到 9 块（分 3 区），好处是方块不会前后串位。"
            "**默认不装，先把 12 块容量拿到手。**")

add_extrude("15_bin_flap", "储仓单向挡板", 3,
            rect(0, 0, 42, 30),
            [circ(6, 6, M25 / 2), circ(36, 6, M25 / 2), circ(21, 20, 4)], 2.0,
            "顶部铰接，只能往里翻。方块进得去出不来 —— 满足『携带多项』的关键件。"
            "注意：手册禁止『固连』，挡板是车体自身机构，不属于固连。")

# ---------------------------------------------------------------- 7. 特殊取块机构
add_extrude("16_rack_hook", "物资架挑杆头", 1,
            [(0, -7), (52, -7), (64, -12), (70, -12), (70, 12), (64, 12), (52, 7), (0, 7)],
            [circ(8, 0, M3 / 2), circ(24, 0, M3 / 2), circ(44, 0, 4)], 6.0,
            "U 型叉口 22mm（< 方块 30mm 也 > 圆柱 14mm），用来卡住挂在 14mm 圆柱上的黄色能量核心。"
            "从下往上挑 + 车体后退，黄块就掉进车头兜里。")

add_extrude("17_cave_probe", "洞窟探杆头", 1,
            [(0, -6), (70, -6), (86, -10), (92, -2), (86, 10), (70, 6), (0, 6)],
            [circ(8, 0, M3 / 2), circ(26, 0, M3 / 2), circ(50, 0, M3 / 2)], 6.0,
            "截面 12x12，能伸进 70x70x50 的洞。前端钩深 12mm，勾住方块后倒车拉出。")

# ---------------------------------------------------------------- 8. 电控与电池
add_extrude("18_electronics_tray", "电控托盘", 1,
            rect(0, -40, 110, 40),
            [circ(8, -32, M3 / 2), circ(8, 32, M3 / 2),
             circ(102, -32, M3 / 2), circ(102, 32, M3 / 2),
             # 扎带槽（固定 STM32 / DRV8833 / 降压模块，不依赖孔位）
             rrect(20, -22, 26, 22, 0.8), rrect(44, -22, 50, 22, 0.8),
             rrect(68, -22, 74, 22, 0.8),
             *[circ(x, y, 3) for x in range(14, 100, 14) for y in (-32, 32)]],
            3.0,
            "★ 装在**夹层**（金属板面上、甲板下方），不是甲板上面 —— 甲板纵向放不下它 + 储仓。"
            "不用纠结模块孔位：扎带穿过槽直接捆。DRV8833 底下垫一层 3M 胶再捆，抗震。")

add_extrude("19_battery_clip", "18650 电池盒卡箍", 1,
            [(0, -26), (54, -26), (54, -18), (10, -18), (10, 18), (54, 18), (54, 26), (0, 26)],
            [circ(6, -22, M3 / 2), circ(6, 22, M3 / 2), circ(44, -22, M3 / 2), circ(44, 22, M3 / 2)],
            3.0,
            "卡住两节 18650 电池盒（宽约 44mm）。装在甲板**下方**，降低重心。")

# ---------------------------------------------------------------- 9. 防护 / 防滑 / 视觉
add_extrude("20_bumper_front", "前保险杠", 1,
            rect(0, -SCOOP_W / 2 - 6, 16, SCOOP_W / 2 + 6),
            [rrect(2, -46, 14, -42, 2.0), rrect(2, 42, 14, 46, 2.0),
             rrect(4, -20, 12, 20, 2.0)], 10.0,
            "填充 45%。手册明确允许『挤压』但禁止『恶意冲撞』，"
            "这根杠是保护自己前铲的，不是拿来撞人的。")

add_extrude("21_anti_tip", "防翻侧杆", 2,
            [(0, -8), (104, -8), (116, -3), (116, 3), (104, 8), (0, 8)],
            [rrect(2, -2.2, 14, 2.2, 2.2), circ(30, 0, M3 / 2),
             circ(94, 0, 4), circ(110, 0, 4)], 5.0,
            "装在车侧偏低位置，端部离地 8mm。侧翻时先着地，保护舵机和储仓。")

add_extrude("22_ramp_anchor", "坡道防滑撑地块", 1,
            [(0, -12), (40, -12), (48, -4), (48, 4), (40, 12), (0, 12)],
            [circ(8, 0, M3 / 2), circ(22, 0, M3 / 2), circ(38, 0, 3)], 8.0,
            "【进阶功能】SG90 驱动。上 26.6° 坡时放下，橡胶垫压地增加正压力，"
            "麦轮就不会在坡上打滑下滑。平地上收起。")

add_extrude("23_cam_mast", "视觉模块支架", 1,
            [(0, -18), (110, -18), (118, -10), (118, 10), (110, 18), (0, 18)],
            [circ(8, 0, M3 / 2), circ(26, 0, M3 / 2),
             rrect(60, -14, 68, 14, 1.0), rrect(84, -14, 92, 14, 1.0),
             circ(104, 0, 4)], 4.0,
            "【进阶功能】K210/OpenMV 支架，两个腰形槽兼容不同孔距的模块。"
            "相机俯角建议 25°~35°，太低看不到地面方块，太高看不到远处。")

# ============================================================================
# SVG 图纸
# ============================================================================
def svg_of(poly, w=320, h=240):
    minx, miny, maxx, maxy = poly.bounds
    sw = maxx - minx or 1
    sh = maxy - miny or 1
    pad = 14
    sc = min((w - 2 * pad) / sw, (h - 2 * pad) / sh)
    ox = (w - sw * sc) / 2 - minx * sc
    oy = (h - sh * sc) / 2 + maxy * sc      # y 翻转

    def tx(x, y):
        return (ox + x * sc, oy - y * sc)

    def path_of(ring):
        pts = list(ring.coords)
        d = "M " + " L ".join("%.1f,%.1f" % tx(x, y) for x, y in pts) + " Z"
        return d

    parts = []
    for g in polys(poly):
        parts.append('<path d="%s" fill="#dbeafe" stroke="#1d4ed8" stroke-width="1.4" fill-rule="evenodd"/>'
                     % path_of(g.exterior))
        for r in g.interiors:
            parts.append('<path d="%s" fill="#ffffff" stroke="#1d4ed8" stroke-width="1.0"/>'
                         % path_of(r))

    # 外廓尺寸标注
    x0, y0 = tx(minx, maxy)
    x1, y1 = tx(maxx, miny)
    dim = ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#dc2626" stroke-width="1"/>'
           '<text x="%.1f" y="%.1f" font-size="11" fill="#dc2626" text-anchor="middle">%.1f</text>'
           '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#dc2626" stroke-width="1"/>'
           '<text x="%.1f" y="%.1f" font-size="11" fill="#dc2626" text-anchor="middle">%.1f</text>')
    dim = dim % (x0, y0 - 8, x1, y0 - 8, (x0 + x1) / 2, y0 - 12, sw,
                 x1 + 8, y0, x1 + 8, y1, x1 + 14, (y0 + y1) / 2, sh)

    return ('<svg viewBox="0 0 %d %d" width="%d" height="%d" xmlns="http://www.w3.org/2000/svg">'
            '<rect width="100%%" height="100%%" fill="#ffffff"/>%s%s</svg>') % (w, h, w, h, "".join(parts), dim)

# ============================================================================
# -------- 打印件总重估算（按 STL 实测实体体积 × PLA 密度 × 用料系数） --------
_tot_g = 0.0
for p in PARTS:
    p["g"] = p.get("vol", 0.0) * PLA_DENSITY * SOLID_RATIO
    _tot_g += p["g"] * p["qty"]

# 按批次排序输出（打印顺序 = 装配顺序）
_ORDER = sorted(range(len(PARTS)), key=lambda i: (BATCH.get(PARTS[i]["f"], 9), i))
PARTS_ORDERED = [PARTS[i] for i in _ORDER]

_batch_g = {}
for p in PARTS_ORDERED:
    b = BATCH.get(p["f"], 9)
    _batch_g[b] = _batch_g.get(b, 0.0) + p["g"] * p["qty"]

HTML = ("""<!DOCTYPE html><html lang="zh-CN"><head><meta charset="utf-8">
<title>CRTC2026 机器人总动员战队 · 3D 打印件图纸集</title>
<style>
body{font-family:-apple-system,"Segoe UI","Microsoft YaHei",sans-serif;background:#f7f8fa;color:#1f2328;margin:0;padding:32px}
h1{font-size:22px;margin:0 0 4px}h2{font-size:15px;margin:0 0 8px;color:#1d4ed8}
.sub{color:#6b7280;font-size:13px;margin-bottom:24px}
table{width:100%%;border-collapse:collapse;background:#fff;border-radius:8px;overflow:hidden;box-shadow:0 1px 3px rgba(0,0,0,.06)}
th,td{padding:10px 12px;border-bottom:1px solid #eef0f2;font-size:13px;text-align:left;vertical-align:top}
th{background:#f1f5f9;font-weight:600}
tr:last-child td{border-bottom:none}
.note{color:#4b5563;font-size:12px;line-height:1.6}
code{background:#f1f5f9;padding:1px 5px;border-radius:3px;font-size:12px}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(340px,1fr));gap:16px;margin-top:24px}
.card{background:#fff;border-radius:8px;padding:14px;box-shadow:0 1px 3px rgba(0,0,0,.06)}
.qty{display:inline-block;background:#dbeafe;color:#1d4ed8;border-radius:10px;padding:1px 8px;font-size:12px}
.box{background:#fffbeb;border-left:4px solid #f59e0b;border-radius:6px;padding:14px 18px;margin-bottom:22px;font-size:13px;line-height:1.75}
table.inner{width:auto;min-width:520px;margin:8px 0 12px;box-shadow:none}
table.inner th,table.inner td{padding:6px 14px;font-size:12.5px}
.b1,.b2,.b3{display:inline-block;border-radius:10px;padding:1px 8px;font-size:12px;white-space:nowrap}
.b1{background:#fee2e2;color:#b91c1c}
.b2{background:#ffedd5;color:#c2410c}
.b3{background:#dcfce7;color:#15803d}
.spare{display:inline-block;background:#fef3c7;color:#92400e;border-radius:10px;padding:1px 7px;font-size:11px;margin-left:5px}
.spec{font-size:12px;color:#374151;background:#f8fafc;border-radius:5px;padding:7px 10px;margin-top:8px;line-height:1.65}
.why{color:#9ca3af}
</style></head><body>
<h1>CRTC2026 · 3D 打印件图纸集（结构件）</h1>
<div class="sub">队伍：机器人总动员战队 ｜ 单位：mm ｜ 材料：PLA ｜ 生成脚本：<code>mechanical/gen_parts.py</code><br>
结构件估算总重 <b>%.0f g</b>（PLA 1.24 g/cm³ × 用料系数 %.2f，<b>打完请实称</b>）｜
全车预算含电子件约 <b>1076 g</b>，距 2.0 kg 上限余 <b>924 g</b>
（打印件 + 金属底板 155 g + 电机/电池/舵机/电子件）——
余量请拿去加底部配重，重心越低越防滑。<b>装完必须实称</b>。</div>

<div class="box">
<b>打印顺序（别乱，底盘不出来后面全卡住）</b>
<table class="inner"><tr><th>批次</th><th>件数</th><th>估算重量</th><th>什么时候打</th></tr>
%s</table>
<b>通用切片参数</b>：层高 0.20mm ｜ 壁厚 3 圈 ｜ 顶/底 4 层 ｜ 喷嘴 200℃ ｜ 热床 60℃ ｜
首层 20mm/s + 关风扇 ｜ 之后 50~60mm/s + 风扇 100%%。
逐件的层高/填充/支撑见下表。打完第一批底盘件<b>立刻试装</b>，孔插不进螺丝是正常的，
用 3mm 钻头扩一下就行，别推倒重打。
</div>

<table><tr><th>批次</th><th>零件</th><th>数量</th><th>外廓（长×宽×厚）</th>
<th>估重</th><th>层高/填充/支撑</th><th>工艺与装配要点</th><th>STL</th></tr>
""" % (_tot_g, SOLID_RATIO,
       "".join('<tr><td><b>%s</b></td><td>%d</td><td>%.0f g</td><td>%s</td></tr>' % (
           BATCH_NAME.get(b, "其他"),
           sum(1 for p in PARTS_ORDERED if BATCH.get(p["f"], 9) == b),
           _batch_g[b],
           {1: "现在就打，装完车能跑",
            2: "底盘装好就打",
            3: "车能跑之后"}.get(b, "有空再打"))
           for b in sorted(_batch_g))))

for i, p in enumerate(PARTS_ORDERED, 1):
    minx, miny, maxx, maxy = p["poly"].bounds
    lay, inf, sup, _why = slice_param(p)
    b = BATCH.get(p["f"], 9)
    spare = ' <span class="spare">建议多打一份</span>' if p["f"] in SPARE else ""
    HTML += ('<tr><td><span class="b%d">%s</span></td><td><b>%s</b>%s</td>'
             '<td><span class="qty">×%d</span></td>'
             '<td>%.0f × %.0f × %.1f</td><td>%.1f g</td>'
             '<td><code>%s / %s / %s</code></td>'
             '<td class="note">%s</td><td><code>%s.stl</code></td></tr>'
             % (b, BATCH_NAME.get(b, "其他"), html.escape(p["cn"]), spare, p["qty"],
                maxx - minx, maxy - miny, p["t"], p["g"],
                lay, inf, html.escape(sup),
                html.escape(p["note"]), p["f"]))

HTML += "</table><div class='grid'>"
for i, p in enumerate(PARTS_ORDERED, 1):
    lay, inf, sup, why = slice_param(p)
    b = BATCH.get(p["f"], 9)
    HTML += ('<div class="card"><h2>%s · %s</h2>%s'
             '<div class="spec">外廓 %.0f × %.0f × %.1f mm ｜ 估重 %.1f g<br>'
             '层高 %s ｜ 填充 %s ｜ 支撑 %s<span class="why">（%s）</span></div>'
             '<div class="note" style="margin-top:8px">%s</div></div>'
             % (BATCH_NAME.get(b, "其他"), html.escape(p["cn"]), svg_of(p["poly"]),
                p["poly"].bounds[2] - p["poly"].bounds[0],
                p["poly"].bounds[3] - p["poly"].bounds[1], p["t"], p["g"],
                lay, inf, html.escape(sup), html.escape(why),
                html.escape(p["note"])))
HTML += "</div></body></html>"

with open(os.path.join(OUT, "图纸集.html"), "w", encoding="utf-8") as f:
    f.write(HTML)

print("生成 %d 个零件 -> %s" % (len(PARTS), STL_DIR))
for p in PARTS:
    print("   %-24s x%d  %.0fx%.0fx%.1f" % (p["cn"], p["qty"],
          p["poly"].bounds[2] - p["poly"].bounds[0],
          p["poly"].bounds[3] - p["poly"].bounds[1], p["t"]))

# ============================================================================
# 给机械队友的 CAD 交付包（2026-10-04 加）
#   他要的是「能建模的东西」，而不是一堆猜出来的数字。
#   本工程每件都是「2D 轮廓 + 拉伸」，一张 DXF 就是一个件的完整定义：
#   导入 CAD → 选外轮廓 → 拉伸到【厚度】→ 打 DXF 里那些内环（就是孔）。
# ============================================================================
_all_dxf = os.path.join(DXF_DIR, "ALL_全部零件.dxf")
_n_polys = write_dxf(_all_dxf, DXF_ALL)

_ROWS = []
for p in PARTS:
    minx, miny, maxx, maxy = p["poly"].bounds
    _ROWS.append((p["f"], p["cn"], p["qty"], maxx - minx, maxy - miny,
                  p["t"], p["g"], BATCH.get(p["f"], 9)))

with open(os.path.join(DXF_DIR, "尺寸表.csv"), "w", encoding="utf-8-sig") as f:
    f.write("DXF文件,中文名,数量,长mm,宽mm,厚度mm,估重g,批次\n")
    for r in _ROWS:
        f.write("%s,%s,%d,%.1f,%.1f,%.1f,%.1f,%d\n" % r)

_MD = [u"# CRTC2026 · 结构件尺寸表（交给机械队友）", u"",
       u"> 单位 mm ｜ 材料 PLA ｜ 生成脚本 `mechanical/gen_parts.py`（改完重跑，图纸和 STL 同源，不会跑偏）",
       u">",
       u"> **每个件都是「2D 轮廓 + 拉伸」**，所以一张 DXF 就是这个件的完整定义。",
       u"> 改图流程：用 SolidWorks / Fusion 360 / Creo 打开 `零件名.dxf` → 选中外面的闭合轮廓 →",
       u"> **拉伸到【厚度】那一列的数值** → 再把 DXF 里剩下的闭合小环（也就是孔）拉伸切除。",
       u"> 想一次看全部就用 `ALL_全部零件.dxf`，每个零件一个图层，图层名末尾带厚度。",
       u"",
       u"## ⚠ 哪些尺寸不能动（规则红线，改了会判不过）",
       u"",
       u"- **整车外廓**：初始状态 ≤ 290 × 195 × 210 mm（长×宽×高），变形后 ≤ 445 × 400 × 450",
       u"- **整车重量** ≤ 2.0 kg（目前预算 1076 g，余 924 g，余量建议拿去做底部配重）",
       u"- 甲板宽 162 mm 是贴着 195 上限留了余量，加宽前先确认还有多少裕度",
       u"",
       u"## ✅ 哪些随你改（不影响规则）",
       u"",
       u"- `01_deck_front` / `02_deck_rear` 上的**安装长圆槽**：这是给金属底板留的调节量，位置/长度随便调",
       u"- 所有减重孔的位置（只要别和安装槽、舵机槽撞在一起）",
       u"- 外观件（`stl_pretty/` 那批）全部不承重，随便改形状",
       u"",
       u"## 📐 全车已经定死的机械接口（改了要同步改代码）",
       u"",
       u"| 参数 | 值 | 谁在用 |",
       u"|---|---|---|",
       u"| 轮距 TRACK | 136 mm（半 68） | `firmware/Core/Inc/board.h` 的 `ROBOT_HALF_TRACK` |",
       u"| 轴距 WHEELBASE | 180 mm（半 90） | `board.h` 的 `ROBOT_HALF_WHEELBASE` |",
       u"| 麦轮外径 | 60 mm（半径 30） | `board.h` 的 `WHEEL_RADIUS` |",
       u"| 能量单元 | 30 mm 立方（V1.4），六面实心 | 夹爪闭合净距、收集铲喉口 46.4 mm |",
       u"| 安装长圆槽 | 中心 ±60，槽长 30，槽宽 4 | 只此一项和买来的金属底板有关 |",
       u"",
       u"## 零件清单",
       u"",
       u"| DXF 文件 | 中文名 | 数量 | 长 | 宽 | 厚度 | 估重 | 批次 |",
       u"|---|---|---|---|---|---|---|---|"]
for r in _ROWS:
    _MD.append(u"| `%s.dxf` | %s | ×%d | %.0f | %.0f | %.1f | %.1f g | %s |"
               % (r[0], r[1], r[2], r[3], r[4], r[5], r[6], BATCH_NAME.get(r[7], u"其他")))
_MD += [u"",
        u"## 联系与背景",
        u"",
        u"重庆大学 CRTC2026 机器人训练大赛，新生组。电控这边已经把固件写完（STM32F103C8T6，",
        u"DRV8833 驱动、麦轮 X 型逆解、PS2 半自动 + 全自动），**不依赖任何 3D 件的精确尺寸**，",
        u"唯一的要求是上面那张「不能动的机械接口」表里的四个数别变。",
        u""]
with open(os.path.join(DXF_DIR, u"给机械队友_尺寸表.md"), "w", encoding="utf-8") as f:
    f.write(u"\n".join(_MD))

print(u"DXF -> %s （%d 个零件 / %d 条闭合轮廓）" % (DXF_DIR, len(PARTS), _n_polys))
