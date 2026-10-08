# -*- coding: utf-8 -*-
"""
gen_gauge.py —— CRTC2026 「量板规」生成器（工装，不是车上零件）

【为什么要有这东西】
    甲板是靠 4 根立柱锁在买来的金属底板上的，这是全车唯一一个
    「必须和买来的东西对上」的尺寸。但金属板是手工折弯件、卖家自注
    ±0.5~1cm 误差，而我拿不到实物。

    拍照让我读数这条路不准：透视 + 摄像头畸变，误差 1~2mm，而孔位
    配合要 0.5mm 级。所以反过来做——出一副 1:1 的量规，打出来往板上
    一放，能不能卡进去是「是/否」问题，不需要读任何数字。

【产出】
    mechanical/stl_gauge/*.stl
      gauge_width_comb.stl   板宽梳   100 / 120 / 140 三档（判断板够不够宽）
      gauge_height.stl       离地规   24 / 28 / 30 三档（判断车底会不会拖地）

    ★ 2026-10-04：原来的 gauge_hole_140（孔位片）已作废、删掉。
      因为甲板安装孔从「固定圆孔」改成了「长圆槽」，能覆盖板宽 100~160mm，
      不再需要拿孔位片去比对孔距。少打一件，省 20 分钟。

【用法】
    python gen_gauge.py
    注意：需要 numpy / shapely / numpy-stl（用 venv 那个 python 跑）

【改尺寸的姿势】
    下面的 G_* 常量就是要测的判定阈值。如果 gen_parts.py 里的
    MOUNT_Y 改了，这里必须跟着改，否则量规会骗人。
    ⚠ COL_H 已经和量规解耦（它是我们自己的设计高度，不再由板高推导），
      所以离地规的结论是"要不要加垫片"，不是"改 COL_H"。
"""

import os
import numpy as np
from shapely.geometry import Polygon, box
from shapely.ops import triangulate, orient, unary_union
from stl import mesh

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(OUT, "stl_gauge")
os.makedirs(STL_DIR, exist_ok=True)

# ============================================================================
# 判定阈值（这几行必须和 gen_parts.py / 装配要求对齐）
# ============================================================================
MOUNT_Y   = 60.0    # 甲板立柱槽中心半距（gen_parts.py 同值，2026-10-04 由 70 改 60）
MOUNT_2Y  = MOUNT_Y * 2.0        # 120
GAUGE_T   = 2.0     # 量规厚度（薄片，平躺打，不要支撑）

# 板宽三档：梳子三个齿的长度（从基准边量起）
#   槽覆盖范围 = MOUNT_Y ± 15 → 45 ~ 75（单边），所以：
#     板宽 ≥ 90  就能用"螺钉 + 长圆槽"
#     板宽 ≥ 100 完全安全（每边留 5mm 余量）
#     板宽 < 90  只能用扎带绕过板边缘
W_A = 100.0   # 能卡进去 -> 板宽 ≥100，长圆槽方案绝对够用
W_B = 120.0   # 中间档，同样够用（只是余量小一点）
W_C = 140.0   # 能到这一档 -> 板比标称还宽，随便装

# 离地高度三档（金属板折弯平面离地）。
#   判定依据：麦轮半径 30mm，轮轴离地必须 ≈30mm 车才不拖地；
#   板面离地低于 28mm 时，TT 马达减速箱和铲唇最容易刮地胶。
H_LOW  = 24.0
H_MID  = 28.0
H_OK   = 30.0


# ============================================================================
# 几何工具（与 gen_parts.py 同一套，独立复制以免互相影响）
# ============================================================================
def build(outline, holes=()):
    poly = unary_union(list(outline)) if not isinstance(outline, Polygon) else Polygon(outline)
    if not poly.is_valid:
        poly = poly.buffer(0)
    for h in holes:
        g = h if isinstance(h, Polygon) else Polygon(h)
        poly = poly.difference(g)
    if not poly.is_valid:
        poly = poly.buffer(0)
    return orient(poly, sign=1.0)


def circ(cx, cy, r, qs=24):
    from shapely.geometry import Point
    return Point(cx, cy).buffer(r, quad_segs=qs)


def polys(poly):
    return list(poly.geoms) if poly.geom_type == "MultiPolygon" else [poly]


def _q(p):
    """顶点量化，用于边的配对比较（避免浮点误差导致配对失败）"""
    return (round(float(p[0]), 5), round(float(p[1]), 5))


def _faces_from_polygon(poly, z0, z1):
    """把带洞多边形挤出成三角面片（水密：侧壁只沿真正的边界边生成）"""
    tris = []
    for g in polys(poly):
        for t in triangulate(g):
            c = t.representative_point()
            if not g.contains(c):
                continue
            t = orient(t, sign=1.0)
            xs, ys = t.exterior.coords.xy
            tris.append([(xs[i], ys[i]) for i in range(3)])
    faces = []
    z0, z1 = float(z0), float(z1)
    for p in tris:
        a, b, c = p
        faces.append([(a[0], a[1], z0), (c[0], c[1], z0), (b[0], b[1], z0)])
        faces.append([(a[0], a[1], z1), (b[0], b[1], z1), (c[0], c[1], z1)])
    # 侧壁：沿多边形真实的边界环走（不用三角剖分的边去凑，
    # 因为 Delaunay 在圆弧处会产生 T 型交点，配不上对 -> 网格不闭合）
    for g in polys(poly):
        # ⚠ 外环 CCW、内环（洞）CW，旋向必须相反，否则孔的体积会被加进去
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


def save_stl(name, faces):
    if not faces:
        print("  !! 空模型:", name)
        return 0
    data = np.zeros(len(faces), dtype=mesh.Mesh.dtype)
    for i, f in enumerate(faces):
        data["vectors"][i] = np.array(f, dtype=np.float64)
    m = mesh.Mesh(data)
    path = os.path.join(STL_DIR, name + ".stl")
    m.save(path)
    vol = abs(m.get_mass_properties()[0]) if hasattr(m, "get_mass_properties") else 0.0
    print("  %-22s %6.1f x %6.1f x %4.1f mm   体积 %8.1f mm3" %
          (name + ".stl",
           m.x.max() - m.x.min(), m.y.max() - m.y.min(), m.z.max() - m.z.min(),
           vol))
    return vol


# ============================================================================
# 件 1：板宽梳 —— 三个齿，长度分别 100 / 120 / 140
#   用法：手柄基准边贴住金属板一条长边，齿朝板内伸。
#         齿尖还落在板面上 = 板宽 ≥ 该档；齿尖悬空 = 板宽 < 该档。
#   结论只有三种：任意齿都卡得住 → 直接打甲板；
#                 只有 100 齿卡住 → 也能打（每边余量小）；
#                 连 100 齿都悬空 → 甲板改用扎带固定（板太窄）。
# ============================================================================
def make_width_comb():
    handle = box(0, 0, 150, 16)
    # 齿越粗 = 档位越大，和齿长一起递增，不容易搞混
    t1 = box(8,  16, 18,  W_A)    # 齿宽 10 -> 100
    t2 = box(60, 16, 74,  W_B)    # 齿宽 14 -> 120
    t3 = box(116, 16, 134, W_C)   # 齿宽 18 -> 140
    poly = build([handle, t1, t2, t3])
    save_stl("gauge_width_comb", _faces_from_polygon(poly, 0, GAUGE_T))
    return poly


# ============================================================================
# 件 2：孔位片 —— ★ 2026-10-04 作废，不再生成
#   甲板安装孔已从「固定圆孔」改为「长圆槽」，槽本身就吃掉 ±15mm 的误差，
#   不需要再拿孔位片去比对孔距。保留这段注释是为了说明"它去哪了"，
#   免得以后有人翻到旧文档又去打印一个没用的件。
# ============================================================================
def make_hole_gauge():
    return None


# ============================================================================
# 件 3：离地高规 —— 24 / 28 / 30 三个台阶
#   用法：把基准底边放在桌面上，从金属板侧面推进去，看"板的下沿"卡在哪一档。
#   结论（注意：不是改 COL_H，COL_H 已经是固定的 22mm）：
#     30 塞得进 → 完美，轮轴离地正好 ≈30mm，不用动
#     只有 28 塞得进 → 差 2mm，在电机和板之间垫 2 片 M3 垫圈
#     只有 24 塞得进 → 差 6mm，垫 4~6 片垫圈，或换 32mm 高的折弯板
#     连 24 都塞不进 → 板太扁，TT 马达减速箱会刮地，必须加高（垫片/换板）
# ============================================================================
def make_height_gauge():
    base = box(0, 0, 86, 24)
    faces = _faces_from_polygon(build(base), 0, GAUGE_T)
    blk_a = box(4,  4, 24, 20)     # 24 档
    blk_b = box(31, 4, 51, 20)     # 28 档
    blk_c = box(58, 4, 78, 20)     # 30 档
    faces += _faces_from_polygon(build(blk_a), GAUGE_T, H_LOW)
    faces += _faces_from_polygon(build(blk_b), GAUGE_T, H_MID)
    faces += _faces_from_polygon(build(blk_c), GAUGE_T, H_OK)
    save_stl("gauge_height", faces)
    return base


if __name__ == "__main__":
    print("生成量板规 -> %s" % STL_DIR)
    make_width_comb()
    make_hole_gauge()
    make_height_gauge()
    print()
    print("判定表（打完量了照着填）：")
    print("  [板宽梳] 140 齿能卡进去 -> 板比标称还宽，随便装")
    print("           120 或 100 齿卡进去 -> 都够用，长圆槽覆盖 100~160mm")
    print("           100 齿也悬空 -> 板宽 <100：甲板改用扎带绕过板边缘固定")
    print("  [离地规] 30 台阶塞得进 -> 完美，不用动")
    print("           只有 28 塞得进 -> 电机与板之间垫 2 片 M3 垫圈")
    print("           只有 24 塞得进 -> 垫 4~6 片，车底会偏低，注意铲唇")
    print("           24 都塞不进 -> 车会拖地，必须加高（垫片/换板）")
