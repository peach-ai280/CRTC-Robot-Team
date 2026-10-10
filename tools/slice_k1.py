# -*- coding: utf-8 -*-
"""
CRTC2026 · 自研切片器（输出 G-code 给 Creality K1 Max）
========================================================

为什么自己写：
    组委会要求「按 K1 Max 参数制作 G-code 提交，不要发 3D 模型文件」。
    本机没有装任何切片软件，所以这里用 shapely 直接对 STL 做平面求交、
    生成墙线 + 填充，输出 Klipper / Marlin 通用 G-code。

适用前提（很重要）：
    本工程的零件都是「2D 轮廓 + 拉伸」结构，摆放方向就是设计方向（Z 朝上），
    **不需要支撑**。这个切片器只做：
        外墙/内墙（3 圈）→ 顶/底实心层（各 4 层）→ 中间稀疏填充（25% 网格）
        → 大件加 Brim
    复杂模型（悬垂、桥接、需要支撑）不要用它，请用 Creality Print。

用法（必须用 venv 的 python，因为要 numpy / shapely / numpy-stl）：
    cd C:\\Users\\hp\\WorkBuddy\\CRTC
    C:\\Users\\hp\\.workbuddy\\binaries\\python\\envs\\default\\Scripts\\python.exe tools/slice_k1.py

输出：
    1D:\\WorkBuddy\\CRTC交付\\K1Max_G代码\\<批次>\\<零件名>.gcode
    + 打印清单.csv（预估时间 / 耗材 / 尺寸校验）
"""

import os
import sys
import math
import csv
from collections import defaultdict

import numpy as np
from shapely.geometry import LineString, MultiLineString, box, Point
from shapely.ops import polygonize, unary_union
from shapely import set_precision

try:
    from stl import mesh as stlmesh
except ImportError:
    print("!! 需要 numpy-stl，请用 venv 的 python 运行本脚本")
    sys.exit(1)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(ROOT, "mechanical", "stl")
GAUGE_DIR = os.path.join(ROOT, "mechanical", "stl_gauge")
OUT_ROOT = r"D:\WorkBuddy\CRTC交付\K1Max_G代码"

# ============================================================================
# 打印机与工艺参数（Creality K1 Max + PLA）
# ============================================================================
BED_X, BED_Y = 300.0, 300.0     # 标称床面
MARGIN = 8.0                    # 距床边留 8mm，= 可用 284x284
NOZZLE = 0.40
BEAD = 0.42                     # 挤出线宽（喷嘴 x1.05）
BEAD_FIRST = 0.46               # 首层线宽（吃牢一点）
LAYER = 0.20                    # 层高
WALLS = 3                       # 墙圈数（2 圈对于受力件偏软，用 3 圈）
SOLID_TOP = 4                   # 顶部实心层数
SOLID_BOTTOM = 4                # 底部实心层数
INFILL = 0.25                   # 稀疏填充密度
GAP = 8.0                       # 多件之间的间距
BRIM_W = 6.0                    # 大面积件加 6mm Brim（防翘边）
BRIM_AREA = 4000.0              # 底面投影面积 > 4000mm² 就加 Brim

T_NOZZLE = 220                  # PLA 喷嘴
T_BED = 60                      # PLA 热床
F_FIRST = 1500                  # 首层速度 25mm/s
F_PRINT = 5400                  # 正常 90mm/s（K1 Max 很轻松）
F_TRAVEL = 9000                 # 空驶 150mm/s
RETRACT = 1.2                   # 回抽长度
FILAMENT_D = 1.75
FIL_AREA = math.pi * (FILAMENT_D / 2.0) ** 2   # 2.405 mm²


# ============================================================================
# 1. 读 STL
# ============================================================================
def load(path):
    m = stlmesh.Mesh.from_file(path)
    return np.asarray(m.vectors, dtype=np.float64)      # (N,3,3)


def slice_at(tris, z):
    """平面 z 与三角网格求交，返回该层的多边形列表"""
    segs = []
    for tri in tris:
        zs = tri[:, 2] - z
        if (zs > 0).all() or (zs < 0).all():
            continue
        pts = []
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            za, zb = zs[i], zs[(i + 1) % 3]
            if za == 0.0:
                pts.append((a[0], a[1]))
            if za * zb < 0:
                t = za / (za - zb)
                pts.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        if len(pts) >= 2:
            segs.append(LineString([pts[0], pts[1]]))
    if not segs:
        return []
    ml = MultiLineString(segs)
    ml = set_precision(ml, 1e-3)            # 吸附，避免浮点断点导致环不闭合
    polys = [p for p in polygonize(ml) if p.area > 0.02]
    if not polys:
        return []
    u = unary_union(polys).buffer(0)
    if u.is_empty:
        return []
    return [g for g in getattr(u, "geoms", [u]) if g.area > 0.02]


# ============================================================================
# 2. 单层路径生成
# ============================================================================
def ring_paths(poly, dist):
    """返回距离原轮廓 dist 处的中心线闭环（外环 + 内孔环）"""
    try:
        g = poly.buffer(-dist)
    except Exception:
        return []
    if g.is_empty:
        return []
    out = []
    for gg in getattr(g, "geoms", [g]):
        if gg.is_empty or gg.area <= 0.0:
            continue
        if gg.geom_type == "Polygon":
            out.append(list(gg.exterior.coords))
            for r in gg.interiors:
                out.append(list(r.coords))
        elif gg.geom_type == "MultiPolygon":
            for p in gg.geoms:
                out.append(list(p.exterior.coords))
                for r in p.interiors:
                    out.append(list(r.coords))
    return out


def infill_paths(region, spacing, angle_deg):
    if region.is_empty:
        return []
    minx, miny, maxx, maxy = region.bounds
    d = math.hypot(maxx - minx, maxy - miny) + 4.0
    cx, cy = (minx + maxx) / 2.0, (miny + maxy) / 2.0
    a = math.radians(angle_deg)
    dx, dy = math.cos(a), math.sin(a)          # 走线方向
    nx, ny = -dy, dx                           # 步进方向
    n = int(d / spacing) + 2
    paths = []
    for i in range(-n, n + 1):
        off = i * spacing
        px, py = cx + nx * off, cy + ny * off
        p1 = (px - dx * d / 2, py - dy * d / 2)
        p2 = (px + dx * d / 2, py + dy * d / 2)
        inter = region.intersection(LineString([p1, p2]))
        if inter.is_empty:
            continue
        for g in getattr(inter, "geoms", [inter]):
            if g.geom_type == "LineString" and g.length > 0.4:
                c = list(g.coords)
                paths.append(c if i % 2 == 0 else c[::-1])   # 蛇形，少走空程
    return paths


# ============================================================================
# 3. G-code 拼装
# ============================================================================
class Gcode:
    def __init__(self):
        self.lines = []
        self.e = 0.0
        self.length = 0.0

    def raw(self, s):
        self.lines.append(s)

    def travel(self, x, y):
        self.raw("G0 X%.3f Y%.3f F%d" % (x, y, F_TRAVEL))

    def do_z(self, z):
        self.raw("G0 Z%.3f F1200" % z)

    def extrude_to(self, x, y, bead, layer, first=False):
        px, py = self.last_x, self.last_y
        L = math.hypot(x - px, y - py)
        if L < 1e-6:
            return
        vol = L * bead * layer
        de = vol / FIL_AREA          # ★ M83 = 相对挤出，这里必须发增量，不能发累计值
        self.e += de
        self.length += L
        f = F_FIRST if first else F_PRINT
        self.raw("G1 X%.3f Y%.3f E%.5f F%d" % (x, y, de, f))
        self.last_x, self.last_y = x, y

    def retract(self):
        self.raw("G1 E-%.3f F2400" % RETRACT)

    def unretract(self):
        self.raw("G1 E%.3f F2400" % RETRACT)


def path_total(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


def emit_layer(g, polys, z, bead, layer, first, solid, infill_region):
    for i in range(WALLS):
        dist = bead * (0.5 + i)
        hit = False
        for poly in polys:
            for ring in ring_paths(poly, dist):
                if len(ring) < 3:
                    continue
                hit = True
                g.retract()
                g.travel(ring[0][0], ring[0][1])
                g.unretract()
                g.last_x, g.last_y = ring[0][0], ring[0][1]
                for p in ring[1:]:
                    g.extrude_to(p[0], p[1], bead, layer, first)
        if not hit:
            break

    if infill_region is None or infill_region.is_empty:
        return
    spacing = bead if solid else bead / INFILL
    ang = 45.0 if solid else (0.0 if int(round(z / LAYER)) % 2 == 0 else 90.0)
    for line in infill_paths(infill_region, spacing, ang):
        if len(line) < 2:
            continue
        g.retract()
        g.travel(line[0][0], line[0][1])
        g.unretract()
        g.last_x, g.last_y = line[0][0], line[0][1]
        for p in line[1:]:
            g.extrude_to(p[0], p[1], bead, layer, first)


# ★ 清理线（purge line）的挤出量必须在「Python 里算好」再写进模板。
#   曾经把它写成模板里的表达式 {100*BEAD_FIRST*0.30/FIL_AREA:.4f}，
#   结果 53 份 G-code 全部输出了字面文本 E{100*...} —— 打印机读到会直接报
#   "Unable to parse move"。模板字符串只能做占位替换，不会帮你算表达式。
#   自检脚本 check_gcode.py 现在有「逐行语法合法性」一项，能抓到这类问题。
PURGE_E = 100.0 * BEAD_FIRST * 0.30 / FIL_AREA     # 100mm 长的清理线
PURGE_F = 1200                                       # 清理线速度（慢一点，出料稳）

HEADER = """; ===========================================================================
; CRTC2026 · 自研切片器生成（tools/slice_k1.py）
; 目标机器：Creality K1 Max  耗材：PLA
; 喷嘴 {noz}mm / 层高 {layer}mm / 墙 {walls} 圈 / 填充 {infill:.0%} / Brim {brim}
; 零件：{name}   预估耗材 {fil:.1f}g   预估时长 {mins:.0f} 分钟
; 说明：本文件是通用 Klipper/Marlin G-code，不依赖任何 Creality 专有宏。
;       若机器改过料/改过 Z-offset，请先在机器上跑一次自动调平再打。
; ===========================================================================
M140 S{bed}
M104 S{tnoz}
M190 S{bed}
M109 S{tnoz}
G21
G90
M83
G28
G1 Z5 F600
; ---- 清理线（在床前缘挤一段，把残料带出来）----
G1 X20 Y20 F9000
G1 Z0.30 F600
G1 X20 Y120 E{purge_e:.4f} F{purge_f}
G1 X21.2 Y120 F3000
G1 X21.2 Y20 E{purge_back:.4f} F{purge_f}
G0 Z2 F1200
"""

FOOTER = """; ---- 结束 ----
M107
M104 S0
M140 S0
M83
G1 E-2 F2400
G91
G1 Z{lift:.1f} F600      ; 抬到零件最高点以上再回原点，别蹭倒零件
G90
G28 X Y
M84
"""


def slice_one(tris, name, plate_xy):
    """把零件放到 plate_xy 指定的床面位置，返回 (gcode文本, 用时分钟, 克数, 尺寸)"""
    minx, miny, _ = tris.reshape(-1, 3).min(axis=0)
    maxx, maxy, maxz = tris.reshape(-1, 3).max(axis=0)
    h = maxz - min(0.0, 0.0)
    zmin = tris.reshape(-1, 3)[:, 2].min()
    h = maxz - zmin
    ox = plate_xy[0] - (minx + maxx) / 2.0
    oy = plate_xy[1] - (miny + maxy) / 2.0
    t2 = tris.copy()
    t2[:, :, 0] += ox
    t2[:, :, 1] += oy

    nlayer = max(1, int(round(h / LAYER)))
    area0 = 0.0
    base = slice_at(t2, zmin + LAYER * 0.5)
    for p in base:
        area0 += p.area
    brim = area0 > BRIM_AREA

    est_vol = 0.0
    g = Gcode()
    footprints = []
    for k in range(nlayer):
        zc = zmin + LAYER * (k + 0.5)
        if k == 0:
            polys = base
        else:
            polys = slice_at(t2, zc)
        if not polys:
            polys = footprints[-1] if footprints else []
        footprints.append(polys)
        if not polys:
            continue
        first = (k == 0)
        bead = BEAD_FIRST if first else BEAD
        solid = (k < SOLID_BOTTOM) or (k >= nlayer - SOLID_TOP) or (nlayer <= SOLID_TOP + SOLID_BOTTOM)
        inner = None
        try:
            u = unary_union(polys)
            inner = u.buffer(-(bead * (0.5 + WALLS - 1) + bead * 0.5))
            if inner.is_empty:
                inner = None
        except Exception:
            inner = None

        g.do_z(zmin + LAYER * (k + 1))      # 本层顶面高度：首层 Z=0.2
        if first and brim:
            loops = int(BRIM_W / bead)
            for i in range(loops, 0, -1):
                for poly in polys:
                    for ring in ring_paths(poly, -bead * i):
                        if len(ring) < 3:
                            continue
                        g.retract()
                        g.travel(ring[0][0], ring[0][1])
                        g.unretract()
                        g.last_x, g.last_y = ring[0][0], ring[0][1]
                        for p in ring[1:]:
                            g.extrude_to(p[0], p[1], bead, LAYER, True)
        emit_layer(g, polys, zmin + LAYER * (k + 0.5), bead, LAYER, first, solid, inner)

    est_vol = g.e * FIL_AREA
    grams = est_vol * 1.24 / 1000.0            # PLA 密度 1.24 g/cm³
    # 时间估算：挤出段 + 空驶段
    seg_speed = (F_PRINT / 60.0)
    t_sec = g.length / max(seg_speed, 1.0) * 1.35
    mins = t_sec / 60.0

    hdr = HEADER.format(noz=NOZZLE, layer=LAYER, walls=WALLS, infill=INFILL,
                        brim=("%.0fmm" % BRIM_W) if brim else "无", name=name,
                        fil=grams, mins=mins,
                        bed=int(T_BED), tnoz=int(T_NOZZLE),
                        purge_e=PURGE_E, purge_back=-PURGE_E, purge_f=PURGE_F)
    # 首层位置已在 header 里写了清理线，这里把绝对定位补上
    txt = (hdr + "; ---- LAYERS START ----\n" + "\n".join(g.lines) +
           "\n; ---- LAYERS END ----\n" + FOOTER.format(lift=h + 15.0))
    dims = (maxx - minx, maxy - miny, h)
    return txt, mins, grams, dims, brim


def main():
    if not os.path.isdir(STL_DIR):
        print("!! 找不到", STL_DIR)
        return
    os.makedirs(OUT_ROOT, exist_ok=True)

    # 打印批次（和 3D打印交付/ 的分组保持一致）
    BATCH = [
        # ⚠ 2026-10-04：甲板固定改成十字槽之后，板宽已经不用量了，
        #   所以旧的「板宽梳 gauge_width_comb」作废，整批只剩 gauge_height 一件。
        #   这一件是确认「板面离地高度」的——太低会让 TT 马达外壳蹭地。
        ("00_先打这个_只有1件", ["gauge_height"]),
        ("01_底盘", ["03_deck_splice", "18_electronics_tray", "19_battery_clip",
                     "20_bumper_front", "21_anti_tip", "04_column",
                     "01_deck_front", "02_deck_rear"]),
        ("02_收集与储仓", ["05_scoop_floor", "05b_scoop_wall_L", "05c_scoop_wall_R",
                           "05d_scoop_back", "06_limit_lever", "12_bin_floor", "13_bin_side",
                           "14_bin_divider", "15_bin_flap"]),
        # ★ 2026-10-10 大改：机械臂从「大臂+小臂两连杆」换成**平行四连杆**
        #   （照用户给的参考视频做的）。所以这一批换了件：
        #     新增 30 臂座 / 31a 回转盘 / 31b 肩架竖板 / 32 平行臂杆 ×2 / 33 肘座
        #     旧的 09 大臂、10 小臂、11 舵盘板、11b 爪转接板、25 增高座 **不再需要**，
        #     已从本批次移除（STL 还留在仓库里，但不建议打，省料）。
        ("03_机械臂", ["30_arm_base", "31a_arm_turntable", "31b_arm_mast",
                       "32_arm_link", "33_arm_elbow",
                       "07_grip_finger_L", "08_grip_finger_R",
                       "16_rack_hook", "17_cave_probe", "22_ramp_anchor", "23_cam_mast"]),
        # ★ 2026-10-04 新增：用户要的「平行夹爪」整套（不是转接板，是爪子本身）
        ("05_平行夹爪", ["24a_claw_base", "24b_claw_slider", "24c_claw_finger",
                        "24d_claw_horn", "24e_claw_link"]),
        # ★ 齿轮齿条版平行夹爪（和用户给的那款市售爪同结构）。和 05 二选一，
        #   建议两套都打 —— 齿轮版齿形万一卡死，立刻换连杆版，车不会开天窗。
        ("06_齿轮夹爪_可选", ["24h_claw_base_gear", "24f_claw_gear",
                            "24g_claw_rack", "24i_claw_finger_gear"]),
        # ★ 2026-10-04 新增：能量块道具（和组委会发的块同规格 30×30×30、内腔 15³、
        #   壁厚 7.5、45° 倒角 2.121）。拍中期考核视频 + 标定夹爪 + 练夹取用，不装车。
        #   放最后一批：万一料不够，先保证装车件，道具可以少打几个。
        ("07_能量块道具_拍视频", ["26_cube_mock"]),
        # ★ 2026-10-04 补：用户说「可能只能打一次」，而上面各批次全是「正好够用」的量，
        #   一个备件都没有。摔断/拧裂/装丢一件就得再求人打一次 —— 赌不起。
        #   下面 8 件是易损件备件，共约 25g / 1 小时，一次打齐，装车时坏了直接换。
        ("08_备件_建议打", [("04_column", 2), ("05_scoop_floor", 1),
                            ("06_limit_lever", 1),
                            ("15_bin_flap", 1), ("24b_claw_slider", 1),
                            ("24g_claw_rack", 1)]),
    ]
    # 外观件（漂亮件）也要能出 G-code —— 单独一批，不承重、可选
    pretty_dir = os.path.join(ROOT, "mechanical", "stl_pretty")
    if os.path.isdir(pretty_dir):
        names = sorted(os.path.splitext(f)[0] for f in os.listdir(pretty_dir) if f.endswith(".stl"))
        BATCH.append(("04_外观件_可选", names))
        for _n in names:
            pass

    QTY = {"04_column": 4, "21_anti_tip": 2, "05b_scoop_wall_L": 1, "05c_scoop_wall_R": 1,
           "06_limit_lever": 2, "13_bin_side": 2, "14_bin_divider": 2, "15_bin_flap": 3,
           # ★ 2026-10-10：11_servo_horn_plate / 11b_claw_adapter 是旧「两连杆臂」的件，
           #   新方案改用平行四连杆后**不再需要**，已从批次和数量表里移除。
           # ★ 平行四连杆臂：两根杆是**完全相同的件**，所以打 2 件
           "32_arm_link": 2,
           # 平行夹爪：滑块/爪指各 2（左右），连杆 3、摇臂 2、爪指 3 —— 多的是备件
           "24b_claw_slider": 2, "24c_claw_finger": 3,
           "24d_claw_horn": 2, "24e_claw_link": 3,
           # 齿轮齿条版：齿轮 2（1 用 1 备）、齿条 2（左右）、爪指 3（2 用 1 备）
           "24f_claw_gear": 2, "24g_claw_rack": 2, "24i_claw_finger_gear": 3,
           # 能量块道具：打 4 个（建议 3 白 + 1 黄，黄色要换料；没黄料就贴黄胶带）
           "26_cube_mock": 4}

    rows = []
    for batch, names in BATCH:
        os.makedirs(os.path.join(OUT_ROOT, batch), exist_ok=True)
        for _item in names:
            # 批次里可以写 (件名, 数量) 来覆盖全局 QTY —— 备件批次用这个
            if isinstance(_item, tuple):
                nm, _qty_override = _item
            else:
                nm, _qty_override = _item, None
            if batch.startswith("00"):
                p = os.path.join(GAUGE_DIR, nm + ".stl")
            elif batch.startswith("04_外观"):
                p = os.path.join(ROOT, "mechanical", "stl_pretty", nm + ".stl")
            else:
                p = os.path.join(STL_DIR, nm + ".stl")
            if not os.path.exists(p):
                print("  !! 缺文件", nm)
                continue
            tris = load(p)
            qty = _qty_override if _qty_override else QTY.get(nm, 1)
            # 排布：件与件间距 8mm，超宽换行
            bb = tris.reshape(-1, 3)
            _area = 0.0
            for _p in slice_at(tris, bb[:, 2].min() + LAYER * 0.5):
                _area += _p.area
            _pad = (2.0 * BRIM_W + 2.0) if _area > BRIM_AREA else 2.0   # Brim 也算进外形
            w = bb[:, 0].max() - bb[:, 0].min() + _pad
            h = bb[:, 1].max() - bb[:, 1].min() + _pad
            usable = BED_X - 2 * MARGIN
            per_row = max(1, int((usable + GAP) // (w + GAP)))
            per_col = int(math.ceil(qty / per_row))
            row_h = h + GAP
            total_h = per_col * row_h
            start_y = (BED_Y - total_h) / 2.0 + row_h / 2.0
            for i in range(qty):
                r, c = divmod(i, per_row)
                if r == 0:
                    n_in_row = min(per_row, qty)
                else:
                    n_in_row = min(per_row, qty - r * per_row)
                row_w = n_in_row * (w + GAP) - GAP
                start_x = (BED_X - row_w) / 2.0 + (w + GAP) / 2.0
                cx = start_x + c * (w + GAP)
                cy = start_y + r * row_h
                txt, mins, grams, dims, brim = slice_one(tris, nm, (cx, cy))
                out = os.path.join(OUT_ROOT, batch, "%s%s.gcode" % (nm, "" if qty == 1 else "_%d" % (i + 1)))
                with open(out, "w", newline="\n") as f:
                    f.write(txt)
                rows.append((batch, nm, i + 1, dims[0], dims[1], dims[2], mins, grams, "是" if brim else "否", out))
            print("  %-22s x%-2d  约 %5.1f 分钟 / %5.1f g" %
                  (nm, qty, sum(r[6] for r in rows if r[1] == nm),
                   sum(r[7] for r in rows if r[1] == nm)))

    with open(os.path.join(OUT_ROOT, "打印清单.csv"), "w", newline="", encoding="utf-8-sig") as f:
        w_ = csv.writer(f)
        w_.writerow(["批次", "零件", "第几件", "长mm", "宽mm", "高mm", "预估分钟", "预估克重", "Brim", "文件"])
        for r in rows:
            w_.writerow([r[0], r[1], r[2], "%.1f" % r[3], "%.1f" % r[4], "%.1f" % r[5],
                         "%.1f" % r[6], "%.2f" % r[7], r[8], r[9]])
    tot_g = sum(r[7] for r in rows)
    tot_m = sum(r[6] for r in rows)
    print("-" * 60)
    print("共 %d 个 gcode 文件，合计 %.0f g / 约 %.1f 小时（单件串打）" % (len(rows), tot_g, tot_m / 60.0))
    print("输出：", OUT_ROOT)


if __name__ == "__main__":
    main()
