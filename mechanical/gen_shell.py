# -*- coding: utf-8 -*-
"""
gen_shell.py —— 外观套件生成器（美观用，不承重）

【为什么单独一个脚本】
    gen_parts.py 里那 25 个是**结构件**，尺寸和受力都验算过，不要动。
    外观件另开一个脚本，装上去不好看可以随便改，拆了也不影响车能跑。

【用法】
    python gen_shell.py
产出：
    mechanical/stl_pretty/*.stl     外观件（单独打印，用另一种颜色更有质感）
    mechanical/外观套件.html          带尺寸的说明页

【美观但合规的三条原则】
    1. 外观件一律 2~3mm 薄壁 + 大减重孔 —— 漂亮不能拿重量换，车限重 2.0kg
    2. 不往外扩尺寸：初始尺寸红线 290 x 195 x 210，我们的车是 236 x 162 x 151，
       外观件最多往外扩 3mm/侧，宽到 168，仍然安全
    3. 不挡功能：前脸罩必须给收集铲留开口，侧裙不能蹭轮子，顶盖不能压住储仓口

【配色建议（单色打印机也能做出双色效果）】
    结构件打黑色或深灰（耐脏、显专业）
    外观件打橙色或白色（跳色，一眼能认出是我们的车）
    这样就不用换料，两盘料各打各的。
"""

import os, math
import numpy as np
from shapely.geometry import Polygon, Point, box
from shapely.ops import triangulate, orient
from stl import mesh

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)))
STL_DIR = os.path.join(OUT, "stl_pretty")
os.makedirs(STL_DIR, exist_ok=True)

# ---- 与 gen_parts.py 保持一致的车体尺寸（改车必改这里） ----
W = 162.0          # 车宽
L = 236.0          # 车长
DECK_T = 4.0
COL_H = 15.0       # 立柱高
HALF_W = W / 2.0
HALF_L = L / 2.0
SHELL_T = 2.5      # 外观件壁厚（轻）
SKIRT_OUT = 3.0    # 侧裙往外的量（每侧）

M3 = 3.2
PLA_DENSITY = 1.24      # g/cm3
# ⚠ 全仓库统一口径，必须和 gen_parts.py 的 SOLID_RATIO、tools/print_bill.py 一致
SOLID_RATE = 0.45
SLICE_TXT = "0.16 / 15% / 否"   # 外观件套用参数：细层高、低填充（不承重）

PARTS = []


# ============================================================ 几何工具
def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def circ(cx, cy, r, qs=12):
    return Point(cx, cy).buffer(r, quad_segs=qs)


def rrect(x0, y0, x1, y1, r, qs=4):
    return box(min(x0, x1) + r, min(y0, y1) + r,
               max(x0, x1) - r, max(y0, y1) - r).buffer(r, quad_segs=qs)


def build(outline, holes=()):
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


def _faces_from_polygon(poly, z0, z1):
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
    # 侧壁：沿真实边界环走（外环 CCW / 内环 CW，旋向必须相反）。
    #   旧写法是沿每个三角形的三条边生成，内部边大量重复 -> 网格不闭合，
    #   切片软件会弹"模型有错误"并自动修复；且 Delaunay 在孔边会跨孔，
    #   算出来的体积偏大（结构件实测高估约 12%）。
    for g in polys(poly):
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


def swap_yz(faces):
    """把"平躺着画"的板立起来：(x, 高, 厚) -> (x, 厚, 高)
    侧裙/轮眉这类竖直板必须用它，否则打出来是躺在地上的。

    ⚠ 交换 y/z 是镜像变换（行列式 -1），会把三角形绕序翻过来 -> 法向朝内。
    所以交换后必须把顶点顺序也倒过来，才能保持法向朝外、体积为正。"""
    return [[(x, z, y) for (x, y, z) in f][::-1] for f in faces]


def volume_of(faces):
    """自己算封闭体积（有符号四面体累加）。
    不用 numpy-stl 的 get_mass_properties —— 它遇到"内部边"会警告 mesh 不闭合，
    那种警告会吓得你以为模型坏了，其实切片软件照样能切。"""
    v = 0.0
    for f in faces:
        (ax, ay, az), (bx, by, bz), (cx, cy, cz) = f
        v += (ax * (by * cz - bz * cy)
              - ay * (bx * cz - bz * cx)
              + az * (bx * cy - by * cx))
    return abs(v) / 6.0


def save_stl(name, faces):
    data = np.zeros(len(faces), dtype=mesh.Mesh.dtype)
    for i, f in enumerate(faces):
        for j in range(3):
            data['vectors'][i][j] = f[j]
    m = mesh.Mesh(data)
    path = os.path.join(STL_DIR, name)
    m.save(path)
    return path, volume_of(faces)


def add(name, cn, qty, faces, note):
    path, vol = save_stl(name + ".stl", faces)
    grams = vol / 1000.0 * PLA_DENSITY * SOLID_RATE
    PARTS.append(dict(f=name + ".stl", cn=cn, qty=qty, vol=vol, g=grams,
                      ntri=len(faces), note=note))
    print("  %-26s %6.1f g  %5d 面  x%d" % (name, grams * qty, len(faces), qty))


# ============================================================ 1. 前脸罩
# 车头一段的 U 形罩：正面和两侧包住，后面开口（让收集铲伸出去）
# 平面形状是 U（俯视），挤出方向是车高 —— 直接水平挤出就行
print("生成外观件：")
nose_out = rrect(-HALF_L - 2, -HALF_W - SKIRT_OUT, -HALF_L + 62, HALF_W + SKIRT_OUT, 10)
# 内腔：往后开穿（+80 超出外轮廓，形成 U 形开口），左右各留 2.5 壁
nose_in = rrect(-HALF_L, -HALF_W - SKIRT_OUT + SHELL_T, -HALF_L + 80,
                HALF_W + SKIRT_OUT - SHELL_T, 8)
# 车头正面开一个喇叭口（收集铲的进块通道），宽 150 深 26
mouth = rrect(-HALF_L - 4, -75, -HALF_L + 22, 75, 6)
nose = build(nose_out, [nose_in, mouth])
add("P1_nose_shell", "前脸罩（U 形，带喇叭口）", 1,
    _faces_from_polygon(nose, COL_H + DECK_T, COL_H + DECK_T + 46),
    "壁厚 2.5，正面喇叭口宽 150 必须对准收集铲；高了挡视线就往下截")

# ============================================================ 2. 侧裙（左右各一，分段）
# 竖直板：在 (车长, 车高) 平面画，挤出厚度=SKIRT_OUT，再立起来
SEG = 150.0    # 单段长（打印床限制，和甲板一样分段）
for side, sy in (("L", 1), ("R", -1)):
    for k in range(2):
        x0 = -HALF_L + k * (L - SEG)
        x1 = x0 + SEG if k == 0 else HALF_L
        plate = rrect(x0, 0, x1, 34, 6)          # (u=车长, v=车高 34mm)
        # 三个装饰长槽（同时减重）：圆角矩形竖排
        slots = []
        for i in range(3):
            cx = x0 + (x1 - x0) * (0.25 + 0.25 * i)
            slots.append(rrect(cx - 16, 8, cx + 16, 26, 5))
        p = build(plate, slots)
        faces = swap_yz(_faces_from_polygon(p, 0, SHELL_T))
        # 平移到该侧
        yoff = sy * (HALF_W + SKIRT_OUT / 2.0)
        faces = [[(x, y + yoff, z) for (x, y, z) in f] for f in faces]
        add("P2_skirt_%s%d" % (side, k + 1), "侧裙（%s 第 %d 段）" % ("左" if sy > 0 else "右", k + 1),
            1, faces, "薄壁 3mm，带 3 个装饰槽；装完手动推一下轮子确认不蹭")

# ============================================================ 3. 顶部盖板（减重孔 + 铭牌区）
# 盖在储仓上方，中间留一块平整区域贴队名贴纸。
# ⚠ 顶盖是全车最大的一块平板，最容易把重量堆上去 —— 厚 2.5 + 打满孔，控制在 25g 以内
TOP_L, TOP_W = 150.0, 120.0
top = rrect(-TOP_L / 2, -TOP_W / 2, TOP_L / 2, TOP_W / 2, 10)
holes = []
# 减重孔：5 x 3 阵列，避开中间铭牌区
for i in range(5):
    for j in range(3):
        cx = -TOP_L / 2 + 20 + i * (TOP_L - 40) / 4.0
        cy = -TOP_W / 2 + 20 + j * (TOP_W - 40) / 2.0
        if abs(cx) < 30 and abs(cy) < 18:
            continue                      # 中间留给铭牌
        holes.append(circ(cx, cy, 9))
# 四个角的安装过孔
for sx in (-1, 1):
    for sy in (-1, 1):
        holes.append(circ(sx * (TOP_L / 2 - 9), sy * (TOP_W / 2 - 9), M3 / 2))
p = build(top, holes)
add("P3_top_cover", "顶部盖板（减重孔 + 铭牌区）", 1,
    _faces_from_polygon(p, 0, 2.5),
    "厚 2.5mm 打满孔，约 25g；中间平整区贴队名贴纸；四角 M3 过孔")

# ============================================================ 4. 轮眉（4 个）
# 半圆拱：外半径 35（比轮顶高 5mm），内半径 30，立起来跨在轮子上
# 轮半径 30，轮眉外 34 内 31 —— 只比轮顶高 4mm，别拱太高顶到 210 的车高红线
R_OUT, R_IN = 34.0, 31.0
ring = circ(0, 0, R_OUT).difference(circ(0, 0, R_IN))
arch = ring.intersection(box(-R_OUT, 0, R_OUT, R_OUT))     # 只取上半
FENDER_W = 26.0     # 轮眉宽度（正好盖住轮宽 26）
for i, (wx, wy) in enumerate([(-90, 68), (90, 68), (-90, -68), (90, -68)]):
    faces = swap_yz(_faces_from_polygon(arch, -FENDER_W / 2, FENDER_W / 2))
    faces = [[(x + wx, y + wy, z + 30) for (x, y, z) in f] for f in faces]
    add("P4_fender_%d" % (i + 1), "轮眉 %d" % (i + 1), 1, faces,
        "半圆拱，跨在轮上方；装完转一下轮子，蹭了就打磨内圈")

# ============================================================ 5. 尾部尾板 + 支臂
tail = rrect(-60, -8, 60, 8, 4)
tail_holes = [circ(-40, 0, M3 / 2), circ(40, 0, M3 / 2),
              rrect(-24, -3, 24, 3, 1.5)]
p = build(tail, tail_holes)
add("P5a_tail_plate", "尾部横板", 1, _faces_from_polygon(p, 0, 3),
    "纯装饰，让车尾不显得被切一刀")

for sx in (-1, 1):
    arm = Polygon([(sx * 44, -10), (sx * 78, -10), (sx * 78, 10), (sx * 44, 22),
                   (sx * 44, 10)])
    arm = build(arm, [circ(sx * 72, 0, M3 / 2), circ(sx * 50, 6, M3 / 2)])
    add("P5b_tail_arm_%s" % ("L" if sx < 0 else "R"), "尾板支臂（三角）", 1,
        _faces_from_polygon(arm, 0, 3), "三角造型，装上去车尾有层次")

# ============================================================ 6. 编号牌（空白！）
# ⚠ 规则：编号贴纸不能带数字。这块牌做成空白，你自己贴字母/队名。
plate = rrect(-32, -16, 32, 16, 3)
ph = [circ(-22, 0, M3 / 2), circ(22, 0, M3 / 2)]
p = build(plate, ph)
add("P6_number_plate", "编号牌（空白，贴字母）", 1,
    _faces_from_polygon(p, 0, 2),
    "★ 空白！贴队名或字母编号，**绝对不能贴带数字的编号**，规则禁止")


# ============================================================ 汇总
def summary():
    tot_g = sum(p['g'] * p['qty'] for p in PARTS)
    lines = []
    lines.append("# 外观套件说明")
    lines.append("")
    lines.append("> 由 `gen_shell.py` 生成，改尺寸改脚本重跑即可。")
    lines.append("> **本套件不承重**，只负责好看。拆掉不影响车能跑。")
    lines.append("")
    lines.append("## 尺寸合规核对")
    lines.append("")
    lines.append("| 项 | 规则上限 | 我们的车 | 装外观件后 | 结论 |")
    lines.append("|---|---|---|---|---|")
    lines.append("| 长 | 290 | %.0f | %.0f | 安全 |" % (L, L + 2))
    lines.append("| 宽 | 195 | %.0f | %.0f | 安全 |" % (W, W + 2 * SKIRT_OUT))
    lines.append("| 高 | 210 | 148 | %.0f | 安全 |" % (148 + 3))
    lines.append("")
    lines.append("## 清单")
    lines.append("")
    lines.append("**打印批次：④ 外观套件 —— 最后打。**")
    lines.append("不影响功能，中期考核看的是车能不能跑，别把料先花在壳子上。")
    lines.append("")
    lines.append("| 文件 | 名称 | 数量 | 单件重 | 层高 / 填充 / 支撑 | 工艺备注 |")
    lines.append("|---|---|---|---|---|---|")
    for p in PARTS:
        lines.append("| `%s` | %s | %d | %.1f g | %s | %s |"
                     % (p['f'], p['cn'], p['qty'], p['g'], SLICE_TXT, p['note']))
    lines.append("")
    lines.append("**外观件合计约 %.0f g**（PLA，按 %.0f%% 实体率估算）"
                 % (tot_g, SOLID_RATE * 100))
    lines.append("")
    lines.append("⚠ 整车限重 **2.0 kg** —— 装完外观件**必须重新称重**，")
    lines.append("超了先把顶盖和尾翼拆掉，这两件纯装饰，各能省十几克。")
    lines.append("")
    lines.append("## 配色建议")
    lines.append("")
    lines.append("单色打印机也能做出双色效果：")
    lines.append("- 结构件（gen_parts.py 那 25 个）打**黑色 / 深灰**：耐脏、显专业")
    lines.append("- 外观件（本套）打**橙色 / 白色**：跳色，场上好认")
    lines.append("")
    lines.append("两盘料各打各的，不用中途换料。")
    return "\n".join(lines) + "\n", tot_g


def _tbl(rows):
    """极简 markdown 表格 -> html（本脚本内部用）"""
    h = "<table>"
    for i, r in enumerate(rows):
        cells = [c.strip() for c in r.strip().strip("|").split("|")]
        if all(set(c) <= set("-: ") for c in cells):
            continue
        tag = "th" if i == 0 else "td"
        h += "<tr>" + "".join("<%s>%s</%s>" % (tag, c, tag) for c in cells) + "</tr>"
    return h + "</table>"


# ============================================================ 输出说明页
import re as _re

txt, tot_g = summary()
html_path = os.path.join(OUT, "外观套件.html")
with open(html_path, "w", encoding="utf-8") as f:
    f.write("<!DOCTYPE html><html lang='zh-CN'><head><meta charset='utf-8'>")
    f.write("<title>CRTC2026 外观套件</title>")
    f.write("<style>body{font-family:'Microsoft YaHei',sans-serif;margin:36px;"
            "line-height:1.7;color:#222;max-width:900px}"
            "table{border-collapse:collapse;margin:14px 0;width:100%}"
            "th,td{border:1px solid #ccc;padding:7px 10px;text-align:left;font-size:14px}"
            "th{background:#f0f4f8}code{background:#f5f5f5;padding:2px 6px;"
            "border-radius:3px}h1{border-bottom:3px solid #ff7a1a;padding-bottom:8px}</style>")
    f.write("</head><body>")
    body = txt
    body = _re.sub(r'^# (.*)$', r'<h1>\1</h1>', body, flags=_re.M)
    body = _re.sub(r'^## (.*)$', r'<h2>\1</h2>', body, flags=_re.M)
    out_lines, in_tbl, buf = [], False, []
    for ln in body.split("\n"):
        if ln.startswith("|"):
            buf.append(ln)
            in_tbl = True
            continue
        if in_tbl:
            out_lines.append(_tbl(buf)); buf = []; in_tbl = False
        out_lines.append(ln)
    if buf:
        out_lines.append(_tbl(buf))
    f.write("\n".join(out_lines))
    f.write("</body></html>")

print("\n外观件合计约 %.0f g" % tot_g)
print("件数：%d 个 STL -> %s" % (len(PARTS), STL_DIR))
print("说明页：%s" % html_path)
