# -*- coding: utf-8 -*-
"""
轮组 / 底板 / 机械臂增高座 的关系图（纯 2D 矢量画，不依赖 3D 渲染）

为什么单独画这一张：
    整车 3D 图只能看出"轮子在板下面"，看不出**为什么**板必须那么高、
    以及轮径换了以后哪个件要跟着改。这张图把几何关系摊平了讲：
        麦轮轴心离地 = 轮半径（这是物理，改不了）
        → 板面必须比轮顶高（否则轮子从板里穿出来）
        → 板一抬高，机械臂肩高跟着涨，就够不到地了
        → 所以「增高座高度 = 90 - 板面高」，把肩高锁死在 136mm

输出：D:\\WorkBuddy\\CRTC交付\\示意图\\04_轮组与底板干涉的三条解法.png
跑法（仓库根）：
    D:\\WorkBuddy\\tools\\pyrender\\Scripts\\python.exe tools/render_wheel_note.py
"""

import os
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = "C:/Windows/Fonts/msyh.ttc"
OUT = r"D:\WorkBuddy\CRTC交付\示意图\04_轮组与底板干涉的三条解法.png"

W, H = 1760, 1420
BG = (250, 250, 251)
INK = (38, 42, 48)
INK2 = (96, 102, 110)
RED = (198, 60, 48)
GRN = (32, 132, 92)
ORA = (228, 118, 46)
BLU = (44, 96, 160)
METAL = (150, 154, 160)
SCALE = 4.0                 # 4 px = 1 mm


def F(size):
    return ImageFont.truetype(FONT_PATH, size)


def txt(d, xy, s, size=20, fill=INK, anchor="la"):
    d.text(xy, s, font=F(size), fill=fill, anchor=anchor)


def textw(d, s, size):
    return d.textlength(s, font=F(size))


def line(d, p, q, fill=INK2, w=2, dash=None):
    if not dash:
        d.line([p, q], fill=fill, width=w)
        return
    import math
    x0, y0 = p
    x1, y1 = q
    tot = math.hypot(x1 - x0, y1 - y0)
    if tot == 0:
        return
    ux, uy = (x1 - x0) / tot, (y1 - y0) / tot
    t = 0.0
    while t < tot:
        a = t
        b = min(t + dash, tot)
        d.line([(x0 + ux * a, y0 + uy * a), (x0 + ux * b, y0 + uy * b)],
               fill=fill, width=w)
        t += dash * 2


def panel(d, x0, y0, x1, y1, title, sub=None, edge=INK2):
    d.rounded_rectangle([x0, y0, x1, y1], 12, fill=(255, 255, 255),
                        outline=edge, width=2)
    txt(d, (x0 + 20, y0 + 14), title, 24, INK)
    if sub:
        txt(d, (x0 + 20, y0 + 46), sub, 19, INK2)


def wheel(d, cx, ground, r=30.0, fill=(58, 62, 68)):
    R = r * SCALE
    cy = ground - R
    d.ellipse([cx - R, cy - R, cx + R, cy + R], fill=fill)
    d.ellipse([cx - 9, cy - 9, cx + 9, cy + 9], fill=(210, 212, 216))
    return cy


def plate(d, x0, x1, top_mm, ground, thick=1.5, fill=METAL):
    y = ground - top_mm * SCALE
    d.rectangle([x0, y, x1, y + thick * SCALE], fill=fill,
                outline=(96, 100, 106))


def dim_z(d, x, y_from, y_to, label, fill=INK2, side="right"):
    d.line([(x, y_from), (x, y_to)], fill=fill, width=2)
    for yy in (y_from, y_to):
        d.line([(x - 7, yy), (x + 7, yy)], fill=fill, width=2)
    if side == "right":
        txt(d, (x + 12, (y_from + y_to) / 2), label, 19, fill, anchor="lm")
    else:
        txt(d, (x - 12, (y_from + y_to) / 2), label, 19, fill, anchor="rm")


def main():
    im = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(im)

    txt(d, (40, 26), "轮组 / 底板 / 机械臂增高座 —— 到底怎么装才不会穿模",
        32, INK)
    txt(d, (40, 72),
        "一句话：麦轮轴心离地 = 轮半径，板面必须比轮顶高 → 板一抬，臂的肩高就涨 → "
        "所以增高座要按「90 − 板面高」切短，肩高锁死 136mm（够地、够 250mm 物资架）",
        21, INK2)

    pw, ph = 540, 700
    py = 130
    px = [60, 620, 1180]

    # ================= 面板 1：旧图（错） =================
    x0 = px[0]
    panel(d, x0, py, x0 + pw, py + ph,
          "① 旧图（错的）", "板面定在 30mm —— 轮子上半截从板里穿出来", edge=RED)
    g = py + ph - 60
    cx = x0 + pw / 2
    line(d, (x0 + 30, g), (x0 + pw - 30, g), INK, 3)          # 地面
    cy = wheel(d, cx, g)
    plate(d, x0 + 50, x0 + pw - 50, 30.0, g)
    # 穿模处打红叉
    for dx in (-1, 1):
        line(d, (cx + dx * 34 - 18, cy - 34 - 18), (cx + dx * 34 + 18, cy - 34 + 18), RED, 5)
        line(d, (cx + dx * 34 - 18, cy - 34 + 18), (cx + dx * 34 + 18, cy - 34 - 18), RED, 5)
    txt(d, (cx, cy - 110), "轮顶 = 60mm", 20, RED, anchor="mb")
    txt(d, (cx, cy + 16), "板面 = 30mm", 20, RED, anchor="mt")
    txt(d, (cx, g - 6), "轴心离地 = 轮半径 30mm", 19, INK2, anchor="mb")
    txt(d, (x0 + 24, py + ph - 38),
        "30 < 60 → 轮子必然穿板，还会穿到甲板和电控托盘上。",
        19, RED)
    txt(d, (cx, py + 300), "轴心离地 = 轮半径，这是物理约束，改不了；\n能改的只有板的安装高度。",
        20, INK2, anchor="mm")

    # ================= 面板 2：解法 A（推荐） =================
    x0 = px[1]
    panel(d, x0, py, x0 + pw, py + ph,
          "② 解法 A（推荐）：板抬到轮子上面", "不切钢板；装好马达后板面自然就是这个高度",
          edge=GRN)
    g = py + ph - 60
    cx = x0 + pw / 2
    line(d, (x0 + 30, g), (x0 + pw - 30, g), INK, 3)
    cy = wheel(d, cx, g)
    plate(d, x0 + 50, x0 + pw - 50, 62.5, g)
    # 马达（吊在板下；侧视图里和轮子重叠，用浅色区分）
    my0 = g - 41.0 * SCALE
    d.rectangle([cx - 30, my0, cx + 26, g - 19.0 * SCALE], fill=(196, 200, 206),
                outline=(120, 124, 130))
    # 吊装角码
    d.rectangle([cx + 15, g - 62.5 * SCALE, cx + 24, my0], fill=(120, 124, 130))
    # 甲板 + 立柱 + 增高座（在板右侧竖着画一段，说明高度怎么叠上去）
    sx = x0 + pw - 96
    d.rectangle([sx, g - 64.0 * SCALE, sx + 34, g - 62.5 * SCALE], fill=METAL)
    d.rectangle([sx + 12, g - 84.5 * SCALE, sx + 22, g - 62.5 * SCALE],
                fill=(185, 179, 166))                       # 立柱 22
    d.rectangle([sx - 4, g - 88.5 * SCALE, sx + 38, g - 84.5 * SCALE],
                fill=(214, 208, 196))                       # 甲板 4
    d.rectangle([sx + 2, g - 116.0 * SCALE, sx + 32, g - 88.5 * SCALE], fill=ORA)
    y136 = g - 136.0 * SCALE
    line(d, (x0 + 250, y136), (x0 + pw - 20, y136), BLU, 2, dash=8)
    wlab = textw(d, "肩高 136mm（和以前一样）", 19)
    d.rectangle([x0 + pw - 28 - wlab, y136 + 6, x0 + pw - 20,
                 y136 + 30], fill=(255, 255, 255))
    txt(d, (x0 + pw - 26, y136 + 26), "肩高 136mm（和以前一样）", 19, BLU,
        anchor="ra")
    dim_z(d, x0 + 78, g, g - 62.5 * SCALE, "板面 62.5", GRN)
    txt(d, (sx + 17, g - 102.0 * SCALE), "增高座\n切到 27.5", 18, (150, 70, 20),
        anchor="mm")
    txt(d, (cx, g - 62.5 * SCALE - 20), "金属底板（欣薇 255×150×1.5）", 19, INK,
        anchor="mb")
    txt(d, (cx - 40, g - 46.0 * SCALE), "TT 马达吊在板下", 19, INK2)
    txt(d, (x0 + pw - 24, g - 6), "轮子完全在板下方 → 零干涉", 19, GRN,
        anchor="rb")

    # ================= 面板 3：解法 B =================
    x0 = px[2]
    panel(d, x0, py, x0 + pw, py + ph,
          "③ 解法 B：板不动，四个轮位开缺口", "板面维持 30mm，夹层 22mm 不变；代价是切板（不可逆）",
          edge=ORA)
    g = py + ph - 60
    cx = x0 + pw / 2
    line(d, (x0 + 30, g), (x0 + pw - 30, g), INK, 3)
    cy = wheel(d, cx, g)
    # 带缺口的板：分段画
    gap = 70 * SCALE / 2          # 缺口要 ≥ 轮宽 60mm，取 70mm
    plate(d, x0 + 50, cx - gap, 30.0, g)
    plate(d, cx + gap, x0 + pw - 50, 30.0, g)
    for xx in (cx - gap, cx + gap):                 # 切口断面
        d.line([(xx, g - 30.0 * SCALE), (xx, g - 1.5 * SCALE)], fill=(96, 100, 106),
               width=3)
    txt(d, (cx, cy - 108), "这四个位置剪掉一块", 20, ORA, anchor="mb")
    line(d, (cx, cy - 100), (cx, cy - 62), ORA, 2, dash=7)
    txt(d, (cx, g - 8), "缺口 ≈ 70 × 62（比轮子大 5mm）", 19, INK2, anchor="mb")
    txt(d, (x0 + 24, py + ph - 38),
        "1.5mm 钢板，铁皮剪/角磨机 10 分钟；好处是重心低、麦轮抓地好。",
        19, ORA)
    txt(d, (cx, py + 330), "重心比解法 A 低 32mm，急停/急转更不容易打滑；\n缺点是钢板开缺口不可逆。",
        20, INK2, anchor="mm")

    # ================= 底部对照表 =================
    ty = py + ph + 52
    txt(d, (60, ty - 40), "按你实测的轮径改一个数就行（增高座切短多少看这张表）", 24, INK)
    cols = [60, 400, 700, 940, 1240]
    rows = [
        ("麦轮外径（实测）", "板面离地", "增高座切到", "整车最高（臂收起）", "代价"),
        ("Φ60（现在的设计值）", "62.5 mm", "27.5 mm", "≈155 mm", "无（切一件打印件）"),
        ("Φ50", "51.5 mm", "38.5 mm", "≈144 mm", "无"),
        ("Φ40", "41.5 mm", "48.5 mm", "≈134 mm", "无"),
        ("板要保持 30mm 不动", "30 mm", "60 mm 不用切", "≈150 mm", "必须给板开缺口（解法 B）"),
    ]
    rh = 46
    for i, r in enumerate(rows):
        y = ty + i * rh
        bgc = (235, 238, 242) if i == 0 else ((244, 248, 245) if i else (255, 255, 255))
        d.rectangle([cols[0], y, cols[-1], y + rh - 4], fill=bgc,
                    outline=(210, 214, 220))
        for j, s in enumerate(r):
            txt(d, (cols[j] + 16, y + rh / 2 - 2), s, 20 if i else 21,
                INK if i else INK2, anchor="lm")
    txt(d, (60, ty + len(rows) * rh + 14),
        "公式：增高座实切高度 = 90 − 板面离地。切的时候从底脚开始切，别碰上面的舵机安装孔；"
        "切歪 1~2mm 不影响（臂的容差有 ±5mm）。", 20, INK2)
    txt(d, (60, ty + len(rows) * rh + 46),
        "本轮结论：图纸按 Φ60 画（板面 62.5 / 增高座 27.5）。你量一下麦轮外径，"
        "对不上就按表里的数切，或者告诉我我重新出图。", 20, INK)

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    im.save(OUT)
    print("saved", OUT, im.size)


if __name__ == "__main__":
    main()
