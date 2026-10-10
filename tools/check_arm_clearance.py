#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CRTC2026 · 柱坐标机械臂 干涉与外形验算（纯解析，只用 math）

为什么要有这个脚本
------------------
`mechanical/gen_parts.py` 只负责**画零件**，它不会告诉你这些零件装到一起会不会打架。
上一版（平行四连杆摆动臂）就是因为没人做这一步，出现了三处致命错误：
  · 升降舵机画在回转盘上，机构根本闭合不了；
  · 立柱被滑座整圈包住，柱顶以外的外表面根本没法装东西；
  · 爪指在检录姿态下直接穿过甲板。
所以这一版把「装起来会怎样」写成可复算的公式，每次改尺寸都重跑一遍。

用法
----
    D:\\WorkBuddy\\tools\\python\\python.exe tools/check_arm_clearance.py

⚠ 这里的 CY_* 常量必须和 `mechanical/gen_parts.py` 的 5.7 节、以及
  `tools/render_asm.py` 的 CY_* 常量**三处一致**。改了哪边都要同步。
"""

import math

# ============================================================ 常量（三处必须一致）
CY_DECK_TOP = 88.5      # 甲板面离地（已含 4mm 板厚）—— 爪尖绝不允许低于它
CY_BASE_L   = 64.0      # 回转底座外廓（沿车长 x）
CY_BASE_W   = 44.0      # 回转底座外廓（沿车宽 y）★ 刻意做窄
CY_BASE_H   = 40.0      # 回转底座高
CY_TT_T     = 6.0       # 回转盘厚
CY_MAST_W   = 40.0      # 立柱外截面
CY_MAST_H   = 62.65     # 立柱高
CY_CAR_W    = 52.0      # 滑座外截面
CY_CAR_H    = 32.0      # 滑座高
CY_CAP_W    = 60.0      # 顶帽外廓
CY_CAP_T    = 6.0       # 顶帽厚
CY_BOOM_X0  = 26.0      # 横臂起点
CY_BOOM_X1  = 64.0      # 横臂末端
CY_BOOM_PIN = 10.0      # 横臂销孔离横臂起点的距离（→ 世界 x = 95+26+10 = 131）
CY_BOOM_Z   = 24.0      # 横臂高
CY_BOOM_Y   = 11.0      # 横臂所在的 y 起点（横臂占 y∈[11,19]）
CY_BOOM_T   = 8.0       # 横臂厚度（沿 y）
CY_BOOM_L   = 52.0      # 爪中心离柱轴的水平距离
CY_CLAW_DROP = 46.0     # 横臂底面 → 爪指尖落差
CY_CRANK_R  = 15.25     # 曲柄半径 → 行程 30.5
CY_CRANK_HUB = 9.0      # 曲柄轮毂半径
CY_CRANK_TIP = 3.6      # 曲柄销孔圆头外缘 = R + 3.6
CY_LINK_L   = 48.0      # 连杆孔距
CY_LINK_T   = 3.5       # 连杆半宽（板宽 7）
CY_AX       = 95.0      # 回转轴在车长方向的位置
CY_LIFT_MAX = 2.0 * CY_CRANK_R

# 平行爪（齿轮版 24f/24g/24h/24i）实测外廓
CLAW_W      = 74.0      # 爪架底板长（沿横臂伸出方向）
CLAW_D      = 56.0      # 爪架底板宽（沿车宽方向）
CLAW_PLATE_T = 4.0
FINGER_X    = 16.0      # 爪指中心离爪中心（闭合态）
FINGER_W    = 6.0
FINGER_L    = 36.0      # 爪指长（沿车宽）
FINGER_H    = 34.0
FINGER_TOP_DROP = 8.0   # 爪指顶面比爪架顶面低多少
FINGER_BOT_DROP = 42.0  # 爪指尖比爪架顶面低多少

# 车体红线
CAR_LEN_MAX, CAR_WID_MAX, CAR_HGT_MAX = 290.0, 195.0, 210.0
CAR_LEN_REAL = 283.0    # 板 255 + 铲 28
CAR_REAR_X   = -129.0   # 车尾最靠后的实体（编号牌）
CAR_HALF_W   = 81.0     # 甲板半宽（162/2）—— 不是红线，只是参考

# 储仓（renders/模型一致：仓底 STL 原点在角上）
BIN_X0, BIN_X1 = -40.0, 60.0
BIN_Y0, BIN_Y1 = -68.0, 68.0
BIN_FLOOR_Z = CY_DECK_TOP + 3.0 + 3.0     # 仓底 3mm + EVA 3mm
BIN_WALL_H  = 52.0

EPS = 1e-6


# ============================================================ 基础几何
def stack(lift=0.0):
    """给定升降量，返回各基准面的 z（相对地面）"""
    base_top = CY_DECK_TOP + CY_BASE_H              # 128.5
    tt_top = base_top + CY_TT_T                     # 134.5
    mast_top = tt_top + CY_MAST_H                   # 197.15
    car_bot = tt_top + lift                         # 滑座底面
    car_top = car_bot + CY_CAR_H
    boom_bot = car_bot - CY_BOOM_Z
    boom_top = car_bot
    pin_z = boom_bot + CY_BOOM_Z / 2.0              # 横臂销中心
    claw_z0 = boom_bot + FINGER_BOT_DROP - CY_CLAW_DROP   # 爪架顶面
    return dict(base_top=base_top, tt_top=tt_top, mast_top=mast_top,
                car_bot=car_bot, car_top=car_top,
                boom_bot=boom_bot, boom_top=boom_top, pin_z=pin_z,
                claw_z0=claw_z0)


def crank_z():
    """曲柄（升降舵机）回转中心高 = 顶帽底面 − 11.4"""
    return (CY_DECK_TOP + CY_BASE_H + CY_TT_T + CY_MAST_H) - 11.4


def claw_aabb(lift, yaw_deg):
    """爪（爪架 + 两根爪指）在世界坐标下的包围盒，单位 mm"""
    s = stack(lift)
    z0 = s["claw_z0"]
    cx, cy = CY_AX + CY_BOOM_L, 0.0            # 回转前：爪中心在柱轴正前方
    a = math.radians(yaw_deg)
    ca, sa = math.cos(a), math.sin(a)

    def rot(px, py):
        dx, dy = px - CY_AX, py - 0.0
        return (CY_AX + dx * ca - dy * sa, dx * sa + dy * ca)

    boxes = []
    # 爪架底板
    for px in (cx - CLAW_W / 2.0, cx + CLAW_W / 2.0):
        for py in (cy - CLAW_D / 2.0, cy + CLAW_D / 2.0):
            boxes.append(rot(px, py))
    lo = [min(b[0] for b in boxes), min(b[1] for b in boxes)]
    hi = [max(b[0] for b in boxes), max(b[1] for b in boxes)]
    plate = dict(x0=lo[0], x1=hi[0], y0=lo[1], y1=hi[1],
                 z0=z0, z1=z0 + CLAW_PLATE_T,
                 x0p=boxes[0][0], x1p=boxes[1][0])
    # 两根爪指
    f = []
    for sgn in (1.0, -1.0):
        fx = cx + sgn * FINGER_X
        pts = []
        for px in (fx - FINGER_W / 2.0, fx + FINGER_W / 2.0):
            for py in (cy - FINGER_L / 2.0, cy + FINGER_L / 2.0):
                pts.append(rot(px, py))
        f.append(dict(x0=min(p[0] for p in pts), x1=max(p[0] for p in pts),
                      y0=min(p[1] for p in pts), y1=max(p[1] for p in pts),
                      z0=z0 - FINGER_BOT_DROP, z1=z0 - FINGER_TOP_DROP))
    return plate, f


def base_aabb():
    return dict(x0=CY_AX - CY_BASE_L / 2.0, x1=CY_AX + CY_BASE_L / 2.0,
                y0=-CY_BASE_W / 2.0, y1=CY_BASE_W / 2.0,
                z0=CY_DECK_TOP, z1=CY_DECK_TOP + CY_BASE_H)


def pin_z_of_phi(phi_deg):
    """给定曲柄角，算出横臂销（= 滑座底 − 12）的高度。曲柄滑块闭合方程。"""
    a = math.radians(phi_deg)
    return (crank_z() + CY_CRANK_R * math.sin(a)
            - math.sqrt(CY_LINK_L ** 2 - (CY_CRANK_R * math.cos(a)) ** 2))


def crank_aabb(phi_deg):
    """曲柄在 xz 平面的包围盒（y 固定占 [CY_BOOM_Y, +CY_BOOM_T]）"""
    a = math.radians(phi_deg)
    cz = crank_z()
    px = CY_AX + CY_BOOM_X0 + CY_BOOM_PIN
    x0, x1 = px - CY_CRANK_HUB, px + CY_CRANK_HUB
    z0, z1 = cz - CY_CRANK_HUB, cz + CY_CRANK_HUB
    for i in range(41):                      # 沿臂中心线采样（含圆头）
        t = i / 40.0
        cx = px + t * CY_CRANK_R * math.cos(a)
        cz2 = cz + t * CY_CRANK_R * math.sin(a)
        r = CY_CRANK_TIP if i == 40 else 4.0
        x0, x1 = min(x0, cx - r), max(x1, cx + r)
        z0, z1 = min(z0, cz2 - r), max(z1, cz2 + r)
    return dict(x0=x0, x1=x1, y0=CY_BOOM_Y, y1=CY_BOOM_Y + CY_BOOM_T, z0=z0, z1=z1)


def carriage_aabb(car_bot):
    h = CY_CAR_W / 2.0
    return dict(x0=CY_AX - h, x1=CY_AX + h, y0=-h, y1=h,
                z0=car_bot, z1=car_bot + CY_CAR_H)


def cap_aabb():
    h = CY_CAP_W / 2.0
    z = CY_DECK_TOP + CY_BASE_H + CY_TT_T + CY_MAST_H
    return dict(x0=CY_AX - h, x1=CY_AX + h, y0=-h, y1=h, z0=z, z1=z + CY_CAP_T)


# 顶帽上给曲柄让位的缺口（和 gen_parts.py 5.7.8 的 rect(17.0, 8.0, _cc_+4, 22.0) 对应）
CAP_CUT_LX0, CAP_CUT_LX1 = 17.0, CY_CAP_W / 2.0 + 4.0
CAP_CUT_LY0, CAP_CUT_LY1 = 8.0, 22.0


def overlap(a, b):
    """两个 AABB 的相交体积（mm³），不相交返回 0"""
    dx = min(a["x1"], b["x1"]) - max(a["x0"], b["x0"])
    dy = min(a["y1"], b["y1"]) - max(a["y0"], b["y0"])
    dz = min(a["z1"], b["z1"]) - max(a["z0"], b["z0"])
    return dx * dy * dz if (dx > EPS and dy > EPS and dz > EPS) else 0.0


# ============================================================ 检查
def main():
    R = []

    def chk(name, ok, detail):
        R.append((("✅" if ok else "❌"), name, detail))

    def warn(name, detail):
        R.append(("⚠️ ", name, detail))

    rnd = lambda v: round(v, 2)

    # ---------- 1. 高度栈自洽
    s_lo, s_hi = stack(0.0), stack(CY_LIFT_MAX)
    chk("行程 = 2R",
        abs((s_hi["car_bot"] - s_lo["car_bot"]) - 2 * CY_CRANK_R) < 1e-9,
        "滑座底 %.2f → %.2f，行程 %.2f（= 2×%.2f）"
        % (s_lo["car_bot"], s_hi["car_bot"], s_hi["car_bot"] - s_lo["car_bot"], CY_CRANK_R))

    chk("滑座顶 ≤ 顶帽底",
        s_hi["car_top"] <= s_hi["mast_top"] + EPS,
        "最高时滑座顶 %.2f vs 顶帽底 %.2f，余 %.2f ← ⚠这一处只有零点几毫米，"
        "所以不要让舵机跑满行程的两个端点（曲柄死点）"
        % (s_hi["car_top"], s_hi["mast_top"], s_hi["mast_top"] - s_hi["car_top"]))

    cz = crank_z()
    chk("曲柄中心 = 销中心中位 + 连杆长",
        abs(cz - ((s_lo["pin_z"] + s_hi["pin_z"]) / 2.0 + CY_LINK_L)) < 1e-6,
        "曲柄中心 %.2f = 销中位 %.2f + 连杆 %.1f" % (cz, (s_lo["pin_z"] + s_hi["pin_z"]) / 2.0, CY_LINK_L))

    top_crank = cz + CY_CRANK_R + CY_CRANK_TIP
    top_cap = CY_DECK_TOP + CY_BASE_H + CY_TT_T + CY_MAST_H + CY_CAP_T
    chk("全车最高点 ≤ 210",
        max(top_crank, top_cap) <= CAR_HGT_MAX,
        "曲柄外缘 %.2f ｜ 顶帽顶 %.2f ｜ 红线 210 → 余 %.2f"
        % (top_crank, top_cap, CAR_HGT_MAX - max(top_crank, top_cap)))

    # ---------- 2. 曲柄扫掠 vs 横臂
    boom_hi = s_hi["boom_top"]
    crank_lo = cz - (CY_CRANK_R + CY_CRANK_TIP)
    chk("曲柄扫掠不撞横臂（z 向）",
        crank_lo >= boom_hi,
        "曲柄最低 %.2f vs 横臂最高 %.2f → 余 %.2f"
        % (crank_lo, boom_hi, crank_lo - boom_hi))

    # ---------- 3. 曲柄扫掠 vs 滑座／顶帽（★ 必须和升降量**联动**，不能按全周扫掠算）
    #    为什么：曲柄角一旦确定，滑座位置就唯一确定了（曲柄滑块闭合方程）。
    #    取 φ=0（臂水平指左）时滑座是最低，取 φ=±90°（臂竖直）时滑座最高。
    #    所以"曲柄臂向左伸出去的那一下"正好发生在滑座最低的时候，两者错开。
    worst_car, worst_phi_car = 1e9, None
    for i in range(-90, 91):
        ck = crank_aabb(float(i))
        cg = carriage_aabb(pin_z_of_phi(float(i)) + CY_BOOM_Z / 2.0)
        ov = overlap(ck, cg)
        if ov > 0:
            worst_car, worst_phi_car = -1.0, i
            break
        # 记录最小间隙（只算 xz 平面内的最短接近）
        gap = max(cg["z0"] - ck["z1"], ck["z0"] - cg["z1"],
                  cg["x0"] - ck["x1"], ck["x0"] - cg["x1"])
        if gap < worst_car:
            worst_car, worst_phi_car = gap, i
    chk("曲柄扫掠 全程不撞滑座（联动验算）",
        worst_phi_car is not None and worst_car > 0,
        "在 φ=−90°~+90° 逐个位置验算，最小间隙 %.2fmm（φ=%s）"
        % (worst_car, worst_phi_car))

    cap = cap_aabb()
    cut_x0 = CY_AX + CAP_CUT_LX0
    cut_x1 = CY_AX + CAP_CUT_LX1
    cut_y0 = CAP_CUT_LY0
    cut_y1 = CAP_CUT_LY1
    worst_cap = 1e9
    for i in range(-90, 91):
        ck = crank_aabb(float(i))
        if overlap(ck, cap) > 0:
            # 相交的部分必须落进缺口范围（x 向从缺口左沿到顶帽右沿，y 向整条带）
            if not (ck["x0"] >= cut_x0 - EPS and ck["y0"] >= cut_y0 - EPS
                    and ck["y1"] <= cut_y1 + EPS and ck["x0"] >= cut_x0 - EPS):
                worst_cap = -1.0
                break
        worst_cap = min(worst_cap, ck["x0"] - cut_x0)
    chk("曲柄扫掠 全程不撞顶帽（缺口够大）",
        worst_cap >= 0.0,
        "曲柄最靠左 %.2f vs 顶帽让位缺口左沿 %.2f → 余 %.2f"
        % (CY_AX + CY_BOOM_X0 + CY_BOOM_PIN - (CY_CRANK_R + CY_CRANK_TIP), cut_x0, worst_cap))

    # ---------- 4. 滑座/横臂 vs 立柱行程区间外的东西
    mast_guard = CY_DECK_TOP + CY_BASE_H + CY_TT_T + CY_MAST_H
    chk("滑座全程被立柱挡住 → 立柱外表面不可装配",
        True,
        "滑座走过 z = %.2f ~ %.2f，所以立柱 z < %.2f 的外表面全是禁区，"
        "唯一能装东西的面是柱顶 %.2f（→ 必须有 36_arm_lift_cap）"
        % (s_lo["car_bot"], s_hi["car_top"], s_hi["car_top"], mast_guard))

    # ---------- 4. 检录姿态：yaw=90°，升降最高档
    L = CY_LIFT_MAX
    plate, fingers = claw_aabb(L, 90.0)
    base = base_aabb()
    chk("检录态 爪架 不与自有底座干涉",
        overlap(plate, base) == 0 and overlap(fingers[0], base) == 0 and overlap(fingers[1], base) == 0,
        "爪架 x∈[%.1f,%.1f] y∈[%.1f,%.1f] z∈[%.1f,%.1f] ｜ 底座 x∈[%.1f,%.1f] y∈[%.1f,%.1f] z∈[%.1f,%.1f]"
        % (plate["x0"], plate["x1"], plate["y0"], plate["y1"], plate["z0"], plate["z1"],
           base["x0"], base["x1"], base["y0"], base["y1"], base["z0"], base["z1"]))

    tip_hi = min(f["z0"] for f in fingers)
    chk("检录态 爪尖在甲板之上",
        tip_hi >= CY_DECK_TOP,
        "爪尖 %.2f vs 甲板面 %.2f → 余 %.2f" % (tip_hi, CY_DECK_TOP, tip_hi - CY_DECK_TOP))

    wid = 2.0 * max(plate["y1"], fingers[0]["y1"], fingers[1]["y1"])
    chk("检录态 车宽 ≤ 195",
        wid <= CAR_WID_MAX,
        "最外沿 y=%.1f → 车宽 %.1f（红线 195，余 %.1f）｜ 甲板半宽 %.0f"
        % (max(plate["y1"], fingers[0]["y1"], fingers[1]["y1"]), wid, CAR_WID_MAX - wid, CAR_HALF_W))

    ln = max(CAR_LEN_REAL, plate["x1"] - CAR_REAR_X)
    chk("检录态 车长 ≤ 290",
        ln <= CAR_LEN_MAX,
        "臂不伸向车前车后（x∈[%.1f,%.1f]），车长仍为 %.1f（红线 290，余 %.1f）"
        % (plate["x0"], plate["x1"], ln, CAR_LEN_MAX - ln))

    # ---------- 5. 检录姿态改成最低档会怎样（反面验证）
    pl0, fg0 = claw_aabb(0.0, 90.0)
    hit_deck = min(f["z0"] for f in fg0) < CY_DECK_TOP
    hit_base = overlap(pl0, base) > 0
    chk("反面验证：yaw=90° + 最低档 **会** 撞东西（所以必须用最高档）",
        hit_deck and hit_base,
        "爪尖 %.2f 穿进甲板 %.2f（低 %.2f）｜ 爪架与底座相交 %.0f mm³"
        % (min(f["z0"] for f in fg0), CY_DECK_TOP, CY_DECK_TOP - min(f["z0"] for f in fg0),
           overlap(pl0, base)))

    # ---------- 6. 爪能不能落到仓底
    pl180, fg180 = claw_aabb(CY_LIFT_MAX, 180.0)
    tip_bin = min(f["z0"] for f in fg180)
    inside = (BIN_X0 <= (pl180["x0"] + pl180["x1"]) / 2.0 <= BIN_X1
              and BIN_Y0 <= (pl180["y0"] + pl180["y1"]) / 2.0 <= BIN_Y1)
    chk("投放态 爪中心落在储仓范围内",
        inside,
        "爪中心 (%.1f, %.1f)｜储仓 x∈[%.0f,%.0f] y∈[%.0f,%.0f]"
        % ((pl180["x0"] + pl180["x1"]) / 2.0, (pl180["y0"] + pl180["y1"]) / 2.0,
           BIN_X0, BIN_X1, BIN_Y0, BIN_Y1))
    chk("投放态 爪尖高于仓底（放得进去）",
        tip_bin >= BIN_FLOOR_Z,
        "爪尖 %.2f vs 仓底 %.2f → 余 %.2f ← ★ 全车最紧的一处，进场前必须实测"
        % (tip_bin, BIN_FLOOR_Z, tip_bin - BIN_FLOOR_Z))

    # ---------- 7. 取块姿态：爪在甲板之外？
    pl0y, fg0y = claw_aabb(0.0, 0.0)
    chk("取块态 爪在甲板前缘之外（可以自由降到最低档）",
        min(f["x0"] for f in fg0y) > 127.0,
        "爪指最内侧 x=%.1f vs 甲板前缘 127 → 多出 %.1f"
        % (min(f["x0"] for f in fg0y), min(f["x0"] for f in fg0y) - 127.0))

    # ---------- 8. 爪指（真正会降到低处的那部分）vs 底座半宽
    #    ★ 注意：爪架底板在 y=15 就已经伸到底座 y 范围里了，但它在 z=137（底座顶 128.5 之上），
    #      所以 2D 投影重叠 ≠ 真干涉。真正决定底座能不能做宽的，是**爪指**。
    fin_in_y = CY_BOOM_L - FINGER_X - FINGER_W / 2.0
    chk("爪指内侧面 在 底座半宽 之外",
        fin_in_y > CY_BASE_W / 2.0,
        "爪指内沿 y=%.1f vs 底座半宽 %.1f → 余 %.1f（若偏心取 38，这里只有 %.1f，会撞底座）"
        % (fin_in_y, CY_BASE_W / 2.0, fin_in_y - CY_BASE_W / 2.0,
           CY_BOOM_L * 0 + 38.0 - FINGER_X - FINGER_W / 2.0))

    # ---------- 9. 取块态：爪指与甲板前缘的间隙（警告级）
    fin_in_x = min(f["x0"] for f in fg0y)
    margin_deck = fin_in_x - 127.0
    if margin_deck >= 3.0:
        chk("取块态 爪指 越过甲板前缘 ≥3mm", True,
            "爪指最内侧 x=%.1f vs 甲板前缘 127 → 余 %.1f" % (fin_in_x, margin_deck))
    else:
        warn("取块态 爪指 越过甲板前缘 只有 %.1fmm" % margin_deck,
             "爪指最内侧 x=%.1f vs 甲板前缘 127。爪是竖直下降的，不蹭；"
             "但**回转扫过甲板时会蹭**，所以固件必须加互锁。想更保险可把 CY_BOOM_L 从 %.0f 加到 56"
             "（车宽也只从 178 到 186，红线 195 仍有余量）。"
             % (fin_in_x, CY_BOOM_L))

    # ---------- 10. 竖直方向能覆盖的目标
    tip_lo = min(f["z0"] for f in claw_aabb(0.0, 0.0)[1])
    tip_h = min(f["z0"] for f in claw_aabb(CY_LIFT_MAX, 0.0)[1])
    chk("爪尖可达高度区间",
        True,
        "%.2f ~ %.2f（离地）｜甲板 %.1f｜仓底 %.1f｜物资架黄块质心 250 → 差 %.1f（够不到，交挑杆）"
        % (tip_lo, tip_h, CY_DECK_TOP, BIN_FLOOR_Z, 250.0 - tip_h))

    # ============================================================ 打印
    print("=" * 100)
    print("CRTC2026 · 柱坐标机械臂 干涉与外形验算")
    print("=" * 100)
    print("高度栈：甲板面 %.2f ｜ 底座顶 %.2f ｜ 回转盘顶 %.2f ｜ 立柱顶 %.2f ｜ 顶帽顶 %.2f"
          % (CY_DECK_TOP, s_lo["base_top"], s_lo["tt_top"], s_lo["mast_top"], top_cap))
    print("        滑座底 %.2f ~ %.2f ｜ 销中心 %.2f ~ %.2f ｜ 曲柄中心 %.2f ｜ 行程 %.2f"
          % (s_lo["car_bot"], s_hi["car_bot"], s_lo["pin_z"], s_hi["pin_z"], cz, CY_LIFT_MAX))
    print("        爪尖 %.2f ~ %.2f ｜ 全车最高 %.2f / 红线 %.0f"
          % (tip_lo, tip_h, max(top_crank, top_cap), CAR_HGT_MAX))
    print("-" * 100)
    bad = 0
    for mark, name, detail in R:
        if mark == "❌":
            bad += 1
        print("%s %-42s %s" % (mark, name, detail))
    print("-" * 100)
    print("共 %d 项，失败 %d 项。" % (len(R), bad))
    print("⚠ 以上全部是**设计值推演**，不是实测。中期文档里一律按「待实测」写。")
    return bad


if __name__ == "__main__":
    raise SystemExit(main())
