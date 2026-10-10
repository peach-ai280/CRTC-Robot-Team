# -*- coding: utf-8 -*-
"""
check_arm_clearance.py —— 机械臂「收起姿态 / 取块姿态」的干涉及限高验算
=========================================================================
【为什么要有这个工具】
  机械臂的模型是靠 gen_parts.py / render_asm.py 两个脚本「摆」出来的，
  但脚本不会告诉你**两个零件有没有撞上**、**有没有超尺寸红线**。
  10-10 就是靠它发现了一个致命问题：
    按「臂朝正后方水平平躺」的收起姿态，爪子的两根手指会**扎进储仓里**
    （手指 x∈[-36,-30] 与 z∈[113.5,147.5]，正好压在储仓前排方块的上方 8mm 里）。
  → 储仓必须整仓前移，且最前面一列的中两排留空当「爪子停靠位」。

【怎么用】
  cd <仓库根>
  D:\\WorkBuddy\\tools\\python\\python.exe tools/check_arm_clearance.py
  （纯 numpy，不需要 shapely / pyrender）

⚠ 本文件里的常数必须和 mechanical/gen_parts.py 的 5.7 节**逐字一致**。
  改了 gen_parts 的 AB_* 常量，这里也要改，然后重跑。
"""
import math

# ---------------------------------------------------------------- 臂的几何
AB_DECK_TOP = 88.5      # 甲板面离地
AB_AX = 106.0           # 臂座回转轴在车长方向的位置（原点 = 金属底板中心）
AB_A_HOLE_DX = 18.0     # 主动轴 A 相对回转轴的 x 偏移
AB_BASE_H = 50.0        # 臂座高
AB_BASE_L = 34.0        # 臂座长（x）
AB_BASE_W = 74.0        # 臂座宽（y）
AB_TT_D = 56.0          # 回转盘直径
AB_TT_T = 8.0           # 回转盘厚
AB_SERVO_DZ = 18.0      # MG995 轴心离盘面
AB_PITCH = 24.0         # 主动轴→从动轴
AB_LINK_L = 105.0       # 杆长
AB_LINK_W = 16.0
AB_MAST_H = 50.0
AB_PLATE_T = 8.0
AB_Y_MAST = 12.0        # 竖板 y ∈ [12,20]
AB_Y_LINK = 20.0        # 杆 / 肘座 y ∈ [20,28]

AB_AZ = AB_DECK_TOP + AB_BASE_H + AB_TT_T + AB_SERVO_DZ     # 164.5
AB_AX_HOLE = AB_AX + AB_A_HOLE_DX                           # 124.0
AB_PLATE_Z = AB_DECK_TOP + AB_BASE_H + AB_TT_T              # 146.5

# 平行爪（24h/24f/24g/24i 齿轮版）
CLAW_BASE_L, CLAW_BASE_W, CLAW_BASE_T = 74.0, 56.0, 4.0
CLAW_FIN_T, CLAW_FIN_W, CLAW_FIN_H = 6.0, 36.0, 34.0        # 厚(x) × 长(y) × 高(z)
CLAW_OFF_CLOSED, CLAW_OFF_OPEN = 3.0, 12.4                  # 齿条本体偏移
CLAW_GAP = AB_PITCH / 2.0                                   # 肘座中心 = A + 6 + L·d

# ---------------------------------------------------------------- 车体
PLATE_TOP = 62.5
PLATE_L, PLATE_W = 255.0, 150.0
DECK_TOP = 88.5
DECK_X = (-118.0, 118.0)                                    # 甲板 x 范围
DECK_W = 162.0
SCOOP_X = (124.5, 152.5)                                    # 收集铲
SCOOP_TOP = 41.0
CAR_REAR = -130.0                                           # 编号牌后缘

# ---------------------------------------------------------------- 储仓
BIN_T = 3.0
BIN_H = 52.0
CUBE = 30.0
PITCH_X, PITCH_Y = 31.0, 31.0


def elbow(pit_deg, yaw_deg=180.0):
    """返回 (A, Ap, B, Bp, E)，世界坐标"""
    d = (math.cos(math.radians(pit_deg)), math.sin(math.radians(pit_deg)))
    def rot(p):
        if abs(yaw_deg) < 1e-9:
            return p
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        dx, dy = p[0] - AB_AX, p[1]
        return (AB_AX + dx * c - dy * s, dx * s + dy * c)
    A = rot((AB_AX_HOLE, 0.0))
    Ap = rot((AB_AX_HOLE, 0.0))
    ex = A[0] + AB_LINK_L * d[0]
    ey = A[1] + AB_LINK_L * d[1]
    return A, ex, ey, AB_AZ, AB_AZ + AB_PITCH


def _yaw_box(bx, yaw_deg):
    """把一个包围盒绕（AB_AX, 0）竖轴转 yaw（取旋转后 4 个角的 AABB）"""
    (x0, x1), (y0, y1), (z0_, z1_) = bx
    if abs(yaw_deg) < 1e-9:
        return bx
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    nx, ny = [], []
    for px in (x0, x1):
        for py in (y0, y1):
            dx, dy = px - AB_AX, py
            nx.append(AB_AX + dx * c - dy * s)
            ny.append(dx * s + dy * c)
    return (min(nx), max(nx)), (min(ny), max(ny)), (z0_, z1_)


def claw_boxes(pit_deg, yaw_deg=180.0, open_state=False):
    """爪子的四个包围盒（世界坐标）：[底板, 爪舵机, 手指左, 手指右]

    做法：先在 yaw = 0 的坐标系里把爪搭好（挂在肘座正下方），
    再整组绕回转轴转 yaw —— 和 render_asm.arm_items() 的摆法完全一致。
    """
    ez = AB_AZ + AB_PITCH / 2.0 + AB_LINK_L * math.sin(math.radians(pit_deg))
    ex0 = AB_AX_HOLE + AB_LINK_L * math.cos(math.radians(pit_deg))   # yaw=0 时的肘座中心 x
    z0 = ez - 21.0                                                   # 爪底板顶面
    off = CLAW_OFF_OPEN if open_state else CLAW_OFF_CLOSED
    boxes = [((ex0 - CLAW_BASE_L / 2, ex0 + CLAW_BASE_L / 2),
              (-CLAW_BASE_W / 2, CLAW_BASE_W / 2), (z0 - CLAW_BASE_T, z0)),
             ((ex0 - 9.5, ex0 + 13.5), (-6.5, 6.5), (z0 + 4.0, z0 + 28.0))]
    for s in (-1, 1):
        cx = ex0 + s * (off + 13.0)
        boxes.append(((cx - CLAW_FIN_T / 2, cx + CLAW_FIN_T / 2),
                      (-CLAW_FIN_W / 2, CLAW_FIN_W / 2),
                      (z0 - CLAW_FIN_H - 8.0, z0 - 8.0)))
    out = [_yaw_box(b, yaw_deg) for b in boxes]
    eccx = (out[0][0][0] + out[0][0][1]) / 2.0
    return out, ez, z0, eccx


def overlap(a, b):
    """两个包围盒的交集尺寸，返回 (dx, dy, dz)；任一为 0 表示不相交"""
    return (max(0.0, min(a[0][1], b[0][1]) - max(a[0][0], b[0][0])),
            max(0.0, min(a[1][1], b[1][1]) - max(a[1][0], b[1][0])),
            max(0.0, min(a[2][1], b[2][1]) - max(a[2][0], b[2][0])))


def hits(a, b):
    o = overlap(a, b)
    return all(v > 1e-6 for v in o), o


def cube_slots(bin_x0, bin_w, ncol, nrow, skip=()):
    """储仓里的方块位（左上角为第 0 列/第 0 排；skip=((col,row),...) 留空）"""
    out = []
    ix0 = bin_x0 + BIN_T
    iy0 = -bin_w / 2 + BIN_T
    for i in range(ncol):
        for j in range(nrow):
            if (i, j) in skip:
                continue
            cx = ix0 + PITCH_X / 2 + i * PITCH_X
            cy = iy0 + PITCH_Y / 2 + j * PITCH_Y
            out.append(((i, j),
                        ((cx - CUBE / 2, cx + CUBE / 2),
                         (cy - CUBE / 2, cy + CUBE / 2),
                         (DECK_TOP + BIN_T, DECK_TOP + BIN_T + CUBE))))
    return out


def report_bin(bin_x0, bin_w, ncol, nrow, skip=()):
    fb, ez, z0, eccx = claw_boxes(0.0, 180.0, False)
    fingers = fb[2:]
    print("=" * 78)
    print("储仓 x∈[%.0f, %.0f]  宽 %.0f   %d 列 × %d 排   留空 %s"
          % (bin_x0, bin_x0 + (ncol * PITCH_X - 1.0), bin_w, ncol, nrow, skip or "无"))
    print("  收起爪子: 肘座中心 x=%.1f z=%.1f ｜ 爪底顶面 z0=%.1f ｜ 手指 z∈[%.1f, %.1f]"
          % (eccx, ez, z0, z0 - 42.0, z0 - 8.0))
    for f in fingers:
        print("     手指 x∈[%.0f, %.0f]  y∈[%.0f, %.0f]"
              % (f[0][0], f[0][1], f[1][0], f[1][1]))
    bad = []
    for key, b in cube_slots(bin_x0, bin_w, ncol, nrow, skip):
        for k, f in enumerate(fingers):
            h, o = hits(b, f)
            if h:
                bad.append((key, k, o))
    if bad:
        print("  ✗ 撞了！%d 个方块位与爪子干涉：" % len(bad))
        for key, k, o in bad:
            print("       位%s  ← 手指%d  重叠量 x=%.1f y=%.1f z=%.1f" % (key, k, o[0], o[1], o[2]))
    else:
        print("  ✓ 全部方块位与爪子无干涉")
    # 爪底板 / 爪舵机 vs 储仓墙
    for name, b in (("爪底板", fb[0]), ("爪舵机", fb[1])):
        wall = ((bin_x0, bin_x0 + (ncol * PITCH_X - 1.0)),
                (-bin_w / 2, bin_w / 2), (DECK_TOP, DECK_TOP + BIN_H))
        h, o = hits(b, wall)
        print("  %s vs 储仓外壳: %s%s" % (name, "✗ 干涉" if h else "✓ 不碰",
              ("  重叠 x=%.1f y=%.1f z=%.1f" % o) if h else ""))
    return len(bad)


def bbox_stow():
    """收起姿态全车包围盒（只算影响 290/195/210 的件）"""
    fb, ez, z0, eccx = claw_boxes(0.0, 180.0, False)
    xs = [CAR_REAR, SCOOP_X[1], DECK_X[1]]
    for b in fb:
        xs += [b[0][0], b[0][1]]
    # 肩架竖板（随 yaw 转）
    c, s = math.cos(math.radians(180.0)), math.sin(math.radians(180.0))
    mxs = []
    for lx in (AB_AX - 10.0, AB_AX + 34.0):
        for ly in (0.0, AB_MAST_H):
            dx = lx - AB_AX
            mxs.append(AB_AX + dx * c)
    xs += [min(mxs), max(mxs)]
    # 臂座
    xs += [AB_AX - AB_BASE_L / 2, AB_AX + AB_BASE_L / 2]
    zmax = max(AB_PLATE_Z + AB_MAST_H,             # 竖板顶 196.5
               ez + 17.0,                          # 肘座上缘
               z0 + 28.0)                          # 爪舵机顶
    zmin = min(b[2][0] for b in fb)
    print("=" * 78)
    print("收起姿态（yaw 180°，pit 0°）")
    print("  全长 %.1f mm（红线 290）  最高 %.1f mm（红线 210）  爪最低点 %.1f mm"
          % (max(xs) - min(xs), zmax, zmin))
    print("  x 范围 [%.1f, %.1f]   z 范围 [%.1f, %.1f]" % (min(xs), max(xs), zmin, zmax))
    return max(xs) - min(xs), zmax


def phi_window(cube_lo, cube_hi):
    """要让 30mm 的方块**整块**落进 34mm 的夹持缝里，φ 的可行区间（可能为空）"""
    ez_lo = cube_hi + 29.0      # E.z ≥ cube_hi + 29（夹持缝上缘要盖住方块顶）
    ez_hi = cube_lo + 63.0      # E.z ≤ cube_lo + 63（夹持缝下缘要低于方块底）
    base = AB_AZ + 12.0         # 176.5
    lo = (ez_lo - base) / AB_LINK_L
    hi = (ez_hi - base) / AB_LINK_L
    if lo > 1.0 or hi < -1.0 or lo > hi:
        return None
    return (math.degrees(math.asin(max(-1.0, lo))),
            math.degrees(math.asin(min(1.0, hi))))


def reach_table():
    print("=" * 78)
    print("取块能力（φ = 俯仰角，0° 水平朝前，+ 抬头；固件把 φ 锁在 −50° ~ +90°）")
    print("  肘座中心 z = %.1f + 105·sinφ    夹持缝 z = [肘座中心 z − 63, 肘座中心 z − 29]"
          % (AB_AZ + 12.0))
    print()
    print("  %-24s %-12s %-24s %s" % ("场景", "方块标高", "φ（整块进缝）", "该 φ 下夹持缝"))
    rows = [
        ("地面白块（地板 0）", 0.0, 30.0, None),
        ("台阶面白块", 150.0, 180.0, None),
        ("中央高台面白块", 135.0, 165.0, None),
        ("焦点黄块", 70.0, 100.0, None),
        ("物资架黄块", 235.0, 265.0, "PARTIAL"),
        ("洞窟黄块", 0.0, 50.0, "CAVE"),
    ]
    for name, lo, hi, flag in rows:
        if flag == "CAVE":
            print("  %-24s %-12s %s" % (name, "%.0f ~ %.0f" % (lo, hi),
                                        "爪外宽 74 > 洞宽 70 → 交给洞窟探杆 17_"))
            continue
        w = phi_window(lo, hi)
        if flag == "PARTIAL":
            ez = AB_AZ + 12.0 + AB_LINK_L      # φ=90 时的最高肘座
            g0, g1 = ez - 63.0, ez - 29.0
            ov = min(g1, hi) - max(g0, lo)
            print("  %-24s %-12s %s" % (name, "%.0f ~ %.0f" % (lo, hi),
                  "✗ 无理数解：φ=90° 时夹持缝 [%.1f, %.1f]，与方块重叠 %.1f mm（只能夹下半截）"
                  % (g0, g1, ov)))
            continue
        if w is None:
            print("  %-24s %-12s ✗ 不可能（超出臂的行程）" % (name, "%.0f ~ %.0f" % (lo, hi)))
            continue
        a, b = max(w[0], -50.0), min(w[1], 90.0)
        if a > b:
            print("  %-24s %-12s ✗ 被 −50° 下限锁死" % (name, "%.0f ~ %.0f" % (lo, hi)))
            continue
        base = AB_AZ + 12.0
        g0 = base + AB_LINK_L * math.sin(math.radians(a)) - 63.0
        g1 = base + AB_LINK_L * math.sin(math.radians(a)) - 29.0
        print("  %-24s %-12s φ = %5.1f° ~ %5.1f°          [%.0f, %.0f]"
              % (name, "%.0f ~ %.0f" % (lo, hi), a, b, g0, g1))
    print()
    print("  ※ 地面白块：φ 要到 −60° 以下才够得着，而主臂在 −50° 以下就扫到收集铲了，")
    print("     所以地面的块一律交给收集铲（分工明确，不许用臂去够地面）。")
    print("  ※ 物资架黄块：方块中心 250，比臂的最高夹持缝还高 12.5mm，只能夹住它下半截；")
    print("     把方块沿 φ14 圆柱轴向外拨出来之后，再夹住下半截拖出来，是可行的。")
    print("     ★ 若现场实测拿不下来 —— 按队长意见**直接放弃**，不要为它超时。")


if __name__ == "__main__":
    print()
    bw, bh = bbox_stow()
    print()
    reach_table()
    print()
    print("### 旧方案：储仓 x∈[-118,-18]（长边 100 沿 x）")
    n_old = report_bin(-118.0, 136.0, 3, 4)
    print()
    print("### 新方案：储仓前移到 x∈[-32,68]，最前一列中两排留空当「爪子停靠位」")
    n_new = report_bin(-32.0, 136.0, 3, 4, skip=((0, 1), (0, 2)))
    print()
    print("结论：旧位置有 %d 个方块位被爪子压住；新位置 %d 个。" % (n_old, n_new))
