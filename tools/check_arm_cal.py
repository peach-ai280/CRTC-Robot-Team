#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_arm_cal.py —— 柱坐标机械臂「标定表」自检（纯标准库，哪个 python 都能跑）

为什么要有它：
  固件编 -Os 时，编译期常量条件会被彻底折掉，光看"编过了"证明不了标定表是对的。
  这个脚本**直接从 firmware/Core/Inc/arm_cal.h 读宏**（唯一出处），然后：
    ① 检查表本身有没有写反、写重、超限（单调性 / 端点一致性 / 与几何参数一致）
    ② 复算每个目标姿态实际会得到多少度（等于把 Arm_LiftDegFromMm 在 PC 上跑一遍）
    ③ 演练 Arm_SetYaw 的互锁判定，确认"低位回转"真的会被拦下

机械组改了尺寸之后，跑一次这个脚本就知道电控这边还对不对得上。
"""
import os
import re
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
CAL = os.path.join(ROOT, 'firmware', 'Core', 'Inc', 'arm_cal.h')

# ---------------------------------------------------------------- 读宏
def load_defines(path):
    d = {}
    pat = re.compile(r'^\s*#define\s+([A-Z0-9_]+)\s+([0-9.]+)f?\s*(?:/\*.*)?$')
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            m = pat.match(line)
            if m:
                d[m.group(1)] = float(m.group(2))
    return d

# ---------------------------------------------------------------- 插值（与 arm.c 同逻辑）
def interp(x, xs, ys):
    if x <= xs[0]:
        return ys[0]
    if x >= xs[-1]:
        return ys[-1]
    for i in range(1, len(xs)):
        if x <= xs[i]:
            span = xs[i] - xs[i - 1]
            t = (x - xs[i - 1]) / span if span > 0 else 0.0
            return ys[i - 1] + t * (ys[i] - ys[i - 1])
    return ys[-1]

def yaw_crosses_sector(cur, tgt, lo, hi):
    """arm.c 的 Arm_SetYaw 互锁判据：路径 [min,max] 与 [lo,hi] 有交集"""
    a, b = (cur, tgt) if cur < tgt else (tgt, cur)
    return (b > lo) and (a < hi)

# ---------------------------------------------------------------- 主流程
def main():
    if not os.path.exists(CAL):
        print('找不到 %s' % CAL)
        return 1
    D = load_defines(CAL)
    fails, warns = [], []

    need = ['ARM_LIFT_N', 'ARM_LIFT_DEG_MIN', 'ARM_LIFT_DEG_MAX',
            'ARM_GEO_DECK_TOP', 'ARM_GEO_CLAW_TIP_LO', 'ARM_GEO_CLAW_TIP_HI',
            'ARM_GEO_STROKE', 'ARM_TIP_SAFE', 'ARM_YAW_SECTOR_LO', 'ARM_YAW_SECTOR_HI']
    for k in need:
        if k not in D:
            fails.append('arm_cal.h 缺少宏 %s' % k)
    if fails:
        print('\n'.join(fails))
        return 1

    n = int(D['ARM_LIFT_N'])
    tips = [D.get('ARM_CAL_TIP_%d' % i) for i in range(n)]
    degs = [D.get('ARM_CAL_DEG_%d' % i) for i in range(n)]
    if any(v is None for v in tips + degs):
        print('标定表缺项：ARM_CAL_TIP_x / ARM_CAL_DEG_x 必须 0..%d 都有' % (n - 1))
        return 1

    # ① 表自查
    for i in range(1, n):
        if tips[i] <= tips[i - 1]:
            fails.append('ARM_CAL_TIP_%d (%.2f) 必须大于 ARM_CAL_TIP_%d (%.2f)'
                         % (i, tips[i], i - 1, tips[i - 1]))
        if degs[i] <= degs[i - 1]:
            fails.append('ARM_CAL_DEG_%d (%.2f) 必须大于 ARM_CAL_DEG_%d (%.2f)'
                         % (i, degs[i], i - 1, degs[i - 1]))
    for i in range(n):
        if not (D['ARM_LIFT_DEG_MIN'] <= degs[i] <= D['ARM_LIFT_DEG_MAX']):
            fails.append('ARM_CAL_DEG_%d = %.1f 超出硬限位 [%.0f, %.0f]'
                         % (i, degs[i], D['ARM_LIFT_DEG_MIN'], D['ARM_LIFT_DEG_MAX']))
    if abs(tips[0] - D['ARM_GEO_CLAW_TIP_LO']) > 0.6:
        warns.append('ARM_CAL_TIP_0 (%.1f) 与几何爪尖最低 %.1f 差了 %.1fmm —— 确认是不是实测值'
                     % (tips[0], D['ARM_GEO_CLAW_TIP_LO'], tips[0] - D['ARM_GEO_CLAW_TIP_LO']))
    if abs(tips[-1] - D['ARM_GEO_CLAW_TIP_HI']) > 0.6:
        warns.append('ARM_CAL_TIP_%d (%.1f) 与几何爪尖最高 %.1f 差了 %.1fmm'
                     % (n - 1, tips[-1], D['ARM_GEO_CLAW_TIP_HI'], tips[-1] - D['ARM_GEO_CLAW_TIP_HI']))
    span = tips[-1] - tips[0]
    if abs(span - D['ARM_GEO_STROKE']) > 1.0:
        warns.append('爪尖行程实测 %.1fmm，几何设计 %.1fmm（差 %.1f）—— 若机械组改过行程，'
                     '记得同步 ARM_GEO_STROKE 和 docs' % (span, D['ARM_GEO_STROKE'], span - D['ARM_GEO_STROKE']))

    # ② 目标姿态复算
    targets = [('ARM_TIP_HOME', '收起/检录/回转安全/投放'),
               ('ARM_TIP_DROP', '投放（储仓）'),
               ('ARM_TIP_FOCUS', '焦点黄块'),
               ('ARM_TIP_PLATFORM', '中央高台面白块'),
               ('ARM_TIP_STEP', '台阶面白块')]
    print('=' * 74)
    print('CRTC2026 柱坐标机械臂 标定表自检')
    print('  来源: %s' % os.path.relpath(CAL, ROOT))
    print('=' * 74)
    print('升降标定表（%d 点，分段线性插值）' % n)
    print('  %-6s %-10s %s' % ('档位', '爪尖(mm)', '升降舵机(度)'))
    for i in range(n):
        print('  %-6d %-10.1f %.1f' % (i, tips[i], degs[i]))
    print()
    print('各目标姿态（爪尖 mm → 升降舵机度）')
    for key, name in targets:
        if key not in D:
            warns.append('arm_cal.h 没有 %s（目标姿态宏）' % key)
            continue
        tip = D[key]
        if tip < tips[0] - 0.01 or tip > tips[-1] + 0.01:
            fails.append('%s = %.1fmm 超出爪尖可达范围 [%.1f, %.1f]'
                         % (key, tip, tips[0], tips[-1]))
        print('  %-22s %-14s 爪尖 %6.1fmm → %.1f 度' % (name, key, tip, interp(tip, tips, degs)))
    print()

    # ③ 互锁演练
    lo, hi = D['ARM_YAW_SECTOR_LO'], D['ARM_YAW_SECTOR_HI']
    safe = D['ARM_TIP_SAFE']
    deck = D['ARM_GEO_DECK_TOP']
    print('回转互锁（危险扇区 %.0f~%.0f 度，安全爪尖 >= %.1fmm，甲板面 %.1fmm）'
          % (lo, hi, safe, deck))
    if safe <= deck:
        fails.append('ARM_TIP_SAFE (%.1f) 必须大于甲板面 (%.1f)，否则低位回转仍会插进甲板'
                     % (safe, deck))
    cases = [(0, 180, '取块位 → 收起位（必须拦）'),
             (180, 0, '收起位 → 取块位（必须拦）'),
             (0, 30, '正前小幅微调（不该拦）'),
             (90, 100, '扇区内微调（必须拦）'),
             (0, 90, '正前 → 正左（必须拦）')]
    for cur, tgt, note in cases:
        cross = yaw_crosses_sector(cur, tgt, lo, hi)
        print('  yaw %3d → %3d  过危险区=%-5s  %s' % (cur, tgt, cross, note))
    if not yaw_crosses_sector(0, 180, lo, hi):
        fails.append('互锁判据漏了 0→180 这条最常见的回转路径')
    if yaw_crosses_sector(0, 30, lo, hi):
        fails.append('互锁判据把 0→30 这种安全动作也拦了（太保守，会让手动模式卡住）')
    print()

    for w in warns:
        print('[警告] %s' % w)
    for f in fails:
        print('[失败] %s' % f)
    if not fails:
        print('结论: 通过（%d 项警告）' % len(warns))
    print('=' * 74)
    return 1 if fails else 0

if __name__ == '__main__':
    sys.exit(main())
