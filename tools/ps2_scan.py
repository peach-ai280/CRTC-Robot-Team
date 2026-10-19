#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ps2_scan.py —— PS2 接线「24 种接法全试一遍」的解读者

配合 firmware/Core/Src/test_ps2scan.c：那个固件用软件打时钟，把
A5 / A7 / A6 / B0 四个脚在「CLK / CMD / CS / DAT」四个角色之间做全排列
（4! = 24 种），每种硬发一帧 9 字节。谁回话，真实接线就是哪一种。

用法规（在 CRTC 目录下，ST-LINK 接好、板子通电）：
    python tools/ps2_scan.py            烧 ps2scan → 读结果 → 给结论
    python tools/ps2_scan.py --read     只读（已经烧过 ps2scan 了）
    python tools/ps2_scan.py --restore  扫完把 full 固件烧回去

━━ 结论怎么用 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
正常接线应该是：  CLK→A5   CMD→A7   CS→B0   DAT→A6

如果扫描说「排列 p 通了」，那就说明现实中是另一套映射，
脚本会直接告诉你「把插在 X 的那根线改插到 Y」，照着挪线就行。
"""

import itertools
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import flash                                   # noqa: E402
import ps2_probe                               # noqa: E402

NAMES = ['A5', 'A7', 'A6', 'B0']               # 索引要和 C 里的 P_A5/P_A7/P_A6/P_B0 一致
ROLES = ['CLK', 'CMD', 'CS', 'DAT']
PERMS = list(itertools.permutations(range(4)))  # 顺序和 C 里四层循环一致（字典序）

# 正确接法：角色 -> 应该在哪个脚
TARGET = {'CLK': 'A5', 'CMD': 'A7', 'CS': 'B0', 'DAT': 'A6'}

MAGIC = 0x50534341


def flash_target(t):
    print('')
    print('>> 烧 %s 固件…' % t)
    r = subprocess.run([sys.executable, os.path.join(HERE, 'flash.py'), t], cwd=ROOT)
    return r.returncode == 0


def read_scan(target='ps2scan', tries=10):
    addr = ps2_probe.sym_addr(target, 'g_scan')
    if addr is None:
        print('[错误] map 里找不到 g_scan。先编译：python tools/build.py %s' % target)
        return None

    oc, scr = flash.find_openocd()
    if not oc:
        print('[错误] 找不到 OpenOCD。')
        return None

    for i in range(tries):
        b = ps2_probe.read_block(oc, scr, addr, 4 * 300)
        if b is None or len(b) < 16:
            print('[错误] 读不到 RAM。')
            return None
        words = [int.from_bytes(b[j:j + 4], 'little') for j in range(0, len(b) // 4 * 4, 4)]
        if words[0] == MAGIC:
            return words
        time.sleep(0.5)
        print('   等它扫完… (%d/%d)' % (i + 1, tries))
    print('[警告] 没看到 magic，固件可能没跑起来（检查灯闪不闪）。')
    return None


def show_elec(sig):
    labels = ['A5', 'A7', 'A6', 'B0']
    print('')
    print('  引脚   下拉读到  上拉读到   状态')
    print('  ' + '-' * 46)
    for i, nm in enumerate(labels):
        pd = (sig >> i) & 1
        pu = (sig >> (8 + i)) & 1
        if pu and pd:
            st = '恒定高（外面在驱动）'
        elif pu and not pd:
            st = '悬空'
        elif not pu and not pd:
            st = '恒定低'
        else:
            st = '异常'
        print('  %-4s   %-8d  %-8d   %s' % (nm, pd, pu, st))


def main():
    args = sys.argv[1:]
    read_only = '--read' in args
    restore = '--restore' in args

    print('')
    print('PS2 接线穷举扫描 —— A5/A7/A6/B0 四个脚 × 四个角色 = 24 种接法全试')

    if not read_only:
        if not flash_target('ps2scan'):
            print('[错误] 烧 ps2scan 失败。先跑 python tools/flash.py check')
            return 1
        time.sleep(1.0)

    w = read_scan()
    if w is None:
        return 1

    show_elec(w[3])

    print('')
    print('=' * 82)
    print('24 种接法对照（每种硬发一帧 9 字节 PS2 轮询）')
    print('=' * 82)
    print('')
    print('   #   CLK  CMD   CS   DAT   收到的 9 字节                                  得分')
    print('  ' + '-' * 94)

    best = None
    for p in range(24):
        base = 8 + p * 10
        rx = [w[base + i] & 0xFF for i in range(9)]
        score = w[base + 9] & 0xFF
        c, m, s, d = [NAMES[x] for x in PERMS[p]]
        txt = ' '.join('%02X' % b for b in rx)
        mark = ' ★通' if score == 0xFF else ('  %d' % score if score else '')
        print('  %2d   %-4s %-4s %-4s %-4s  %s%s' % (p, c, m, s, d, txt, mark))
        if score == 0xFF and best is None:
            best = p

    print('')
    print('=' * 82)
    if best is not None:
        c, m, s, d = [NAMES[x] for x in PERMS[best]]
        mode = w[2] & 0xFF
        print('★★★ 找到了！排列 #%d 通了' % best)
        print('')
        print('  你的真实接线是：')
        print('    %s 那根线 → 接收器 CLK' % c)
        print('    %s 那根线 → 接收器 CMD' % m)
        print('    %s 那根线 → 接收器 CS' % s)
        print('    %s 那根线 → 接收器 DAT' % d)
        print('')
        print('  状态字节 = 0x%02X（%s）'
              % (mode, {0x41: '数字模式', 0x73: '模拟·绿灯', 0x79: '模拟·红灯'}.get(mode, '未知')))

        have = {c: 'CLK', m: 'CMD', s: 'CS', d: 'DAT'}
        moves = []
        for pin in NAMES:
            role = have[pin]
            if TARGET[role] != pin:
                moves.append((pin, role, TARGET[role]))
        print('')
        if not moves:
            print('  → 你现在的接线本来就是对的，问题在固件，我来改。')
        else:
            print('  ★ 改法（先拔电，再动线）：')
            for pin, role, want in moves:
                print('     把现在插在【%s】的那根（其实是 %s）改插到【%s】' % (pin, role, want))
    else:
        print('24 种接法全都没回话。')
        print('')
        print('  这一下可以确定：**不是接线顺序的问题**了。')
        print('  接收器被供电、被驱动着，但对任何时钟都不回应。真正的可能只剩：')
        print('')
        print('   ① 接收器要 5V 才能工作（很多便宜模块是 5V 设计的，3.3V 下')
        print('      指示灯亮但逻辑走不动）。→ 把 XTARK 的 VCC 接到 STM32 的 5V 孔试一次。')
        print('   ② 手柄和接收器没对上频（手柄那颗灯应该是"闪几下后常亮"）。')
        print('   ③ 接收器坏了 / 引脚被焊坏 / 那 6 个孔有虚焊。')

    if restore:
        if flash_target('full'):
            print('')
            print('>> 已把 full 固件烧回去。')
    else:
        print('')
        print('（板子现在跑的是 ps2scan；python tools/ps2_scan.py --restore 可烧回 full）')

    print('')
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
