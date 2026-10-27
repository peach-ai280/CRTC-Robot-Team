#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ps2_scan.py —— PS2 接线「24 种接法全试一遍」的解读者

配合 firmware/Core/Src/test_ps2scan.c：那个固件用软件打时钟，把
A5 / A7 / A6 / B0 四个脚在「CLK / CMD / CS / DAT」四个角色之间做全排列
（4! = 24 种），每种硬发一帧 9 字节。谁回话，真实接线就是哪一种。

用法规（在 CRTC 目录下，ST-LINK 接好、板子通电）：
    python tools/ps2_scan.py                烧 ps2scan → 读结果 → 给结论
    python tools/ps2_scan.py --read         只读（已经烧过 ps2scan 了）
    python tools/ps2_scan.py --restore v2s  扫完把 v2s（车一）固件烧回去
    python tools/ps2_scan.py --restore full 扫完把 full 固件烧回去

⚠ 跑之前先确认接收器真的通着电（VCC–GND 之间要有 3.2~3.3V 或 4.8~5V）。
  接收器没电的话，24 种接法全都不会回话，这个扫描就白跑了。

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
    restore = None
    if '--restore' in args:
        i = args.index('--restore')
        nxt = args[i + 1] if i + 1 < len(args) else ''
        restore = nxt if nxt and not nxt.startswith('-') else 'full'

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

    # ---- 从 24 行数据反推每个脚的真实电平 -------------------------------
    # 一根脚只要在某一轮里当过 DAT，它读到的值就直接反映这根线当时的电平：
    #   9 个字节全 00 → 这根线被钉在低电平（被短到 GND，或者芯片引脚已损坏）
    #   9 个字节全 FF → 这根线被钉在高电平（被短到 3.3V/5V）
    # 正常接线下，DAT 在"没被选中"时应该是高，在传输中会有 0/1 交替，
    # 所以四种读数都会出现。
    levels = {}
    for p in range(24):
        base = 8 + p * 10
        d_pin = NAMES[PERMS[p][3]]
        rx = [w[base + i] & 0xFF for i in range(9)]
        if all(b == 0x00 for b in rx):
            v = 'L'
        elif all(b == 0xFF for b in rx):
            v = 'H'
        else:
            v = '?'
        levels.setdefault(d_pin, set()).add(v)

    pinned = {}
    for pin in NAMES:
        s = levels.get(pin, set())
        if s == {'L'}:
            pinned[pin] = 'L'
        elif s == {'H'}:
            pinned[pin] = 'H'

    if pinned:
        print('')
        print('=' * 82)
        print('★ 电气结论（按每个脚当 DAT 时的读数反推）')
        print('=' * 82)
        print('')
        for pin in NAMES:
            if pin in pinned:
                why = '低电平 0V' if pinned[pin] == 'L' else '高电平 3.3V'
                print('   %-3s 被钉死在%s' % (pin, why))
            else:
                print('   %-3s 有变化（正常，或者接的是接收器的输出脚）' % pin)
        print('')
        print('  一根脚被钉死的意思是：把它的内部上拉/下拉打开，电平都不跟着走。')
        print('  这已经不是"接错了对象"了——接错对象只是两根线互换，')
        print('  交换之后扫描能查出来。钉死是这根线被短到电源或地，')
        print('  或者是这颗芯片上这个脚已经坏了（ESD 打穿后内部对地/对电源短路）。')
        print('')
        print('  ★ 先断线再量一次，区分是"线短了"还是"芯片坏了"：')
        print('     1) 拔掉接收器那 4 根信号线，重新跑一遍本脚本')
        print('     2) 还是钉死 → 芯片引脚坏了，换一块 Blue Pill（备用板）')
        print('     3) 不钉死了 → 是线的问题，一根一根插回去，插一根量一次')

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
        print('  四个脚随便怎么排都不回话，说明接收器根本没有在 DAT 上开口。')
        print('  按这个顺序查（第 1 条最常见）：')
        print('')
        print('   ① 接收器没通电。它不吃 ST-LINK 那点 3.3V 也要有电——量一下')
        print('      接收器 VCC 和 GND 两个孔之间的电压，应该是 3.2~3.3V；')
        print('      如果是 0V，就是电源没接上（继电器没开 / 接在 COM 上被切断了）。')
        print('   ② 接收器要 5V。有些便宜模块 3.3V 下指示灯亮但逻辑走不动，')
        print('      把 VCC 换到 5V 那一路再扫一次。')
        print('   ③ 手柄和接收器没对上频。手柄那颗灯应该是"闪几下后常亮"，')
        print('      一直慢闪就是没对上。')
        print('   ④ 那几个孔虚焊 / 杜邦头没压好。换线、换孔各试一次。')

    if restore:
        print('')
        if flash_target(restore):
            print('>> 已把 %s 固件烧回去。' % restore)
        else:
            print('[警告] 烧回 %s 失败，板子现在还是 ps2scan。' % restore)
    else:
        print('')
        print('（板子现在跑的是 ps2scan；python tools/ps2_scan.py --restore 可烧回 full）')

    print('')
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
