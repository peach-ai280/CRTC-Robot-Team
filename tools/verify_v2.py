#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
verify_v2.py —— 烧完 firmware_v2 后，直接读芯片内存判断 PS2 到底通没通。

不靠看灯、不靠摇杆试，只读两个变量（地址每次从 ELF 符号表里现取）：
  g_debug            帧计数 / 模式 / 状态 / 四轮输出
  g_ps2_last_bytes   接收器最后一帧的原始字节

为什么不在代码里写死地址：bss 的布局会随代码改动整体平移。
2026-10-27 加了两个变量之后，g_debug 就从 0x20000000 挪到了 0x20000004，
写死地址的版本会把每个字段都读错 4 个字节，然后给出完全错误的结论。
现在改成用 arm-none-eabi-nm 读 ELF，布局怎么变都不会错。

判断标准：
  valid_frames > 0             → 握手成功，PS2 通了
  last_length == 9 且 [1] 是 41/73/79 → 收到合法帧
  valid_frames == 0 且 last_length == 0 → 一帧都没收到（接线/接收器问题）
  last_bytes 全 FF                       → 接收器一个字节都不回（经典没回应）

用法：python tools/verify_v2.py
      加 --samples N 多采几次（默认 3 次，间隔 1.2s）
      加 --elf 路径 指定要读符号的 ELF（默认取 build 目录里最新的那个）
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import flash   # 复用 find_openocd / IFACE_CFG / TARGET_CFG / ROOT

# 这三个是兜底值，正常情况下 resolve_symbols() 会用 ELF 里的真实地址覆盖。
# 兜底值对应 firmware_servo.bin 2026-10-27 那一版，仅供参考。
G_DEBUG = 0x20000004
G_LEN   = 0x20000048
G_BYTES = 0x20000049

MODE_NAME = {0x41: '数字红灯', 0x73: '模拟绿灯', 0x79: '模拟红灯'}


def find_nm():
    """找 arm-none-eabi-nm：先问 build_v2 拿工具链路径，再退回 PATH。"""
    cand = []
    try:
        import build_v2
        cand.append(os.path.join(build_v2.TOOLS, 'arm-none-eabi-nm.exe'))
    except Exception:
        pass
    cand.append('arm-none-eabi-nm.exe')
    cand.append('arm-none-eabi-nm')
    for c in cand:
        try:
            p = subprocess.run([c, '--version'], capture_output=True,
                               text=True, timeout=20)
            if p.returncode == 0:
                return c
        except Exception:
            continue
    return None


def pick_elf():
    if '--elf' in sys.argv:
        return sys.argv[sys.argv.index('--elf') + 1]
    build = os.path.join(flash.ROOT, 'firmware_v2', 'build')
    try:
        elfs = [os.path.join(build, f) for f in os.listdir(build)
                if f.endswith('.elf')]
    except OSError:
        return None
    if not elfs:
        return None
    # 取最新的那个：谁刚编完就读谁，避免读了上一次编译的地址
    return max(elfs, key=os.path.getmtime)


def resolve_symbols():
    """返回 (g_debug, g_ps2_last_length, g_ps2_last_bytes, 说明文字)"""
    global G_DEBUG, G_LEN, G_BYTES
    nm = find_nm()
    elf = pick_elf()
    if not nm or not elf or not os.path.exists(elf):
        return G_DEBUG, G_LEN, G_BYTES, '找不到 nm 或 ELF，用内置地址（可能与固件不符）'
    p = subprocess.run([nm, '-n', elf], capture_output=True, text=True, timeout=60)
    sym = {}
    for line in (p.stdout or '').splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                sym[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    need = ['g_debug', 'g_ps2_last_length', 'g_ps2_last_bytes']
    if not all(k in sym for k in need):
        return G_DEBUG, G_LEN, G_BYTES, 'ELF 里没找到符号，用内置地址（可能与固件不符）'
    G_DEBUG, G_LEN, G_BYTES = sym['g_debug'], sym['g_ps2_last_length'], sym['g_ps2_last_bytes']
    return G_DEBUG, G_LEN, G_BYTES, '符号地址取自 %s' % os.path.basename(elf)


def read_mem(oc, scr, addr, words):
    """读 words 个 32bit 字，返回 (字节列表, 原始输出)"""
    cmds = ['init', 'halt',
            'mdw 0x%08X %d' % (addr, words),
            'resume', 'shutdown']
    cmd = [oc, '-s', scr, '-f', flash.IFACE_CFG, '-f', flash.TARGET_CFG]
    for c in cmds:
        cmd += ['-c', c]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=90)
    out = (p.stdout or '') + (p.stderr or '')

    data = []
    for m in re.finditer(r'0x([0-9a-fA-F]{8}):((?:\s+[0-9a-fA-F]{8})+)', out):
        for w in m.group(2).split():
            v = int(w, 16)
            data += [v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, (v >> 24) & 0xFF]
    return data, out


def u32(b, off):
    return b[off] | (b[off + 1] << 8) | (b[off + 2] << 16) | (b[off + 3] << 24)


def s16(b, off):
    v = b[off] | (b[off + 1] << 8)
    return v - 65536 if v & 0x8000 else v


def report(dbg, ln, raw):
    print('  —— 帧统计 ——')
    print('    valid_frames   = %-8d  invalid_frames = %d'
          % (u32(dbg, 0), u32(dbg, 4)))
    print('    last_loop_ms   = %d ms' % u32(dbg, 8))

    print('  —— 连接状态 ——')
    connected = dbg[0x0D] != 0
    armed = dbg[0x0E] != 0
    emg = dbg[0x0F] != 0
    print('    connected = %-5s  armed = %-5s  emergency = %s'
          % (connected, armed, emg))

    mode = dbg[0x0C]
    print('    ps2_mode  = 0x%02X %s'
          % (mode, MODE_NAME.get(mode, '(未知/未收到)')))

    print('  —— 四轮输出 —— LF %d  RF %d  LB %d  RB %d'
          % (s16(dbg, 0x10), s16(dbg, 0x12), s16(dbg, 0x14), s16(dbg, 0x16)))

    print('  —— 接收器最后回帧 ——')
    print('    last_length = %d' % ln)
    if ln == 0:
        print('    原始字节 = (空，一个字节都没收到)')
    else:
        n = min(ln, 21)
        print('    原始字节 = ' + ' '.join('%02X' % raw[i] for i in range(n)))

    print()
    print('  ===== 判定 =====')
    vf = u32(dbg, 0)
    if vf > 0:
        print('    ★★★ PS2 已通！收到 %d 帧合法数据，模式 0x%02X' % (vf, mode))
        return 'OK'
    if ln == 0:
        print('    ✗ 一帧都没收到 → 接收器完全没回应')
        print('      → 查 6 根线（尤其 DI/DO 有没有反）、VCC 是不是 3.3V、共地')
        return 'DEAD'
    raw_n = [raw[i] for i in range(min(ln, 21))]
    if raw_n and all(b == 0xFF for b in raw_n):
        print('    ✗ 收到 %d 个字节，但全是 FF → 接收器在线但不应答' % ln)
        print('      → 这是之前一直遇到的现象：命令发出去了，对方不返回值')
        print('      → 最大嫌疑：CS/CLK 到了但 CMD(DI) 没到，或 DAT(DO) 断开')
        return 'FF'
    if ln >= 2 and raw[1] in MODE_NAME:
        print('    ★ 收到合法帧但 valid_frames 还是 0（可能刚上电）')
        return 'OK'
    print('    ? 收到数据但不像合法帧，看上面原始字节')
    return 'ODD'


def main():
    samples = 3
    if '--samples' in sys.argv:
        samples = int(sys.argv[sys.argv.index('--samples') + 1])

    oc = flash.find_openocd()
    if not oc[0]:
        print('[!] 找不到 OpenOCD')
        return 1
    oc, scr = oc[0], oc[1]

    print('=' * 58)
    print(' firmware_v2 运行状态（直接读芯片内存，不看灯）')
    print('=' * 58)
    _, _, _, note = resolve_symbols()
    print(' %s' % note)
    print(' g_debug=0x%08X  last_length=0x%08X  last_bytes=0x%08X'
          % (G_DEBUG, G_LEN, G_BYTES))

    last = None
    for i in range(samples):
        dbg, out = read_mem(oc, scr, G_DEBUG, 6)
        if len(dbg) < 24:
            print('[!] 读内存失败，OpenOCD 原始输出末尾：')
            print('\n'.join(out.splitlines()[-8:]))
            return 1
        ln, _ = read_mem(oc, scr, G_LEN, 1)
        raw, _ = read_mem(oc, scr, G_BYTES, 6)

        print('\n--- 第 %d/%d 次采样 ---' % (i + 1, samples))
        verdict = report(dbg, ln[0] if ln else 0, raw)
        last = verdict

        if i != samples - 1:
            import time
            time.sleep(1.2)

    print()
    print('=' * 58)
    print(' 结论: %s' % {
        'OK':   'PS2 通了，这套固件可以进入实车',
        'DEAD': '一帧都没有 —— 接收器没回应',
        'FF':   '全 FF —— 接收器在线但不应答（老问题）',
        'ODD':  '收到异常数据，看原始字节',
    }.get(last, '?'))
    print('=' * 58)
    return 0


if __name__ == '__main__':
    sys.exit(main())
