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
# 兜底值仅对应 2026-10-27 那一版，仅供参考，别当准。
G_DEBUG = 0x20000004
G_LEN   = 0x20000054
G_BYTES = 0x20000055
G_PS2   = None      # Ps2Service 的地址，用来读 stage / fail_count

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


def elf_reset(elf):
    """读同名 .bin 的向量表第 2 个字，返回不带 Thumb 位的 Reset 地址。

    用 .bin 而不是 ELF 的 entry，是因为 .bin 和我们烧进板子的东西是
    同一个文件，比对起来没有中间环节。
    """
    b = os.path.splitext(elf)[0] + '.bin'
    try:
        d = open(b, 'rb').read(8)
    except OSError:
        return None
    if len(d) < 8:
        return None
    return int.from_bytes(d[4:8], 'little') & ~1


def pick_elf(board_reset=None):
    """选 ELF。

    不能只按"最新那个"选：编过一次 --servotest 之后，build 目录里最新的
    ELF 是舵机自检固件，它根本没有 g_debug，脚本会退回内置地址乱读。
    所以有板上的 Reset 时，优先挑 .bin 向量表能对上的那一份。
    """
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
    if board_reset is not None:
        hit = [e for e in elfs if elf_reset(e) == (board_reset & ~1)]
        if hit:
            return max(hit, key=os.path.getmtime)
    return max(elfs, key=os.path.getmtime)


def elf_symbols(elf, nm):
    p = subprocess.run([nm, '-n', elf], capture_output=True, text=True, timeout=60)
    sym = {}
    for line in (p.stdout or '').splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                sym[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    return sym


def resolve_symbols(board_reset=None):
    """返回 (g_debug, g_ps2_last_length, g_ps2_last_bytes, 说明文字)"""
    global G_DEBUG, G_LEN, G_BYTES, G_PS2
    nm = find_nm()
    elf = pick_elf(board_reset)
    if not nm or not elf or not os.path.exists(elf):
        return G_DEBUG, G_LEN, G_BYTES, '找不到 nm 或 ELF，用内置地址（可能与固件不符）'
    sym = elf_symbols(elf, nm)
    need = ['g_debug', 'g_ps2_last_length', 'g_ps2_last_bytes']
    if not all(k in sym for k in need):
        return G_DEBUG, G_LEN, G_BYTES, 'ELF 里没找到符号，用内置地址（可能与固件不符）'
    G_DEBUG, G_LEN, G_BYTES = sym['g_debug'], sym['g_ps2_last_length'], sym['g_ps2_last_bytes']
    # g_ps2 可能不存在（老版本固件里它是 main 的局部变量），有就读
    G_PS2 = sym.get('g_ps2')
    return G_DEBUG, G_LEN, G_BYTES, '符号地址取自 %s' % os.path.basename(elf)


def read_mem(oc, scr, addr, words):
    """读 words 个 32bit 字，返回 (字节列表, 原始输出)

    speed 固定 500kHz：这块克隆版 ST-LINK 在默认 4MHz 下读几次就会掉
    （Failed to write memory at 0xe00fffec / Examination failed），
    降速之后基本一次就成。诊断脚本不在乎这点速度。
    """
    cmds = ['adapter speed 500', 'init', 'halt',
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


def check_firmware(oc, scr, board_reset=None):
    """比对板上的 Reset 向量和 ELF 里的 Reset_Handler。

    这一步不能省。读内存诊断的前提是"板上跑的正是这份 ELF"，
    否则每个变量的地址都是别的固件的布局，脚本照样能拼出
    看起来很像样的数字（2026-10-27 就因此误报过一次"PS2 已通"：
    板上是 demo 固件，g_debug 那个地址上其实是 demo 的 g_wheel，
    两个 500 的速度值被当成了帧计数）。
    """
    nm = find_nm()
    elf = pick_elf(board_reset)
    if not (nm and elf and os.path.exists(elf)):
        return None, None, None
    exp = elf_symbols(elf, nm).get('Reset_Handler')
    if exp is None:
        return None, None, None
    if board_reset is None:
        d, _ = read_mem(oc, scr, 0x08000000, 2)
        if len(d) < 8:
            return None, None, exp
        board_reset = int.from_bytes(bytes(d[4:8]), 'little')
    brd = board_reset
    # 向量表里的 Reset 带 Thumb 位（最低位恒为 1），符号表里的地址不带，
    # 所以只比高位，不能直接相等比较。
    return ((brd & ~1) == (exp & ~1)), brd, exp


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

    # 先验合理性，再看帧计数。顺序不能反：
    # 地址错位时 valid_frames 照样可能是个大数，先看它就会误判成"通了"。
    if ln > 21:
        print('    ✗ 数据不可信：last_length = %d，PS2 帧长只可能是 5/9/21（0 = 还没收到）' % ln)
        print('      → 这个地址上的东西不是 PS2 帧缓冲')
        print('      → 大概率板上跑的不是这份 ELF，先确认固件版本')
        return 'BAD'
    if vf > 0 and ln == 0:
        print('    ✗ 自相矛盾：valid_frames = %d 但 last_length = 0' % vf)
        print('      → 这两个变量不在同一个结构里，地址或固件版本对不上')
        return 'BAD'
    if vf > 0:
        print('    ★★★ PS2 已通！收到 %d 帧合法数据，模式 0x%02X' % (vf, mode))
        return 'OK'
    if ln == 0:
        print('    ✗ 一帧都没收到 → 接收器完全没回应')
        print('      → 查 6 根线（尤其 DI/DO 有没有反）、VCC 是不是 3.3V、共地')
        return 'DEAD'
    raw_n = [raw[i] for i in range(min(ln, 21))]
    if raw_n and all(b == 0xFF for b in raw_n):
        print('    ✗ 回帧 %d 个字节全是 FF → DAT 线从头到尾没被拉低过' % ln)
        print('      这不等于"接收器在线但装死"，更常见的是它根本没开口：')
        print('      · 接收器没电或电压不够（VCC 要 3.3V，别接 5V；GND 要共）')
        print('      · DAT(DO) 没接到 PA6，或者 DI/DO 接反了（这是最常见的一种）')
        print('      · CS / CLK 没到，接收器压根没被选中')
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

    # 先读板上的 Reset，再用它去挑对应的 ELF —— 顺序不能反。
    d0, _ = read_mem(oc, scr, 0x08000000, 2)
    brd = int.from_bytes(bytes(d0[4:8]), 'little') if len(d0) >= 8 else None
    _, _, _, note = resolve_symbols(brd)
    print(' 板上 Reset = %s' % ('0x%08X' % brd if brd else '(读不到)'))
    print(' %s' % note)
    print(' g_debug=0x%08X  last_length=0x%08X  last_bytes=0x%08X'
          % (G_DEBUG, G_LEN, G_BYTES))

    ok, brd, exp = check_firmware(oc, scr, brd)
    if ok is False:
        print()
        print('[!] 板上固件和这份 ELF 不是同一版')
        print('    板上 Reset = 0x%08X' % brd)
        print('    ELF  期望 = 0x%08X' % exp)
        print('    读内存诊断的前提是两者一致，否则每个变量的地址都错位，')
        print('    读出来的数字看着像样但全是别的变量。先烧录：')
        print('      python tools/flash.py v2s')
        return 2
    if ok is True:
        print(' 固件校验：板上 Reset = 0x%08X，与 ELF 一致' % brd)

    # 服务自己的状态：stage 非 0 说明正卡在配置事务里，
    # 这才是"被锁"的直接证据，比看帧数更快。
    if G_PS2:
        svc, _ = read_mem(oc, scr, G_PS2, 3)
        if len(svc) >= 12:
            st = svc[0]
            fail = int.from_bytes(bytes(svc[2:4]), 'little')
            print(' 服务状态：stage = %d   fail_count = %d' % (st, fail))
            if st != 0:
                print('   ⚠ stage = %d → 正卡在配置事务（1=enter 2=mode 3=rumble 4=leave）' % st)
                print('     这就是被锁的样子。断电重上电，或再烧一次都能解。')

    last = None
    prev_vf = None
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
        vf_now = u32(dbg, 0)
        if prev_vf is not None and vf_now < prev_vf:
            print('    ⚠ valid_frames 比上次采样还小（%d → %d）：' % (prev_vf, vf_now))
            print('      要么中间复位过，要么读的不是同一个变量')
        prev_vf = vf_now

        if i != samples - 1:
            import time
            time.sleep(1.2)

    print()
    print('=' * 58)
    print(' 结论: %s' % {
        'OK':   'PS2 通了，这套固件可以进入实车',
        'DEAD': '一帧都没有 —— 接收器没回应',
        'FF':   '全 FF —— DAT 线一直是高，接收器没有回话（多半没电/线没到位）',
        'ODD':  '收到异常数据，看原始字节',
        'BAD':  '读数不可信 —— 先确认板上固件和 ELF 是同一版',
    }.get(last, '?'))
    print('=' * 58)
    return 0


if __name__ == '__main__':
    sys.exit(main())
