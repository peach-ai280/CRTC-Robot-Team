#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ps2_probe.py —— 不用串口、不用看灯，直接读芯片内存判断 PS2 手柄到底通没通

用法：
    python tools/ps2_probe.py                # 默认探 full 固件
    python tools/ps2_probe.py ps2            # 探 ps2 固件
    python tools/ps2_probe.py full -n 5      # 连读 5 次，每次间隔 1 秒

原理：
    芯片跑着固件，用 ST-LINK 把 CPU 暂停一瞬，读全局变量 `ps2` 的内存，
    解出 connected / mode / raw[9] / lost_cnt，读完立刻让 CPU 继续跑。
    所以它不打断你的车，只在暂停的那几毫秒内看不到数据。

为什么比看灯强：
    full 固件的灯只有"常亮/慢闪"两态，只能告诉你"通/不通"；
    这里能看出 raw[9] 到底收到了什么，直接把问题分成三类。

符号地址从 firmware/build/<目标>/firmware.map 里解析，不用 arm-none-eabi-nm。
"""
import os
import re
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

sys.path.insert(0, HERE)
import flash  # noqa: E402  复用它的 find_openocd()

# ---------------------------------------------------------------- 结构体布局
# PS2_t 的偏移 —— ★ 已用"lost_cnt 每 5ms +1"这个活体征实测校准过。
# 注意各成员最大对齐只有 2（uint16），所以**没有任何填充**：
#   btn(2) btn_prev(2) btn_click(2) lx(2) ly(2) rx(2) ry(2) mode(1) connected(1)
#   = 14 字节，紧接着 lost_cnt 落在偏移 16（天然 2 对齐），raw 落在 18。
OFF_BTN       = 0
OFF_BTN_PREV  = 2
OFF_BTN_CLICK = 4
OFF_LX        = 6
OFF_LY        = 8
OFF_RX        = 10
OFF_RY        = 12
OFF_MODE      = 14
OFF_CONNECTED = 15
OFF_LOST      = 16
OFF_RAW       = 18
SIZEOF_PS2    = 28          # 18 + 9 = 27，按 2 对齐补到 28

MODE_NAME = {
    0x41: '数字模式（摇杆只回 0/255）',
    0x73: '模拟模式 · 绿灯',
    0x79: '模拟模式 · 红灯',
    0xF3: '模拟模式 · 绿灯（部分克隆手柄）',
    0x00: '还没握上手（一次都没成功）',
}

# 硬件寄存器（诊断用）
REG = {
    'SPI1_CR1':  0x40013000,
    'SPI1_CR2':  0x40013004,
    'SPI1_SR':   0x40013008,
    'SPI1_DR':   0x4001300C,
    'GPIOA_IDR': 0x40010808,
    'GPIOB_IDR': 0x40010C08,
}


# ============================ 找符号地址 ============================

def sym_addr(target, name):
    """从 map 文件里取全局变量地址。返回 None 表示没找到（多半是被 -Os 折掉了）。"""
    mp = os.path.join(ROOT, 'firmware', 'build', target, 'firmware.map')
    if not os.path.isfile(mp):
        print('[错误] 找不到 %s' % mp)
        print('       先编译：python tools/build.py %s' % target)
        return None
    pat = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+%s\s*$' % re.escape(name))
    with open(mp, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = pat.match(line)
            if m:
                return int(m.group(1), 16)
    return None


# ============================ 读内存 ============================

def read_block(oc, scr, addr, nbytes):
    """读一段连续内存，返回 bytes。"""
    base = addr & ~3
    nwords = (addr - base + nbytes + 3) // 4
    cmds = ['init', 'halt', 'mdw 0x%08X %d' % (base, nwords), 'resume', 'exit']
    cmd = [oc, '-s', scr, '-f', flash.IFACE_CFG, '-f', flash.TARGET_CFG]
    for c in cmds:
        cmd += ['-c', c]

    try:
        p = subprocess.run(cmd, capture_output=True, timeout=90)
    except subprocess.TimeoutExpired:
        print('[错误] OpenOCD 超时。ST-LINK 插好了吗？')
        return None
    out = (p.stdout or b'').decode('utf-8', 'ignore') + \
          (p.stderr or b'').decode('utf-8', 'ignore')

    buf = bytearray()
    for line in out.splitlines():
        m = re.match(r'^0x([0-9a-fA-F]{8}):\s+(.*)$', line.strip())
        if not m:
            continue
        for w in m.group(2).split():
            if re.fullmatch(r'[0-9a-fA-F]{8}', w):
                buf += int(w, 16).to_bytes(4, 'little')
    if not buf:
        print('[错误] 没读到内存。OpenOCD 输出：')
        for ln in out.splitlines()[-12:]:
            print('   ' + ln)
        return None
    off = addr - base
    return bytes(buf[off:off + nbytes])


# ============================ 解读 ============================

def u16(b, o):
    return b[o] | (b[o + 1] << 8)


def u8(b, o):
    return b[o]


def s16(b, o):
    v = u16(b, o)
    return v - 0x10000 if v & 0x8000 else v


def probe_once(target, oc, scr, ps2_addr, quiet=False):
    raw_blk = read_block(oc, scr, ps2_addr, SIZEOF_PS2)
    if raw_blk is None:
        return False

    btn       = u16(raw_blk, OFF_BTN)
    lx        = s16(raw_blk, OFF_LX)
    ly        = s16(raw_blk, OFF_LY)
    rx        = s16(raw_blk, OFF_RX)
    ry        = s16(raw_blk, OFF_RY)
    mode      = u8(raw_blk, OFF_MODE)
    connected = u8(raw_blk, OFF_CONNECTED)
    lost      = u16(raw_blk, OFF_LOST)
    raw       = raw_blk[OFF_RAW:OFF_RAW + 9]

    secs = lost * 5.0 / 1000.0    # PS2_Scan 每 5ms 一次

    if not quiet:
        print('=' * 68)
        print('ps2 结构体 @ 0x%08X' % ps2_addr)
        print('=' * 68)
        print('  connected = %d      %s' % (
            connected, '在线' if connected else '不在线'))
        print('  mode      = 0x%02X   %s' % (
            mode, MODE_NAME.get(mode, '未知')))
        print('  lost_cnt  = %-6d  ≈ 连续失败 %.1f 秒' % (lost, secs))
        print('  raw[9]    = %s' % ' '.join('%02X' % b for b in raw))
        print('  按键位图  = 0x%04X' % btn)
        print('  摇杆      = 左(%4d,%4d)  右(%4d,%4d)' % (lx, ly, rx, ry))
        print('  原始字节  = %s' % ' '.join('%02X' % b for b in raw_blk))
        print('-' * 68)

    # ---- 判定 ----
    # 注意：CPU 是在 memcpy 中途被暂停的，raw 末尾可能有几个字节是上一帧残留，
    #       所以用"9 个字节里有 7 个以上是 FF"来判断"对面没说话"。
    ff_cnt = sum(1 for b in raw if b == 0xFF)
    all_ff = ff_cnt >= 7

    if connected and mode in (0x73, 0x79, 0xF3):
        print('  [OK] 手柄通了，模拟模式。摇杆可以用了。')
        if not quiet:
            print('       → 推一下摇杆再跑一次本脚本，看 lx/ly 有没有跟着变。')
        return True

    if connected and mode == 0x41:
        print('  [半通] 握手成功但停在数字模式（摇杆只会是 0 或 255）。')
        print('         把接收器断电重上电一次（拔 ST-LINK 的 USB 等 3 秒再插）。')
        return False

    if all_ff:
        print('  [不通] raw 全是 FF —— 接收器一个字节都没回。')
        print('')
        print('  说明：MISO(A6) 线上永远是高，接收器从来没把它拉低过。')
        print('  按这个顺序查（前面的是最可能的）：')
        print('   1. 手柄开了没？开关推到 ON，手柄自己的灯要变常亮')
        print('   2. 接线时板子断电了没？没断电就拔 ST-LINK 的 USB 等 3 秒再插')
        print('   3. VCC 到底有没有 3.3V？——万用表量 XTARK 板 VCC 孔和 GND 孔')
        print('   4. DAT(④) 这根接上了没？它在 A6（上排⑩）')
        print('   5. CS(②) 接 B0（上排⑧）、CLK(①) 接 A5（上排⑪），有无插错')
        print('   6. GND(⑥) 接上了没？不共地 = SPI 必废')
        return False

    print('  [不通] 有线上的数据，但握手失败（rx[2] != 0x5A）。')
    print('         raw = %s' % ' '.join('%02X' % b for b in raw))
    if raw[1] == 0xFF and raw[2] == 0xFF:
        print('         像是 CMD/DAT 反了或 CS 没拉低。')
    print('         先试：把 CMD(③) 和 DAT(④) 两根对调，再跑一次。')
    return False


def main():
    args = [a for a in sys.argv[1:] if not a.startswith('-')]
    target = args[0] if args else 'full'

    times = 1
    if '-n' in sys.argv:
        i = sys.argv.index('-n')
        if i + 1 < len(sys.argv):
            times = max(1, int(sys.argv[i + 1]))

    ps2_addr = sym_addr(target, 'ps2')
    if ps2_addr is None:
        print('[错误] map 里找不到 ps2 符号（可能被 -Os 优化掉了）。')
        return 2

    oc, scr = flash.find_openocd()
    if oc is None:
        print('[错误] 找不到 OpenOCD。跑 python tools/env_check.py 看看。')
        return 2

    print('目标固件 %s ｜ ps2 @ 0x%08X' % (target, ps2_addr))
    if times > 1:
        print('连读 %d 次，每次间隔 1 秒（推摇杆看数值变化）' % times)

    ok = False
    for i in range(times):
        if i:
            time.sleep(1.0)
            print('')
        ok = probe_once(target, oc, scr, ps2_addr, quiet=False)

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
