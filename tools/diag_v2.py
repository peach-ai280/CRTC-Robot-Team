#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
diag_v2.py —— firmware_v2 跑不起来时的根因诊断。

一次连接里读全套证据：
  PC / SP                卡在哪条指令
  RCC_CSR                复位标志（是不是 IWDG 看门狗复位）
  RCC_CR / RCC_CFGR      时钟有没有切到 PLL
  DBGMCU_CR              调试冻结位
  g_debug                主循环有没有跑过

用法：python tools/diag_v2.py
"""
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import flash

REG = {
    'RCC_CR':    0x40021000,
    'RCC_CFGR':  0x40021004,
    'RCC_CSR':   0x40021024,
    'DBGMCU_CR': 0xE0042004,
    'IWDG_KR':   0x40003000,
    'IWDG_PR':   0x40003004,
    'IWDG_RLR':  0x40003008,
    'IWDG_SR':   0x4000300C,
    'SYST_CSR':  0xE000E010,
    'SYST_RVR':  0xE000E014,
    'SYST_CVR':  0xE000E018,
    'GPIOB_CRH': 0x40010C04,
    'TIM4_CR1':  0x40000800,
    'TIM4_ARR':  0x4000082C,
}

# RCC_CSR 复位标志位
RST_FLAG = {
    26: 'PORRSTF  上电/掉电复位',
    27: 'PINRSTF  NRST 引脚复位',
    28: 'SFTRSTF  软件复位',
    29: 'IWDGRSTF ★独立看门狗复位',
    30: 'WWDGRSTF  窗口看门狗复位',
    31: 'LPWRRSTF 低功耗复位',
}


def oc_run(oc, scr, cmds, timeout=90):
    cmd = [oc, '-s', scr, '-f', flash.IFACE_CFG, '-f', flash.TARGET_CFG]
    for c in cmds:
        cmd += ['-c', c]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    return (p.stdout or '') + (p.stderr or '')


def mdw(out, addr, n=1):
    """从 openocd 输出里抠出 mdw 结果"""
    hexaddr = '%08x' % addr
    for m in re.finditer(r'0x(%s):((?:\s+[0-9a-fA-F]{8})+)' % hexaddr, out):
        return [int(w, 16) for w in m.group(2).split()][:n]
    return None


def main():
    oc = flash.find_openocd()
    if not oc[0]:
        print('[!] 找不到 OpenOCD')
        return 1
    oc, scr = oc[0], oc[1]

    cmds = ['init', 'halt', 'reg pc', 'reg sp', 'reg xpsr']
    for name, addr in REG.items():
        cmds.append('mdw 0x%08X 1' % addr)
    cmds.append('mdw 0x20000000 6')     # g_debug 24B
    cmds.append('resume')
    cmds.append('shutdown')

    for attempt in range(4):
        out = oc_run(oc, scr, cmds)
        if 'Cortex-M3' in out or 'device id' in out:
            break
        print('  连接失败，重试 %d/4 ...' % (attempt + 1))
        import time
        time.sleep(2)
    else:
        print('[!] 连不上。原始输出末尾：')
        print('\n'.join(out.splitlines()[-10:]))
        return 1

    print('=' * 60)
    print(' firmware_v2 运行诊断')
    print('=' * 60)

    # ---- PC / SP ----
    m = re.search(r'\bpc\s*\(/32\)\s*[:=]\s*(0x[0-9a-fA-F]+)', out)
    pc = int(m.group(1), 16) if m else None
    m = re.search(r'\bsp\s*\(/32\)\s*[:=]\s*(0x[0-9a-fA-F]+)', out)
    sp = int(m.group(1), 16) if m else None
    print('\n[程序位置]')
    print('  PC = %s' % ('0x%08X' % pc if pc is not None else '读不到'))
    print('  SP = %s' % ('0x%08X' % sp if sp is not None else '读不到'))
    if pc is not None:
        # 用 nm 把 PC 映射到最近的函数符号
        nm = os.path.join(os.path.dirname(flash.find_openocd()[0]),
                          'arm-none-eabi-nm.exe')
        syms = []
        elf = os.path.join(os.path.dirname(HERE), 'firmware_v2', 'build', 'firmware.elf')
        nm2 = os.path.join(
            'D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/'
            'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.'
            '14.3.rel1.win32_1.0.100.202602081740/tools/bin',
            'arm-none-eabi-nm.exe')
        if os.path.isfile(elf) and os.path.isfile(nm2):
            p = subprocess.run([nm2, '-n', elf], capture_output=True, text=True)
            for line in p.stdout.splitlines():
                parts = line.split()
                if len(parts) == 3:
                    try:
                        a = int(parts[0], 16)
                    except ValueError:
                        continue
                    syms.append((a, parts[2]))
            syms.sort()
            func = None
            for a, n in syms:
                if a <= pc:
                    func = (a, n)
                else:
                    break
            if func:
                print('  → 落在函数 %s + 0x%X  (%s)'
                      % (func[1], pc - func[0], '0x%08X' % func[0]))
        if pc < 0x08000000:
            print('  ⚠ PC 不在 Flash 里！HardFault 或启动失败')
        elif sp is not None and sp == 0x20005000:
            print('  ⚠ SP 还是复位初值 → 可能卡在重置循环/未进 main')

    # ---- 复位原因 ----
    print('\n[复位原因 RCC_CSR]')
    csr = mdw(out, REG['RCC_CSR'])
    if csr:
        v = csr[0]
        print('  RCC_CSR = 0x%08X' % v)
        hit = [txt for bit, txt in RST_FLAG.items() if v & (1 << bit)]
        for h in hit:
            print('    ✓ ' + h)
        if not hit:
            print('    (无复位标志)')
        if v & (1 << 29):
            print('  ★★★ 独立看门狗复位 = 程序卡在某个地方没喂狗')
    else:
        print('  读不到')

    # ---- 时钟 ----
    print('\n[时钟状态]')
    cr = mdw(out, REG['RCC_CR'])
    cfgr = mdw(out, REG['RCC_CFGR'])
    if cr and cfgr:
        c, f = cr[0], cfgr[0]
        print('  RCC_CR   = 0x%08X  HSIRDY=%d PLLRDY=%d'
              % (c, (c >> 1) & 1, (c >> 25) & 1))
        sw = f & 3
        sws = (f >> 2) & 3
        print('  RCC_CFGR = 0x%08X  SW=%d SWS=%d  PLLMUL=%d PLLSRC=%d'
              % (f, sw, sws, (f >> 18) & 0xF, (f >> 17) & 1))
        print('  → 时钟源: %s'
              % {0: 'HSI 8MHz', 1: 'HSE', 2: 'PLL 64MHz', 3: '备用'}.get(sws, '?'))
        if sws != 2:
            print('  ⚠ 时钟没切到 PLL（应在 64MHz）。clock_init() 可能失败了。')
    else:
        print('  读不到')

    # ---- 主循环有没有跑 ----
    print('\n[主循环 g_debug @0x20000000]')
    db = mdw(out, 0x20000000, 6)
    if db and len(db) >= 6:
        valid = db[0]
        invalid = db[1]
        last_ms = db[2]
        mode = db[3] & 0xFF
        flags = (db[3] >> 8) & 0xFF
        print('  valid_frames   = %d' % valid)
        print('  invalid_frames = %d' % invalid)
        print('  last_loop_ms   = %d' % last_ms)
        print('  ps2_mode       = 0x%02X' % mode)
        print('  connected=%d armed=%d emergency=%d'
              % (flags & 1, (flags >> 1) & 1, (flags >> 2) & 1))
        print()
        if valid == 0 and invalid == 0 and last_ms == 0:
            print('  ★★★ 全 0 = 主循环一次都没进去')
            print('      要么 board_init() 返回 false 卡在 for(;;)，')
            print('      要么一直在被看门狗复位（BSS 每次被清 0）。')
        elif invalid > 0 and valid == 0:
            print('  ★ 主循环在跑，但 PS2 一帧有效数据都没收到 → 纯 PS2 问题')
        elif valid > 0:
            print('  ★★★ PS2 通了，收到 %d 帧' % valid)
    else:
        print('  读不到')

    # ---- SysTick ----
    print('\n[SysTick 时基]')
    csr = mdw(out, REG['SYST_CSR'])
    rvr = mdw(out, REG['SYST_RVR'])
    cvr = mdw(out, REG['SYST_CVR'])
    if csr and rvr and cvr:
        c, r, v = csr[0], rvr[0], cvr[0]
        print('  SYST_CSR = 0x%08X  ENABLE=%d TICKINT=%d CLKSOURCE=%d'
              % (c, c & 1, (c >> 1) & 1, (c >> 2) & 1))
        print('  SYST_RVR = %d  (应 63999 = 64MHz/1000-1)' % r)
        print('  SYST_CVR = %d' % v)
        if not (c & 1):
            print('  ⚠ SysTick 没使能 → board_millis() 恒 0 → 主循环永不执行！')
        elif r == 0:
            print('  ⚠ RVR=0 → SysTick 不产生中断')
    else:
        print('  读不到')

    # ---- IWDG ----
    print('\n[独立看门狗 IWDG]')
    sr = mdw(out, REG['IWDG_SR'])
    pr = mdw(out, REG['IWDG_PR'])
    rlr = mdw(out, REG['IWDG_RLR'])
    if sr and pr and rlr:
        print('  IWDG_SR  = 0x%08X  PVU=%d RVU=%d WVU=%d'
              % (sr[0], sr[0] & 1, (sr[0] >> 1) & 1, (sr[0] >> 2) & 1))
        print('  IWDG_PR  = %d   IWDG_RLR = %d' % (pr[0], rlr[0]))
        if sr[0] != 0:
            print('  ★★★ IWDG_SR 非 0 = board_init() 会卡在等待循环里！')
            print('      这段 while(IWDG_SR!=0) 若一直等不到清零，')
            print('      board_init 返回 false → main 卡在 for(;;){} → 主循环永不进入')
    else:
        print('  读不到')

    # ---- DBGMCU ----
    d = mdw(out, REG['DBGMCU_CR'])
    if d:
        print('\n[调试冻结 DBGMCU_CR = 0x%08X]' % d[0])
        print('  DBG_IWDG_STOP(bit8) = %d  (1=调试停下时冻结看门狗)' % ((d[0] >> 8) & 1))

    return 0


if __name__ == '__main__':
    sys.exit(main())
