# -*- coding: utf-8 -*-
"""
run_cpp_sim.py —— 在 ARM 模拟器里跑 C++ 版的三块逻辑

不是"我照着公式又写了一遍"，而是把 cpp/src 下的 chassis.cpp、motor.cpp、
arm.cpp、vision.cpp 编成真正的 ARM 机器码，让模拟器去执行，再把 printf
出来的日志读回来。测的东西比 C 版多：连机构序列耗时和视觉帧解析一起测。

用法：
    python tools/sim/run_cpp_sim.py
"""

import os
import struct
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
CPP  = os.path.join(ROOT, 'cpp')

TOOLS = ('D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/'
         'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.'
         '14.3.rel1.win32_1.0.100.202602081740/tools/bin')
NM      = os.path.join(TOOLS, 'arm-none-eabi-nm.exe')
OBJCOPY = os.path.join(TOOLS, 'arm-none-eabi-objcopy.exe')

BASE   = 0x08000000
PERIPH = 0x40000000


def sh(cmd):
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.returncode, p.stdout.decode('utf-8', errors='replace')


def symbols(elf):
    rc, out = sh([NM, elf])
    syms = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3:
            try:
                syms[parts[2]] = int(parts[0], 16)
            except ValueError:
                pass
    return syms


def main():
    elf = os.path.join(CPP, 'build', 'sim', 'sim.elf')
    if not os.path.exists(elf):
        print('找不到 %s' % elf)
        print('先跑: python tools/build_cpp.py sim')
        return 1

    binf = os.path.join(CPP, 'build', 'sim', 'sim.bin')
    rc, out = sh([OBJCOPY, '-O', 'binary', elf, binf])
    if rc != 0:
        print('[objcopy 失败]'); print(out); return 1

    syms = symbols(elf)
    for need in ('main', 'g_done', 'g_log', 'g_logLen'):
        if need not in syms:
            print('符号表里找不到 %s' % need)
            return 1

    from unicorn import (Uc, UC_ARCH_ARM, UC_MODE_THUMB,
                         UC_PROT_ALL, UC_PROT_READ, UC_PROT_WRITE)
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC

    mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    mu.mem_map(BASE, 0x80000, UC_PROT_ALL)
    mu.mem_map(PERIPH, 0x100000, UC_PROT_READ | UC_PROT_WRITE)
    mu.mem_map(0xE000E000, 0x10000, UC_PROT_READ | UC_PROT_WRITE)

    with open(binf, 'rb') as f:
        img = f.read()
    mu.mem_write(BASE, img)

    mu.reg_write(UC_ARM_REG_SP, BASE + 0x70000)
    mu.emu_start(syms['main'] | 1, 0, count=200_000_000)   # Cortex-M 纯 Thumb

    done = struct.unpack('<I', mu.mem_read(syms['g_done'], 4))[0]
    n = struct.unpack('<I', mu.mem_read(syms['g_logLen'], 4))[0]
    log = bytes(mu.mem_read(syms['g_log'], min(n, 16384))).decode('utf-8', 'replace')

    print('=' * 78)
    print('CRTC2026 C++ 核心逻辑 · 离线验证（ARM 模拟器执行真实代码）')
    print('=' * 78)
    print(log.replace('\r\n', '\n'))

    if done != 1:
        print('[失败] 程序没跑完（g_done=%d），可能死循环了' % done)
        return 1

    if 'ALL PASS' in log:
        print('模拟器结论：全部通过。')
        print('注意：这只证明算法逻辑自洽；真机上轮子转不转、舵机会不会抖，')
        print('      还得烧进去实测。')
        return 0
    print('模拟器结论：有失败项，见上面的 [FAIL]。')
    return 1


if __name__ == '__main__':
    sys.exit(main())
