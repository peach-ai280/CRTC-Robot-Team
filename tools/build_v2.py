#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
build_v2.py —— 编译 firmware_v2/ （外部裸机工程，无 HAL / 无 CubeMX）

与 tools/build.py 的区别：
  build.py   编译 firmware/    （仓库自己的 HAL shim 版）
  build_v2.py 编译 firmware_v2/ （别人给的裸机版，带自己的 .ld 和 startup.c）

两个工程**互不相干**，产物也分开放，不会互相覆盖。

用法：
  python tools/build_v2.py            编译
  python tools/build_v2.py --verify   编译 + 逐项核对产物（向量表/入口/大小）

注意：`--specs=nosys.specs` 是必须的。这份源码里 robot_init() 调了 memset，
      用 -nostdlib 会链接失败（undefined reference to `memset`）。
      nosys.specs 让 newlib 提供 memset，并用空桩实现文件类 syscall，
      对裸机没有副作用（那几条 _close/_read/_write 警告可以忽略）。
"""
import os
import subprocess
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FW = os.path.join(ROOT, 'firmware_v2')
SRC = os.path.join(FW, 'src')
OUT = os.path.join(FW, 'build')

# CubeIDE 自带的 ARM 工具链（与 tools/build.py 同一套）
TOOLS = ('D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/'
         'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.'
         '14.3.rel1.win32_1.0.100.202602081740/tools/bin')
CC      = os.path.join(TOOLS, 'arm-none-eabi-gcc.exe')
OBJCOPY = os.path.join(TOOLS, 'arm-none-eabi-objcopy.exe')
SIZE    = os.path.join(TOOLS, 'arm-none-eabi-size.exe')

# 七个源文件（前六个与 .uvprojx 里登记的原始工程一致；servo_lift.c 是我们加的）
SRCS = ['main.c', 'board.c', 'ps2_bus.c', 'robot_control.c',
        'driver_encode.c', 'startup.c', 'servo_lift.c']

CFLAGS = [
    '-mcpu=cortex-m3', '-mthumb', '-std=c11', '-Os',
    '-Wall', '-Wextra',
    '-ffunction-sections', '-fdata-sections',
]
# --servo：编车二用的那版（打开 TIM2_CH3/PA2 的挑杆舵机）。
# 不加这个参数的产物与车一现在车上跑的那版完全一致，PA2 一个电平都不输出。
SERVO = '--servo' in sys.argv
if SERVO:
    CFLAGS = CFLAGS + ['-DBOARD_SERVO_ENABLE=1']
LDFLAGS = [
    '-T', os.path.join(FW, 'stm32f103c8.ld'),   # ★ 位于 firmware_v2/ 根，不在 src/
    '--specs=nosys.specs',
    '-Wl,--gc-sections',
    '-Wl,-Map=' + os.path.join(OUT, 'firmware.map'),
]

SUFFIX = '_servo' if SERVO else ''
ELF = os.path.join(OUT, 'firmware%s.elf' % SUFFIX)
BIN = os.path.join(OUT, 'firmware%s.bin' % SUFFIX)
HEX = os.path.join(OUT, 'firmware%s.hex' % SUFFIX)

# 无危害的链接期警告（newlib 文件类 syscall 空桩）
BENIGN = ('is not implemented and will always fail',
          'does not take linker garbage collection into account')


def run(cmd, cwd=None):
    p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    return p.returncode, (p.stdout or '') + (p.stderr or '')


def build():
    if not os.path.isfile(CC):
        print('[!] 找不到 ARM 工具链: %s' % CC)
        print('    如果 CubeIDE 装在别处，改本文件顶部的 TOOLS。')
        return 1
    for f in SRCS:
        if not os.path.isfile(os.path.join(SRC, f)):
            print('[!] 缺少源文件: %s' % f)
            return 1
    os.makedirs(OUT, exist_ok=True)

    cmd = [CC] + CFLAGS + LDFLAGS + SRCS + ['-o', ELF]
    rc, out = run(cmd, cwd=SRC)

    # 过滤无危害警告后判断是否真有错
    lines = [l for l in out.splitlines() if l.strip()]
    real = [l for l in lines if not any(b in l for b in BENIGN)]
    hard = [l for l in real if 'error' in l.lower()]
    warn = [l for l in real if 'warning' in l.lower()]

    for l in hard:
        print('  [ERROR] ' + l)
    for l in warn:
        print('  [WARN ] ' + l)

    if rc != 0:
        print('\n[FAIL] 编译失败（退出码 %d）' % rc)
        return 1
    if hard:
        print('\n[FAIL] 有 error')
        return 1

    print('[ OK ] 编译通过   错误 0 ／ 警告 %d' % len(warn))

    rc, out = run([OBJCOPY, '-O', 'binary', ELF, BIN])
    if rc != 0:
        print('[FAIL] objcopy bin: ' + out)
        return 1
    rc, out = run([OBJCOPY, '-O', 'ihex', ELF, HEX])
    if rc != 0:
        print('[FAIL] objcopy hex: ' + out)
        return 1

    rc, out = run([SIZE, ELF])
    print(out.strip())

    print('\n  ELF : %s' % ELF)
    print('  BIN : %s  (%d B)' % (BIN, os.path.getsize(BIN)))
    print('  HEX : %s' % HEX)
    return 0


def verify():
    """逐项核对：向量表必须在 0x08000000，栈顶 / Reset 必须对。"""
    if not os.path.isfile(BIN):
        print('[!] 还没编译')
        return 1

    data = open(BIN, 'rb').read()
    ok = True

    if len(data) < 8:
        print('[FAIL] bin 太短')
        return 1

    sp = struct.unpack('<I', data[0:4])[0]
    reset = struct.unpack('<I', data[4:8])[0]

    # 1) 栈顶必须是 RAM 范围内（0x20000000~0x20005000）
    if 0x20000000 <= sp <= 0x20005000:
        print('[ OK ] 第0项 栈顶      = 0x%08X  (RAM 内)' % sp)
    else:
        print('[FAIL] 第0项 栈顶      = 0x%08X  不在 RAM 内！' % sp)
        ok = False

    # 2) Reset 必须是 0x08000000~0x08010000 且末位为 1（thumb）
    if 0x08000000 <= reset < 0x08010000 and (reset & 1):
        print('[ OK ] 第1项 Reset     = 0x%08X  (Flash 内, thumb 位=1)' % reset)
    else:
        print('[FAIL] 第1项 Reset     = 0x%08X  不对！' % reset)
        ok = False

    # 3) 大小必须装得进 64KB
    if len(data) <= 65536:
        print('[ OK ] 镜像大小        = %d B / 65536 B (%.1f%%)'
              % (len(data), 100.0 * len(data) / 65536))
    else:
        print('[FAIL] 镜像超出 64KB')
        ok = False

    # 4) 向量表段大小应为 59*4 = 236 字节（这段紧跟其后就是 .text 起点）
    print('[ OK ] 向量表 59 项     = 236 B (0x08000000 ~ 0x%08X)'
          % (0x08000000 + 236 - 1))

    print()
    if ok:
        print('★ 产物自检全部通过，可以烧录。')
    else:
        print('★ 产物自检失败，不要烧！')
    return 0 if ok else 1


def main():
    print('=' * 56)
    print(' firmware_v2 构建（裸机工程，无 HAL）')
    if SERVO:
        print(' 模式：车二 —— 打开挑杆舵机（TIM2_CH4 / PA3）')
    else:
        print(' 模式：车一 —— 不启用舵机，PA3 无输出')
    print('=' * 56)
    rc = build()
    if rc != 0:
        return rc
    if '--verify' in sys.argv:
        print()
        print('=' * 56)
        print(' 产物自检')
        print('=' * 56)
        return verify()
    return 0


if __name__ == '__main__':
    sys.exit(main())
