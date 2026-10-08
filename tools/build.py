# -*- coding: utf-8 -*-
"""
build.py —— CRTC2026 固件构建脚本（代替 make）

本机没有 make，所以直接用 Python 调 arm-none-eabi-gcc。
编译器用的是 CubeIDE 2.2.0 自带的 GNU Tools for STM32 14.3.1。

用法：
    python tools/build.py full      # 完整固件（手柄 + 底盘 + 机构 + 视觉）
    python tools/build.py motion    # 只测底盘运动（串口发指令）
    python tools/build.py arm       # 只测舵机/机械臂
    python tools/build.py vision    # 只测视觉串口解析
    python tools/build.py all       # 四个都编

产物在 firmware/build/<target>/ 下：
    firmware.elf   调试用
    firmware.bin   串口/ST-Link 烧录用（起始地址 0x08000000）
    firmware.hex   CubeProgrammer 烧录用
    firmware.map   看占用多少 Flash / RAM
"""

import os
import sys
import subprocess
import shutil

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
FW = os.path.join(ROOT, 'firmware')
SRC = os.path.join(FW, 'Core', 'Src')
INC = os.path.join(FW, 'Core', 'Inc')

# CubeIDE 自带的 ARM 工具链。如果你的 CubeIDE 装在别处 / 版本不同，改这里。
TOOLS = ('D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/'
         'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.'
         '14.3.rel1.win32_1.0.100.202602081740/tools/bin')
CC = os.path.join(TOOLS, 'arm-none-eabi-gcc.exe')
OBJCOPY = os.path.join(TOOLS, 'arm-none-eabi-objcopy.exe')
SIZE = os.path.join(TOOLS, 'arm-none-eabi-size.exe')

# 每个目标要编的源文件（公共的三个会自动加上）
COMMON = ['hal_shim.c', 'startup_stm32f103xb.c', 'syscalls.c']

TARGETS = {
    'full': ['main.c', 'app.c', 'ps2.c', 'mecanum.c', 'motor.c',
             'servo.c', 'arm.c', 'sensor.c', 'vision.c',
             'auto_task.c', 'auto_full.c', 'bsp.c'],
    'motion': ['test_motion.c', 'mecanum.c', 'motor.c', 'sensor.c', 'bsp.c'],
    'arm': ['test_arm.c', 'servo.c', 'arm.c', 'bsp.c'],
    'ps2':     ['test_ps2.c', 'ps2.c', 'bsp.c'],
}

# 每个目标额外加的宏
EXTRA_FLAGS = {
    'vision': ['-DUSE_VISION=1'],   # 视觉测试固件必须开，否则 vision.c 是空实现
}

CFLAGS = [
    '-mcpu=cortex-m3', '-mthumb',
    '-std=gnu11', '-Os', '-g3',
    '-ffunction-sections', '-fdata-sections',
    '-Wall', '-Wno-unused-parameter', '-Wno-unused-variable',
    '-DSTM32F103xB',
]

LDFLAGS = [
    '-mcpu=cortex-m3', '-mthumb',
    '-T', os.path.join(FW, 'STM32F103C8TX_FLASH.ld'),
    '-Wl,--gc-sections',
    '--specs=nano.specs',
    '-u', '_printf_float',          # 没有这个，printf("%f") 打印出来是空的
    # 注意：不能加 -lnosys。libnosys 里有一份 _write/_read/_sbrk 的实体实现，
    # 会和我们 syscalls.c 里的实现打架，报 multiple definition。
    '-lm', '-lc',
    '-Wl,--no-warn-rwx-segments',
]


def run(cmd, cwd=None):
    p = subprocess.run(cmd, cwd=cwd, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT)
    return p.returncode, p.stdout.decode('utf-8', errors='replace')


def build(target):
    files = TARGETS[target] + COMMON
    outdir = os.path.join(FW, 'build', target)
    objects = []

    print('=' * 74)
    print('目标: %s' % target)
    print('=' * 74)

    os.makedirs(outdir, exist_ok=True)

    for f in files:
        src = os.path.join(SRC, f)
        obj = os.path.join(outdir, f.replace('.c', '.o'))
        objects.append(obj)
        if not os.path.exists(src):
            print('  [缺失] %s 不存在' % f)
            return False
        cmd = [CC] + CFLAGS + EXTRA_FLAGS.get(target, []) + ['-I', INC, '-c', src, '-o', obj]
        rc, out = run(cmd)
        if rc != 0:
            print('  [编译失败] %s' % f)
            print(out)
            return False
        if out.strip():
            for line in out.strip().splitlines():
                if 'warning' in line.lower():
                    print('    警告 %s: %s' % (f, line.strip()))

    elf = os.path.join(outdir, 'firmware.elf')
    mapf = os.path.join(outdir, 'firmware.map')
    ld = LDFLAGS + ['-Wl,-Map=' + mapf]
    rc, out = run([CC] + ld + objects + ['-o', elf])
    if rc != 0:
        print('  [链接失败]')
        print(out)
        return False
    if out.strip():
        print(out.strip())

    rc, out = run([OBJCOPY, '-O', 'binary', elf, os.path.join(outdir, 'firmware.bin')])
    rc, out = run([OBJCOPY, '-O', 'ihex', elf, os.path.join(outdir, 'firmware.hex')])

    rc, out = run([SIZE, elf])
    print('  ' + out.strip().replace('\n', '\n  '))
    print('  产物: %s' % outdir)
    return True


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1

    arg = sys.argv[1]
    targets = list(TARGETS.keys()) if arg == 'all' else [arg]

    if targets[0] not in TARGETS:
        print('未知目标: %s，可选: %s 或 all' % (arg, '/'.join(TARGETS)))
        return 1

    if not os.path.exists(CC):
        print('找不到编译器: %s' % CC)
        print('请改 build.py 里的 TOOLS 路径，指向你 CubeIDE 安装目录下的 tools/bin')
        return 1

    ok = True
    for t in targets:
        if not build(t):
            ok = False
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
