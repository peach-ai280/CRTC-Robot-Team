# -*- coding: utf-8 -*-
"""
build_cpp.py —— C++ 版固件的构建脚本

为什么要单独一个脚本，而不并进 build.py：
    C++ 要用 g++ 编、C 文件（启动文件 / HAL 兼容层）要用 gcc 编，
    两套编译器混着来，混在一起会把 build.py 搞得很乱。分开更清楚。

用法：
    python tools/build_cpp.py mcu      # 真机固件，产物 cpp/build/mcu/firmware.bin
    python tools/build_cpp.py sim      # 模拟器用的 ELF，给 run_cpp_sim.py 吃
    python tools/build_cpp.py all      # 两个都编

产物：
    mcu:  firmware.elf / .bin / .hex / .map   烧录用
    sim:  sim.elf                             模拟器用，不烧
"""

import os
import sys
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..')
FW   = os.path.join(ROOT, 'firmware')
CPP  = os.path.join(ROOT, 'cpp')
SIM  = os.path.join(HERE, 'sim')

TOOLS = ('D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/'
         'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.'
         '14.3.rel1.win32_1.0.100.202602081740/tools/bin')
CC   = os.path.join(TOOLS, 'arm-none-eabi-gcc.exe')
CXX  = os.path.join(TOOLS, 'arm-none-eabi-g++.exe')
OBJCOPY = os.path.join(TOOLS, 'arm-none-eabi-objcopy.exe')
SIZE    = os.path.join(TOOLS, 'arm-none-eabi-size.exe')

INC = [os.path.join(CPP, 'inc'), os.path.join(FW, 'Core', 'Inc')]

# C 语言公共部分：启动文件 + HAL 兼容层 + 板级初始化（只有真机需要）
C_FILES_MCU = ['startup_stm32f103xb.c', 'hal_shim.c', 'bsp.c', 'syscalls.c']

# C++ 源文件
CPP_FILES = ['chassis.cpp', 'motor.cpp', 'servo.cpp', 'arm.cpp',
             'vision.cpp', 'params.cpp']

CFLAGS = [
    '-mcpu=cortex-m3', '-mthumb', '-std=gnu11', '-Os', '-g3',
    '-ffunction-sections', '-fdata-sections',
    '-Wall', '-Wno-unused-parameter', '-DSTM32F103xB',
]

# C++ 必须关掉异常和 RTTI，否则会链进 libstdc++ 一大坨，64KB Flash 顶不住
CXXFLAGS = [
    '-mcpu=cortex-m3', '-mthumb', '-std=gnu++17', '-Os', '-g3',
    '-ffunction-sections', '-fdata-sections',
    '-fno-exceptions', '-fno-rtti',
    '-fno-threadsafe-statics', '-fno-use-cxa-atexit',
    '-Wall', '-Wno-unused-parameter', '-DSTM32F103xB',
]

LDFLAGS_MCU = [
    '-mcpu=cortex-m3', '-mthumb',
    '-T', os.path.join(FW, 'STM32F103C8TX_FLASH.ld'),
    '--specs=nano.specs',
    '-u', '_printf_float',          # 没有这个 printf("%f") 打出来是空的
    '-Wl,--gc-sections',
    '-Wl,--no-warn-rwx-segments',
    '-lm', '-lc',
]

LDFLAGS_SIM = [
    '-mcpu=cortex-m3', '-mthumb',
    '-T', os.path.join(SIM, 'sim.ld'),
    '-nostartfiles', '-e', 'main',
    '--specs=nano.specs',
    '-u', '_printf_float',
    '-Wl,--gc-sections',
    '-Wl,--no-warn-rwx-segments',
    '-lm', '-lc', '-lgcc',
]


def run(cmd):
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.returncode, p.stdout.decode('utf-8', errors='replace')


def inc_flags():
    out = []
    for d in INC:
        out += ['-I', d]
    return out


def compile_one(compiler, src, obj, flags):
    rc, out = run([compiler] + flags + inc_flags() + ['-c', src, '-o', obj])
    if rc != 0:
        print('  [编译失败] %s' % os.path.basename(src))
        print(out)
        return False
    for line in out.strip().splitlines():
        if 'warning' in line.lower():
            print('    警告 %s: %s' % (os.path.basename(src), line.strip()))
    return True


def build_mcu():
    outdir = os.path.join(CPP, 'build', 'mcu')
    os.makedirs(outdir, exist_ok=True)
    print('=' * 74)
    print('目标: mcu（C++ 真机固件）')
    print('=' * 74)

    objs = []
    for f in C_FILES_MCU:
        src = os.path.join(FW, 'Core', 'Src', f)
        obj = os.path.join(outdir, f.replace('.c', '.o'))
        if not compile_one(CC, src, obj, CFLAGS):
            return False
        objs.append(obj)

    for f in CPP_FILES + ['hal_mcu.cpp']:
        src = os.path.join(CPP, 'src', f)
        obj = os.path.join(outdir, f.replace('.cpp', '.o'))
        if not compile_one(CXX, src, obj, CXXFLAGS):
            return False
        objs.append(obj)

    src = os.path.join(CPP, 'main_mcu.cpp')
    obj = os.path.join(outdir, 'main_mcu.o')
    if not compile_one(CXX, src, obj, CXXFLAGS):
        return False
    objs.append(obj)

    elf  = os.path.join(outdir, 'firmware.elf')
    mapf = os.path.join(outdir, 'firmware.map')
    rc, out = run([CXX] + LDFLAGS_MCU + ['-Wl,-Map=' + mapf] + objs + ['-o', elf])
    if rc != 0:
        print('  [链接失败]'); print(out); return False
    if out.strip():
        print(out.strip())

    run([OBJCOPY, '-O', 'binary', elf, os.path.join(outdir, 'firmware.bin')])
    run([OBJCOPY, '-O', 'ihex',  elf, os.path.join(outdir, 'firmware.hex')])

    rc, out = run([SIZE, elf])
    print('  ' + out.strip().replace('\n', '\n  '))
    print('  产物: %s' % outdir)
    return True


def build_sim():
    outdir = os.path.join(CPP, 'build', 'sim')
    os.makedirs(outdir, exist_ok=True)
    print('=' * 74)
    print('目标: sim（模拟器离线验证）')
    print('=' * 74)

    objs = []
    for f in CPP_FILES + ['hal_sim.cpp']:
        src = os.path.join(CPP, 'src', f)
        obj = os.path.join(outdir, f.replace('.cpp', '.o'))
        if not compile_one(CXX, src, obj, CXXFLAGS):
            return False
        objs.append(obj)

    src = os.path.join(CPP, 'main_sim.cpp')
    obj = os.path.join(outdir, 'main_sim.o')
    if not compile_one(CXX, src, obj, CXXFLAGS):
        return False
    objs.append(obj)

    elf = os.path.join(outdir, 'sim.elf')
    # 库必须写在源文件后面：ld 是按顺序解析符号的
    rc, out = run([CXX] + LDFLAGS_SIM + objs + ['-o', elf])
    if rc != 0:
        print('  [链接失败]'); print(out); return False
    if out.strip():
        print(out.strip())

    print('  产物: %s' % elf)
    return True


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    arg = sys.argv[1]
    if arg == 'all':
        targets = ['mcu', 'sim']
    elif arg in ('mcu', 'sim'):
        targets = [arg]
    else:
        print('未知目标: %s，可选: mcu / sim / all' % arg)
        return 1

    if not os.path.exists(CXX):
        print('找不到编译器: %s' % CXX)
        print('请改 build_cpp.py 里的 TOOLS 路径')
        return 1

    ok = True
    for t in targets:
        if t == 'mcu':
            ok = build_mcu() and ok
        else:
            ok = build_sim() and ok
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
