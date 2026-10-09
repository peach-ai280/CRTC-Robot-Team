# -*- coding: utf-8 -*-
"""
build_cpp.py —— C++ 版固件的构建脚本

为什么要单独一个脚本，而不并进 build.py：
    C++ 要用 g++ 编、C 文件（启动文件 / HAL 兼容层）要用 gcc 编，
    两套编译器混着来，混在一起会把 build.py 搞得很乱。分开更清楚。

用法：
    python tools/build_cpp.py a        # A 车固件（麦轮 kMecanum），产物 cpp/build/mcu/
    python tools/build_cpp.py b        # B 车固件（橡胶轮差速 kDiff），产物 cpp/build/mcu_b/
    python tools/build_cpp.py sim      # 模拟器用的 ELF，给 run_cpp_sim.py 吃
    python tools/build_cpp.py all      # 三个都编

产物：
    a:    cpp/build/mcu/firmware.elf / .bin / .hex / .map     烧录 A 车
    b:    cpp/build/mcu_b/firmware.elf / .bin / .hex / .map   烧录 B 车
    sim:  cpp/build/sim/sim.elf                               模拟器用，不烧

★ 两车差别的全部内容就是：CXXFLAGS 里多/少一个 -DCRTC_CHASSIS_KIND=0 或 1。
  源文件一个字节都不差 —— 报"底盘运动（全部）"时，这就是代码层面的证据。
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

# ★ 车型运动学内核开关（对应 crtc.hpp 的 kChassisKind）
#   A 车 = 0（kMecanum 麦轮）；B 车 = 1（kDiff 橡胶轮差速）
CXXFLAGS_A = CXXFLAGS + ['-DCRTC_CHASSIS_KIND=0']
CXXFLAGS_B = CXXFLAGS + ['-DCRTC_CHASSIS_KIND=1']

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


def build_mcu(kind='a'):
    """kind='a' -> A 车（麦轮）；kind='b' -> B 车（橡胶轮差速）。

    两套的差别**只有一个编译宏** -DCRTC_CHASSIS_KIND=0/1，
    源文件完全一样。产物分别落在 build/mcu 和 build/mcu_b，互不覆盖。
    这么做的意义：两台车共用同一份代码 + 同一份参数表，
    只有运动学那一行不同 —— 报"底盘运动（全部）"时这是最干净的代码证据。
    """
    extra = CXXFLAGS_A if kind == 'a' else CXXFLAGS_B
    name  = 'A 车 麦轮' if kind == 'a' else 'B 车 橡胶轮差速'

    outdir = os.path.join(CPP, 'build', 'mcu' if kind == 'a' else 'mcu_b')
    os.makedirs(outdir, exist_ok=True)
    print('=' * 74)
    print('目标: mcu%s（C++ 真机固件 · %s）' % ('' if kind == 'a' else '_b', name))
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
        if not compile_one(CXX, src, obj, extra):
            return False
        objs.append(obj)

    src = os.path.join(CPP, 'main_mcu.cpp')
    obj = os.path.join(outdir, 'main_mcu.o')
    if not compile_one(CXX, src, obj, extra):
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


def build_sim(kind='a'):
    """模拟器 ELF。kind 决定编进哪套运动学内核，理由见下面注释。"""
    extra = CXXFLAGS_A if kind == 'a' else CXXFLAGS_B
    name  = 'A 车 麦轮' if kind == 'a' else 'B 车 橡胶轮差速'

    outdir = os.path.join(CPP, 'build', 'sim' if kind == 'a' else 'sim_b')
    os.makedirs(outdir, exist_ok=True)
    print('=' * 74)
    print('目标: sim%s（模拟器离线验证 · %s）' % ('' if kind == 'a' else '_b', name))
    print('=' * 74)

    objs = []
    for f in CPP_FILES + ['hal_sim.cpp']:
        src = os.path.join(CPP, 'src', f)
        obj = os.path.join(outdir, f.replace('.cpp', '.o'))
        if not compile_one(CXX, src, obj, extra):
            return False
        objs.append(obj)

    src = os.path.join(CPP, 'main_sim.cpp')
    obj = os.path.join(outdir, 'main_sim.o')
    if not compile_one(CXX, src, obj, extra):
        return False
    objs.append(obj)

    # ⚠ 产物名固定叫 sim.elf，因为 run_cpp_sim.py 就认这个名。
    #   A/B 两份落在不同目录，不会互相覆盖。
    #   kinematics() 是编译期分派的（-Os 会把常量条件彻底折掉），
    #   所以"用同一份 sim 测两台车"是测不出来的 —— 必须编两份、各跑一遍。
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
        targets = ['a', 'b', 'sim', 'sim_b']
    elif arg in ('a', 'b', 'sim', 'sim_b'):
        targets = [arg]
    else:
        print('未知目标: %s，可选: a(A车麦轮) / b(B车差速) / sim(A车模拟) / sim_b(B车模拟) / all' % arg)
        return 1

    if not os.path.exists(CXX):
        print('找不到编译器: %s' % CXX)
        print('请改 build_cpp.py 里的 TOOLS 路径')
        return 1

    ok = True
    for t in targets:
        if t in ('a', 'b'):
            ok = build_mcu(t) and ok
        elif t == 'sim_b':
            ok = build_sim('b') and ok
        else:
            ok = build_sim('a') and ok
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
