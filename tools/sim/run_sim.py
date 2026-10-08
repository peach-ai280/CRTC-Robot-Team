# -*- coding: utf-8 -*-
"""
run_sim.py —— 在 ARM 模拟器（unicorn）里真跑一遍底盘运动学

不是"我照着公式又写了一遍"，而是把 firmware/Core/Src 下的 mecanum.c、
motor.c 原封不动编译成 ARM 机器码，让模拟器去执行，再把结果读出来判定。

判定三件事：
  1. 三个特例的方向对不对（纯前进 / 纯横移 / 纯自转）
  2. 归一化限幅有没有生效（四轮解算值的绝对值不能超过 1）
  3. 归一化有没有保住速度矢量方向（正解反算的角度 == 指令角度）

用法：
    python tools/sim/run_sim.py
"""

import os
import re
import subprocess
import struct
import math
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, '..', '..')
FW = os.path.join(ROOT, 'firmware')
SRC = os.path.join(FW, 'Core', 'Src')
INC = os.path.join(FW, 'Core', 'Inc')

TOOLS = ('D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/'
         'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.'
         '14.3.rel1.win32_1.0.100.202602081740/tools/bin')
CC = os.path.join(TOOLS, 'arm-none-eabi-gcc.exe')
OBJCOPY = os.path.join(TOOLS, 'arm-none-eabi-objcopy.exe')
NM = os.path.join(TOOLS, 'arm-none-eabi-nm.exe')

BASE = 0x08000000
PERIPH = 0x40000000
NCASE, NW = 8, 14

SRC_FILES = ['sim_kinematics.c', 'mecanum.c', 'motor.c', 'hal_shim.c']


def sh(cmd):
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.returncode, p.stdout.decode('utf-8', errors='replace')


def compile_sim():
    elf = os.path.join(HERE, 'sim.elf')
    srcs = [os.path.join(HERE, 'sim_kinematics.c')] + \
           [os.path.join(SRC, f) for f in ['mecanum.c', 'motor.c', 'hal_shim.c']]
    cmd = [CC, '-mcpu=cortex-m3', '-mthumb', '-std=gnu11', '-Os',
           '-I', INC, '-I', os.path.join(HERE),
           '-T', os.path.join(HERE, 'sim.ld'),
           '-nostartfiles', '-e', 'main',
           # 必须带 nano.specs，否则 -lm 会解析到 ARM 模式的库，
           # 报 "Unknown destination type (ARM/Thumb)"
           '--specs=nano.specs',
           '-Wl,--gc-sections', '-Wl,--no-warn-rwx-segments',
           ] + srcs + [
           # 库必须写在源文件后面，ld 是按顺序解析符号的
           '-lm', '-lc', '-lgcc',
           ] + ['-o', elf]
    rc, out = sh(cmd)
    if rc != 0:
        print('[编译失败]'); print(out); return None, None
    b = os.path.join(HERE, 'sim.bin')
    rc, out = sh([OBJCOPY, '-O', 'binary', elf, b])
    if rc != 0:
        print('[objcopy 失败]'); print(out); return None, None
    return elf, b


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


def run(binfile, main_addr):
    from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_PROT_ALL, UC_PROT_READ, UC_PROT_WRITE
    from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC

    mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    mu.mem_map(BASE, 0x80000, UC_PROT_ALL)                       # 代码 + 数据 + 栈
    mu.mem_map(PERIPH, 0x100000, UC_PROT_READ | UC_PROT_WRITE)   # 外设寄存器（当 RAM 用）
    mu.mem_map(0xE000E000, 0x10000, UC_PROT_READ | UC_PROT_WRITE)  # SysTick / NVIC

    with open(binfile, 'rb') as f:
        img = f.read()
    mu.mem_write(BASE, img)

    mu.reg_write(UC_ARM_REG_SP, BASE + 0x70000)
    # Cortex-M 是纯 Thumb，跳转地址最低位要为 1
    mu.emu_start(main_addr | 1, 0, count=80_000_000)
    return mu


def main():
    print('=' * 76)
    print('底盘运动学离线验证（ARM 模拟器执行真实固件代码）')
    print('=' * 76)

    elf, binf = compile_sim()
    if elf is None:
        return 1

    syms = symbols(elf)
    for need in ('main', 'g_out', 'g_done'):
        if need not in syms:
            print('符号表里找不到 %s' % need)
            return 1

    mu = run(binf, syms['main'])

    done = struct.unpack('<I', mu.mem_read(syms['g_done'], 4))[0]
    if done != 1:
        print('[失败] 程序没跑完（g_done=%d），可能是死循环或异常' % done)
        return 1

    raw = mu.mem_read(syms['g_out'], NCASE * NW * 4)
    g = []
    for c in range(NCASE):
        row = list(struct.unpack('<%df' % NW, raw[c * NW * 4:(c + 1) * NW * 4]))
        g.append(row)

    names = ['纯前进', '纯左移', '纯逆时针', '前进+自转', '斜走45度',
             '三轴全给', '前进-自转', '纯后退']

    print('\n用例                 指令(vx,vy,wz)      四轮解算值                        |w|max')
    print('-' * 76)
    fails, warns = [], []
    for c in range(NCASE):
        vx, vy, wz = g[c][0], g[c][1], g[c][2]
        w = g[c][3:7]
        mx = max(abs(x) for x in w)
        print('%-18s (%+.1f,%+.1f,%+.1f)   [%+.3f %+.3f %+.3f %+.3f]   %.3f'
              % (names[c], vx, vy, wz, w[0], w[1], w[2], w[3], mx))
        if mx > 1.001:
            fails.append('%s: 四轮解算值超过 1（归一化没生效，max=%.3f）' % (names[c], mx))

    # ---- 特例方向校验 ----
    print('\n方向校验（四轮符号必须满足的模式）')
    print('-' * 76)
    # 阈值不能按"满量程 1.0"来定：Chassis_SetTarget 会给每个轴乘 SPEED_GAIN，
    # 比如前进增益 0.85，纯前进时四轮解算值就是 0.85 而不是 1.0。
    # 所以这里只判"符号 + 一致性"，具体幅值交给增益决定。
    checks = [
        (0, '纯前进：四轮同号同值',
         lambda w: all(x > 0.4 for x in w) and (max(w) - min(w)) < 0.02),
        (1, '纯左移：LF/RB 负、RF/LB 正',
         lambda w: w[0] < -0.4 and w[3] < -0.4 and w[1] > 0.4 and w[2] > 0.4),
        (2, '纯逆时针：左两轮负、右两轮正',
         lambda w: w[0] < -0.3 and w[2] < -0.3 and w[1] > 0.3 and w[3] > 0.3),
        (7, '纯后退：四轮同为负',
         lambda w: all(x < -0.4 for x in w) and (max(w) - min(w)) < 0.02),
    ]
    for idx, desc, fn in checks:
        ok = fn(g[idx][3:7])
        print('  [%s] %s' % ('通过' if ok else '失败', desc))
        if not ok:
            fails.append('方向校验失败：%s' % desc)

    # ---- 归一化保方向 ----
    print('\n归一化是否保住速度矢量方向（正解反算 vs 指令）')
    print('-' * 76)
    print('用例                 指令角    增益后期望角   反算角    反算-期望   反算-指令')
    for c in range(NCASE):
        if abs(g[c][0]) < 1e-6 and abs(g[c][1]) < 1e-6:
            continue    # 纯自转用例没有平移方向可比
        a_cmd, a_exp, a_out = g[c][7], g[c][8], g[c][9]
        d1 = a_out - a_exp
        d2 = a_out - a_cmd
        print('%-18s %7.2f   %10.2f   %8.2f   %+8.3f   %+8.3f'
              % (names[c], a_cmd, a_exp, a_out, d1, d2))
        if abs(d1) > 0.5:
            fails.append('%s：归一化后方向偏了 %.2f 度（期望误差 <0.5 度）' % (names[c], d1))
        # d2 是"指令方向"和"实际方向"的差，它只由三个 SPEED_GAIN 不相等引起，
        # 不是归一化的锅。超过 1 度就提醒，但不算失败。
        if abs(d2) > 1.0:
            warns.append('%s：实际走的方向比指令偏了 %.2f 度 —— '
                         '原因是 SPEED_GAIN_FORWARD / STRAFE 不相等。'
                         '想要严格按指令方向走，把这两个增益设成一样。'
                         % (names[c], d2))

    # ---- PWM 实际输出 ----
    # 从 config.h 读四个轮的转向修正符号（顺序 LF/RF/LB/RB，对应 TIM4 的 CH1~CH4）
    dir_sign = [1, 1, 1, 1]
    cfg = open(os.path.join(INC, 'config.h'), encoding='utf-8').read()
    for i, k in enumerate(('MOTOR_LF_DIR', 'MOTOR_RF_DIR',
                           'MOTOR_LB_DIR', 'MOTOR_RB_DIR')):
        m = re.search(r'#define\s+' + k + r'\s+(-?\d+)', cfg)
        if m:
            dir_sign[i] = int(m.group(1))

    # ⚠ 这张表最容易看错，先说清楚口径：
    #   DRV8833 用的是 PHASE/ENABLE 接法（IN1=PWM，IN2=方向）。
    #   正转时 DIR=0，CCR =  duty    * 7200
    #   反转时 DIR=1，CCR = (1-duty) * 7200   <-- 所以反转的 CCR 反而小
    #   左右轮是镜像安装的（config.h 里 RF/RB 的 MOTOR_x_DIR = -1），
    #   因此"车往前走"时右两轮走的就是反转分支，CCR 会是 885 而不是 6314。
    #   **看到一大一小不要以为车跑偏了，要看下面的"有效占空比"列。**
    print('\n写进 TIM4->CCRx 的 PWM 比较值（周期 7200，CCR/7200 只是原始值）')
    print('-' * 76)
    print('%-18s %-28s %s' % ('用例', 'CCR[CH1 CH2 CH3 CH4]',
                              '有效占空比+转向（+正转 / -反转 / =静止刹车）'))
    for c in range(NCASE):
        pwm = g[c][10:14]
        w = g[c][3:7]
        eff, sign = [], []
        for i in range(4):
            s = w[i] * dir_sign[i]          # 经过转向修正后的轮速
            if s > 0.001:                   # DIR=0 正转
                eff.append(pwm[i] / 7200.0);      sign.append('+')
            elif s < -0.001:                # DIR=1 反转
                eff.append(1.0 - pwm[i] / 7200.0); sign.append('-')
            else:                           # 静止（按 MOTOR_IDLE_BRAKE 刹车或滑行）
                eff.append(0.0);                  sign.append('=')
        print('%-18s [%5.0f %5.0f %5.0f %5.0f]      '
              '[%s%.2f %s%.2f %s%.2f %s%.2f]'
              % (names[c], pwm[0], pwm[1], pwm[2], pwm[3],
                 sign[0], eff[0], sign[1], eff[1], sign[2], eff[2], sign[3], eff[3]))

    print('\n' + '=' * 76)
    if warns:
        print('提示（不算错误，但你可能想调）：')
        for w in warns:
            print('  · ' + w)
        print('')
    if fails:
        print('发现 %d 个问题：' % len(fails))
        for f in fails:
            print('  ! ' + f)
        return 1
    print('全部通过。')
    print('注意：这是模拟器执行固件代码的结果，证明算法逻辑自洽；')
    print('      真机上轮子转不转、转速够不够，还得看电机接线和实际标定。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
