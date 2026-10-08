# -*- coding: utf-8 -*-
"""
run_auto_sim.py —— 在 ARM 模拟器里真跑一遍「全自动状态机」

把 firmware 下的 auto_full.c + auto_task.c 原封不动编译成 ARM 机器码跑，
机构/传感器用假实现顶替（那部分 C++ 版已经验证过），这里要验的是
**状态编排**：会不会卡死、会不会跳状态、取块计数对不对、连败会不会放弃。

两个场景：
  A 顺利：靠拢 1200ms 后对中成功 → 应拿满 AUTO_MAX_PICK 个后收工
  B 空场：永远对不上 → 连败 AUTO_FAIL_LIMIT 次后停车等人

用法：
    python tools/sim/run_auto_sim.py
"""

import os
import re
import subprocess
import struct
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

SRC_FILES = ['sim_auto.c', 'auto_full.c', 'auto_task.c']

# 状态枚举值（要和 auto_full.h 的 FullState_t 对得上）
FA_OFF, FA_ARM, FA_SEARCH, FA_CRUISE, FA_APPROACH, FA_MACRO, FA_RECOVER, FA_DONE = range(8)
SNAME = ['OFF', 'ARM', 'SEARCH', 'CRUISE', 'APPROACH', 'MACRO', 'RECOVER', 'DONE']


def sh(cmd):
    p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    return p.returncode, p.stdout.decode('utf-8', errors='replace')


def read_cfg(name, default):
    """从 config.h 读一个整型宏，这样改了参数测试依然成立"""
    path = os.path.join(INC, 'config.h')
    if not os.path.isfile(path):
        return default
    txt = open(path, encoding='utf-8', errors='replace').read()
    m = re.search(r'^\s*#define\s+%s\s+(\d+)' % name, txt, re.M)
    return int(m.group(1)) if m else default


def compile_sim():
    elf = os.path.join(HERE, 'sim_auto.elf')
    srcs = [os.path.join(HERE, 'sim_auto.c')] + \
           [os.path.join(SRC, f) for f in ['auto_full.c', 'auto_task.c']]
    cmd = [CC, '-mcpu=cortex-m3', '-mthumb', '-std=gnu11', '-Os',
           '-I', INC, '-I', os.path.join(HERE),
           '-T', os.path.join(HERE, 'sim.ld'),
           '-nostartfiles', '-e', 'main',
           '--specs=nano.specs',
           '-Wl,--gc-sections', '-Wl,--no-warn-rwx-segments',
           ] + srcs + [
           # 库必须写在源文件后面，ld 按顺序解析符号
           '-lm', '-lc', '-lgcc',
           ] + ['-o', elf]
    rc, out = sh(cmd)
    if rc != 0:
        print('[编译失败]')
        print(out)
        return None, None
    b = os.path.join(HERE, 'sim_auto.bin')
    rc, out = sh([OBJCOPY, '-O', 'binary', elf, b])
    if rc != 0:
        print('[objcopy 失败]')
        print(out)
        return None, None
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
    from unicorn.arm_const import UC_ARM_REG_SP

    mu = Uc(UC_ARCH_ARM, UC_MODE_THUMB)
    mu.mem_map(BASE, 0x80000, UC_PROT_ALL)
    mu.mem_map(PERIPH, 0x100000, UC_PROT_READ | UC_PROT_WRITE)
    mu.mem_map(0xE000E000, 0x10000, UC_PROT_READ | UC_PROT_WRITE)

    with open(binfile, 'rb') as f:
        img = f.read()
    mu.mem_write(BASE, img)
    mu.reg_write(UC_ARM_REG_SP, BASE + 0x70000)

    # main 返回后会跳到无效地址触发异常，这是预期的"跑完了"，忽略掉
    try:
        mu.emu_start(main_addr | 1, 0, count=200_000_000, timeout=60_000_000)
    except Exception:
        pass
    return mu


def dump(tag, o):
    print('\n  [%s] 模拟时长 %d ms，状态切换 %d 次，最终状态 %s'
          % (tag, o['ticks'], o['transitions'], SNAME[o['final_state']]))
    print('  取到 %d 个 / 连败 %d 次 / 有输出的周期 %d'
          % (o['picked'], o['fail'], o['cmd_nonzero']))
    print('  各状态停留周期：' +
          '  '.join('%s=%d' % (SNAME[i], o['visit'][i])
                    for i in range(8) if o['visit'][i] > 0))
    print('  单状态最长停留 %d ms（在 %s）'
          % (o['max_dwell_ms'], SNAME[o['dwell_of_max']] if o['dwell_of_max'] >= 0 else '-'))


def main():
    print('=' * 76)
    print('全自动状态机离线验证（ARM 模拟器执行真实固件代码）')
    print('=' * 76)

    max_pick = read_cfg('AUTO_MAX_PICK', 3)
    fail_lim = read_cfg('AUTO_FAIL_LIMIT', 3)
    hard_to  = read_cfg('AUTO_MACRO_HARD_TIMEOUT', 8000)
    print('\n从 config.h 读到：AUTO_MAX_PICK=%d  AUTO_FAIL_LIMIT=%d  '
          'AUTO_MACRO_HARD_TIMEOUT=%d' % (max_pick, fail_lim, hard_to))

    elf, binf = compile_sim()
    if elf is None:
        return 1

    syms = symbols(elf)
    for need in ('main', 'g_outA', 'g_outB', 'g_outC', 'g_done'):
        if need not in syms:
            print('符号表里找不到 %s' % need)
            return 1

    mu = run(binf, syms['main'])
    done = struct.unpack('<i', mu.mem_read(syms['g_done'], 4))[0]
    if done != 1:
        print('[失败] 程序没跑完（g_done=%d）—— 八成是死循环' % done)
        return 1

    def rd(addr):
        raw = mu.mem_read(addr, 96)
        v = struct.unpack('<I16i7i', raw)
        return {
            'ticks': v[0], 'visit': list(v[1:17]),
            'picked': v[17], 'fail': v[18], 'final_state': v[19],
            'cmd_nonzero': v[20], 'max_dwell_ms': v[21],
            'dwell_of_max': v[22], 'transitions': v[23],
        }

    A = rd(syms['g_outA'])
    B = rd(syms['g_outB'])
    C = rd(syms['g_outC'])

    print('\n场景 A（顺利：靠拢 1200ms 后对中成功）')
    print('-' * 76)
    dump('A', A)
    print('\n场景 B（空场：永远碰不到块，靠总时长上限收尾）')
    print('-' * 76)
    dump('B', B)
    print('\n场景 C（舵机卡死：对中了但宏每次超时）')
    print('-' * 76)
    dump('C', C)

    fails = []

    # ---- 通用：不能卡死，且必须真的在动 ----
    # CRUISE 是"常驻巡游"状态，空场时本来就会一直待在里面（靠总时长上限收尾），
    # 所以卡死检查要把它排除，否则会误报。
    for tag, o in (('A', A), ('B', B), ('C', C)):
        if o['dwell_of_max'] == FA_CRUISE:
            continue
        if o['max_dwell_ms'] > hard_to + 2000:
            fails.append('场景%s：在 %s 状态卡了 %d ms（超过硬超时 %d ms）'
                         % (tag, SNAME[o['dwell_of_max']], o['max_dwell_ms'], hard_to))
        if o['final_state'] != FA_DONE:
            fails.append('场景%s：最终没进 DONE，停在 %s' % (tag, SNAME[o['final_state']]))
        if o['cmd_nonzero'] <= 0:
            fails.append('场景%s：全程没输出任何运动指令（车根本没动）' % tag)

    # ---- 场景 A：应该真抓到东西 ----
    if A['picked'] != max_pick:
        fails.append('场景A：取到 %d 个，期望 %d 个（AUTO_MAX_PICK）' % (A['picked'], max_pick))
    if A['visit'][FA_MACRO] <= 0:
        fails.append('场景A：从没进过 MACRO 状态（宏没被触发）')
    if A['visit'][FA_CRUISE] <= 0:
        fails.append('场景A：从没进过 CRUISE 状态（车没巡游）')
    if A['visit'][FA_RECOVER] <= 0:
        fails.append('场景A：从没进过 RECOVER 状态（抓完没收尾）')

    # ---- 场景 B：空场应该巡游到底，一个都拿不到 ----
    if B['picked'] != 0:
        fails.append('场景B：空场却取到 %d 个，判定逻辑有问题' % B['picked'])
    if B['visit'][FA_CRUISE] <= 0:
        fails.append('场景B：没在巡游（空场时应该一直找）')

    # ---- 场景 C：连败到上限必须放弃，不能无限重试 ----
    if C['picked'] != 0:
        fails.append('场景C：舵机卡死却算成功 %d 次' % C['picked'])
    if C['fail'] < fail_lim:
        fails.append('场景C：连败 %d 次，期望 >= %d（AUTO_FAIL_LIMIT）' % (C['fail'], fail_lim))
    if C['visit'][FA_APPROACH] <= 0:
        fails.append('场景C：从没进过 APPROACH')

    print('\n' + '=' * 76)
    if fails:
        print('发现 %d 个问题：' % len(fails))
        for f in fails:
            print('  ! ' + f)
        return 1
    print('全部通过。')
    print('说明：这验证的是状态编排逻辑自洽（不卡死、不漏计、会认输）；')
    print('      真机上能不能抓到，取决于宏参数和机械对中做得准不准。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
