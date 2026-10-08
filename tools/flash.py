#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
一键烧录脚本（小白用这一个就够了）

用法（在 CRTC 目录下打开终端）：
    python tools/flash.py check     只检查 ST-Link 和芯片有没有连上（第一次先跑这个）
    python tools/flash.py motion    烧底盘测试固件
    python tools/flash.py ps2       烧 PS2 手柄测试固件（不初始化电机舵机，车不会动）
    python tools/flash.py arm       烧机构测试固件
    python tools/flash.py vision    烧视觉测试固件
    python tools/flash.py full      烧完整固件（打比赛用这个）

前置条件（缺一不可）：
    1. ST-Link 的 SWDIO / SWCLK / GND / 3V3 四根线接到开发板
    2. ST-Link 插电脑 USB
    3. 开发板供电（电池或 ST-Link 的 3V3 二选一，别同时供）
    4. BOOT0 跳线帽接在 0（GND 一侧）

烧录完程序会自动开始运行，不用按复位。
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# CubeIDE 自带的烧录工具。如果你的 CubeIDE 装在别处或版本不同，改这一行。
CLI_CANDIDATES = [
    r"D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
    r"\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304"
    r"\tools\bin\STM32_Programmer_CLI.exe",
]

# 每个目标对应的 bin 文件
BINS = {
    'full':   os.path.join(ROOT, 'firmware', 'build', 'full',   'firmware.bin'),
    'motion': os.path.join(ROOT, 'firmware', 'build', 'motion', 'firmware.bin'),
    'arm':    os.path.join(ROOT, 'firmware', 'build', 'arm',    'firmware.bin'),
    'ps2':    os.path.join(ROOT, 'firmware', 'build', 'ps2',    'firmware.bin'),
    'vision': os.path.join(ROOT, 'firmware', 'build', 'vision', 'firmware.bin'),
}

ADDR = '0x08000000'


def find_cli():
    for p in CLI_CANDIDATES:
        if os.path.isfile(p):
            return p
    # 兜底：扫一遍 D 盘的 CubeIDE 安装目录
    base = r"D:\ST"
    if os.path.isdir(base):
        for dirpath, _dirnames, filenames in os.walk(base):
            if 'STM32_Programmer_CLI.exe' in filenames:
                return os.path.join(dirpath, 'STM32_Programmer_CLI.exe')
    return None


def run(cmd, tag):
    print('-' * 60)
    print(tag)
    print('$ ' + ' '.join(cmd[:1] + cmd[1:]))
    print('-' * 60)
    r = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8',
                       errors='replace')
    out = (r.stdout or '') + (r.stderr or '')
    print(out)
    return r.returncode, out


def main():
    arg = sys.argv[1].lower() if len(sys.argv) > 1 else 'full'

    if arg in ('-h', '--help', 'help'):
        print(__doc__)
        return 0

    cli = find_cli()
    if cli is None:
        print('找不到 STM32_Programmer_CLI.exe。')
        print('请打开 D:\\ST 确认 CubeIDE 的安装目录名，然后修改本脚本的 CLI_CANDIDATES。')
        return 2
    print('烧录工具：%s' % cli)

    # ---------- check：只连线看看芯片在不在 ----------
    if arg == 'check':
        code, out = run([cli, '-c', 'port=SWD', 'freq=4000'], '检查连接')
        if 'Device ID' in out or 'STM32' in out:
            print('\n[OK] 芯片连上了。可以开始烧录：python tools/flash.py motion')
            return 0
        print('\n[失败] 没连上。按这个顺序排查：')
        print('  1. ST-Link 的灯亮不亮？不亮 = USB 没插好或线坏了')
        print('  2. SWDIO/SWCLK 接反没有？这两根最容易反')
        print('  3. 开发板有没有供电？只靠 ST-Link 的 3V3 也行，但别和电池同时供')
        print('  4. BOOT0 是不是接在 1 了？必须接 0')
        print('  5. 线是不是太长（>20cm）？太长会通信失败')
        return 1

    if arg == 'unlock':
        # 芯片被读保护锁住时的解锁。解锁会擦掉整片 Flash，属正常现象。
        code, out = run([cli, '-c', 'port=SWD', 'freq=4000', '-ob', 'RDP=0xAA'],
                        '解除读保护')
        if code == 0:
            print('\n[OK] 解锁完成（整片已擦除）。现在重新烧录即可。')
            return 0
        print('\n[失败] 解锁没成功，多半是连线问题。先跑 python tools/flash.py check')
        return 1

    if arg not in BINS:
        print('不认识的目标：%s' % arg)
        print('可选：check / unlock / ps2 / motion / arm / vision / full')
        return 2

    bin_path = BINS[arg]
    if not os.path.isfile(bin_path):
        print('找不到 %s' % bin_path)
        print('先编译：python tools/build.py %s' % arg)
        return 2

    # ---------- 烧录 ----------
    # -c port=SWD   用 SWD 接口（ST-Link 默认）
    # -w 文件 地址  写入
    # -v            写完后读回来比对，防止写坏
    # -s            烧完直接运行（不加这个要手动按复位）
    code, out = run([cli, '-c', 'port=SWD', 'freq=4000',
                     '-w', bin_path, ADDR, '-v', '-s'],
                    '烧录 %s -> %s' % (os.path.basename(bin_path), ADDR))

    if code == 0 and ('Download verified' in out or 'successfully' in out.lower()):
        print('\n[OK] 烧好了，程序已经在跑。')
        print('接下来：串口助手打开 COM 口，波特率 115200，看有没有打印。')
        return 0

    print('\n[失败] 退出码 %d。常见原因：' % code)
    print('  - 芯片被读保护锁住了：先跑一次 python tools/flash.py unlock')
    print('  - BOOT0 在 1：改回 0')
    print('  - 线太长或接触不良：换短线，重新插拔')
    return 1


if __name__ == '__main__':
    sys.exit(main())
