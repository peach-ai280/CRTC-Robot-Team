# -*- coding: utf-8 -*-
"""
env_check.py —— 一条命令查清「电脑上的开发环境到底搭全了没有」

用法（在 CRTC 目录下打开终端）：
    python tools/env_check.py          完整检查（含「ST-Link 能不能连上芯片」）
    python tools/env_check.py --fast   只查文件和驱动，不连芯片（快）

【为什么需要它】
    板子连不上时，最怕分不清到底是
        ① 电脑环境没搭全  还是  ② 那 4 根线没接
    这个脚本把这两件事**分开报**：
      凡是打 [OK] 的，就都不是电脑的问题；
      只有最后一项（连芯片）红了，说明电脑这边一切正常，去查线。

【不改动任何东西】纯只读检查，不会烧录、不会写文件。
"""

import glob
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
FW = os.path.join(ROOT, 'firmware')

# 复用 flash.py 里那套「找烧录程序 + 判断连没连上」的逻辑，保证两边口径一致
sys.path.insert(0, HERE)
try:
    import flash
except Exception:
    flash = None

GCC_CANDIDATES = [
    'D:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/'
    'com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.'
    '14.3.rel1.win32_1.0.100.202602081740/tools/bin/arm-none-eabi-gcc.exe',
]

OK, BAD, WARN = '[OK]', '[缺]', '[注意]'
items = []


def add(name, level, detail):
    items.append((name, level, detail))


def find_gcc():
    for p in GCC_CANDIDATES:
        if os.path.isfile(p):
            return p
    base = r'D:\ST'
    if os.path.isdir(base):
        for dp, _dn, fn in os.walk(base):
            if 'arm-none-eabi-gcc.exe' in fn:
                return os.path.join(dp, 'arm-none-eabi-gcc.exe')
    return None


def ps(query):
    """跑一段 PowerShell，拿它的标准输出（失败就返回空串，不抛异常）"""
    try:
        r = subprocess.run(
            ['powershell', '-NoProfile', '-NonInteractive', '-Command', query],
            capture_output=True, text=True, encoding='utf-8', errors='replace',
            timeout=25)
        return ((r.stdout or '') + (r.stderr or '')).strip()
    except Exception:
        return ''


def vlen(s):
    """按终端显示宽度算长度（中文占 2 列），用于对齐"""
    return sum(2 if ord(c) > 0x2E80 else 1 for c in s)


def pad(s, width):
    return s + ' ' * max(1, width - vlen(s))


def main():
    fast = '--fast' in sys.argv

    # ---------- 1. Python ----------
    add('Python 解释器', OK, '%s   (%s)' % (sys.version.split()[0], sys.executable))

    # ---------- 2. 交叉编译器 ----------
    gcc = find_gcc()
    if gcc:
        ver = '?'
        try:
            r = subprocess.run([gcc, '--version'], capture_output=True, text=True,
                               timeout=20)
            line = (r.stdout or '').splitlines()
            if line:
                ver = line[0].strip()
        except Exception:
            pass
        add('交叉编译器 arm-none-eabi-gcc', OK, ver)
    else:
        add('交叉编译器 arm-none-eabi-gcc', BAD,
            '没找到 —— CubeIDE 没装，或装到了非 D:\\ST 的目录')

    # ---------- 3. 烧录程序（OpenOCD 首选，CubeProgrammer 备选）----------
    # ⚠ 2026-10-13 实测：我们的 ST-LINK 是克隆版（固件 V2J29S7），
    #   CubeProgrammer 跟它握手会一直报 ST-LINK error (DEV_CONNECT_ERR)
    #   —— 这句话看着像"线接错了"，其实是工具本身兼容问题。
    #   同一块板、同一把线，OpenOCD 一次就连上。所以首选 OpenOCD。
    oc = scr = cli = None
    if flash is not None:
        oc, scr = flash.find_openocd()
        cli = flash.find_cli()
    if oc:
        add('烧录程序 OpenOCD（首选）', OK, oc)
    else:
        add('烧录程序 OpenOCD（首选）', BAD, '没找到 —— CubeIDE 没装全')
    if cli:
        add('烧录程序 CubeProgrammer（备选）', OK, cli)
    else:
        add('烧录程序 CubeProgrammer（备选）', WARN, '没找到（不影响，有 OpenOCD 就够）')

    # ---------- 4. 构建脚本 ----------
    bp = os.path.join(HERE, 'build.py')
    fp = os.path.join(HERE, 'flash.py')
    if os.path.isfile(bp) and os.path.isfile(fp):
        add('构建/烧录脚本', OK, 'tools/build.py  tools/flash.py 都在')
    else:
        add('构建/烧录脚本', BAD, 'tools/ 下缺文件，仓库不完整')

    # ---------- 5. 固件产物 ----------
    bins = sorted(glob.glob(os.path.join(FW, 'build', '*', 'firmware.bin')))
    if bins:
        names = []
        for b in bins:
            t = os.path.basename(os.path.dirname(b))
            sz = os.path.getsize(b)
            names.append('%s(%dKB)' % (t, (sz + 1023) // 1024))
        add('已编译好的固件', OK, '%d 份：%s' % (len(bins), '  '.join(names)))
    else:
        add('已编译好的固件', BAD, '一份都没有 —— 先跑 python tools/build.py all')

    # ---------- 6. ST-LINK 驱动 ----------
    out = ps("Get-PnpDevice -PresentOnly | Where-Object { "
             "$_.FriendlyName -match 'ST-?Link|STMicro' } | "
             "Select-Object -ExpandProperty FriendlyName")
    if 'STLink' in out or 'ST-Link' in out:
        first = [l for l in out.splitlines() if l.strip()]
        add('ST-LINK 被系统认到', OK, first[0].strip() if first else 'STM32 STLink')
    elif 'STM32' in out:
        add('ST-LINK 被系统认到', OK, out.splitlines()[0].strip())
    else:
        add('ST-LINK 被系统认到', BAD,
            '没认到 —— USB 没插紧，或 ST-LINK 驱动没装')

    # ---------- 7. 串口 ----------
    ports = ps("(Get-ItemProperty -Path 'HKLM:\\HARDWARE\\DEVICEMAP\\SERIALCOMM' "
               "-ErrorAction SilentlyContinue).PSObject.Properties | "
               "Where-Object { $_.Name -notlike 'PS*' } | "
               "ForEach-Object { $_.Name + '=' + $_.Value }")
    ports = [l.strip() for l in ports.splitlines() if l.strip()]
    if ports:
        add('串口（COM 口）', OK, ' '.join(ports))
    else:
        add('串口（COM 口）', WARN,
            '0 个 —— 想看串口打印/发指令，需要插一个 USB 转串口模块(CH340)')

    # ---------- 8. 连芯片（最关键的一项）----------
    if fast:
        add('ST-LINK 能连上芯片', WARN, '本次 --fast 跳过')
    elif oc:
        out = ''
        try:
            r = subprocess.run(flash._oc_cmd(oc, scr, ['init', 'flash probe 0', 'exit']),
                               capture_output=True, text=True, encoding='utf-8',
                               errors='replace', timeout=40)
            out = (r.stdout or '') + (r.stderr or '')
        except Exception:
            out = ''
        if flash.oc_ok(out):
            det = []
            m = re.search(r'STLINK\s+(\S+)', out)
            if m:
                det.append('探针 %s' % m.group(1))
            m = re.search(r'Target voltage:\s*([\d.]+)', out)
            if m:
                det.append('电压 %sV' % m.group(1))
            m = re.search(r'device id = (\S+)', out)
            if m:
                det.append('ID %s' % m.group(1))
            m = re.search(r'flash size = (\S+ \S+)', out)
            if m:
                det.append('Flash %s' % m.group(1))
            add('ST-LINK 能连上芯片', OK, ' ｜ '.join(det) if det else '连上了')
        else:
            add('ST-LINK 能连上芯片', BAD,
                '连不上 —— 查那 4 根线（3V3/GND/SWDIO/SWCLK）')
    elif cli:
        out = ''
        try:
            r = subprocess.run([cli, '-c', 'port=SWD', 'freq=4000', 'mode=normal'],
                               capture_output=True, text=True, encoding='utf-8',
                               errors='replace', timeout=40)
            out = (r.stdout or '') + (r.stderr or '')
        except Exception as e:
            out = 'ERROR %s' % e
        connected = flash.conn_ok(out) if flash else ('Device ID' in out)
        if connected:
            m = re.search(r'Device\s*ID\s*:\s*(\S+)', out)
            add('ST-LINK 能连上芯片', OK,
                'Device ID = %s' % (m.group(1) if m else '?'))
        elif 'DEV_CONNECT_ERR' in out:
            add('ST-LINK 能连上芯片', BAD,
                'DEV_CONNECT_ERR —— 板子没应答：查那 4 根线')
        elif 'No ST-LINK' in out:
            add('ST-LINK 能连上芯片', BAD, 'No ST-LINK detected —— USB 问题')
        else:
            add('ST-LINK 能连上芯片', BAD, '连不上（看上面的原始输出）')
    else:
        add('ST-LINK 能连上芯片', BAD, '烧录程序都找不到，无法测试')

    # ---------- 打印 ----------
    print()
    print('=' * 66)
    print('  CRTC 单片机开发环境体检')
    print('=' * 66)
    for i, (name, level, detail) in enumerate(items):
        print('  %s %s%s' % (level, pad(name, 30), detail))
    print('-' * 66)

    bad = [x for x in items if x[1] == BAD]
    if not bad:
        print('  结论：电脑这边全部齐活，可以开始烧录。')
    elif len(bad) == 1 and bad[0][0] == 'ST-LINK 能连上芯片':
        print('  结论：★ 电脑环境是齐全的 ★ —— 打 [OK] 的项全都跑得起来。')
        print('        唯一没过的是最后一项，那属于「ST-LINK 到板子」的硬件接线，')
        print('        不是电脑环境问题。检查：')
        print('          1) 那 4 根线接上了吗？（3V3 / GND / SWDIO / SWCLK）')
        print('          2) SWDIO 和 SWCLK 是不是接反了？（最常见）')
        print('          3) 板子供电了吗？（可以只靠 ST-LINK 的 3V3）')
        print('        ⚠ 顺带一句：CubeProgrammer 单跑报 DEV_CONNECT_ERR 不代表线错，')
        print('          我们的克隆版 ST-LINK 它本来就握不上手 —— 以本项为准。')
    else:
        print('  结论：上面打 [缺] 的项要先补齐，其余没问题。')
    print('=' * 66)
    print()

    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
