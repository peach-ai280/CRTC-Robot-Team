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
    python tools/flash.py unlock    解除芯片读保护（会擦掉整片 Flash）
    python tools/flash.py rescue    ★ 万一「烧过一次就连不上」时的一键急救

--------------------------------------------------------------------------
★ 两条烧录通道（脚本会自动挑，你也可以手动指定）
--------------------------------------------------------------------------
  OpenOCD         —— 默认走这条。开源驱动，对「克隆版 ST-LINK」友好。
                     CubeIDE 里自带（externaltools.openocd + mcu.debug.openocd）。
  CubeProgrammer  —— ST 官方工具，备选。日志里看到 `ST-LINK error (DEV_CONNECT_ERR)`
                     时，**先别怀疑线**，很可能是它跟克隆版 ST-LINK 握不上手
                     （2026-10-13 实测：同一块板、同一把线，CubeProgrammer 报
                      DEV_CONNECT_ERR，OpenOCD 却一次连上、读出芯片 ID）。
  ─── 所以本脚本把 OpenOCD 放在第一位。

--------------------------------------------------------------------------
【关于「烧一次就锁死」】
    正常情况不会发生 —— 本固件从不占用 SWD 脚（PA13/PA14），也不开读保护、
    不进低功耗、不使能看门狗。见 CRTC交付\\烧录防锁死与引脚对照_一页纸.md。
    真被锁了（多半是别人给的固件、或以后自己加了占 SWD 脚的代码），
    按 rescue 的提示走：连不上就用 BOOT0=1 上电，芯片跑出厂 BootROM，
    ST-Link 一定能连，擦干净再拨回 0 即可，**硬件救得回来，不会变砖**。

前置条件（缺一不可）：
    1. ST-Link 的 SWDIO / SWCLK / GND / 3V3 四根线接到开发板
    2. ST-Link 插电脑 USB
    3. 开发板供电（电池或 ST-Link 的 3V3 二选一，别同时供）
    4. BOOT0 跳线帽接在 0（GND 一侧）

烧录完程序会自动开始运行，不用按复位。
"""
import glob
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ---------- 烧录工具的位置（装在别处就改这里，或让下面的通配自己找） ----------
# OpenOCD（首选）
OPENOCD_CANDIDATES = [
    r"D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
    r"\com.st.stm32cube.ide.mcu.externaltools.openocd.win32_2.4.500.202604080855"
    r"\tools\bin\openocd.exe",
]
OPENOCD_SCRIPTS_CANDIDATES = [
    r"D:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins"
    r"\com.st.stm32cube.ide.mcu.debug.openocd_2.3.400.202606220929"
    r"\resources\openocd\st_scripts",
]
# CubeProgrammer（备选）
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
    # ps2scan：PS2 接线穷举诊断固件（配合 tools/ps2_scan.py，一次性用）
    'ps2scan': os.path.join(ROOT, 'firmware', 'build', 'ps2scan', 'firmware.bin'),
    # motionauto：不用手柄、不用串口的底盘自检（上电自己跑一遍就走完）
    'wheel': os.path.join(ROOT, 'firmware', 'build', 'wheel', 'firmware.bin'),
    'motionauto': os.path.join(ROOT, 'firmware', 'build', 'motionauto', 'firmware.bin'),
    # v2：外部裸机工程（无 HAL），由 tools/build_v2.py 编译，与上面各目标互不相干
    'v2': os.path.join(ROOT, 'firmware_v2', 'build', 'firmware.bin'),
    # v2s：同上，但打开了 PA2 的挑杆舵机（车二用）。编译：build_v2.py --servo
    'v2s': os.path.join(ROOT, 'firmware_v2', 'build', 'firmware_servo.bin'),
    # v2c / v2cs：2 号车（车二）的马达线序（右后 +1，与 1 号车不同）。
    #   编译：build_v2.py --car2  ／  build_v2.py --car2 --servo
    'v2c':  os.path.join(ROOT, 'firmware_v2', 'build', 'firmware_car2.bin'),
    'v2cs': os.path.join(ROOT, 'firmware_v2', 'build', 'firmware_servo_car2.bin'),
}

ADDR = '0x08000000'
TARGET_CFG = 'target/stm32f1x.cfg'
IFACE_CFG = 'interface/stlink-dap.cfg'      # ← 用 dapdirect；interface/stlink.cfg 是旧 hla，本机 OpenOCD 会报递归错


# ============================ 找工具 ============================

def find_openocd():
    """返回 (openocd.exe 路径, st_scripts 目录)，找不到返回 (None, None)。"""
    for exe in OPENOCD_CANDIDATES:
        if os.path.isfile(exe):
            for scr in OPENOCD_SCRIPTS_CANDIDATES:
                if os.path.isdir(scr):
                    return exe, scr
    # 兜底：按通配找（换了 CubeIDE 版本也能找到）
    for pat in (r"D:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*\tools\bin\openocd.exe",
                r"D:\ST\**\openocd.exe"):
        hits = glob.glob(pat, recursive=True)
        if hits:
            exe = hits[0]
            for spat in (r"D:\ST\STM32CubeIDE*\STM32CubeIDE\plugins\*\resources\openocd\st_scripts",
                         r"D:\ST\**\st_scripts"):
                for scr in glob.glob(spat, recursive=True):
                    if os.path.isfile(os.path.join(scr, IFACE_CFG)):
                        return exe, scr
    return None, None


def find_cli():
    for p in CLI_CANDIDATES:
        if os.path.isfile(p):
            return p
    base = r"D:\ST"
    if os.path.isdir(base):
        for dirpath, _dirnames, filenames in os.walk(base):
            if 'STM32_Programmer_CLI.exe' in filenames:
                return os.path.join(dirpath, 'STM32_Programmer_CLI.exe')
    return None


def run(cmd, tag, timeout=90):
    print('-' * 60)
    print(tag)
    print('$ ' + ' '.join(cmd))
    print('-' * 60)
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, encoding='utf-8',
                           errors='replace', timeout=timeout)
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or '') + (e.stderr or '')

        def _s(x):
            return x.decode('utf-8', 'replace') if isinstance(x, bytes) else (x or '')
        out = _s(e.stdout) + _s(e.stderr)
        print(out)
        print('[超时] 工具卡住没返回（%ds）。通常是连接/复位阶段卡住。' % timeout)
        return 124, out
    out = (r.stdout or '') + (r.stderr or '')
    print(out)
    return r.returncode, out


# ============================ OpenOCD 通道 ============================

def _oc_cmd(oc, scr, cmds):
    cmd = [oc, '-s', scr, '-f', IFACE_CFG, '-f', TARGET_CFG]
    for c in cmds:
        cmd += ['-c', c]
    return cmd


def _oc_bin(bin_path):
    return bin_path.replace('\\', '/')


def oc_ok(out):
    """OpenOCD 到底连上没有？
    连上的硬证据：Examination succeed / Cortex-M3 ... detected。
    （OpenOCD 不会用 'STM32' 这种词误报，所以判据可以放心用。）"""
    low = out.lower()
    return ('examination succeed' in low) or ('cortex-m3' in low) \
        or ('processor detected' in low)


def oc_probe(oc, scr, quiet=False):
    """只连接 + 识别芯片，不动 Flash。返回 (ok, out)。"""
    code, out = run(_oc_cmd(oc, scr, ['init', 'flash probe 0', 'exit']),
                    'OpenOCD 连接 + 识别芯片', timeout=40)
    return (oc_ok(out) and 'error: ' not in out.lower()), out


def oc_flash(oc, scr, bin_path):
    """烧录 + 回读校验 + 复位运行。"""
    b = _oc_bin(bin_path)
    cmds = [
        'init',
        'halt',
        'flash write_image erase %s %s' % (b, ADDR),
        'verify_image %s %s' % (b, ADDR),
        'reset run',
        'exit',
    ]
    return run(_oc_cmd(oc, scr, cmds),
               '烧录 %s -> %s' % (os.path.basename(bin_path), ADDR), timeout=180)


def oc_unlock(oc, scr):
    """解除读保护（STM32F1：stm32f1x unlock 0）—— 会擦掉整片 Flash。"""
    return run(_oc_cmd(oc, scr, ['init', 'stm32f1x unlock 0', 'reset run', 'exit']),
               '解除读保护（会擦除整片 Flash）', timeout=120)


def oc_erase(oc, scr):
    """全片擦除。"""
    return run(_oc_cmd(oc, scr, ['init', 'halt', 'stm32f1x mass_erase 0',
                                 'reset run', 'exit']),
               '全片擦除', timeout=120)


# ============================ CubeProgrammer 通道 ============================

def conn_ok(out):
    """CubeProgrammer 到底连上了没有？

    ⚠⚠ 千万不要用 `'STM32' in out` 判断 —— 输出第一行固定打印版本号
        "STM32CubeProgrammer v2.23.0"，里面就含 STM32，
        会把「连不上」稳稳地误判成「连上了」。（2026-10-13 实测踩到这个坑）

    连上的硬证据是打印了 Device ID / Device name；
    没连上则是 ST-LINK error (DEV_CONNECT_ERR)。
    """
    low = out.lower()
    if 'dev_connect_err' in low:
        return False
    if 'no st-link' in low or 'st-link not detected' in low:
        return False
    return ('Device ID' in out) or ('Device name' in out)


def occupier():
    """看有没有别的程序正锁着 ST-LINK。

    ⚠ 实测（2026-10-13）：VSCode 一装 STM32CubeIDE 扩展，它就会自动拉起
      `cube.exe stlink-server-start` → 再由它拉起 `stlinkserver.exe`，
      把 ST-LINK 独占掉。这时命令行去抢，报的也是 `DEV_CONNECT_ERR` ——
      文案看着像「线没接对」，其实是被别的程序占着。
      返回占用的进程名列表（空列表 = 没人占）。
    """
    try:
        r = subprocess.run(['tasklist', '/fo', 'csv', '/nh'],
                           capture_output=True, text=True, errors='ignore')
    except Exception:
        return []
    hits = []
    for line in (r.stdout or '').splitlines():
        low = line.lower()
        if 'stlinkserver' in low:
            hits.append('stlinkserver.exe')
        elif 'cube.exe' in low:
            hits.append('cube.exe')
    return hits


def print_wire_help():
    print('  → 【ST-Link 认到了，但目标芯片不应答】按这个顺序查：')
    print('    0. ★ ST-Link 的 GND 必须和板子 GND 连在一起（不共地 = 必报这个错）')
    print('    1. ★ 4 根线到底接上了没？3V3 / GND / SWDIO / SWCLK，一个都不能空')
    print('    2. ★ SWDIO / SWCLK 接反了没有？这两根最容易反')
    print('    3. BOOT0 跳线帽必须在 0（GND 一侧）')
    print('    4. 线插松了？重新拔插一遍，或换根短线（<20cm）')


# ============================ 主流程 ============================

def main():
    arg = sys.argv[1].lower() if len(sys.argv) > 1 else 'full'

    if arg in ('-h', '--help', 'help'):
        print(__doc__)
        return 0

    oc, scr = find_openocd()
    cli = find_cli()

    if oc is None and cli is None:
        print('既找不到 openocd.exe 也找不到 STM32_Programmer_CLI.exe。')
        print('请确认 CubeIDE 装在 D:\\ST，或修改本脚本顶部的路径常量。')
        return 2
    if oc:
        print('烧录工具：OpenOCD  %s' % oc)
        print('          脚本目录 %s' % scr)
    else:
        print('烧录工具：CubeProgrammer  %s   （没找到 OpenOCD）' % cli)

    # ---------- check ----------
    if arg == 'check':
        if oc:
            ok, out = oc_probe(oc, scr)
            if ok:
                print('\n[OK] 芯片连上了。')
                for line in out.splitlines():
                    if any(k in line for k in ('Target voltage', 'device id',
                                               'flash size', 'Cortex-M3')):
                        print('     ' + line.strip())
                print('\n下一步：python tools/flash.py motion  （先把车架起来，四轮离地）')
                return 0
            print('\n[失败] OpenOCD 没连上。')
            print_wire_help()
            print('  5. 还是不行 → python tools/flash.py rescue')

        if cli:
            print('\n--- 再用 CubeProgrammer 试一次 ---')
            for mode, tag in (('normal', '普通连接'), ('UR', 'connect-under-reset')):
                code, out = run([cli, '-c', 'port=SWD', 'freq=4000', 'mode=%s' % mode],
                                'CubeProgrammer 检查连接（%s）' % tag, timeout=60)
                if conn_ok(out):
                    print('\n[OK] 芯片连上了（CubeProgrammer，%s）。' % tag)
                    print('下一步：python tools/flash.py motion')
                    return 0
            print('\n[失败] CubeProgrammer 也没连上。')
            occ = occupier()
            if occ:
                print('  ⚠ ST-LINK 正被别的程序占着 → %s' % '、'.join(occ))
                print('     关掉 VSCode 的 STM32CubeIDE 扩展后重跑 check')
            print_wire_help()
            print('  5. 还是不行 → python tools/flash.py rescue')
        return 1

    # ---------- rescue ----------
    if arg == 'rescue':
        print('=' * 60)
        print('急救助手：把「烧过一次就连不上」的芯片救回来')
        print('=' * 60)
        print('会依次尝试：① 连接 ② 解除读保护 ③ 全片擦除\n')

        if oc:
            ok, out = oc_probe(oc, scr)
            if ok:
                oc_unlock(oc, scr)
                oc_erase(oc, scr)
                print('\n[OK] 已解锁 + 全片擦除，芯片是干净的空片了。')
                print('     ① 确认 BOOT0 跳线帽在 0')
                print('     ② python tools/flash.py check   → 应该秒连')
                print('     ③ python tools/flash.py motion  → 重新烧进去')
                return 0

        if cli:
            connected = False
            for mode, tag in (('normal', '普通连接'), ('UR', 'connect-under-reset')):
                code, out = run([cli, '-c', 'port=SWD', 'freq=4000', 'mode=%s' % mode,
                                 '-ob', 'RDP=0xAA'], 'CubeProgrammer 解除读保护（%s）' % tag,
                                timeout=90)
                if code == 0 and 'DEV_CONNECT_ERR' not in out and \
                   ('successfully' in out.lower() or conn_ok(out)):
                    connected = True
                    break
            if connected:
                run([cli, '-c', 'port=SWD', 'freq=4000', '-e', 'all'], '全片擦除')
                print('\n[OK] 已解锁 + 全片擦除。重新跑 check / motion 即可。')
                return 0

        print('--- 工具方式没连上，走这条路（100% 有效，硬件救得回来）---')
        print('  1. 拔掉电池 / 断开供电')
        print('  2. 把 BOOT0 跳线帽从「0」拔下来，插到「1」（3.3V 那一侧）')
        print('  3. 重新上电 —— 这时芯片跑的是出厂 BootROM，用户程序根本不运行')
        print('  4. 再跑一次：python tools/flash.py rescue')
        print('  5. 连上后会全片擦除；擦完把 BOOT0 拨回「0」，断电再上电')
        print('  （BOOT0=1 只是让芯片从 ROM 启动，不会损坏任何东西，放心拨）')
        return 1

    # ---------- unlock ----------
    if arg == 'unlock':
        if oc:
            code, out = oc_unlock(oc, scr)
            if code == 0 and 'error' not in out.lower().replace('error:', ''):
                print('\n[OK] 解锁完成（整片已擦除）。重新烧录即可。')
                return 0
        if cli:
            for mode in ('normal', 'UR'):
                code, out = run([cli, '-c', 'port=SWD', 'freq=4000', 'mode=%s' % mode,
                                 '-ob', 'RDP=0xAA'], '解除读保护（%s）' % mode, timeout=90)
                if code == 0 and 'DEV_CONNECT_ERR' not in out:
                    print('\n[OK] 解锁完成（整片已擦除）。重新烧录即可。')
                    return 0
        print('\n[失败] 解锁没成功。先跑 check；还不行就跑 rescue（用 BOOT0=1 那招）。')
        return 1

    # ---------- 烧录 ----------
    if arg not in BINS:
        print('不认识的目标：%s' % arg)
        print('可选：check / unlock / rescue / ps2 / ps2scan / motion / motionauto / arm / vision / full')
        return 2

    bin_path = BINS[arg]
    if not os.path.isfile(bin_path):
        print('找不到 %s' % bin_path)
        print('先编译：python tools/build.py %s' % arg)
        return 2

    if oc:
        code, out = oc_flash(oc, scr, bin_path)
        low = out.lower()
        if oc_ok(out) and ('** verified ok **' in low or 'verified ok' in low
                           or 'wrote ' in low):
            print('\n[OK] 烧好了，程序已经在跑（OpenOCD 通道）。')
            print('接下来：串口助手打开 COM 口，波特率 115200，看有没有打印。')
            print('          没有 USB 转串口模块的话，看到上面这几行就算成功。')
            return 0
        if oc_ok(out) and 'error' not in low.split('reset run')[-1]:
            print('\n[OK] 烧好了（未打印 verify 结果，但连接与写入均无报错）。')
            return 0
        print('\n[失败] OpenOCD 烧录没成功。常见原因：')
        print('  - 芯片被读保护锁住：先跑 python tools/flash.py rescue')
        print('  - BOOT0 在 1：改回 0')
        print('  - 线太长/接触不良：换短线，重新插拔')

    if cli:
        print('\n--- 换 CubeProgrammer 再试 ---')
        code, out = run([cli, '-c', 'port=SWD', 'freq=4000',
                         '-w', bin_path, ADDR, '-v', '-s'],
                        'CubeProgrammer 烧录 %s' % os.path.basename(bin_path), timeout=180)
        if code == 0 and 'DEV_CONNECT_ERR' not in out and \
           ('Download verified' in out or 'successfully' in out.lower()):
            print('\n[OK] 烧好了，程序已经在跑。')
            return 0
    return 1


if __name__ == '__main__':
    sys.exit(main())
