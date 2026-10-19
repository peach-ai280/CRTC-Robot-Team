"""
verify_wheel.py —— 用调试器直接读寄存器，验证「一次只有一个轮子通电」

这不是看轮子转不转，而是读芯片内部的真实数值：
    TIM4_CCR1..4  四个通道的比较值（= 占空比），应该同一时刻只有一个非 0
    GPIOB_ODR     四个方向脚 PB12~PB15 的电平

【为什么值得做】
    眼睛看轮子会被很多因素骗（地面摩擦、轮子卡住、线松）。
    寄存器不会骗人：CCR 是 0 就说明固件根本没给那一路 PWM。

【前提】
    1. ST-LINK 插在电脑上
    2. 已经烧了 wheel 固件（python tools/flash.py wheel）
    3. 四轮离地

【用法】
    D:\\WorkBuddy\\tools\\python.exe tools\\verify_wheel.py
    D:\\WorkBuddy\\tools\\python.exe tools\\verify_wheel.py --samples 14
"""

import argparse
import re
import subprocess
import sys

import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import flash  # noqa: E402

# TIM4 基址 0x40000800：CCR1=+0x34, CCR2=+0x38, CCR3=+0x3C, CCR4=+0x40
TIM4_CCR_BASE = 0x40000834
GPIOB_ODR = 0x40010C0C

NAMES = ["左前LF", "右前RF", "左后LB", "右后RB"]
# 固件里的 dir_sign：LF=+1 RF=-1 LB=+1 RB=-1
# 实际效果：想让 LF/LB 正转 -> DIR 低；想让 RF/RB 正转 -> DIR 高
DIR_FORWARD_LEVEL = [0, 1, 0, 1]


def sample(oc, scr, n_samples, interval_ms, settle_ms):
    """返回 [(ccr[4], odr), ...]

    先复位再放开，保证采样从测试序列的第 0 秒开始，
    不会因为 openocd 启动慢而只采到尾巴。

    ⚠ 必须用 -c 逐个传命令：把 mdw 写在 -f 的 cfg 文件里，OpenOCD 会
      静默不执行（不会报错，只是没有输出），踩过一次。
    """
    cmds = ["init", "reset halt", "resume", "sleep %d" % settle_ms]
    for i in range(n_samples):
        cmds += ["halt",
                 "mdw 0x%08X 4" % TIM4_CCR_BASE,
                 "mdw 0x%08X" % GPIOB_ODR]
        if i != n_samples - 1:
            cmds += ["resume", "sleep %d" % interval_ms]
    cmds += ["resume", "shutdown"]

    cmd = [oc, "-s", scr, "-f", flash.IFACE_CFG, "-f", flash.TARGET_CFG]
    for c in cmds:
        cmd += ["-c", c]

    p = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    out = p.stdout + p.stderr

    ccrs, odrs = [], []
    for m in re.finditer(r"0x40000834:\s*((?:[0-9a-fA-F]{8}\s*){4})", out):
        ccrs.append([int(x, 16) for x in m.group(1).split()])
    for m in re.finditer(r"0x40010c0c:\s*([0-9a-fA-F]{8})", out):
        odrs.append(int(m.group(1), 16))

    if not ccrs:
        print(out[-1500:])
    return list(zip(ccrs, odrs + [0] * len(ccrs))), out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--samples", type=int, default=11, help="采样次数")
    ap.add_argument("--interval", type=int, default=2000, help="采样间隔 ms")
    ap.add_argument("--settle", type=int, default=3500, help="复位后等多久开始采样 ms（覆盖上电 3 秒延时）")
    args = ap.parse_args()

    oc, scr = flash.find_openocd()
    if not oc:
        print("[X] 找不到 openocd.exe（需要有 STM32CubeIDE）")
        return 1

    print("=" * 72)
    print("读寄存器验证（请确认：ST-LINK 已插、已烧 wheel、四轮离地）")
    print("=" * 72)
    print("采样 %d 次，间隔 %d ms，总时长约 %.0f 秒\n"
          % (args.samples, args.interval, args.samples * args.interval / 1000.0))

    rows, raw = sample(oc, scr, args.samples, args.interval, args.settle)
    if not rows or not any(any(c) for c, _ in rows):
        print("[!] 没采到有效数据。检查 ST-LINK 是否插好。")
        print("    如果报 'unable to connect to the target'，等 3 秒重跑一次即可。")
        return 2

    print("  采样   CCR1(左前)  CCR2(右前)  CCR3(左后)  CCR4(右后)   非零通道   方向脚PB15..12")
    print("  " + "-" * 84)
    bad = []
    for i, (ccr, odr) in enumerate(rows):
        nz = [j for j, v in enumerate(ccr) if v > 0]
        dirs = "".join(str((odr >> b) & 1) for b in (15, 14, 13, 12))
        if len(nz) == 0:
            tag = "（全停/间隙）"
            ok = True
        elif len(nz) == 1:
            j = nz[0]
            tag = NAMES[j]
            # 校验该路方向脚电平是否符合固件的 dir_sign 约定
            want_fwd = DIR_FORWARD_LEVEL[j]
            got = (odr >> [12, 13, 14, 15][j]) & 1
            ok = True   # 正转/反转两种都合法，只记录
            tag += " 方向=%s" % ("正转" if got == want_fwd else "反转")
        else:
            tag = "!!! 多个通道同时输出: " + ",".join(NAMES[j] for j in nz)
            ok = False
            bad.append(i)
        print("  %4d   %10d %11d %11d %11d   %-30s %s   %s"
              % (i, ccr[0], ccr[1], ccr[2], ccr[3], tag, dirs,
                 "OK" if ok else "X"))

    print()
    print("=" * 72)
    if bad:
        print("[X] 有 %d 次采样出现多个通道同时有输出 —— 这是「控制一个、别的也动」的硬证据。"
              % len(bad))
        return 1
    print("[OK] 所有采样里，同一时刻最多只有一个通道有 PWM 输出。")
    print("     固件侧的「串动」已经不存在；如果实车还有轮子跟着动，")
    print("     那就一定是硬件侧：共地没接好 / 两个模块的 GND 串了 / 方向线接错模块。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
