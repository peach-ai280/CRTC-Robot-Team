#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
check_interface.py —— 引脚接口「冻结基线」校验

【为什么要有这个工具】
    改算法的时候手一滑，很容易顺手把 board.h 里的引脚宏改了，
    然后车就不动了，还要从头查线。
    这个脚本把「确认过的引脚」存成基线，每次改完跑一遍，
    一眼看出哪个宏被动了、从什么变成什么。

【用法】
    python tools/check_interface.py            # 校验（默认）
    python tools/check_interface.py --save     # 把当前 board.h 存成新基线
    python tools/check_interface.py --diff     # 只显示有差异的项

【退出码】
    0 = 全部一致        1 = 有差异（或者文件读不到）

【注意】
    只校验 board.h（引脚唯一真源）。
    config.h 是「调参文件」，故意改它不算错，不在校验范围内。
"""

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BOARD_H = ROOT / "firmware" / "Core" / "Inc" / "board.h"
BASELINE = Path(__file__).resolve().parent / "interface_baseline.json"

# #define NAME  value   <- 只认对象式宏（没有参数表）
DEFINE_RE = re.compile(r"^\s*#\s*define\s+([A-Za-z_]\w*)(?!\()\s+(.+?)\s*$", re.M)

# 要冻结的宏（按功能分组，只列真正决定接线的）
WATCHED = [
    # --- PS2 ---
    "PS2_CS_PORT", "PS2_CS_PIN",
    # --- 电机 PWM（TIM4） ---
    "MOTOR_PWM_TIM",
    "MOTOR_TIM_CH_LF", "MOTOR_TIM_CH_RF", "MOTOR_TIM_CH_LB", "MOTOR_TIM_CH_RB",
    # --- 电机方向 ---
    "MOTOR_DIR_PORT",
    "MOTOR_LF_DIR_PIN", "MOTOR_RF_DIR_PIN", "MOTOR_LB_DIR_PIN", "MOTOR_RB_DIR_PIN",
    # --- 轮序 ---
    "MOTOR_LF", "MOTOR_RF", "MOTOR_LB", "MOTOR_RB",
    # --- 舵机 ---
    "SERVO_TIM", "SERVO_TIM_CH_GRIP", "SERVO_TIM_CH_ARM",
    "SERVO_TIM_CH_SCOOP", "SERVO_TIM_CH_EXTRA",
    "SERVO_TIM_YAW", "SERVO_TIM_CH_YAW",
    # --- 传感器 / 交互 ---
    "LIMIT_L_PORT", "LIMIT_L_PIN", "LIMIT_R_PORT", "LIMIT_R_PIN",
    "VBAT_ADC_CHANNEL",
    "LED_PORT", "LED_PIN", "KEY_PORT", "KEY_PIN", "BUZZ_PORT", "BUZZ_PIN",
    # --- 车体尺寸（改错会「横移时自己转圈」） ---
    "ROBOT_HALF_WHEELBASE", "ROBOT_HALF_TRACK", "WHEEL_RADIUS",
]


def read_macros(path):
    """解析 board.h 里所有对象式宏，去掉行尾注释"""
    txt = path.read_text(encoding="utf-8", errors="replace")
    out = {}
    for m in DEFINE_RE.finditer(txt):
        name, val = m.group(1), m.group(2)
        # 去掉 /* ... */ 和 // ... 注释
        val = re.sub(r"/\*.*?\*/", "", val)
        val = re.sub(r"//.*$", "", val)
        out[name] = val.strip()
    return out


def to_pin_name(port, pin):
    """GPIOB + GPIO_PIN_12 -> PB12，人看着直观"""
    m = re.search(r"GPIO([A-G])", port or "")
    n = re.search(r"GPIO_PIN_(\d+)", pin or "")
    if m and n:
        return "P%s%s" % (m.group(1), n.group(1))
    return None


def humanize(macros):
    """把 PORT/PIN 成对的宏额外算出一个可读引脚名，方便看"""
    extra = {}
    for name in list(macros):
        if name.endswith("_PIN"):
            base = name[:-4]
            port = macros.get(base + "_PORT")
            pin = to_pin_name(port, macros[name])
            if pin:
                extra[name] = pin
    return extra


def load_baseline():
    if not BASELINE.exists():
        return None
    try:
        return json.loads(BASELINE.read_text(encoding="utf-8"))
    except Exception as e:
        print("[ERR] 基线文件损坏：%s" % e)
        return None


def save_baseline(macros):
    data = {k: macros.get(k, None) for k in WATCHED}
    data["_meta"] = {
        "note": "引脚冻结基线。确认接线后再 --save。",
        "file": "firmware/Core/Inc/board.h",
    }
    BASELINE.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("[OK] 基线已保存 -> %s（共 %d 项）" % (BASELINE.name, len(WATCHED)))


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

    if not BOARD_H.exists():
        print("[ERR] 找不到 %s" % BOARD_H)
        return 1

    cur = read_macros(BOARD_H)
    pins = humanize(cur)

    if "--save" in sys.argv:
        save_baseline(cur)
        return 0

    base = load_baseline()
    if base is None:
        print("[WARN] 还没有基线文件，先跑一次：")
        print("       python tools/check_interface.py --save")
        return 1

    only_diff = "--diff" in sys.argv

    ok, changed, missing, added = [], [], [], []

    for name in WATCHED:
        old = base.get(name)
        new = cur.get(name)
        if old is None and new is None:
            continue
        if old is None:
            added.append((name, "-", new))
        elif new is None:
            missing.append((name, old))
        elif str(old) != str(new):
            changed.append((name, old, new))
        else:
            ok.append((name, new, pins.get(name)))

    print("=" * 66)
    print(" 引脚接口基线校验   board.h")
    print("=" * 66)

    if not only_diff:
        for name, value, pin in ok:
            extra = ("  ->  %s" % pin) if pin else ""
            print("  [ OK ]  %-24s = %s%s" % (name, value, extra))

    for name, old, new in changed:
        print("  [改动]  %-24s %s  ->  %s" % (name, old, new))
    for name, old in missing:
        print("  [丢失]  %-24s 原来 = %s，现在找不到" % (name, old))
    for name, _, new in added:
        print("  [新增]  %-24s = %s（基线里没有，确认是有意加的）" % (name, new))

    print("-" * 66)
    print(" 一致 %d ／ 改动 %d ／ 丢失 %d ／ 新增 %d"
          % (len(ok), len(changed), len(missing), len(added)))

    if changed or missing:
        print("")
        print(" ★ 引脚被动过了。如果确实重新焊了线，确认无误后跑：")
        print("     python tools/check_interface.py --save")
        print("   如果只是改算法时手滑，把上面这些改回原值再编译。")
        return 1

    print("")
    print(" ★ 引脚全部一致，烧录不会因为引脚改动而出问题。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
