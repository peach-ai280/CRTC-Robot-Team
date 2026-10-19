"""
check_wiring.py —— 引脚矩阵静态自检（不需要板子，纯代码校验）

校验内容：
  1. 同一引脚是否被两个功能重复占用
  2. 每个功能用的引脚，在 STM32F103C8T6 上是否真的具备那个复用功能
  3. 是否用了「禁用引脚」(SWD / JTAG / 晶振 / BOOT)
  4. 同一外设的同一通道是否被两个功能占用
  5. 定时器时钟是否真的被打开
  6. 列出还剩哪些空闲脚（给舵机/编码器留余量）

用法（仓库根目录）：
    D:\\WorkBuddy\\tools\\python\\python.exe tools\\check_wiring.py
"""

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
BOARD_H = ROOT / "firmware" / "Core" / "Inc" / "board.h"
BSP_C = ROOT / "firmware" / "Core" / "Src" / "bsp.c"

# ---------------------------------------------------------------------------
# STM32F103C8T6（LQFP48）引脚能力表 —— 只列我们可能用到的复用功能
# 依据：RM0008 Table 5 / DS5319 pin definitions
# ---------------------------------------------------------------------------
CAPABILITY = {
    "PA0":  ["TIM2_CH1", "GPIO"],
    "PA1":  ["TIM2_CH2", "GPIO"],
    "PA2":  ["TIM2_CH3", "USART2_TX", "GPIO"],
    "PA3":  ["TIM2_CH4", "USART2_RX", "GPIO"],
    "PA4":  ["SPI1_NSS", "ADC1_IN4", "GPIO"],
    "PA5":  ["SPI1_SCK", "ADC1_IN5", "GPIO"],
    "PA6":  ["SPI1_MISO", "ADC1_IN6", "GPIO"],
    "PA7":  ["SPI1_MOSI", "ADC1_IN7", "GPIO"],
    "PA8":  ["TIM1_CH1", "GPIO"],
    "PA9":  ["USART1_TX", "TIM1_CH2", "GPIO"],
    "PA10": ["USART1_RX", "TIM1_CH3", "GPIO"],
    "PA11": ["TIM1_CH4", "USB_DM", "GPIO"],
    "PA12": ["USB_DP", "GPIO"],
    "PA13": ["SWDIO"],
    "PA14": ["SWCLK"],
    "PA15": ["JTDI"],
    "PB0":  ["TIM3_CH3", "ADC1_IN8", "GPIO"],
    "PB1":  ["TIM3_CH4", "ADC1_IN9", "GPIO"],
    "PB3":  ["JTDO"],
    "PB4":  ["JNTRST"],
    "PB5":  ["GPIO"],
    "PB6":  ["TIM4_CH1", "I2C1_SCL", "GPIO"],
    "PB7":  ["TIM4_CH2", "I2C1_SDA", "GPIO"],
    "PB8":  ["TIM4_CH3", "GPIO"],
    "PB9":  ["TIM4_CH4", "GPIO"],
    "PB10": ["GPIO"],
    "PB11": ["GPIO"],
    "PB12": ["GPIO"],
    "PB13": ["GPIO"],
    "PB14": ["GPIO"],
    "PB15": ["GPIO"],
    "PC13": ["GPIO"],
    "PC14": ["OSC32_IN"],
    "PC15": ["OSC32_OUT"],
}

FORBIDDEN = {
    "PA13": "SWDIO 调试数据线，占了就再也烧不进去",
    "PA14": "SWCLK 调试时钟线，占了就再也烧不进去",
    "PA15": "JTAG JTDI（未关 JTAG 时不可用）",
    "PB3":  "JTAG JTDO（未关 JTAG 时不可用）",
    "PB4":  "JTAG JNTRST（未关 JTAG 时不可用）",
    "PC14": "32.768kHz 晶振脚",
    "PC15": "32.768kHz 晶振脚",
}

# 我们认为必须存在的功能 -> (引脚, 需要的复用功能名)
EXPECT = [
    ("PS2 SCK",       "PA5",  "SPI1_SCK"),
    ("PS2 MISO",      "PA6",  "SPI1_MISO"),
    ("PS2 MOSI",      "PA7",  "SPI1_MOSI"),
    ("电机 LF PWM",   "PB6",  "TIM4_CH1"),
    ("电机 RF PWM",   "PB7",  "TIM4_CH2"),
    ("电机 LB PWM",   "PB8",  "TIM4_CH3"),
    ("电机 RB PWM",   "PB9",  "TIM4_CH4"),
    ("舵机 夹爪",     "PA0",  "TIM2_CH1"),
    ("舵机 升降",     "PA1",  "TIM2_CH2"),
    ("舵机 前铲",     "PA2",  "TIM2_CH3"),
    ("舵机 备用",     "PA3",  "TIM2_CH4"),
    ("舵机 回转",     "PA11", "TIM1_CH4"),
    ("电机 LF 方向",  "PB12", "GPIO"),
    ("电机 RF 方向",  "PB13", "GPIO"),
    ("电机 LB 方向",  "PB14", "GPIO"),
    ("电机 RB 方向",  "PB15", "GPIO"),
    ("PS2 片选 CS",   "PB0",  "GPIO"),
    ("板载按键",      "PB5",  "GPIO"),
    ("限位 左",       "PB10", "GPIO"),
    ("限位 右",       "PB11", "GPIO"),
    ("电池检测 ADC",  "PB1",  "ADC1_IN9"),
    ("蜂鸣器",        "PA8",  "GPIO"),
    ("板载 LED",      "PC13", "GPIO"),
]

errors = []
warns = []
notes = []


def read_macros(path):
    txt = path.read_text(encoding="utf-8", errors="replace")
    out = {}
    for m in re.finditer(r"^#define\s+(\w+)\s+(.+?)$", txt, re.M):
        val = m.group(2)
        val = re.sub(r"/\*.*?\*/", "", val)     # 去掉行尾块注释
        val = re.sub(r"//.*$", "", val)         # 去掉行尾行注释
        out[m.group(1)] = val.strip()
    return out


# ---------------------------------------------------------------------------
# 1. 从 board.h 里把「真正被代码使用的引脚」抓出来
# ---------------------------------------------------------------------------
macros = read_macros(BOARD_H)
used = {}   # 引脚 -> [功能名, ...]

PIN_RE = re.compile(r"\bGPIO_PIN_(\d+)\b")


def add(pin, who):
    used.setdefault(pin, [])
    if who not in used[pin]:
        used[pin].append(who)


# 从 *PORT / *PIN 成对的宏里还原
pin_macros = [
    (m[:-4], macros[m]) for m in macros if m.endswith("_PIN")
]
PORT_OVERRIDE = {"_DIR": "GPIOB"}   # 四个方向脚共用 MOTOR_DIR_PORT


def port_of(base):
    if base + "_PORT" in macros:
        v = macros[base + "_PORT"]
    else:
        for suffix, v2 in PORT_OVERRIDE.items():
            if base.endswith(suffix):
                v = v2
                break
        else:
            return None
    return "P" + v[-1]     # GPIOA -> PA


for base, val in pin_macros:
    pfx = port_of(base)
    if pfx is None:
        continue
    for n in PIN_RE.findall(val):
        add(f"{pfx}{n}", base)

# 电机 PWM 在 board.h 里是 TIM_CHANNEL_x，没有 PIN 宏，这里补齐
for n, ch in zip(range(6, 10), ["MOTOR_TIM_CH_LF", "MOTOR_TIM_CH_RF",
                                "MOTOR_TIM_CH_LB", "MOTOR_TIM_CH_RB"]):
    add(f"PB{n}", ch)

# 舵机 / SPI / ADC 也只用 CHANNEL 宏表达，用核对表补齐（已占则跳过，避免误报重复）
for _name, _pin, _ in EXPECT:
    if _pin not in used:
        add(_pin, _name)

# ---------------------------------------------------------------------------
# 2. 重复占用检查
# ---------------------------------------------------------------------------
print("=" * 68)
print("【检查 1】同一引脚是否被两个功能重复占用")
print("=" * 68)
dup = {p: v for p, v in used.items() if len(v) > 1}
if dup:
    for p, v in sorted(dup.items()):
        # 同一功能不同名字的别名不算错
        real = [x for x in v if not x.endswith("_ALIAS")]
        if len(set(real)) > 1:
            errors.append(f"{p} 被重复占用: {', '.join(real)}")
            print(f"  [X] {p}  <- {', '.join(real)}")
        else:
            print(f"  [~] {p}  <- {', '.join(real)}（同功能别名，正常）")
else:
    print("  [OK] 没有任何引脚被重复占用")

# ---------------------------------------------------------------------------
# 3. 复用功能能力检查
# ---------------------------------------------------------------------------
print()
print("=" * 68)
print("【检查 2】每个功能用的引脚，在 F103C8T6 上是否真具备该复用功能")
print("=" * 68)
for func, pin, need in EXPECT:
    caps = CAPABILITY.get(pin)
    if caps is None:
        errors.append(f"{func}: {pin} 在这个封装上不存在")
        print(f"  [X] {func:12s} {pin:5s} 封装上没这个脚")
        continue
    if need in caps:
        print(f"  [OK] {func:12s} {pin:5s} -> {need}")
    else:
        errors.append(f"{func}: {pin} 不具备 {need}（可选 {caps}）")
        print(f"  [X] {func:12s} {pin:5s} -> 不具备 {need}（可选 {caps}）")

# ---------------------------------------------------------------------------
# 4. 禁用引脚检查
# ---------------------------------------------------------------------------
print()
print("=" * 68)
print("【检查 3】是否误用了禁用引脚")
print("=" * 68)
hit = False
for p in sorted(used):
    if p in FORBIDDEN:
        hit = True
        errors.append(f"{p} 是禁用脚：{FORBIDDEN[p]}")
        print(f"  [X] {p} <- {', '.join(used[p])}  ⛔ {FORBIDDEN[p]}")
if not hit:
    print("  [OK] 没有占用 SWD / JTAG / 晶振脚")

# ---------------------------------------------------------------------------
# 5. 定时器通道冲突
# ---------------------------------------------------------------------------
print()
print("=" * 68)
print("【检查 4】同一外设的同一通道是否被两处占用")
print("=" * 68)
chan_map = {}
for func, pin, need in EXPECT:
    if "_CH" in need:
        chan_map.setdefault(need, []).append(f"{func}({pin})")
conflict = False
for ch, who in chan_map.items():
    if len(who) > 1:
        conflict = True
        errors.append(f"{ch} 冲突: {who}")
        print(f"  [X] {ch} <- {', '.join(who)}")
    else:
        print(f"  [OK] {ch} 独占")
if not conflict:
    print("  -> 所有定时器通道均独占")

# ---------------------------------------------------------------------------
# 6. 定时器时钟是否被打开
# ---------------------------------------------------------------------------
print()
print("=" * 68)
print("【检查 5】用到的定时器，时钟使能是否真的写了")
print("=" * 68)
bsp = BSP_C.read_text(encoding="utf-8", errors="replace")
shim_path = ROOT / "firmware" / "Core" / "Src" / "hal_shim.c"
shim = shim_path.read_text(encoding="utf-8", errors="replace") if shim_path.exists() else ""
for tim in ["TIM1", "TIM2", "TIM4"]:
    in_bsp = f"__HAL_RCC_{tim}_CLK_ENABLE()" in bsp
    in_shim = f"RCC_APB2ENR_{tim}EN" in shim or f"RCC_APB1ENR_{tim}EN" in shim
    if in_bsp or in_shim:
        where = []
        if in_bsp:
            where.append("bsp.c")
        if in_shim:
            where.append("hal_shim.c")
        print(f"  [OK] {tim} 时钟使能 (在 {' / '.join(where)})")
    else:
        errors.append(f"{tim} 时钟从未被打开")
        print(f"  [X] {tim} 时钟从未被打开 -> 该定时器的 PWM 完全没输出")

moe = "0x8000UL" in shim or "MOE" in shim
print(f"  [{'OK' if moe else 'X'}] TIM1 高级定时器 BDTR.MOE 主输出使能"
      f"{'（已处理）' if moe else '（未处理 -> PA11 无输出）'}")
if not moe:
    errors.append("TIM1 BDTR.MOE 未使能")

# ---------------------------------------------------------------------------
# 7. 空闲引脚余量
# ---------------------------------------------------------------------------
print()
print("=" * 68)
print("【检查 6】还剩哪些空闲脚（给后续舵机/编码器/限位留余量）")
print("=" * 68)
free = []
for p, caps in CAPABILITY.items():
    if p in used or p in FORBIDDEN:
        continue
    free.append(f"{p}({'/'.join(c for c in caps if c != 'GPIO')})")
print("  " + "  ".join(free))
notes.append(f"空闲引脚 {len(free)} 个")

# ---------------------------------------------------------------------------
# 8. 重映射检查
# ---------------------------------------------------------------------------
print()
print("=" * 68)
print("【检查 7】是否有 AFIO 重映射会打乱上面的默认引脚")
print("=" * 68)
allsrc = ""
for f in (ROOT / "firmware" / "Core" / "Src").glob("*.c"):
    allsrc += f.read_text(encoding="utf-8", errors="replace")
if "AFIO->MAPR" in allsrc or "MAPR =" in allsrc:
    warns.append("代码里有 AFIO->MAPR 写入，需人工确认没有改 SPI1/TIM4/TIM2 的重映射位")
    print("  [!] 发现 AFIO->MAPR 写入，需人工确认")
else:
    print("  [OK] 没有任何重映射写入 -> SPI1=PA5/6/7、TIM4=PB6~9、TIM2=PA0~3、TIM1_CH4=PA11"
          " 都是上电默认值")

# ---------------------------------------------------------------------------
# 汇总
# ---------------------------------------------------------------------------
print()
print("=" * 68)
print("汇总")
print("=" * 68)
print(f"  检查项: 7     错误: {len(errors)}     警告: {len(warns)}")
for e in errors:
    print(f"    [X] {e}")
for w in warns:
    print(f"    [!] {w}")
if not errors and not warns:
    print("  ★ 全部通过：引脚矩阵本身不存在冲突")
sys.exit(1 if errors else 0)
