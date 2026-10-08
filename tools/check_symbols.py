# -*- coding: utf-8 -*-
"""
check_symbols.py —— CRTC2026 固件静态自检（不需要编译器）

为什么要这个脚本：
  本机没有 arm-none-eabi 的 HAL 库，编译不了整个工程。但在拿到 CubeIDE
  之前，90% 的"编不过"其实只是三类低级错误：
    1. 调用了没实现的函数（漏文件 / 函数名打错 / .h 声明了但 .c 没写）
    2. 用了没定义的宏（config.h 里改名了但代码没跟着改）
    3. .c 里定义了但 .h 没声明（别的模块调不到）
  这个脚本就查这三件事。它不是编译器，抓不到类型不匹配、参数个数错，
  但能把上面这些"一眼看不出来"的问题提前揪出来。

用法：
  python tools/check_symbols.py
"""

import re
import os
import glob
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'firmware')
INC_DIR = os.path.join(ROOT, 'Core', 'Inc')
SRC_DIR = os.path.join(ROOT, 'Core', 'Src')

# 本工程模块前缀 —— 只检查这些，避免把 HAL / 标准库的函数卷进来
PREFIXES = (
    'PS2_', 'Motor_', 'Chassis_', 'Mecanum_', 'Servo_', 'Arm_',
    'Auto_', 'Sensor_', 'Vision_', 'App_', 'Limit_',
)

KEYWORDS = {
    'if', 'for', 'while', 'switch', 'return', 'sizeof', 'else', 'do',
    'case', 'break', 'continue', 'int', 'float', 'void', 'uint8_t',
    'uint16_t', 'uint32_t', 'int8_t', 'int16_t', 'int32_t',
}

# HAL / 编译器提供的符号，出现在调用位置但不需要本工程定义
EXTERNAL_CALL = {
    'HAL_TIM_PWM_Start', 'HAL_TIM_PWM_Init', 'HAL_TIM_PWM_ConfigChannel',
    'HAL_SPI_Init', 'HAL_UART_Init', 'HAL_ADC_Init', 'HAL_ADC_ConfigChannel',
    'HAL_ADCEx_Calibration_Start', 'HAL_GPIO_Init', 'HAL_GPIO_WritePin',
    'HAL_GPIO_ReadPin', 'HAL_GPIO_TogglePin', 'HAL_UART_Transmit',
    'HAL_UART_Receive_IT', 'HAL_SPI_TransmitReceive', 'HAL_ADC_Start',
    'HAL_ADC_PollForConversion', 'HAL_ADC_GetValue', 'HAL_GetTick',
    'HAL_Delay', 'HAL_Init', 'HAL_RCC_OscConfig', 'HAL_RCC_ClockConfig',
    'HAL_NVIC_SetPriority', 'HAL_NVIC_EnableIRQ', 'Error_Handler',
    'SystemClock_Config', 'MX_TIM4_Init', 'MX_GPIO_Init',
}

MACRO_WHITELIST_PREFIX = (
    'GPIO_PIN_', 'GPIO_', 'TIM_CHANNEL_', 'TIM_', 'ADC_CHANNEL_', 'ADC_',
    'SPI_', 'UART_', 'USART', 'RCC_', 'FLASH_', 'HAL_', '__HAL_', 'PSB_',
    'MOTOR_TIM_CH_', 'SERVO_TIM_CH_', 'LIMIT_', 'LED_', 'KEY_', 'BUZZ_',
    'PS2_CS_', 'MOTOR_LF_DIR_PIN', 'MOTOR_RF_DIR_PIN',
    'MOTOR_LB_DIR_PIN', 'MOTOR_RB_DIR_PIN',
)


def read(p):
    with open(p, encoding='utf-8', errors='ignore') as f:
        return f.read()


def strip_comments(s):
    """去掉注释和字符串字面量。
    字符串一定要剥掉：串口打印里的 "[ESTOP] ON"、"SEMI-AUTO" 这些会被
    误判成未定义的宏，报一堆没意义的错误，反而把真问题淹掉。"""
    s = re.sub(r'/\*.*?\*/', '', s, flags=re.S)
    s = re.sub(r'//[^\n]*', '', s)
    s = re.sub(r'"(?:\\.|[^"\\])*"', '""', s)
    s = re.sub(r"'(?:\\.|[^'\\])*'", "''", s)
    return s


def collect_files():
    hs = sorted(glob.glob(os.path.join(INC_DIR, '*.h')))
    cs = sorted(glob.glob(os.path.join(SRC_DIR, '*.c')))
    return hs, cs


def find_definitions(files):
    """函数定义：返回 name -> (file, line)"""
    out = {}
    # 匹配 "类型 名字(参数...) {" ，跨行到 {
    pat = re.compile(
        r'^\s*(?:static\s+|extern\s+|inline\s+)*'
        r'[A-Za-z_][\w<>\*\s]*?[\s\*](\w+)\s*\([^;{)]*\)\s*\{',
        re.M)
    for p in files:
        s = strip_comments(read(p))
        for m in pat.finditer(s):
            name = m.group(1)
            if name in KEYWORDS:
                continue
            line = s[:m.start()].count('\n') + 1
            out.setdefault(name, (os.path.basename(p), line))
    return out


def find_declarations(files):
    """函数声明（.h 里的原型）"""
    out = set()
    pat = re.compile(
        r'^\s*(?:extern\s+)?[A-Za-z_][\w\*\s]*?[\s\*](\w+)\s*\([^;{)]*\)\s*;',
        re.M)
    for p in files:
        s = strip_comments(read(p))
        for m in pat.finditer(s):
            name = m.group(1)
            if name in KEYWORDS:
                continue
            out.add(name)
    return out


def find_calls(files):
    """调用点：name -> set(file)"""
    out = {}
    pat = re.compile(r'\b([A-Za-z_]\w*)\s*\(')
    for p in files:
        s = strip_comments(read(p))
        for m in pat.finditer(s):
            name = m.group(1)
            if name in KEYWORDS:
                continue
            out.setdefault(name, set()).add(os.path.basename(p))
    return out


def find_defined_macros(files):
    out = set()
    fn_like = set()      # 函数式宏，如 PS2_CS_LOW()，它们不是函数，别报"缺失定义"
    pat_any = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)\s*(\()?', re.M)
    for p in files:
        s = strip_comments(read(p))
        for m in pat_any.finditer(s):
            out.add(m.group(1))
            if m.group(2):
                fn_like.add(m.group(1))
    return out, fn_like


def find_enum_members(files):
    """枚举成员不是 #define，但不算未定义宏，要单独收集"""
    out = set()
    pat = re.compile(r'\benum\s*(?:\w+\s*)?\{([^}]*)\}', re.S)
    for p in files:
        s = strip_comments(read(p))
        for m in pat.finditer(s):
            for item in m.group(1).split(','):
                item = item.strip().split('=')[0].strip()   # 去掉 "= 0" 之类的初值
                if re.match(r'^[A-Za-z_]\w*$', item):
                    out.add(item)
    return out


def find_global_objects(files):
    """全局变量 / 常量（YAW_GEOM 之类）。它们也不是宏，别报。"""
    out = set()
    pat = re.compile(
        r'^\s*(?:static\s+|extern\s+)*(?:const\s+|volatile\s+)*'
        r'(?:uint\d+_t|int\d+_t|float|double|char|int)\s+(\w+)\s*[=;]', re.M)
    for p in files:
        s = strip_comments(read(p))
        for m in pat.finditer(s):
            out.add(m.group(1))
    return out


def find_used_macros(files):
    """只统计"疑似宏"的全大写标识符。
    要先把 ->成员 / .成员 去掉，否则寄存器结构体的成员名（CRL、ARR、CCR1…）
    会被当成未定义的宏报一大堆，把真问题淹掉。"""
    out = {}
    pat = re.compile(r'\b([A-Z][A-Z0-9_]{2,})\b')
    for p in files:
        # 寄存器头文件里全是寄存器名（CRL / ARR / CCR1…），不是宏，跳过
        if os.path.basename(p) in ('stm32f103_regs.h',):
            continue
        s = strip_comments(read(p))
        s = re.sub(r'^\s*#.*$', '', s, flags=re.M)   # 去掉预处理行
        s = re.sub(r'->\s*[A-Za-z_]\w*', ' ', s)     # 去掉 ->member
        s = re.sub(r'\.\s*[A-Za-z_]\w*', ' ', s)     # 去掉 .member
        for m in pat.finditer(s):
            name = m.group(1)
            if name.startswith(MACRO_WHITELIST_PREFIX):
                continue
            out.setdefault(name, set()).add(os.path.basename(p))
    return out


def main():
    hs, cs = collect_files()
    allf = hs + cs
    defs = find_definitions(allf)
    decls = find_declarations(allf)
    calls = find_calls(allf)
    mac_def, fn_macros = find_defined_macros(allf)
    mac_def |= find_enum_members(allf)          # 枚举成员
    mac_def |= find_global_objects(allf)        # 全局变量/常量
    mac_def |= set(defs.keys())                 # 函数名（LOG 这种全大写的）
    mac_def |= {'DISABLE', 'ENABLE', 'NULL', 'RESET', 'SET',
                'EBADF', 'ENOMEM', 'S_IFCHR', 'NSS', 'PLL', 'PLLMUL'}
    mac_use = find_used_macros(allf)

    problems = []

    # --- 1. 调用了本工程前缀函数但没定义 ---
    for name, where in sorted(calls.items()):
        if not name.startswith(PREFIXES):
            continue
        if name in defs:
            continue
        if name in EXTERNAL_CALL:
            continue
        if name in fn_macros:              # 是函数式宏，不是函数
            continue
        problems.append(
            "[缺失定义] 调用了 %-24s 但没有任何 .c 定义它  (出现在: %s)"
            % (name, ', '.join(sorted(where))))

    # --- 2. .h 声明了但没实现 ---
    for name in sorted(decls):
        if not name.startswith(PREFIXES):
            continue
        if name not in defs and name not in EXTERNAL_CALL:
            problems.append("[声明未实现] %s 在头文件里有原型，但没有实现" % name)

    # --- 3. 用了没定义的宏 ---
    for name, where in sorted(mac_use.items()):
        if name in mac_def:
            continue
        if name.startswith(MACRO_WHITELIST_PREFIX):
            continue
        problems.append(
            "[宏未定义?] %-28s (出现在: %s)" % (name, ', '.join(sorted(where))))

    # --- 4. 定义了但从未被调用（提示，不算错误）---
    unused = []
    for name, (f, line) in sorted(defs.items()):
        if not name.startswith(PREFIXES):
            continue
        if name in calls or name in decls:
            continue
        unused.append("  %-24s (%s:%d) 定义了但没人调用" % (name, f, line))

    print("=" * 78)
    print("CRTC2026 固件静态自检")
    print("  头文件 %d 个，源文件 %d 个" % (len(hs), len(cs)))
    print("  本工程函数定义 %d 个，声明 %d 个" % (len(defs), len(decls)))
    print("=" * 78)

    if problems:
        print("\n发现 %d 个问题：\n" % len(problems))
        for p in problems:
            print("  " + p)
    else:
        print("\n未发现符号缺失 / 宏缺失问题。")

    if unused:
        print("\n未被调用的函数（不一定有问题，只是提醒）：")
        for u in unused:
            print(u)

    print("\n注：本脚本只做符号级检查，类型不匹配、参数个数错、HAL API 版本差异")
    print("    它抓不到。最终请以 CubeIDE 的实际编译结果为准。")
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
