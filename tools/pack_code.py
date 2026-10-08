# -*- coding: utf-8 -*-
"""
pack_code.py —— 把全部代码打包成 zip，交给队友 / 存档

【为什么要有这个脚本】
用户是纯新手，不会用 git，也不会手动挑文件。他需要"一个 zip 拿到所有代码"。
改完代码重新跑一次就能刷新 zip，避免发出去的是旧版。

【用法】在仓库根目录执行：
    python tools/pack_code.py

产出：
    D:\\WorkBuddy\\CRTC交付\\CRTC_全部代码.zip
    同时在仓库根写出 代码文件清单.md（告诉他每个文件是干嘛的）
"""

import os
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_ZIP = r"D:\WorkBuddy\CRTC交付\CRTC_代码包_只有代码.zip"

# 要打包的目录（相对仓库根）
# ⚠ 2026-10-04：原来把 mechanical / docs / 3D打印交付 一起打进去了，
#   用户打开 zip 看到一堆 3D 建模图和"先打量规量板宽"的旧说明 —— 那是已经作废的旧版交付，
#   混进代码包里只会误导。代码包就只放代码。
#   建模脚本（mechanical/gen_parts.py 等）单独在 给机械队友_CAD包 / 建模脚本包 里给。
INCLUDE_DIRS = ["firmware", "cpp", "tools", "vision"]
# 要打包的根部文件
INCLUDE_FILES = ["README.md", "现在该干什么.md", "代码说明_先看这个.md"]
# 要排除的东西（这些是产物/缓存，不是源码）
EXCLUDE_DIRS = {"build", "__pycache__", ".git", ".workbuddy", "stl", "stl_pretty",
                "stl_gauge", "dxf", "sim"}
EXCLUDE_EXT = {".elf", ".bin", ".hex", ".map", ".o", ".gcode", ".stl", ".pyc", ".zip"}

# 每个文件的用途说明（写给新手看的）
FILE_DESC = {
    "firmware/Core/Src/main.c": "主程序：上电初始化 + 主循环",
    "firmware/Core/Src/mecanum.c": "麦轮运动解算（横移/斜走/自转的数学）",
    "firmware/Core/Src/motor.c": "电机输出层（PWM + 方向 + 死区补偿）",
    "firmware/Core/Src/ps2.c": "PS2 手柄通信（SPI 读写 + 按键解析）",
    "firmware/Core/Src/servo.c": "舵机控制（角度插值 + 姿态）",
    "firmware/Core/Src/arm.c": "机械臂动作序列（取块/举升/入仓）",
    "firmware/Core/Src/auto_task.c": "半自动宏（一键收集/挑架/探洞/贴边）",
    "firmware/Core/Src/auto_full.c": "全自动调度器（在半自动宏之上）",
    "firmware/Core/Src/bsp.c": "外设初始化（时钟/GPIO/定时器/SPI/串口）",
    "firmware/Core/Src/sensor.c": "限位开关 + 电池电压检测",
    "firmware/Core/Src/vision.c": "视觉模块串口协议解析（可选）",
    "firmware/Core/Src/hal_shim.c": "★ 自写的 HAL 替代层（官方 HAL 下载不到）",
    "firmware/Core/Src/startup_stm32f103xb.c": "启动文件（C 写的，不调 .init_array）",
    "firmware/Core/Src/test_motion.c": "测试固件：只测底盘跑动",
    "firmware/Core/Src/test_arm.c": "测试固件：只测舵机/机械臂",
    "firmware/Core/Src/test_vision.c": "测试固件：只测摄像头",
    "firmware/Core/Inc/config.h": "★★ 全车唯一调参入口（95% 的时间只改这个）",
    "firmware/Core/Inc/board.h": "硬件引脚映射表（改接线前先看这个）",
    "tools/build.py": "编译固件（替代 make）",
    "tools/flash.py": "一键烧录（自动找 CubeIDE 的烧录器）",
    "tools/check_symbols.py": "静态自检（查符号/宏缺失）",
    "tools/print_bill.py": "打印清单与重量估算",
    "tools/slice_k1.py": "自研切片器（STL → K1 Max 的 G-code）",
    "tools/check_gcode.py": "G-code 六项自检",
    "tools/sim/run_sim.py": "运动学离线验证（不需要硬件）",
    "tools/sim/run_auto_sim.py": "全自动三场景验证",
    "mechanical/gen_parts.py": "3D 结构件建模（改尺寸改这里）",
    "mechanical/gen_shell.py": "外观件建模",
    "mechanical/gen_gauge.py": "1:1 量规建模",
}


def should_skip(rel_path):
    parts = rel_path.replace("\\", "/").split("/")
    for p in parts[:-1]:          # 只看目录部分
        if p in EXCLUDE_DIRS:
            return True
    ext = os.path.splitext(rel_path)[1].lower()
    return ext in EXCLUDE_EXT


def main():
    os.makedirs(os.path.dirname(OUT_ZIP), exist_ok=True)
    packed = []

    with zipfile.ZipFile(OUT_ZIP, "w", zipfile.ZIP_DEFLATED) as zf:
        for d in INCLUDE_DIRS:
            abs_d = os.path.join(ROOT, d)
            if not os.path.isdir(abs_d):
                continue
            for dirpath, dirnames, filenames in os.walk(abs_d):
                dirnames[:] = [x for x in dirnames if x not in EXCLUDE_DIRS]
                for fn in filenames:
                    abs_f = os.path.join(dirpath, fn)
                    rel = os.path.relpath(abs_f, ROOT)
                    if should_skip(rel):
                        continue
                    zf.write(abs_f, rel)
                    packed.append(rel.replace("\\", "/"))

        for f in INCLUDE_FILES:
            abs_f = os.path.join(ROOT, f)
            if os.path.isfile(abs_f):
                zf.write(abs_f, f)
                packed.append(f)

    packed.sort()

    # 写出清单
    lines = ["# 代码文件清单", "",
             f"打包文件：`{OUT_ZIP}`", f"共 {len(packed)} 个文件", "",
             "## 最重要 / 最常用的文件", ""]
    for k, v in FILE_DESC.items():
        if k in packed:
            lines.append(f"- `{k}` —— {v}")
    lines += ["", "## 完整清单", ""]
    for p in packed:
        lines.append(f"- `{p}`")

    list_path = os.path.join(ROOT, "代码文件清单.md")
    with open(list_path, "w", encoding="utf-8") as fp:
        fp.write("\n".join(lines) + "\n")

    print(f"打包完成：{OUT_ZIP}")
    print(f"文件数：{len(packed)}")
    print(f"清单：{list_path}")
    print(f"大小：{os.path.getsize(OUT_ZIP)/1024:.1f} KB")


if __name__ == "__main__":
    main()
