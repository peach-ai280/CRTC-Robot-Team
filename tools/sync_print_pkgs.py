# -*- coding: utf-8 -*-
"""重建 3D打印交付/ 的批次目录（按 gen_parts.py 的 BATCH / SPARE 表驱动）。

为什么需要它：
    批次目录是**手工复制 STL** 建起来的，一旦臂改了型号（比如 10-10 从
    「平行四连杆」改成「柱坐标」），旧目录里会残留作废件、又缺新件 ——
    而负责打印的同学是**照着文件夹数件**的，这样一定会打错。
    本脚本把这件事变成一条命令。

用法（普通 python 即可）：
    D:\\WorkBuddy\\tools\\python\\python.exe tools\\sync_print_pkgs.py

⚠ 会**删掉** 3D打印交付/ 下所有 `0X_` 开头的批次目录再重建。
   两个手写的说明文件（`00_先打这个_量板规/怎么用.md`、`01_底盘必打/先读我.txt`）
   不在保护范围内 —— 重建后若丢了，用 `git checkout HEAD -- 3D打印交付/` 取回。

产出：3D打印交付/00..07 八个批次目录；跑完请再执行一次
    python tools/print_bill.py        # 刷新 打印清单.md / .csv
"""
import os, re, shutil

R = r"C:\Users\hp\WorkBuddy\CRTC"
STL = os.path.join(R, "mechanical", "stl")
GAUGE = os.path.join(R, "mechanical", "stl_gauge")
PRETTY = os.path.join(R, "mechanical", "stl_pretty")
OUT = os.path.join(R, "3D打印交付")

src = open(os.path.join(R, "mechanical", "gen_parts.py"), encoding="utf-8").read()

BATCH = {}
m = re.search(r'^BATCH\s*=\s*\{(.*?)^\}', src, re.S | re.M)
for k, v in re.findall(r'"([0-9A-Za-z_]+)"\s*:\s*(\d+)', m.group(1)):
    BATCH[k] = int(v)

SPARE = set(re.findall(r'"([0-9A-Za-z_]+)"',
                       re.search(r'^SPARE\s*=\s*\{(.*?)\}', src, re.S | re.M).group(1)))

b1 = [k for k, v in BATCH.items() if v == 1]
b2 = [k for k, v in BATCH.items() if v == 2]
b3 = [k for k, v in BATCH.items() if v == 3]
b1.sort(); b2.sort(); b3.sort()
assert len(b1) + len(b2) + len(b3) == len(BATCH), "有零件没归到 1/2/3 批"

CLAW = ["24a_claw_base", "24b_claw_slider", "24c_claw_finger", "24d_claw_horn",
        "24e_claw_link", "24f_claw_gear", "24g_claw_rack", "24h_claw_base_gear",
        "24i_claw_finger_gear"]

PLAN = [
    ("00_先打这个_量板规",   ["gauge_height"]),
    ("01_底盘必打",         b1),
    ("02_收集与储仓",       b2),
    ("03_机械臂与机构",     b3),
    ("04_外观套件",         None),          # 从 stl_pretty 取
    ("05_备件多打一份",     [k for k in BATCH if k in SPARE]),
    ("06_能量块道具_拍视频用", ["26_cube_mock"]),
    ("07_夹爪_可选两套",    CLAW),
]

# 1) 删掉所有旧的批次目录（只删 0*_ 开头的）
for d in os.listdir(OUT):
    p = os.path.join(OUT, d)
    if os.path.isdir(p) and re.match(r"^0\d_", d):
        shutil.rmtree(p)
        print("removed", d)

# 2) 重建
try:
    GUIDE = open(os.path.join(OUT, "03_机械臂与机构", "先读我.txt"), encoding="utf-8").read()
except Exception:
    GUIDE = None

missing = []
for name, items in PLAN:
    dst = os.path.join(OUT, name)
    os.makedirs(dst, exist_ok=True)
    if items is None:
        for f in sorted(os.listdir(PRETTY)):
            if f.endswith(".stl"):
                shutil.copy2(os.path.join(PRETTY, f), dst)
    else:
        for it in items:
            for base in (STL, GAUGE):
                s = os.path.join(base, it + ".stl")
                if os.path.exists(s):
                    shutil.copy2(s, dst)
                    break
            else:
                missing.append(it)
    n = len([f for f in os.listdir(dst) if f.endswith(".stl")])
    print("%-24s %2d stl" % (name, n))

print("缺失:", missing or "无")

# 3) 数量核对：批次里每个件都要能对上
allstl = set(os.path.splitext(f)[0] for f in os.listdir(STL))
should = set(BATCH) | {"gauge_height", "26_cube_mock"} | set(CLAW)
print("BATCH 里有但 stl 目录没有:", sorted(set(BATCH) - allstl) or "无")
print("stl 目录有但没进任何批次:", sorted(allstl - should) or "无")
