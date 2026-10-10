# -*- coding: utf-8 -*-
"""
重打交付用 zip —— 把 D:\\WorkBuddy\\CRTC交付\\ 下的成品重新打包。

为什么要有这个：
    交付目录里的 zip 是"快照"。只要动过 mechanical/（改了模型）、
    K1Max_G代码/（重切了）、示意图/（重渲染了）或 docs/（改了文档），
    那堆 zip 就是**旧的**，发出去会让队友按老图纸干活。
    这个脚本把该打的包一次打齐，避免"忘了重打"。

用法（普通 python 即可，不需要 venv）：
    D:\\WorkBuddy\\tools\\python\\python.exe tools\\repack_zips.py

产出：
    CRTC交付\\CRTC_3D打印包.zip
    CRTC交付\\K1Max_G代码.zip
    CRTC交付\\分组发包\\①..⑤ 五个分组 zip
（代码包那个 zip 由 tools/pack_code.py 负责，本脚本不碰。）
"""
import os, sys, zipfile

D = r"D:\WorkBuddy\CRTC交付"
R = r"C:\Users\hp\WorkBuddy\CRTC"

# 找文件时的搜索路径（先近后远），找不到再全盘 walk
SEARCH = [D,
          os.path.join(D, "CRTC_3D打印包"),
          os.path.join(D, "给机械队友_CAD包"),
          os.path.join(D, "示意图"),
          os.path.join(R, "mechanical"),
          os.path.join(R, "docs"),
          R]

_walk_cache = {}


def find(name):
    for d in SEARCH:
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    for base in (D, R):
        if base not in _walk_cache:
            hit = None
            for root, _dirs, files in os.walk(base):
                if name in files:
                    hit = os.path.join(root, name)
                    break
            _walk_cache[base] = hit
        if _walk_cache[base]:
            return _walk_cache[base]
    return None


def add_file(zf, src, arc):
    if src and os.path.exists(src):
        zf.write(src, arc)
        return True
    print("   !! 缺文件:", arc)
    return False


def add_dir(zf, src_dir, arc_prefix, skip_names=(), exts=None):
    n = 0
    for root, dirs, files in os.walk(src_dir):
        dirs[:] = [d for d in dirs if d not in ("__pycache__", ".git")]
        for f in files:
            if f in skip_names:
                continue
            if exts and os.path.splitext(f)[1].lower() not in exts:
                continue
            full = os.path.join(root, f)
            rel = os.path.relpath(full, src_dir).replace("\\", "/")
            zf.write(full, arc_prefix + "/" + rel)
            n += 1
    return n


def build(zip_path, entries, dirs=()):
    """entries = [(源文件名, zip 内路径)]；dirs = [(源目录, zip 内前缀)]"""
    os.makedirs(os.path.dirname(zip_path), exist_ok=True)
    n = 0
    with zipfile.ZipFile(zip_path, "w", zipfile.ZIP_DEFLATED) as zf:
        for name, arc in entries:
            if add_file(zf, find(name), arc):
                n += 1
        for src, pref in dirs:
            if os.path.isdir(src):
                n += add_dir(zf, src, pref)
            else:
                print("   !! 缺目录:", src)
    size = os.path.getsize(zip_path) / 1024.0 / 1024.0
    print("   -> %s  (%d 项, %.2f MB)" % (os.path.basename(zip_path), n, size))


IMG = os.path.join(D, "示意图")
PKG = os.path.join(D, "CRTC_3D打印包")
GCODE = os.path.join(D, "K1Max_G代码")
CAD = os.path.join(D, "给机械队友_CAD包")
STEP = os.path.join(CAD, "STEP_可直接编辑")

print("=== 顶层两个大包 ===")
build(os.path.join(D, "CRTC_3D打印包.zip"), [], [(PKG, "CRTC_3D打印包")])
build(os.path.join(D, "K1Max_G代码.zip"), [], [(GCODE, "K1Max_G代码")])

print("=== 分组发包 ===")
G = os.path.join(D, "分组发包")

# ① 打印负责人：整套 G-code
build(os.path.join(G, "①发给打印负责人_K1Max_G代码.zip"), [], [(GCODE, "K1Max_G代码")])

# ② 机械部：打印件说明 + 装配图 + 全部 CAD
build(os.path.join(G, "②发给机械部_打印件与装配图.zip"),
      [("平行夹爪_装配图.html", "平行夹爪_装配图.html"),
       ("齿轮夹爪_装配图.html", "齿轮夹爪_装配图.html"),
       ("打印件示意图_全部零件.html", "打印件示意图_全部零件.html"),
       ("打印件齐全性核对_一台车全在这了.md", "打印件齐全性核对_一台车全在这了.md"),
       ("打印顺序_机械臂还在改怎么打.md", "打印顺序_机械臂还在改怎么打.md"),
       ("明天去买的五金清单_两台车往多了备.md", "明天去买的五金清单_两台车往多了备.md"),
       ("给机械队友_怎么在SW和Fusion里打开.md", "给机械队友_怎么在SW和Fusion里打开.md"),
       ("装配图_照着装车.html", "装配图_照着装车.html"),
       ("附录_补件采购清单.md", "附录_补件采购清单.md"),
       ("机械臂_装配与运动范围.html", "机械臂_装配与运动范围.html")],
      dirs=[(IMG, "示意图"), (CAD, "给机械队友_CAD包")])

# ③ 电气同学：焊接排线（本轮未改动，重新打一遍保持一致）
build(os.path.join(G, "③发给电气同学_焊接排线与硬件布置.zip"),
      [("硬件接线与焊接指南_完整版.md", "硬件接线与焊接指南_完整版.md"),
       ("给电气同学_焊接排线与硬件布置.md", "给电气同学_焊接排线与硬件布置.md"),
       ("装配图_照着装车.html", "装配图_照着装车.html"),
       ("附录_布线图与分层布局图.html", "附录_布线图与分层布局图.html"),
       ("附录_补件采购清单.md", "附录_补件采购清单.md"),
       ("打印件示意图_全部零件.html", "打印件示意图_全部零件.html"),
       ("明天去买的五金清单_两台车往多了备.md", "明天去买的五金清单_两台车往多了备.md"),
       ("01_整车装配示意.png", "示意图/01_整车装配示意.png"),
       ("04_轮组与底板干涉的三条解法.png", "示意图/04_轮组与底板干涉的三条解法.png")])

# ④ 算法组：代码 + 调参 + 机械臂运动范围
build(os.path.join(G, "④发给算法组_代码与调参.zip"),
      [("CRTC_代码包_只有代码.zip", "CRTC_代码包_只有代码.zip"),
       ("机械臂_装配与运动范围.html", "机械臂_装配与运动范围.html"),
       ("机械臂是怎么动起来的.html", "机械臂是怎么动起来的.html"),
       ("各地形抓取可行性确认表.html", "各地形抓取可行性确认表.html"),
       ("调参指南.md", "调参指南.md"),
       ("需要你自己填的数据清单.md", "需要你自己填的数据清单.md"),
       ("01_整车装配示意.png", "示意图/01_整车装配示意.png"),
       ("04_轮组与底板干涉的三条解法.png", "示意图/04_轮组与底板干涉的三条解法.png"),
       ("05_机械臂_三姿态与限高.png", "示意图/05_机械臂_三姿态与限高.png")])

# ⑤ 建模同学：看图 + 可编辑模型
build(os.path.join(G, "⑤发给建模同学_零件图与整车示意.zip"),
      [("STEP_装配位置表.csv", "B_要改模型才用_STEP_装配位置表.csv"),
       ("STEP_零件厚度对照表.csv", "B_要改模型才用_STEP_零件厚度对照表.csv")])

# ⑤ 的 A_看图 目录要重命名图片（图片带前缀说明），所以手写一遍
with zipfile.ZipFile(os.path.join(G, "⑤发给建模同学_零件图与整车示意.zip"), "a",
                     zipfile.ZIP_DEFLATED) as zf:
    a_entries = [("图纸集_结构件.html", "A_看图_先开这个/图纸集_结构件_带尺寸.html"),
                 ("图纸集_外观套件.html", "A_看图_先开这个/图纸集_外观套件_带尺寸.html"),
                 ("平行夹爪_装配图.html", "A_看图_先开这个/平行夹爪_装配图.html"),
                 ("齿轮夹爪_装配图.html", "A_看图_先开这个/齿轮夹爪_装配图.html"),
                 ("机械臂_装配与运动范围.html", "A_看图_先开这个/机械臂_装配与运动范围.html"),
                 ("机械臂是怎么动起来的.html", "A_看图_先开这个/机械臂是怎么动起来的.html"),
                 ("给机械队友_怎么在SW和Fusion里打开.md", "A_看图_先开这个/给机械队友_怎么在SW和Fusion里打开.md"),
                 ("装配图_照着装车.html", "A_看图_先开这个/装配图_照着装车.html"),
                 ("01_整车装配示意.png", "A_看图_先开这个/示意图_01_整车装配示意.png"),
                 ("02_平行夹爪_连杆版.png", "A_看图_先开这个/示意图_02_平行夹爪_连杆版.png"),
                 ("03_平行夹爪_齿轮版.png", "A_看图_先开这个/示意图_03_平行夹爪_齿轮版.png"),
                 ("04_轮组与底板干涉的三条解法.png", "A_看图_先开这个/示意图_04_轮组与底板干涉的三条解法.png"),
                 ("05_机械臂_三姿态与限高.png", "A_看图_先开这个/示意图_05_机械臂_三姿态与限高.png")]
    for name, arc in a_entries:
        add_file(zf, find(name), arc)
    # DXF：只要 .dxf，外加 尺寸表.csv（不要自动带上 STEP 子目录）
    add_dir(zf, CAD, "B_要改模型才用_DXF", exts={".dxf"})
    add_file(zf, find("尺寸表.csv"), "B_要改模型才用_DXF/尺寸表.csv")
    add_dir(zf, STEP, "B_要改模型才用_STEP")
    # STL 目录里还留着旧「两连杆臂」的 5 个件（仓库保留用于追溯），
    # 但发给机械队友的包里**不能带**，否则会照着旧件干活。
    DEPRECATED_STL = {"09_arm_upper.stl", "10_arm_fore.stl", "11_servo_horn_plate.stl",
                      "11b_claw_adapter.stl", "25_arm_riser.stl"}
    add_dir(zf, os.path.join(R, "mechanical", "stl"), "B_要改模型才用_STL",
            skip_names=DEPRECATED_STL)

print("\n全部完成。")
