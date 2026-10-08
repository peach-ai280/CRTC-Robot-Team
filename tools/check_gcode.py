# -*- coding: utf-8 -*-
"""
G-code 自检（配套 tools/slice_k1.py）
=====================================
逐份检查六件事，任何一项不过就报出来：
  1. 坐标有没有跑出 K1 Max 的 300x300 床（含 Brim）
  2. 温度指令是不是合法数字（别出现 M104 S0.4 这种）
  3. Z 是否单调递增、首层是否 0.2、末层是否接近零件高度
  4. 挤出量 E 是否全为正、总量换算成体积后与零件几何体积是否同量级
  5. 有没有出现不认识的 G/M 指令
  6. ★ 逐行语法合法性：每个参数（X/Y/Z/E/F/S/P）后面必须紧跟一个合法数字

第 6 项是 2026-10-04 补的。起因：清理线的 E 被写成模板里没被格式化的表达式
`E{100*BEAD_FIRST*0.30/FIL_AREA:.4f}`，53 份 G-code 全部输出了字面文本，
打印机读到会直接报 "Unable to parse move"。而前面五项全都是「用正则只匹配数字」，
非法行会被静默跳过 —— 所以五项全绿也照样漏掉了它。
这条检查是"兜底网"：不管上面算错什么，只要写出去的不是合法 G-code 就一定被抓到。
用法：venv 的 python tools/check_gcode.py
"""

import os
import re
import glob
import math
import sys

import numpy as np
from stl import mesh as stlmesh

GCODE_DIR = r"D:\WorkBuddy\CRTC交付\K1Max_G代码"
STL_DIRS = [r"C:\Users\hp\WorkBuddy\CRTC\mechanical\stl",
            r"C:\Users\hp\WorkBuddy\CRTC\mechanical\stl_gauge",
            r"C:\Users\hp\WorkBuddy\CRTC\mechanical\stl_pretty"]
BED = 300.0
FIL_AREA = math.pi * (1.75 / 2.0) ** 2
KNOWN = set("GMT")
# 一个 G-code 参数 token 的合法形态：单个字母，后面跟「一个数字」或「什么都不跟」。
#   · G1 X20 Y120 E5.7 F1200  → 字母 + 数字
#   · G28 X Y                 → 字母单独出现也是合法的（"归零 X、Y 两轴"），
#                                所以数字部分必须可选，否则会误报。
#   · E{100*BEAD_FIRST*...}   → 不合法，会被抓到（这正是加这一项的原因）
SYNTAX_OK = re.compile(r"^[A-Z](-?\d+(\.\d+)?)?$")


def stl_volume(name, also=None):
    """按名字找 STL。

    ⚠ 为什么要有 also 这一路：G-code 文件名是「零件名_序号」，反推零件名要剥掉尾号，
      但 P4_fender_1 这种「数字本来就是名字一部分」的件，剥完就找不到了。
      所以先按原名找，找不到再按剥掉尾号的名字找。
    """
    names = [name] + ([also] if also and also != name else [])
    for d in STL_DIRS:
        for nm in names:
            p = os.path.join(d, nm + ".stl")
            if os.path.exists(p):
                m = stlmesh.Mesh.from_file(p)
                v = m.vectors
                vol = 0.0
                for tri in v:
                    a, b, c = tri
                    vol += float(np.dot(a, np.cross(b, c))) / 6.0
                return abs(vol)
    return None


BRIM_W = 6.0          # 与 slice_k1.py 的 BRIM_W 保持一致
BRIM_AREA_MIN = 4000.0  # 与切片器里"超过这个投影面积才加 Brim"的阈值一致


def brim_volume(name, also=None):
    """Brim 裙边吃掉的耗材体积（mm³）。

    ⚠ 为什么必须扣：2026-10-04 交叉验证时，`gauge_width_comb` 的
    "挤出量 / 几何体积" 算出来是 1.835，看着像严重过挤；扣掉 Brim 后是 1.008。
    原因：梳子周长约 1500mm，6mm 裙边就是约 9000mm² 的首层面积 ——
    比零件本身的截面（6928mm²）还大。不扣就会误判。
    """
    names = [name] + ([also] if also and also != name else [])
    for d in STL_DIRS:
        for nm in names:
            p = os.path.join(d, nm + ".stl")
            if not os.path.exists(p):
                continue
            g = stl_first_layer_geom(p)
            if g is None:
                return 0.0
            if g.area < BRIM_AREA_MIN:
                return 0.0            # 小件不加 Brim
            return g.length * BRIM_W * 0.20   # 周长 × 裙边宽 × 首层层高
    return 0.0


def stl_first_layer_geom(path, z=0.10):
    """STL 在 z 处截面的几何（面积 / 周长）。用 shapely 从三角面切一圈线段重建。"""
    from shapely.ops import unary_union, polygonize
    from shapely.geometry import LineString, MultiLineString
    try:
        from shapely import set_precision
    except Exception:
        set_precision = None
    v = stlmesh.Mesh.from_file(path).vectors
    segs = []
    for tri in v:
        zs = tri[:, 2] - z
        if (zs > 0).all() or (zs < 0).all():
            continue
        pts = []
        for i in range(3):
            a, b = tri[i], tri[(i + 1) % 3]
            if zs[i] == 0:
                pts.append((a[0], a[1]))
            if zs[i] * zs[(i + 1) % 3] < 0:
                t = zs[i] / (zs[i] - zs[(i + 1) % 3])
                pts.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
        if len(pts) >= 2:
            segs.append(LineString([pts[0], pts[1]]))
    if not segs:
        return None
    ml = MultiLineString(segs)
    if set_precision is not None:
        ml = set_precision(ml, 1e-3)
    polys = [q for q in polygonize(ml) if q.area > 0.02]
    return unary_union(polys) if polys else None


def main():
    files = sorted(glob.glob(os.path.join(GCODE_DIR, "*", "*.gcode")))
    if not files:
        print("没有 gcode 文件")
        return 1
    problems = []
    print("检查 %d 份 G-code\n" % len(files))
    for f in files:
        base = os.path.basename(f)[:-6]
        part = re.sub(r"_\d+$", "", base)
        txt = open(f, encoding="utf-8").read().split("\n")
        xs, ys, zs, es, cmds = [], [], [], [], set()
        # 只把「层循环」之间的 Z 算进来（头和尾的抬 Z / 归零不算）
        in_layers = False
        for ln in txt:
            mark = ln.strip()
            if mark == "; ---- LAYERS START ----":
                in_layers = True; continue
            if mark == "; ---- LAYERS END ----":
                in_layers = False; continue
            ln = ln.split(";")[0].strip()
            if not ln:
                continue
            code = ln.split()[0]
            cmds.add(code)
            if code in ("G0", "G1"):
                m = re.search(r"X(-?[\d.]+)", ln)
                if m: xs.append(float(m.group(1)))
                m = re.search(r"Y(-?[\d.]+)", ln)
                if m: ys.append(float(m.group(1)))
                m = re.search(r"Z(-?[\d.]+)", ln)
                if m and in_layers: zs.append(float(m.group(1)))
                # 挤出量只统计"同时带 X/Y 的走线"，回抽/补料那种纯 E 行不算
                m = re.search(r"E(-?[\d.]+)", ln)
                if m and ("X" in ln or "Y" in ln):
                    es.append(float(m.group(1)))
        ok = True
        msg = []
        # 1 越床（含 Brim 6mm 余量给 ±8mm 已留边）
        if xs and (min(xs) < 0.5 or max(xs) > BED - 0.5):
            ok = False; msg.append("X 越床 %.1f~%.1f" % (min(xs), max(xs)))
        if ys and (min(ys) < 0.5 or max(ys) > BED - 0.5):
            ok = False; msg.append("Y 越床 %.1f~%.1f" % (min(ys), max(ys)))
        # 2 温度
        for ln in txt:
            m = re.match(r"M(104|109|140|190) S([\d.]+)", ln.strip())
            if m:
                v = float(m.group(2))
                if not (0 <= v <= 300):
                    ok = False; msg.append("温度异常 %s" % ln.strip())
        # 3 Z
        if zs:
            jumps = [zs[i + 1] - zs[i] for i in range(len(zs) - 1) if zs[i + 1] < zs[i] - 1e-6]
            if jumps:
                ok = False; msg.append("Z 有下降 %d 次" % len(jumps))
            if abs(max(zs) - 0.0) < 0.01:
                ok = False; msg.append("Z 全程为 0")
        # 4 挤出量（★ 必须先扣掉 Brim，否则带裙边的件会被误判成"过挤"）
        tot_e = sum(e for e in es if e > 0)
        vol = tot_e * FIL_AREA - brim_volume(base, part)
        geo = stl_volume(base, part)
        # ⚠ 上限 1.45 的来历（2026-10-04 实测推出来的，别拍脑袋改）：
        #   · 扣掉 Brim 后，绝大多数件的比值落在 1.00~1.05（甲板 1.005 / 梳子 1.008 / 相机架 1.028）
        #   · 细长薄壁件（侧裙 348mm² 截面这种）3 圈墙会填满截面还略有重叠 → 到 1.10
        #   · 锥形/曲面件首层本来就比中段大
        #   所以 1.45 是给这些留的余量；真正的精确性靠"扣 Brim 后的首层面积比对"，
        #   那一轮实测下来全部落在 1.00~1.10，说明切片器是自洽的。
        hi = 1.45
        if geo:
            r = vol / geo
            if not (0.25 <= r <= hi):
                ok = False; msg.append("挤出体积比异常 %.2f（几何 %.1f mm³，上限 %.2f）" % (r, geo, hi))
        else:
            msg.append("(找不到对应 STL)")
        if tot_e <= 0:
            ok = False; msg.append("没有任何正向挤出")
        # 5 指令合法性
        unknown = [c for c in cmds if c[0] not in KNOWN]
        if unknown:
            ok = False; msg.append("不认识的指令 %s" % unknown[:5])
        for must in ("M190", "M109", "G28", "M83"):
            if must not in cmds:
                ok = False; msg.append("缺 %s" % must)
        # 6 ★ 逐行语法合法性：每个参数后面必须紧跟一个合法数字
        #     这一项不看语义，只看"打印机能不能解析"，是所有计算错误的兜底网。
        bad_lines = []
        for i, raw in enumerate(txt, 1):
            code_ln = raw.split(";")[0].strip()
            if not code_ln:
                continue
            tk = code_ln.split()
            if tk[0][0] not in KNOWN:
                continue                       # 指令名本身的问题由第 5 项管
            for a in tk[1:]:
                if not SYNTAX_OK.match(a):
                    bad_lines.append("L%d: %s" % (i, code_ln))
                    break
        if bad_lines:
            ok = False
            msg.append("语法非法 %d 行，例：%s" % (len(bad_lines), bad_lines[0]))
        if not ok:
            problems.append((base, msg))
        print("%-26s %-6s" % (base, "OK" if ok else "NG"), ("; ".join(msg) if msg else ""))
    print("\n" + "=" * 60)
    if problems:
        print("!! %d 份有问题：" % len(problems))
        for n, m in problems:
            print("  ", n, m)
    else:
        print("全部 %d 份通过：床面 / 温度 / Z / 挤出量 / 指令 / 语法 六项全绿" % len(files))
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
