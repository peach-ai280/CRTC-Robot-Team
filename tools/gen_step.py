# -*- coding: utf-8 -*-
"""
把 gen_parts.py 建模出的零件导出成 STEP（*.step / *.stp）

为什么要有这个：
    队友要在 SolidWorks / Fusion360 里**打开并改动**我们的零件。
    STL 是三角网格，SW 导入后是"图形实体"，Fusion 里是 mesh —— 都**不能编辑特征**
    （不能改孔径、不能在上面继续画草图）。
    DXF 是 2D 轮廓，能编辑但要自己拉伸。
    STEP 是**实体**（B-rep），SW / Fusion / Creo / Rhino / FreeCAD 全都能直接打开、
    直接量尺寸、直接在上面打孔改形 —— 这才是给机械队友的正确格式。

原理：
    本工程所有零件都是「2D 轮廓（可能带孔）+ 沿 Z 拉伸」，所以我不需要 CAD 内核，
    直接用 shapely 的轮廓环写出 STEP 的 advanced_brep 实体：
        顶面 + 底面 + 每条边一个侧面 = 一个封闭壳（CLOSED_SHELL）
    这样导出的 STEP 是**精确几何**（不是三角面片近似），体积和 STL 应当完全一致。

⚠ 用 venv 跑（需要 numpy / shapely / numpy-stl）：
    C:\\Users\\hp\\.workbuddy\\binaries\\python\\envs\\default\\Scripts\\python.exe tools\\gen_step.py
    C:\\Users\\hp\\.workbuddy\\binaries\\python\\envs\\default\\Scripts\\python.exe tools\\gen_step.py --check
      （--check 会用 gmsh 把每个 STEP 读回来算体积，和 STL 的体积对账）

输出： D:\\WorkBuddy\\CRTC交付\\给机械队友_CAD包\\STEP_可直接编辑\\
"""
import os, sys, runpy, math, datetime

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

# ---------------------------------------------------------------- 加载建模脚本
# gen_parts.py 是脚本式的（顶层直接建模），用 run_path 跑一遍拿到 PARTS 列表。
# 副作用：会顺带把 stl / dxf / 图纸集.html 重新生成一遍（几何不变，只是刷新）。
GP = os.path.join(ROOT, "mechanical", "gen_parts.py")
g = runpy.run_path(GP)
PARTS = g["PARTS"]

OUT_DIR = r"D:\WorkBuddy\CRTC交付\给机械队友_CAD包\STEP_可直接编辑"
os.makedirs(OUT_DIR, exist_ok=True)


def f(v):
    return "%.6f" % float(v)


def esc(s):
    """STEP 是 ASCII 格式，中文要写成 \\X2\\XXXX\\X0\\（ISO 10303-21 的转义写法）"""
    out, buf = [], []
    for ch in str(s):
        if ord(ch) < 128:
            if buf:
                out.append("\\X2\\" + "".join(buf) + "\\X0\\")
                buf = []
            out.append(ch)
        else:
            buf.append("%04X" % ord(ch))
    if buf:
        out.append("\\X2\\" + "".join(buf) + "\\X0\\")
    return "".join(out).replace("'", "")


class W:
    """STEP 实体累加器（ISO-10303-21 物理文件）"""
    def __init__(self):
        self.lines = []
        self.n = 0
        self._pt = {}
        self._dir = {}
        self._ax = {}

    def add(self, s):
        self.n += 1
        self.lines.append("#%d = %s;" % (self.n, s))
        return self.n

    def point(self, x, y, z):
        k = (round(x, 6), round(y, 6), round(z, 6))
        if k in self._pt:
            return self._pt[k]
        i = self.add("CARTESIAN_POINT('',(%s,%s,%s))" % (f(x), f(y), f(z)))
        self._pt[k] = i
        return i

    def direction(self, x, y, z):
        k = (round(x, 6), round(y, 6), round(z, 6))
        if k in self._dir:
            return self._dir[k]
        i = self.add("DIRECTION('',(%s,%s,%s))" % (f(x), f(y), f(z)))
        self._dir[k] = i
        return i

    def axis(self, x, y, z, nx, ny, nz, rx, ry, rz):
        """AXIS2_PLACEMENT_3D（位置 + Z 轴法向 + 参考 X 方向）"""
        k = (round(x, 6), round(y, 6), round(z, 6),
             round(nx, 6), round(ny, 6), round(nz, 6))
        if k in self._ax:
            return self._ax[k]
        loc = self.point(x, y, z)
        ax = self.direction(nx, ny, nz)
        rd = self.direction(rx, ry, rz)
        i = self.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)" % (loc, ax, rd))
        self._ax[k] = i
        return i

    def plane(self, x, y, z, nx, ny, nz, rx, ry, rz):
        a = self.axis(x, y, z, nx, ny, nz, rx, ry, rz)
        return self.add("PLANE('',#%d)" % a)

    def polyloop(self, pts3):
        """pts3: [(x,y,z), ...] 至少 3 点，首尾不重复"""
        ids = [self.point(*p) for p in pts3]
        return self.add("POLY_LOOP('',(%s))" % ",".join("#%d" % i for i in ids))

    def face(self, loops, plane_id, outer_first=True):
        """loops: [loop_id, ...]，第一个是外环，其余是孔"""
        bs = []
        for j, lid in enumerate(loops):
            if j == 0 and outer_first:
                bs.append(self.add("FACE_OUTER_BOUND('',#%d,.T.)" % lid))
            else:
                bs.append(self.add("FACE_BOUND('',#%d,.T.)" % lid))
        return self.add("ADVANCED_FACE('',(%s),#%d,.T.)"
                        % (",".join("#%d" % b for b in bs), plane_id))


def _blocks(poly):
    """按连通块分组取出环：[[外环, 孔...], [外环, 孔...], ...]

    ⚠ 必须分组！poly 在减去一堆孔之后可能断开成 MultiPolygon
      （外购爪转接板 11b 就是这样）。如果把别的连通块当成"孔"挖掉，
      体积会凭空少一大块 —— 2026-10-05 踩过：11b 体积差了 68%。
    """
    geoms = list(poly.geoms) if poly.geom_type == "MultiPolygon" else [poly]
    blocks = []
    for gm in geoms:
        rings = [list(gm.exterior.coords)] + [list(r.coords) for r in gm.interiors]
        rs = []
        for r in rings:
            if len(r) > 1 and r[0] == r[-1]:
                r = r[:-1]
            if len(r) >= 3:
                rs.append([(float(p[0]), float(p[1])) for p in r])
        if rs:
            blocks.append(rs)
    return blocks


def shell_of(w, rings, z0, z1):
    """把一个连通块（外环 + 若干孔）挤出成 CLOSED_SHELL，返回 shell 实体号"""
    faces = []
    # 第一个环是外环（CCW），其余是孔（CW）—— 由 build() 的 orient 保证
    outer = rings[0]
    holes = rings[1:]

    # ---- 顶面（法向 +Z）：外环原序(CCW)、孔原序(CW)
    top = [w.polyloop([(x, y, z1) for x, y in outer])]
    for h in holes:
        top.append(w.polyloop([(x, y, z1) for x, y in h]))
    # 所有环都在同一平面，plane 只用一个（位置取外环首点）
    pl_top = w.plane(outer[0][0], outer[0][1], z1, 0, 0, 1, 1, 0, 0)
    faces.append(w.face(top, pl_top))

    # ---- 底面（法向 -Z）：所有环反向
    bot = [w.polyloop([(x, y, z0) for x, y in reversed(outer)])]
    for h in holes:
        bot.append(w.polyloop([(x, y, z0) for x, y in reversed(h)]))
    pl_bot = w.plane(outer[0][0], outer[0][1], z0, 0, 0, -1, 1, 0, 0)
    faces.append(w.face(bot, pl_bot))

    # ---- 侧壁：每条边一个面
    # 统一规则（已推导验证）：沿环方向取边 a→b，外法向 = (dy,-dx,0)
    #   外环 CCW  → 指向实体外侧 ✓
    #   孔环   CW  → 指向孔中心（即材料之外）✓
    for ring in rings:
        n = len(ring)
        for i in range(n):
            ax_, ay_ = ring[i]
            bx_, by_ = ring[(i + 1) % n]
            dx, dy = bx_ - ax_, by_ - ay_
            L = math.hypot(dx, dy)
            if L < 1e-9:
                continue
            nx, ny = dy / L, -dx / L
            loop = w.polyloop([(ax_, ay_, z0), (bx_, by_, z0),
                               (bx_, by_, z1), (ax_, ay_, z1)])
            pl = w.plane(ax_, ay_, z0, nx, ny, 0, 0, 0, 1)
            faces.append(w.face([loop], pl))

    return w.add("CLOSED_SHELL('',(%s))" % ",".join("#%d" % i for i in faces))


def write_step(path, name, poly, z0, z1, note=""):
    w = W()
    # ---- 几何（一个零件可能由多个连通块组成 → 多个实体）
    shells = [shell_of(w, b, z0, z1) for b in _blocks(poly)]
    breps = [w.add("MANIFOLD_SOLID_BREP('%s',#%d)" % (name, s)) for s in shells]

    # ---- 单位 / 精度上下文
    lu = w.add("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
    au = w.add("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )")
    sau = w.add("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )")
    unc = w.add("UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#%d,"
                "'distance_accuracy_value','')" % lu)
    ctx = w.add("( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
                "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#%d)) "
                "GLOBAL_UNIT_ASSIGNED_CONTEXT((#%d,#%d,#%d)) "
                "REPRESENTATION_CONTEXT('','') )" % (unc, lu, au, sau))

    # ---- 原点坐标系
    o = w.point(0, 0, 0)
    dz = w.direction(0, 0, 1)
    dx_ = w.direction(1, 0, 0)
    origin = w.add("AXIS2_PLACEMENT_3D('',#%d,#%d,#%d)" % (o, dz, dx_))

    items = ",".join("#%d" % i for i in [origin] + breps)
    sr = w.add("SHAPE_REPRESENTATION('',(%s),#%d)" % (items, ctx))

    # ---- 产品结构（给零件起名字，SW / Fusion 打开后能看到件名）
    ac = w.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')")
    p = w.add("PRODUCT('%s','%s','%s',(#%d))"
              % (name, name, esc(note),
                 w.add("PRODUCT_CONTEXT('',#%d,'mechanical')" % ac)))
    pdc = w.add("PRODUCT_DEFINITION_CONTEXT('part definition',#%d,'design')" % ac)
    pd = w.add("PRODUCT_DEFINITION('design','',#%d,#%d)" % (p, pdc))
    pds = w.add("PRODUCT_DEFINITION_SHAPE('','',#%d)" % pd)
    w.add("SHAPE_DEFINITION_REPRESENTATION(#%d,#%d)" % (pds, sr))

    now = datetime.datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
    head = (
        "ISO-10303-21;\n"
        "HEADER;\n"
        "FILE_DESCRIPTION((''),'2;1');\n"
        "FILE_NAME('%s.step','%s',('%s'),('%s'),'gen_step.py','gen_parts.py','');\n"
        "FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 -1 1 1 1 1 }'));\n"
        "ENDSEC;\n" % (name, now, esc("CRTC2026 机器人总动员战队"), esc("重庆大学"))
    )
    with open(path, "w", encoding="ascii", newline="\n") as fp:
        fp.write(head + "DATA;\n" + "\n".join(w.lines) + "\nENDSEC;\nEND-ISO-10303-21;\n")
    return path


# ---------------------------------------------------------------- 主流程
def main():
    check = "--check" in sys.argv
    rows = []
    for p in PARTS:
        z0, z1 = p["zs"]
        t = z1 - z0
        # 权威体积 = 2D 轮廓面积 × 厚度（轮廓是建模的唯一源头，不受网格缺陷影响）
        v_ref = p["poly"].area * t
        path = os.path.join(OUT_DIR, p["f"] + ".step")
        write_step(path, p["f"], p["poly"], z0, z1, p["cn"] + " " + (p["note"] or ""))
        rows.append((p["f"], p["cn"], t, p["vol"], v_ref, path))

    print("已导出 %d 个 STEP -> %s" % (len(rows), OUT_DIR))
    # 顺手写一份「件名 / 中文名 / 厚度」对照，队友照着拉伸用
    lst = os.path.join(os.path.dirname(OUT_DIR), "STEP_零件厚度对照表.csv")
    with open(lst, "w", encoding="utf-8-sig", newline="\n") as fp:
        fp.write("零件,中文名,厚度mm,实体体积mm3,STEP文件\n")
        for nm, cn, t, v_stl, v_ref, path in rows:
            fp.write("%s,%s,%.1f,%.1f,%s\n" % (nm, cn, t, v_ref, os.path.basename(path)))
    print("对照表:", lst)

    if check:
        import gmsh
        gmsh.initialize([])
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.option.setNumber("Geometry.OCCBoundsUseStl", 1)
        bad = 0
        for nm, cn, t, v_stl, v_ref, path in rows:
            gmsh.model.add("t")
            try:
                gmsh.model.occ.importShapes(path)
                gmsh.model.occ.synchronize()
                ents = gmsh.model.getEntities(3)
                vol = 0.0
                for dim, tag in ents:
                    vol += gmsh.model.occ.getMass(dim, tag)
                if not ents:
                    raise RuntimeError("读进来了但没识别成实体（0 个 solid）")
                gmsh.model.remove()
            except Exception as e:
                print("  !! %s 读不回来: %s" % (nm, e)); bad += 1; continue
            ok = abs(vol - v_ref) < max(1.0, v_ref * 0.01)
            if not ok:
                bad += 1
                print("  !! %-22s STEP %.1f vs 轮廓×厚度 %.1f  差 %.1f%%  (STL网格算的是 %.1f)"
                      % (nm, vol, v_ref, abs(vol - v_ref) / v_ref * 100, v_stl))
        gmsh.finalize()
        print("gmsh 回读校验：%d 件，%d 件异常（基准 = 2D轮廓面积 × 厚度）"
              % (len(rows), bad))


if __name__ == "__main__":
    main()
