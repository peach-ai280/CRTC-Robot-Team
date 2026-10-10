# -*- coding: utf-8 -*-
"""
CRTC2026 装配示意图渲染器
=====================================================================
用真实的 STL 做离线软渲染（纯 numpy + PIL，不需要 OpenGL / matplotlib），
所以图里的每一块形状，就是你拿去切片打印的那一块，不是画出来的示意图。

跑法（在仓库根目录执行）：
  D:\\WorkBuddy\\tools\\pyrender\\Scripts\\python.exe tools\\render_asm.py

输出：
  D:\\WorkBuddy\\CRTC交付\\示意图\\01_整车装配示意.png
  D:\\WorkBuddy\\CRTC交付\\示意图\\02_平行夹爪_连杆版.png
  D:\\WorkBuddy\\CRTC交付\\示意图\\03_平行夹爪_齿轮版.png
"""

import os
import math
import numpy as np
from stl import mesh
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STL_DIR = os.path.join(ROOT, "mechanical", "stl")
OUT_DIR = r"D:\WorkBuddy\CRTC交付\示意图"
FONT_PATH = "C:/Windows/Fonts/msyh.ttc"

# ---------------------------------------------------------------- 颜色
C_PLA = (232, 230, 224)      # 本色打印件
C_DECK = (214, 208, 196)     # 甲板
C_ARM = (228, 118, 46)       # 机械臂 / 增高座（橙）
C_CLAW = (79, 163, 199)      # 夹爪（蓝）
C_BIN = (159, 179, 160)      # 储仓（灰绿）
C_SCOOP = (224, 163, 62)     # 收集铲（黄）
C_COL = (185, 179, 166)      # 立柱
C_METAL = (107, 111, 117)    # 金属底板
C_WHEEL = (51, 55, 60)       # 麦轮
C_MOTOR = (85, 90, 96)       # TT 马达
C_BATT = (46, 125, 91)       # 18650 电池
C_PCB = (46, 139, 87)        # STM32 板
C_DRV = (192, 57, 43)        # DRV8833
C_PS2 = (142, 68, 173)       # PS2 接收模块
C_SERVO = (43, 95, 158)      # 舵机
C_WIRE = (60, 64, 72)        # 线束
C_CUBE = (250, 250, 250)     # 能量块道具（白）
C_PROBE = (214, 90, 140)     # 洞窟探杆（洋红，醒目）

# ---------------------------------------------------------------- 几何工具


def stl_tris(name):
    """读一个 STL，返回 (M,3,3) 三角面（世界单位 mm）"""
    m = mesh.Mesh.from_file(os.path.join(STL_DIR, name + ".stl"))
    return m.vectors.astype(np.float64)


def _rot(axis, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def xform(tris, local_rz=0.0, amap=("x", "y", "z"), rot=(0.0, 0.0, 0.0),
          trans=(0.0, 0.0, 0.0)):
    """
    local_rz : 先绕零件自己的 z 轴转（度）—— 摇臂转角、齿条掉头都用它
    amap     : 轴映射，('x','z','y') 表示 零件y -> 世界z、零件z -> 世界y（把平放件立起来）
    rot      : 绕世界轴 X->Y->Z 依次旋转（度）
    trans    : 平移
    """
    p = np.asarray(tris, dtype=np.float64).reshape(-1, 3)
    if local_rz:
        p = p @ _rot("z", local_rz).T
    M = np.zeros((3, 3))
    for j, a in enumerate(amap):
        sgn = -1.0 if a.startswith("-") else 1.0
        M["xyz".index(a.lstrip("-")), j] = sgn
    p = p @ M.T
    R = _rot("z", rot[2]) @ _rot("y", rot[1]) @ _rot("x", rot[0])
    p = p @ R.T
    p = p + np.asarray(trans, dtype=np.float64)
    return p.reshape(-1, 3, 3)


def box(cx, cy, cz, sx, sy, sz):
    """长方体（中心 + 三边尺寸）"""
    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    v = np.array([
        [cx - hx, cy - hy, cz - hz], [cx + hx, cy - hy, cz - hz],
        [cx + hx, cy + hy, cz - hz], [cx - hx, cy + hy, cz - hz],
        [cx - hx, cy - hy, cz + hz], [cx + hx, cy - hy, cz + hz],
        [cx + hx, cy + hy, cz + hz], [cx - hx, cy + hy, cz + hz],
    ])
    f = [(0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6), (0, 4, 5), (0, 5, 1),
         (1, 5, 6), (1, 6, 2), (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0)]
    return v[np.array(f)]


def cyl(cx, cy, cz, axis, r, L, seg=26):
    """圆柱：axis 决定轴向，L 为长度，中心在 (cx,cy,cz)"""
    ax = "xyz".index(axis)
    a1, a2 = [i for i in range(3) if i != ax]
    t = np.linspace(0, 2 * math.pi, seg, endpoint=False)
    dirs = np.zeros((seg, 3))
    dirs[:, a1] = np.cos(t) * r
    dirs[:, a2] = np.sin(t) * r
    c = np.array([cx, cy, cz], dtype=float)
    half = np.zeros(3)
    half[ax] = L / 2.0
    bot = c - half + dirs
    top = c + half + dirs
    tris = []
    for i in range(seg):
        j = (i + 1) % seg
        tris.append([bot[i], bot[j], top[j]])
        tris.append([bot[i], top[j], top[i]])
        tris.append([c - half, top[j], top[i]])
        tris.append([c + half, bot[i], bot[j]])
    return np.array(tris)


# ---------------------------------------------------------------- 渲染器


def _cam_basis(eye, target, up):
    fwd = np.asarray(target, float) - np.asarray(eye, float)
    fwd /= np.linalg.norm(fwd)
    up = np.asarray(up, float)
    right = np.cross(fwd, up)
    right /= np.linalg.norm(right)
    cup = np.cross(right, fwd)
    return np.stack([right, cup, fwd])


def render(items, eye, target, W, H, f_ratio=1.9, up=(0.0, 0.0, 1.0),
           bg=(244, 245, 247), ss=2, grid_ext=260.0, grid_step=40.0):
    """items = [(tris, color), ...]；弱透视 + z-buffer + 双面 Lambert"""
    W2, H2 = W * ss, H * ss
    f = f_ratio * H2
    img = Image.new("RGB", (W2, H2), bg)
    drw = ImageDraw.Draw(img)
    basis = _cam_basis(eye, target, up)
    eye = np.asarray(eye, float)

    def proj(p):
        q = (np.atleast_2d(np.asarray(p, float)) - eye) @ basis.T
        z = q[:, 2]
        z = np.where(z < 1.0, 1.0, z)
        return np.stack([W2 / 2 + f * q[:, 0] / z, H2 / 2 - f * q[:, 1] / z], 1), z

    # 地面网格
    n = int(grid_ext / grid_step)
    for i in range(-n, n + 1):
        v = i * grid_step
        for seg in (((-grid_ext, v, 0), (grid_ext, v, 0)),
                    ((v, -grid_ext, 0), (v, grid_ext, 0))):
            s, _ = proj(seg)
            drw.line([tuple(s[0]), tuple(s[1])], fill=(214, 217, 221), width=1)

    zbuf = np.full((H2, W2), np.inf)
    cbuf = np.zeros((H2, W2, 3), np.uint8)
    L = np.array([0.42, -0.58, 0.70])
    L /= np.linalg.norm(L)

    for tris, color in items:
        tris = np.asarray(tris, float)
        v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
        nrm = np.cross(v1 - v0, v2 - v0)
        ln = np.linalg.norm(nrm, axis=1)
        good = ln > 1e-9
        nrm = np.where(ln[:, None] > 1e-9, nrm / (ln[:, None] + 1e-9), 0.0)
        shade = 0.34 + 0.66 * np.abs(nrm @ L)
        pc = (tris - eye) @ basis.T          # (M,3,3)
        zc = pc[:, :, 2]
        ok = good & (zc.min(1) > 2.0)
        if not ok.any():
            continue
        zc = np.where(zc < 2.0, 2.0, zc)
        sx = W2 / 2 + f * pc[:, :, 0] / zc
        sy = H2 / 2 - f * pc[:, :, 1] / zc
        base = np.asarray(color, float)
        for k in np.nonzero(ok)[0]:
            x0, y0 = sx[k, 0], sy[k, 0]
            x1, y1 = sx[k, 1], sy[k, 1]
            x2, y2 = sx[k, 2], sy[k, 2]
            area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
            if abs(area) < 1e-9:
                continue
            minx = max(int(min(x0, x1, x2)) - 1, 0)
            maxx = min(int(max(x0, x1, x2)) + 1, W2 - 1)
            miny = max(int(min(y0, y1, y2)) - 1, 0)
            maxy = min(int(max(y0, y1, y2)) + 1, H2 - 1)
            if minx > maxx or miny > maxy:
                continue
            xs = np.arange(minx, maxx + 1)
            ys = np.arange(miny, maxy + 1)
            X = xs[None, :]
            Y = ys[:, None]
            w0 = ((x1 - X) * (y2 - Y) - (x2 - X) * (y1 - Y)) / area
            w1 = ((x2 - X) * (y0 - Y) - (x0 - X) * (y2 - Y)) / area
            w2 = 1.0 - w0 - w1
            inside = (w0 >= -0.002) & (w1 >= -0.002) & (w2 >= -0.002)
            if not inside.any():
                continue
            invz = (w0 / zc[k, 0] + w1 / zc[k, 1] + w2 / zc[k, 2])
            dep = 1.0 / np.maximum(invz, 1e-9)
            dep = np.where(inside, dep, np.inf)
            sub = zbuf[miny:maxy + 1, minx:maxx + 1]
            hit = dep < sub
            if not hit.any():
                continue
            col = (base * shade[k]).clip(0, 255).astype(np.uint8)
            sub[hit] = dep[hit]
            cbuf[miny:maxy + 1, minx:maxx + 1][hit] = col

    img = Image.fromarray(cbuf, "RGB")
    if ss != 1:
        img = img.resize((W, H), Image.LANCZOS)
    return img, basis, proj


# ---------------------------------------------------------------- 标注


def annotate(img, eye, target, W, H, notes, f_ratio=1.9, up=(0.0, 0.0, 1.0),
             font_size=22, title=None, subtitle=None):
    drw = ImageDraw.Draw(img)
    fnt = ImageFont.truetype(FONT_PATH, font_size)
    fnt_s = ImageFont.truetype(FONT_PATH, int(font_size * 0.82))
    fnt_t = ImageFont.truetype(FONT_PATH, int(font_size * 1.45))
    basis = _cam_basis(eye, target, up)
    eye = np.asarray(eye, float)
    f = f_ratio * H

    def scr(p):
        q = (np.asarray(p, float) - eye) @ basis.T
        z = max(q[2], 1.0)
        return (W / 2 + f * q[0] / z, H / 2 - f * q[1] / z)

    for item in notes:
        pt = scr(item[0])
        dx, dy = item[2], item[3]
        tx, ty = pt[0] + dx, pt[1] + dy
        lines = item[1].split("\n")
        lh = int(font_size * 1.32)
        # 多行文本的整体包围盒
        w_max = max(drw.textlength(s, font=fnt) for s in lines)
        if dx < 0:
            tx = tx - w_max
        # 夹到画布内，防止文字被裁掉
        tx = min(max(tx, 8.0), W - 8.0 - w_max)
        ty = min(max(ty, 8.0), H - 8.0 - (lh * (len(lines) - 1) + font_size))
        bbtop = ty
        bbbot = ty + lh * (len(lines) - 1) + font_size
        pad = 5
        drw.rectangle([tx - pad, bbtop - pad // 2, tx + w_max + pad, bbbot + pad // 2],
                      fill=(255, 255, 255))
        drw.line([pt[0], pt[1], tx if dx >= 0 else tx + (w_max if dx < 0 else 0),
                  bbtop + (bbbot - bbtop) / 2],
                 fill=(70, 74, 80), width=2)
        drw.ellipse([pt[0] - 3.5, pt[1] - 3.5, pt[0] + 3.5, pt[1] + 3.5],
                    fill=(228, 118, 46), outline=(255, 255, 255), width=1)
        for i, s in enumerate(lines):
            drw.text((tx, ty + i * lh), s, font=fnt, fill=(35, 38, 42))

    if title:
        drw.text((26, 22), title, font=fnt_t, fill=(25, 27, 30))
    if subtitle:
        drw.text((28, 22 + int(font_size * 1.9)), subtitle, font=fnt_s, fill=(90, 95, 100))
    return img


def footer(img, lines, font_size=21):
    """图下方写一段说明（按像素宽度自动换行，不用手数汉字）"""
    fnt = ImageFont.truetype(FONT_PATH, font_size)
    W, H = img.size
    pad = 26
    maxw = W + 100 - 2 * pad
    wrapped = []
    nopunct = "。，、；：）】》”！？%"
    for s in lines:
        cur = ""
        for ch in s:
            if fnt.getlength(cur + ch) > maxw and cur:
                if ch in nopunct:            # 标点别掉到行首
                    wrapped.append(cur + ch)
                    cur = ""
                    continue
                wrapped.append(cur)
                cur = ch
            else:
                cur += ch
        wrapped.append(cur)
    lh = int(font_size * 1.5)
    h = len(wrapped) * lh + pad
    canvas = Image.new("RGB", (W + 100, H + h), (250, 250, 251))
    canvas.paste(img, (50, 0))
    d2 = ImageDraw.Draw(canvas)
    for i, s in enumerate(wrapped):
        d2.text((pad + 24, H + 12 + i * lh), s, font=fnt, fill=(55, 58, 62))
    return canvas


def paste_inset(img, small, corner="rb", margin=26, title=None):
    """把一张小插图贴到主图角落（rb 右下 / lb 左下 / rt 右上 / lt 左上）"""
    W, H = img.size
    w, h = small.size
    x = margin if corner in ("lb", "lt") else W - w - margin
    y = margin + 34 if corner in ("lt", "rt") else H - h - margin
    img.paste(small, (x, y))
    d = ImageDraw.Draw(img)
    d.rectangle([x - 2, y - 2, x + w + 1, y + h + 1], outline=(120, 124, 130), width=3)
    if title:
        f = ImageFont.truetype(FONT_PATH, 20)
        d.rectangle([x - 2, y - 36, x + w + 1, y - 3], fill=(235, 237, 240))
        d.text((x + 8, y - 32), title, font=f, fill=(35, 38, 42))
    return img


def claw_link_core(open_state=True):
    """连杆版内部机构（隐去爪指/滑块）：底板 + 轴 + 摇臂 + 两根交叉连杆"""
    r, L, xs = 13.5, 28.5, (25.1 if open_state else 15.0)
    ang0 = 90.0 if open_state else 0.0
    th = math.radians(ang0)
    out = [(xform(stl_tris("24a_claw_base")), C_DECK),
           (cyl(0.0, 0.0, 1.0, "z", 2.6, 16.0), C_MOTOR),
           (xform(stl_tris("24d_claw_horn"), local_rz=ang0, trans=(0, 0, -4.0)), C_CLAW)]
    for sg in (1, -1):
        A = np.array([sg * r * math.cos(th), sg * r * math.sin(th)])
        B = np.array([-sg * xs, 0.0])
        mid = (A + B) / 2.0
        ang = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))
        out.append((xform(stl_tris("24e_claw_link"), local_rz=ang,
                          trans=(mid[0], mid[1], -11.0)), C_ARM))
    return out


def claw_gear_core(open_state=True):
    """齿轮版内部机构（隐去爪指）：底板 + 齿轮 + 两条齿条"""
    off = 12.4 if open_state else 3.0
    return [(xform(stl_tris("24h_claw_base_gear")), C_DECK),
            (cyl(0.0, 0.0, 1.0, "z", 2.6, 16.0), C_MOTOR),
            (xform(stl_tris("24f_claw_gear"), local_rz=20.0, trans=(0, 0, -6.0)), C_ARM),
            (xform(stl_tris("24g_claw_rack"), trans=(off, 0, -8.0)), C_CLAW),
            (xform(stl_tris("24g_claw_rack"), local_rz=180.0, trans=(-off, 0, -8.0)), C_CLAW)]


# ================================================================ 1. 整车


# ================================================================ 3b. 柱坐标机械臂
# 这套参数必须和 mechanical/gen_parts.py 的 5.7 节**逐字一致**，改了要两边一起改。
CY_DECK_TOP  = 88.5     # 甲板面离地（**已含 4mm 板厚**）—— 爪尖绝不允许低于这个数
CY_BASE_L    = 64.0     # 回转底座外廓（沿 x）；座顶 128.5
CY_BASE_W    = 44.0     # 回转底座外廓（沿 y）★ 刻意做窄：爪内侧指 23.6 > 22 才擦得过去
CY_BASE_H    = 40.0     # 回转底座高
CY_TT_D      = 64.0     # 回转盘长（沿 x）
CY_TT_W      = 44.0     # 回转盘宽（沿 y）
CY_TT_T      = 6.0      # 回转盘厚
CY_MAST_H    = 62.65    # 立柱高 → 柱顶 197.15
CY_CAP_W     = 60.0     # 顶帽外廓
CY_CAP_T     = 6.0      # 顶帽厚 → 帽顶 203.15
CY_CAR_H     = 32.0     # 滑座高
CY_BOOM_X0   = 26.0     # 横臂起点（= 滑座外壁）
CY_BOOM_X1   = 64.0     # 横臂末端
CY_BOOM_Z    = 24.0     # 横臂高
CY_BOOM_Y    = 11.0     # 横臂/曲柄/连杆所在的 y 起点（横臂占 y∈[11,19]）
CY_BOOM_L    = 52.0     # ★ 爪中心离柱轴的水平距离（第三次改定）
CY_BOOM_PIN  = 10.0     # 横臂销孔离横臂起点的距离
CY_CLAW_DROP = 46.0     # ★ 横臂底面 → 爪指尖的落差（爪架 4 + 齿条 8 + 爪指 34）
CY_CRANK_R   = 15.25    # 曲柄半径 → 行程 2R = 30.5
CY_LINK_L    = 48.0     # 连杆孔距（定曲柄中心高 = 137.75 + 48 = 185.75）
CY_AX        = 95.0     # 回转轴在车长方向的位置
CY_LIFT_MAX  = 2.0 * CY_CRANK_R          # 30.5

CY_BASE_Z    = CY_DECK_TOP               # 88.5   底座底
CY_TT_Z      = CY_BASE_Z + CY_BASE_H     # 128.5  回转盘底
CY_MAST_Z    = CY_TT_Z + CY_TT_T         # 134.5  立柱底（= 回转盘顶）
CY_MAST_TOP  = CY_MAST_Z + CY_MAST_H     # 197.15 立柱顶
CY_CAP_Z     = CY_MAST_TOP               # 197.15 顶帽底
CY_CRANK_Z   = CY_CAP_Z - 11.4           # 185.75 曲柄/升降舵机轴心高
CY_PIN_X     = CY_AX + CY_BOOM_X0 + CY_BOOM_PIN   # 131.0 横臂销 x（= 曲柄中心 x）
CY_LINK_Y    = 20.0                      # 连杆在 y∈[20,26]，刻意离开横臂那一层


def cy_pose(lift=0.0):
    """给一个升降量 lift（0 = 最低，30.5 = 最高），返回这套臂的四个 z 基准"""
    car_bot = CY_MAST_Z + lift                  # 滑座底面
    boom_bot = car_bot - CY_BOOM_Z              # 横臂底面
    boom_zc = boom_bot + CY_BOOM_Z / 2.0        # 横臂中面
    pin_z = boom_zc                             # 横臂销孔在横臂中面上
    claw_z0 = boom_bot + 42.0 - CY_CLAW_DROP     # 爪底板顶面（claw_gear_items 里爪尖在 z0−42）
    return car_bot, boom_zc, pin_z, claw_z0


def crank_phi(lift):
    """由升降量反解曲柄角 φ（度）。
       z_s = CY_CRANK_Z + R·sinφ − sqrt(L² − (R·cosφ)²)   （φ ∈ [−90, 90] 单调升）
    """
    _, _, pin_z, _ = cy_pose(lift)
    lo, hi = -90.0, 90.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        a = math.radians(mid)
        z = (CY_CRANK_Z + CY_CRANK_R * math.sin(a)
             - math.sqrt(CY_LINK_L ** 2 - (CY_CRANK_R * math.cos(a)) ** 2))
        if z < pin_z:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2.0


def _crank_pin(phi):
    a = math.radians(phi)
    return (CY_PIN_X + CY_CRANK_R * math.cos(a),
            CY_CRANK_Z + CY_CRANK_R * math.sin(a))


def _yaw_about(tris, yaw_deg):
    """把已经摆好的零件绕「回转轴那根竖直轴」转 yaw 度"""
    R = _rot("z", yaw_deg)
    c = np.array([CY_AX, 0.0, 0.0])
    p = np.asarray(tris, dtype=np.float64).reshape(-1, 3)
    return ((p - c) @ R.T + c).reshape(-1, 3, 3)


def arm_items(lift=0.0, yaw_deg=0.0, open_state=False, show_claw=True):
    """
    柱坐标机械臂整套（含平行爪）。
      lift    : 升降量 0 ~ 30.5mm（0 = 爪最低）
      yaw_deg : 0 朝车头（取块）｜90 朝车侧（**检录姿态**）｜180 朝车尾（投放）
    """
    out = []
    car_bot, boom_zc, pin_z, claw_z0 = cy_pose(lift)
    phi = crank_phi(lift)

    # 30 回转底座 —— 不随 yaw 转（舵机在里面，柱体在它上面转）
    out.append((xform(stl_tris("30_arm_base"), trans=(CY_AX, 0.0, CY_BASE_Z)), C_ARM))

    # ---- 以下都随 yaw 转
    sw = [
        (xform(stl_tris("31a_arm_turntable"), trans=(CY_AX, 0.0, CY_TT_Z)), C_ARM),
        (xform(stl_tris("31b_arm_mast"), trans=(CY_AX, 0.0, CY_MAST_Z)), C_ARM),
        (xform(stl_tris("32_arm_carriage"), trans=(CY_AX, 0.0, car_bot)), C_ARM),
        (xform(stl_tris("33_arm_boom"), amap=("x", "z", "y"),
               trans=(CY_AX, CY_BOOM_Y, boom_zc)), C_ARM),
        (xform(stl_tris("36_arm_lift_cap"), trans=(CY_AX, 0.0, CY_CAP_Z)), C_CLAW),
        # 34 曲柄：绕水平轴（世界 y）转 φ
        (xform(stl_tris("34_arm_lift_crank"), amap=("x", "z", "y"),
               rot=(0.0, phi, 0.0), trans=(CY_PIN_X, CY_BOOM_Y, CY_CRANK_Z)), C_MOTOR),
    ]
    # 35 连杆：从曲柄销连到横臂销
    px, pz = _crank_pin(phi)
    dx, dz = CY_PIN_X - px, pin_z - pz
    ang = math.degrees(math.atan2(-dz, dx))
    sw.append((xform(stl_tris("35_arm_lift_link"), amap=("x", "z", "y"),
                     rot=(0.0, ang, 0.0),
                     trans=((px + CY_PIN_X) / 2.0, CY_LINK_Y, (pz + pin_z) / 2.0)),
               C_CLAW))
    # 升降舵机（吊在顶帽下面，轴沿 y）
    sw.append((box(CY_PIN_X + 0.3, -0.4, CY_CRANK_Z, 12.6, 22.8, 22.8), C_SERVO))
    for t, c in sw:
        out.append((_yaw_about(t, yaw_deg), c))

    # 两根销轴（M3×25 + 防松螺母）
    for py0 in (CY_BOOM_Y, CY_LINK_Y + 3.0):
        out.append((_yaw_about(cyl(CY_PIN_X, py0, pin_z, "y", 1.6, 16.0),
                               yaw_deg), C_MOTOR))
    # 回转舵机（塞在底座内腔里）+ 回转盘上的舵机
    out.append((box(CY_AX, 0.0, CY_BASE_Z + 17.0, 23.0, 13.0, 24.0), C_SERVO))

    # ---- 平行爪：吊在横臂正下方，爪中心在柱轴前方 CY_BOOM_L
    if show_claw:
        for tris, col in claw_gear_items(open_state, claw_z0):
            out.append((_yaw_about(xform(tris, trans=(CY_AX + CY_BOOM_L, 0.0, 0.0)),
                                   yaw_deg), col))
    return out


def build_vehicle():
    items = []
    A = items.append

    # ★★ 高度基准（2026-10-04 修正，全车所有 z 都从这里推） ★★
    #   旧写法把「板面离地」定成 30mm —— 那是错的，Φ60 麦轮轴心就离地 30mm，
    #   轮顶到 60mm，板面 30 等于让轮子上半截从底板和甲板里穿出来。
    #   真相是：**底板是装在马达上面的**，装好后板面自然就在轮子之上。
    #   板面 = 轮外径 + 2.5（马达吊装余量）→ Φ60 轮 = 62.5mm。
    #   于是「肩高 136mm」这个关键约束要靠**切短机械臂增高座**来保住：
    #       增高座高 = 90 - 板面高  →  Φ60: 27.5 ｜ Φ50: 38.5 ｜ Φ40: 48.5
    #   这样大臂/小臂/夹爪的绝对高度和以前完全一样（够地、够 250mm 物资架都不变）。
    PLATE_TOP = 62.5          # 板面离地（Φ60 麦轮）
    RISER_H = 27.5            # 增高座实切高度（= 90 - PLATE_TOP）
    DZ = PLATE_TOP - 30.0     # 相对旧基准的整体抬升量

    # ---- 金属底板（欣薇 255x150x1.5，装在四个马达上方）
    A((box(0, 0, PLATE_TOP - 0.75, 255, 150, 1.5), C_METAL))

    # ---- 四个麦轮 + TT 马达（马达吊在板下，轮子完全在板下方 → 不干涉）
    for sx in (1, -1):
        for sy in (1, -1):
            wx, wy = sx * 90.0, sy * 68.0
            A((cyl(wx, wy, 30.0, "y", 30.0, 26.0), C_WHEEL))
            A((box(wx, wy - sy * 15.0, 30.0, 30.0, 26.0, 26.0), C_WHEEL))
            A((box(wx, wy - sy * 32.0, 30.0, 24.0, 22.0, 22.0), C_MOTOR))
            A((cyl(wx, wy - sy * 44.0, 30.0, "y", 3.0, 12.0), C_MOTOR))
            # 马达→底板的吊装支架（外购 L 角码或打印件，板面高就是它定的）
            A((box(wx, wy - sy * 20.0, (41.0 + PLATE_TOP) / 2, 8.0, 26.0,
                   PLATE_TOP - 41.0), C_MOTOR))

    # ---- 立柱 x8（M3x30 从板面穿到甲板）
    #      ★ 中间那 4 根从 x=±4 挪到 x=−45 / +78：把甲板中段整块让给储仓。
    #        甲板 162 宽、4 厚，中间最长的无支撑跨距只有 120mm，
    #        载荷 3N 时下垂 ≈0.05mm，可以忽略；何况储仓托盘本身就是一道加强梁。
    for sx in (104.0, -104.0, -45.0, 78.0):
        for sy in (60.0, -60.0):
            A((xform(stl_tris("04_column"), trans=(sx, sy, PLATE_TOP)), C_COL))

    # ---- 夹层：电控托盘 + STM32 + 2xDRV8833 + PS2 接收
    A((xform(stl_tris("18_electronics_tray"), trans=(-55.0, -40.0, PLATE_TOP)), C_PLA))
    A((box(-20.0, -12.0, 34.0 + DZ, 53.0, 22.0, 1.6), C_PCB))       # STM32F103C8T6
    A((box(-20.0, -12.0, 37.0 + DZ, 53.0, 22.0, 8.0), (30, 90, 60)))  # 排针+元件高度
    A((box(22.0, -20.0, 34.5 + DZ, 21.0, 16.0, 6.0), C_DRV))        # DRV8833 #1
    A((box(22.0, 4.0, 34.5 + DZ, 21.0, 16.0, 6.0), C_DRV))          # DRV8833 #2
    A((box(-20.0, 18.0, 35.0 + DZ, 46.0, 26.0, 9.0), C_PS2))        # PS2 接收模块

    # ---- 夹层：电池夹 + 2 节 18650
    A((xform(stl_tris("19_battery_clip"), trans=(-81.0, -26.0, PLATE_TOP)), C_PLA))
    for dy in (-10.0, 10.0):
        A((cyl(-95.0, dy, 39.0 + DZ, "x", 9.0, 65.0), C_BATT))
        A((box(-95.0 + 34.0, dy, 39.0 + DZ, 3.0, 12.0, 12.0), (180, 180, 180)))

    # ---- 甲板：前段 + 后段 + 中间搭接板
    #      ★ 两段改成「端面对接」而不是「重叠 20」：重叠会让两块 4mm 板占同一段空间，
    #        物理上根本装不进去。对接后总长 254，正好压在 255 的金属底板上。
    A((xform(stl_tris("01_deck_front"), trans=(-1.0, -0.0, 52.0 + DZ)), C_DECK))
    A((xform(stl_tris("02_deck_rear"), local_rz=180.0, trans=(1.0, 0.0, 52.0 + DZ)), C_DECK))
    A((xform(stl_tris("03_deck_splice"), trans=(-30.0, -30.0, 49.0 + DZ)), C_PLA))

    # ---- 储仓（100(x)×136(y) 大兜 = 3 列 × 4 排；★ 长边沿车宽 y）
    #      ★★ 2026-10-10 二次改：装车位置 x ∈ [−40, +60]。移动过两次：
    #         ① 原来在 x∈[−118,−18] 时会被「收起姿态」的爪子扎进去；
    #         ② 前移到 [−32,68] 后，为给「检录态爪朝车侧」让位，整体再后移 8mm。
    BX0, BZ = -40.0, 56.0 + DZ
    A((xform(stl_tris("12_bin_floor"), trans=(BX0, 0.0, BZ)), C_BIN))
    A((xform(stl_tris("13_bin_side"), amap=("x", "z", "-y"),
             trans=(BX0, -65.0, BZ + 3.0)), C_BIN))
    A((xform(stl_tris("13_bin_side"), amap=("x", "z", "y"),
             trans=(BX0, 65.0, BZ + 3.0)), C_BIN))
    # 单向翻板挂在朝车头的投料口
    A((xform(stl_tris("15_bin_flap"), amap=("y", "-z", "x"),
             trans=(BX0 + 97.0, -21.0, BZ + 55.0)), C_PLA))
    # 方块：3 列（x）× 4 排（y）= 12 位，**最后列的中两排留空**给爪子停车 → 实装 10 块
    BIN_SKIP = ((2, 1), (2, 2))
    for i in range(3):
        cx = BX0 + 5 + i * 31.0 + 15.0
        for j, cy in enumerate((-46.5, -15.5, 15.5, 46.5)):
            if (i, j) in BIN_SKIP:
                continue                      # 爪子停靠位：留空，也是第一个投料口
            col = (245, 214, 90) if (i, j) == (0, 0) else C_CUBE
            A((box(cx, cy, BZ + 3 + 15.0, 30.0, 30.0, 30.0), col))

    # ---- ★ 机械臂：柱坐标（Cylindrical Robot）。这张图画的是 **检录/开局姿态**：
    #      yaw = 90°（爪朝车侧）、升降最低。为什么不能 yaw=0 或 180：
    #        · yaw=0（朝车头）→ 爪中心到 x=133+i，全车长会爆到 292，过不了 290 检录；
    #        · yaw=180（朝车尾）→ 爪正好吊进储仓（仓口 88.5~140.5），撞仓壁；
    #        · 只有 yaw=90 时爪架落在 x∈[67,123]、y∈[1,75]，整台车一项红线都不碰。
    for tris, col in arm_items(lift=0.0, yaw_deg=90.0, open_state=False):
        A((tris, col))

    # ---- 洞窟探杆（17_cave_probe，SERVO_EXTRA 驱动，□ 键摆出）
    #     装在车头左前、底板下方（挂耳吊在板底）；这里画的是「伸出去勾洞窟黄块」的姿态。
    #     ⚠ 启动时收回（贴着车身或转到车侧），不占 290mm 初始车长。
    pa = 30.0
    pz = PLATE_TOP - 2.0
    A((xform(stl_tris("17_cave_probe"), rot=(0.0, pa, 0.0),
             trans=(124.0, -72.0, pz)), C_PROBE))
    A((box(120.0, -72.0, pz + 4.0, 23.0, 13.0, 24.0), C_SERVO))      # 探杆舵机 SG90
    A((cyl(124.0, -72.0, pz, "y", 3.0, 24.0), C_MOTOR))              # 摆臂轴

    # ---- 收集铲（底板 + 两侧斜壁 + 后壁）+ 吊到板下的两根吊耳
    A((xform(stl_tris("05_scoop_floor"), trans=(141.5, 0.0, 1.0)), C_SCOOP))
    A((xform(stl_tris("05b_scoop_wall_L"), trans=(141.5, 0.0, 7.0)), C_SCOOP))
    A((xform(stl_tris("05c_scoop_wall_R"), trans=(141.5, 0.0, 7.0)), C_SCOOP))
    A((xform(stl_tris("05d_scoop_back"), trans=(129.5, 0.0, 7.0)), C_SCOOP))
    for sy in (1, -1):
        A((box(133.0, sy * 40.0, (33.0 + PLATE_TOP - 1.5) / 2, 10.0, 6.0,
               PLATE_TOP - 34.5), C_MOTOR))                          # 铲吊耳（M3 螺柱）
    # 铲里兜着一块（示意「推过去就进兜」）
    A((box(143.0, 0.0, 16.0, 30.0, 30.0, 30.0), C_CUBE))

    # ---- 车尾编号牌（不能带数字）
    A((xform(stl_tris("P6_number_plate"), trans=(-127.5, -20.0, 40.0 + DZ))
       if os.path.exists(os.path.join(STL_DIR, "P6_number_plate.stl"))
       else box(-129.0, 0.0, 56.0 + DZ, 3.0, 60.0, 30.0), (240, 240, 240)))

    return items


# ================================================================ 2. 夹爪（连杆版）


def claw_link_items(open_state=True, z0=0.0):
    """05 批次平行夹爪（导轨滑块 + 交叉连杆）。底板中心在原点、底板顶面 z=z0"""
    xs = 25.1 if open_state else 15.0       # 滑块到中心距离
    r, L = 13.5, 28.5
    th = math.radians(90.0 if open_state else 0.0)
    out = []
    # 底板 z:[0,4]
    out.append((xform(stl_tris("24a_claw_base"), trans=(0, 0, z0)), C_DECK))
    # 舵机（轴朝下穿过中心孔）
    out.append((box(2.0, 0.0, z0 + 16.0, 23.0, 13.0, 24.0), C_SERVO))
    out.append((cyl(0.0, 0.0, z0 + 1.0, "z", 2.6, 10.0), C_MOTOR))
    # 摇臂 z:[-4,-1]
    out.append((xform(stl_tris("24d_claw_horn"), local_rz=math.degrees(th),
                      trans=(0, 0, z0 - 4.0)), C_CLAW))
    # 两根交叉连杆 z:[-11,-8]
    for s in (1, -1):
        A = np.array([s * r * math.cos(th), s * r * math.sin(th)])
        B = np.array([-s * xs, 0.0])
        mid = (A + B) / 2.0
        ang = math.degrees(math.atan2(B[1] - A[1], B[0] - A[0]))
        out.append((xform(stl_tris("24e_claw_link"), local_rz=ang,
                          trans=(mid[0], mid[1], z0 - 11.0)), C_ARM))
    # 滑块 z:[-8,4]
    for s in (1, -1):
        out.append((xform(stl_tris("24b_claw_slider"), trans=(s * xs, 0, z0 - 8.0)), C_CLAW))
    # 爪指 z:[-42,-8]
    for s in (1, -1):
        out.append((xform(stl_tris("24c_claw_finger"), trans=(s * xs, 0, z0 - 42.0)), C_CLAW))
    return out


# ================================================================ 3. 夹爪（齿轮版）


def claw_gear_items(open_state=True, z0=0.0):
    """06 批次平行夹爪（齿轮 + 双齿条）。底板中心在原点、底板顶面 z=z0"""
    off = 12.4 if open_state else 3.0        # 齿条本体偏移（爪指中心 = off + 13）
    out = []
    out.append((xform(stl_tris("24h_claw_base_gear"), trans=(0, 0, z0)), C_DECK))
    out.append((box(2.0, 0.0, z0 + 16.0, 23.0, 13.0, 24.0), C_SERVO))
    out.append((cyl(0.0, 0.0, z0 + 1.0, "z", 2.6, 10.0), C_MOTOR))
    # 齿轮 z:[-6,0]
    out.append((xform(stl_tris("24f_claw_gear"), local_rz=20.0,
                      trans=(0, 0, z0 - 6.0)), C_ARM))
    # 两条齿条（第二件绕竖直轴转 180°）z:[-8,4]
    out.append((xform(stl_tris("24g_claw_rack"), local_rz=0.0,
                      trans=(off, 0, z0 - 8.0)), C_CLAW))
    out.append((xform(stl_tris("24g_claw_rack"), local_rz=180.0,
                      trans=(-off, 0, z0 - 8.0)), C_CLAW))
    # 爪指 z:[-42,-8]
    for s in (1, -1):
        out.append((xform(stl_tris("24i_claw_finger_gear"),
                          trans=(s * (off + 13.0), 0, z0 - 42.0)), C_CLAW))
    return out


# ================================================================ main


def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    # ---------------- 图 1：整车
    W, H = 1760, 1080
    FR = 2.62
    eye, tgt = (700.0, -620.0, 300.0), (18.0, 0.0, 82.0)
    items = build_vehicle()
    img, _, _ = render(items, eye, tgt, W, H, f_ratio=FR, ss=2,
                       grid_ext=300.0, grid_step=50.0)
    notes = [
        # 车头侧（屏幕右下）
        ((143.0, -25, 22), "收集铲（两侧斜壁 + 后壁，唇口贴地）", 120, 230),
        ((150.0, 20, 200.0), "★ 柱坐标机械臂：立柱 + 滑座 + 偏心横臂 + 平行夹爪", 120, -170),
        ((131.0, 30, 186.0), "★ 升降曲柄（单臂，R=15.25 → 行程 30.5）", -150, -190),
        ((95.0, 0, 205.0), "★ 顶帽 60×60 —— 升降舵机吊在它下面（立柱唯一能装东西的地方）", -420, -120),
        ((95.0, 52, 120.0), "★ 爪停在车侧（yaw 90°）· 爪尖 95 > 甲板 88.5，余 6.5mm", -300, 90),
        ((70.0, 81, 86), "甲板（前后两段端面对接 + 搭接板）", -400, -80),
        ((170.0, -74, 40), "★ 洞窟探杆（□ 键摆出）", 60, 150),
        # 车尾侧（屏幕左上）
        ((10.0, 0, 112), "★ 储仓 100×136：3 列 × 4 排，最后一列中两排留作爪停车 → 实装 10 块", -300, -170),
        ((-60.0, -70, 62.5), "金属底板 255×150×1.5（装在马达上方）", -300, 130),
        ((-95.0, 10, 76), "2×18650 电池（7.4V）", -300, -40),
        ((-129.0, 0, 88), "编号牌（不能有数字）", -300, 40),
        # 底部
        ((-30.0, 20, 68), "夹层：STM32 + 2×DRV8833 + PS2", -180, 250),
        ((90.0, -68, 30), "麦轮 ×4 + TT 马达 ×4（全在板下方）", -70, 300),
    ]
    img = annotate(img, eye, tgt, W, H, notes, f_ratio=FR, font_size=22,
                   title="CRTC2026 整车装配示意（结构件，外观壳未画）—— 机械臂「检录姿态」",
                   subtitle="长 283 × 宽 162 × 高 204.6mm ｜ 初始尺寸红线 290×195×210 ｜ "
                            "臂 yaw 90° 朝车侧、升降在最高档")
    img = footer(img, [
        "读图说明：",
        "① 这张图里的每一块，都是「3D打印交付」里那个 STL 打出来的实物，不是手绘示意；"
        "只有金属底板、麦轮、TT 马达、电池、电路板、舵机是外购件，用简化形状代替。",
        "② ★ 图上就是**比赛开始那一瞬间的姿态**（也是检录姿态）：机械臂 yaw=90° 转到车侧、"
        "升降推到最高档。长 283、宽 162、高 204.6，三项红线全在里侧。",
        "③ ★ 机械臂这一版换成了**圆柱坐标机械臂（柱坐标 / Cylindrical Robot）**："
        "θ 轴 = 整根立柱绕底座竖直轴回转（0~180°，1 个舵机）；Z 轴 = 滑座沿立柱直线升降"
        "（曲柄滑块，行程 30.5，1 个舵机）；第三个舵机管夹爪开合。"
        "**三个自由度完全解耦、不需要逆运动学**，PS2 直接控各个舵机角度 —— 这是新生队最稳的构型。",
        "④ ★ 上一版把它读成了「平行四连杆摆动臂」，那是**错的**，已推翻；新构型的所有零件、"
        "尺寸、装配关系都重画了一遍，见《机械臂_装配与运动范围.html》。",
        "⑤ ★ 储仓：100(沿车长) × 136(沿车宽)，3 列 × 4 排。"
        "**最后一列的中两排刻意留空**：那里是收起时爪子的停车位，同时也是投料口。"
        "所以随车携带 10 块 + 爪里还能再夹 1 块。中间 4 根立柱已从 x=±4 挪到 x=−45/+78 给它让位。",
        "⑥ 铲子吊在底板下方两根吊耳上（M3 螺柱），唇口贴地；两侧斜壁把方块收窄到 50mm 喉口。"
        "★ 分工：**地面上的方块一律用铲收**；物资架黄块用挑杆 16_rack_hook、"
        "洞窟黄块用探杆 17_cave_probe、高处的白块/焦点黄块才归机械臂。",
    ], font_size=21)
    p1 = os.path.join(OUT_DIR, "01_整车装配示意.png")
    img.save(p1)
    print("saved", p1)

    # ---------------- 图 2：连杆版平行夹爪（主视图 + 底部机构特写）
    W, H = 1520, 660
    FR = 3.2
    eye, tgt = (70.0, -560.0, 70.0), (70.0, 0.0, -14.0)
    items = []
    for tris, col in claw_link_items(True, 0.0):
        items.append((tris, col))
    items.append((xform(stl_tris("26_cube_mock"), trans=(0.0, -15.0, -40.0)), C_CUBE))
    for tris, col in claw_link_items(False, 0.0):
        items.append((xform(tris, trans=(140.0, 0.0, 0.0)), col))
    items.append((xform(stl_tris("26_cube_mock"), trans=(140.0, -15.0, -42.0)), C_CUBE))
    img, _, _ = render(items, eye, tgt, W, H, f_ratio=FR, ss=2,
                       grid_ext=150.0, grid_step=25.0)
    notes = [
        ((0.0, 0.0, 28.0), "SG90 舵机（扎带绑在底板上）", 80, -55),
        ((18.0, 0.0, 4.0), "爪架底板 24a（中间贯通槽 = 导轨）", -300, -60),
        ((-25.1, 0.0, -4.0), "滑块 24b（在导槽里平移）", -330, 20),
        ((-25.1, 0.0, -30.0), "爪指 24c（内侧贴 1mm EVA 防滑）", -360, 100),
        ((25.1, 0.0, -30.0), "爪指 24c", 60, 20),
        ((0.0, 0.0, -48.0), "左套：张开 44mm（方块自由进出）", -90, 150),
        ((140.0, 0.0, 28.0), "同一套的闭合态", 60, -120),
        ((140.0, 0.0, -48.0), "右套：闭合 24mm（方块被夹住）", -40, 150),
    ]
    img = annotate(img, eye, tgt, W, H, notes, f_ratio=FR, font_size=21,
                   title="平行夹爪 A 款：连杆版（05 批次，推荐先装这套）",
                   subtitle="导轨滑块 + 交叉连杆｜张开净距 44mm / 闭合净距 24mm（方块 30mm）")
    # 底部：机构特写（同一个夹爪的张开态 + 闭合态，隐去爪指与滑块）
    core = []
    for tris, col in claw_link_core(True):
        core.append((tris, col))
    for tris, col in claw_link_core(False):
        core.append((xform(tris, trans=(150.0, 0.0, 0.0)), col))
    ins, _, _ = render(core, (100.0, -350.0, -330.0), (75.0, 0.0, -6.0),
                       1520, 300, f_ratio=9.4, ss=2, grid_ext=120.0, grid_step=25.0)
    band = Image.new("RGB", (1520, 300 + 56), (250, 250, 251))
    band.paste(ins, (0, 46))
    db = ImageDraw.Draw(band)
    ft = ImageFont.truetype(FONT_PATH, 22)
    db.text((16, 12), "↓ 把底板翻过来看（隐去爪指和滑块）：橙色摇臂转，两根连杆交叉推两个滑块 → 两片爪指永远平行",
            font=ft, fill=(35, 38, 42))
    canvas = Image.new("RGB", (1520, H + band.size[1] + 22), (250, 250, 251))
    canvas.paste(img, (0, 0))
    canvas.paste(band, (0, H + 22))
    img = canvas
    img = footer(img, [
        "图上两套是同一个夹爪的两种状态：左边张开（方块自由进出）、右边闭合（夹住方块）。"
        "颜色只为了让零件分得清：米色=底板，蓝=滑块与爪指，橙=连杆与摇臂，深蓝=舵机。",
        "零件：24a 底板 ×1、24b 滑块 ×2、24c 爪指 ×2、24d 摇臂 ×1、24e 连杆 ×2（共 8 件，含备件）。",
        "为什么推荐它：全机构只有销钉铰接，没有齿啮合，打印误差 0.2mm 也照样能动，失败率最低。",
        "装配顺序：滑块穿进底板导槽 → 爪指从下方用 M2.5 自攻拧到滑块 → 舵机用扎带绑在底板上 → "
        "装摇臂 → 上两根连杆（两根各占一层，不会打架）。",
    ], font_size=20)
    p2 = os.path.join(OUT_DIR, "02_平行夹爪_连杆版.png")
    img.save(p2)
    print("saved", p2)

    # ---------------- 图 3：齿轮版平行夹爪（主视图 + 底部机构特写）
    W, H = 1520, 660
    FR = 3.2
    eye, tgt = (70.0, -560.0, 70.0), (70.0, 0.0, -14.0)
    items = []
    for tris, col in claw_gear_items(True, 0.0):
        items.append((tris, col))
    items.append((xform(stl_tris("26_cube_mock"), trans=(0.0, -15.0, -40.0)), C_CUBE))
    for tris, col in claw_gear_items(False, 0.0):
        items.append((xform(tris, trans=(140.0, 0.0, 0.0)), col))
    items.append((xform(stl_tris("26_cube_mock"), trans=(140.0, -15.0, -42.0)), C_CUBE))
    img, _, _ = render(items, eye, tgt, W, H, f_ratio=FR, ss=2,
                       grid_ext=150.0, grid_step=25.0)
    notes = [
        ((0.0, 0.0, 28.0), "SG90 / MG996R 舵机", 80, -55),
        ((18.0, 0.0, 4.0), "齿轮爪底板 24h（两条导槽）", -320, -60),
        ((-15.0, 0.0, -4.0), "齿条滑块 24g ×2（第二件转 180°）", -370, 30),
        ((-25.4, 0.0, -30.0), "爪指 24i（36mm 长条，内贴 EVA）", -370, 110),
        ((25.4, 0.0, -30.0), "爪指 24i", 60, 20),
        ((0.0, 0.0, -48.0), "左套：张开 44.8mm", -60, 150),
        ((140.0, 0.0, 28.0), "同一套的闭合态", 60, -120),
        ((140.0, 0.0, -48.0), "右套：闭合 26mm，夹住方块", -30, 150),
    ]
    img = annotate(img, eye, tgt, W, H, notes, f_ratio=FR, font_size=21,
                   title="平行夹爪 B 款：齿轮齿条版（06 批次，和市售那款同结构）",
                   subtitle="齿轮 + 双齿条｜张开净距 44.8mm / 闭合净距 26mm（方块 30mm）")
    # 底部：机构特写（隐去底板和爪指，只看齿轮怎么带两条齿条）
    core = []
    for tris, col in claw_gear_core(True):
        core.append((tris, col))
    for tris, col in claw_gear_core(False):
        core.append((xform(tris, trans=(150.0, 0.0, 0.0)), col))
    ins, _, _ = render(core, (80.0, -330.0, -300.0), (75.0, 0.0, -4.0),
                       1520, 300, f_ratio=8.2, ss=2, grid_ext=120.0, grid_step=25.0)
    band = Image.new("RGB", (1520, 300 + 56), (250, 250, 251))
    band.paste(ins, (0, 46))
    db = ImageDraw.Draw(band)
    ft = ImageFont.truetype(FONT_PATH, 22)
    db.text((16, 12), "↓ 把底板翻过来看（隐去底板和爪指）：舵机转齿轮，齿轮同时推两条齿条反向移动 → 两片爪指平行开合",
            font=ft, fill=(35, 38, 42))
    canvas = Image.new("RGB", (1520, H + band.size[1] + 22), (250, 250, 251))
    canvas.paste(img, (0, 0))
    canvas.paste(band, (0, H + 22))
    img = canvas
    img = footer(img, [
        "图上两套是同一个夹爪的两种状态：左边张开、右边闭合夹住方块。米色=底板，蓝=齿条滑块与爪指，"
        "橙=齿轮，深蓝=舵机。",
        "零件：24h 底板 ×1、24f 齿轮 ×2（1 用 1 备）、24g 齿条滑块 ×2、24i 爪指 ×3（2 用 1 备）。",
        "注意：第二件齿条要「在桌面上转 180°」再装（绕竖直轴转，不是翻面！），这样两块爪指才面对面。",
        "注意：齿必须先用锉刀把毛刺修一遍，PLA 齿有毛刺会卡死。真卡死了就换 A 款连杆版 —— "
        "两套都打了，装车时二选一，07/08 钩形指留作最后兜底。",
    ], font_size=20)
    p3 = os.path.join(OUT_DIR, "03_平行夹爪_齿轮版.png")
    img.save(p3)
    print("saved", p3)

    # ---------------- 图 4：机械臂三姿态（每个姿态单独渲一张再横拼）
    #   柱坐标臂只有两个动作量：yaw（回转）+ lift（升降）。
    #   每一张里那根**红色横杠就是 210mm 限高线**，肉眼就能看出还剩多少余量。
    POSES = [
        (CY_LIFT_MAX, 90.0, 100.0,
         "① 检录 / 收起姿态",
         "yaw 90° 朝车侧 · 升降到最高档 · 全车最高 204.6mm（红线 210，余 5.4）"),
        (0.0, 0.0, 130.0,
         "② 取块姿态（最低档）",
         "yaw 0° 朝车头 · 爪在 x=147（已在甲板前缘 127 之外）· 爪尖 64.5"),
        (CY_LIFT_MAX, 180.0, 60.0,
         "③ 投放姿态（朝储仓）",
         "yaw 180° 朝车尾 · 爪在 x=43 正落在储仓停车格上方 · 爪尖 95（仓底 94.5）"),
    ]
    sub_w, sub_h = 720, 800
    subs = []
    for lift, yaw, cx, t1, t2 in POSES:
        it = list(arm_items(lift=lift, yaw_deg=yaw, open_state=False))
        it.append((box(cx, -240.0, 210.0, 560.0, 7.0, 3.5), (206, 74, 74)))  # 210 限高线
        if abs(yaw - 180.0) < 1e-6:      # 投放态：把储仓画出来，证明「落得进去」
            it.append((box(10.0, 0.0, 88.5 + 1.5, 100.0, 136.0, 3.0), (150, 190, 165)))
            for i in range(3):
                for j, cy in enumerate((-46.5, -15.5, 15.5, 46.5)):
                    if (i, j) in ((2, 1), (2, 2)):
                        continue                  # ★ 爪的停车位：留空
                    it.append((box(-40.0 + 5.0 + i * 31.0 + 15.0, cy,
                                   88.5 + 3.0 + 15.0, 30.0, 30.0, 30.0),
                               (150, 190, 165)))
        eye, tgt = (cx + 300.0, -640.0, 360.0), (cx, 0.0, 140.0)
        s, _, _ = render(it, eye, tgt, sub_w, sub_h, f_ratio=1.95, ss=2,
                         grid_ext=340.0, grid_step=50.0)
        s = annotate(s, eye, tgt, sub_w, sub_h, [],
                     f_ratio=1.95, font_size=20, title=t1, subtitle=t2)
        subs.append(s)
    canvas = Image.new("RGB", (sub_w * 3 + 24, sub_h + 16), (250, 250, 251))
    for i, s in enumerate(subs):
        canvas.paste(s, (i * (sub_w + 12), 8))
    canvas = footer(canvas, [
        "★ 柱坐标机械臂只有两个动作量：**yaw（整根立柱绕底座回转 0~180°）+ lift（滑座沿立柱升降，行程 30.5）**，"
        "第三个自由度是夹爪开合。三自由度完全解耦、不需要逆运算。",
        "★ 三张图共用同一把尺子 —— 红色横杠是同一条 210mm 限高线，可以横向对比。",
        "① 检录 / 收起：yaw 90°（爪朝车侧）+ 升降到最高档。全车最高点是曲柄外缘 204.6mm，离红线余 5.4mm。"
        "★ 这里必须用最高档：爪尖在最高档是 95mm，比甲板顶 88.5 高 6.5mm；"
        "**换成最低档爪尖只有 64.5mm，会直接插进甲板** —— 这就是为什么固件要加互锁。",
        "② 取块：yaw 0° 朝车头，爪中心在 x=147，已经在甲板前缘（x=127）之外，"
        "所以这一侧可以放心把升降放到最低档（爪尖 64.5）。",
        "③ 投放：yaw 180° 朝车尾，爪中心落在 x=43，正对储仓停车格上方；"
        "浅绿色是储仓（100×136，3 列 × 4 排，**最后一列中两排刻意留空**当爪停车位 + 投料口）。"
        "爪尖最高 95 vs 仓底 94.5 —— **只剩 0.5mm，这是全车最紧的一处**，进场前必须实测。",
        "★ 三个舵机怎么分工：SG90 管回转（PA11）｜SG90 管升降（PA1，曲柄滑块）｜SG90 管夹爪开合（PA0）。"
        "**同一台 ≥3 舵机**，中期指标 5 直接满足。",
        "⚠ 局限：爪尖最高只有 95mm，**够不到离地 250 的物资架黄块**（差 155mm），"
        "也够不到地面。物资架改用挑杆 16_rack_hook、地面方块改用前铲、洞穴黄块改用探杆 17_cave_probe。",
    ], font_size=20)
    p4 = os.path.join(OUT_DIR, "05_机械臂_三姿态与限高.png")
    canvas.save(p4)
    print("saved", p4)


if __name__ == "__main__":
    main()
