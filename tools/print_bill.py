# -*- coding: utf-8 -*-
"""
扫描 mechanical/stl 和 mechanical/stl_pretty 下所有 STL，
算出：包围盒尺寸、体积、PLA 估算重量、建议切片参数、分批建议。
输出 3D打印交付/打印清单.csv 和 打印清单.md

用法：
    python tools/print_bill.py
"""
import os
import re
import struct
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
STL_DIR = os.path.join(ROOT, 'mechanical', 'stl')
STL_PRETTY = os.path.join(ROOT, 'mechanical', 'stl_pretty')
OUT = os.path.join(ROOT, '3D打印交付')

PLA_DENSITY = 1.24          # g/cm^3，PLA 常用值
# ⚠ 全仓库统一口径：必须和 mechanical/gen_parts.py 的 SOLID_RATIO、
#   gen_shell.py 的 SOLID_RATE 一致，否则几处重量加不起来。
#   打印件实际用料 ≈ 实体体积 × 这个系数（外壳实心 + 内部填充）
SHELL_RATIO = 0.45

# 中文名 / 数量 / 批次：直接从生成脚本里抓，避免两处各写一份慢慢跑偏
#   gen_parts.py:  add_extrude("名", "中文名", 数量, ...)  /  add_loft(同)
#   gen_shell.py:  add("名", "中文名", 数量, ...)
NAMES, QTY = {}, {}
for script in ('gen_parts.py', 'gen_shell.py'):
    p = os.path.join(ROOT, 'mechanical', script)
    if not os.path.exists(p):
        continue
    src = open(p, encoding='utf-8').read()
    for m in re.finditer(r'add(?:_extrude|_loft)?\(\s*"([^"]+)"\s*,\s*"([^"]*)"\s*,\s*(\d+)', src):
        NAMES[m.group(1)] = m.group(2)
        QTY[m.group(1)] = int(m.group(3))

# 批次映射：抓 gen_parts.py 里的 BATCH = { ... }
BATCH, SPARE = {}, set()
_p = os.path.join(ROOT, 'mechanical', 'gen_parts.py')
if os.path.exists(_p):
    _src = open(_p, encoding='utf-8').read()
    m = re.search(r'^BATCH\s*=\s*\{(.*?)^\}', _src, re.S | re.M)
    if m:
        for k, v in re.findall(r'"([^"]+)"\s*:\s*(\d+)', m.group(1)):
            BATCH[k] = int(v)
    m = re.search(r'^SPARE\s*=\s*\{(.*?)^\}', _src, re.S | re.M)
    if m:
        SPARE = set(re.findall(r'"([^"]+)"', m.group(1)))

BATCH_NAME = {1: '① 底盘必打', 2: '② 收集与储仓', 3: '③ 机械臂与机构',
              4: '④ 外观套件（最后打）'}


def read_stl_binary(path):
    """手工解析二进制 STL，返回三角面列表（不依赖 numpy-stl）"""
    with open(path, 'rb') as f:
        f.read(80)
        n_decl = struct.unpack('<I', f.read(4))[0]
        data = f.read()
    # 以实际字节数为准：有的导出器声明的面数会多/少一个，按声明读会越界
    n = len(data) // 50
    if n != n_decl:
        print('  [提示] %s 声明 %d 面，实际 %d 面，按实际算'
              % (os.path.basename(path), n_decl, n))
    tris = []
    for i in range(n):
        off = i * 50
        # 面记录 50 字节 = normal(12) + v1(12) + v2(12) + v3(12) + attr(2)
        # 跳过法向量，只读 3 个顶点 = 9 个 float = 36 字节
        v = struct.unpack_from('<9f', data, off + 12)
        tris.append((v[0:3], v[3:6], v[6:9]))
    return tris


def read_stl(path):
    """自动判断 ASCII / 二进制"""
    with open(path, 'rb') as f:
        head = f.read(80)
    try:
        txt = head.decode('ascii')
        if 'solid' in txt.lower() and 'facet' in head.decode('ascii', 'ignore'):
            pass
    except Exception:
        pass
    # 简单判断：前 5 字节是 'solid' 且整个文件能 utf-8 解码 → ASCII
    with open(path, 'rb') as f:
        raw = f.read()
    try:
        s = raw.decode('utf-8')
        if s.lstrip().startswith('solid'):
            return parse_ascii(s)
    except Exception:
        pass
    return read_stl_binary(path)


def parse_ascii(s):
    tris = []
    cur = []
    for ln in s.splitlines():
        ln = ln.strip()
        if ln.startswith('vertex'):
            p = ln.split()[1:4]
            cur.append(tuple(float(x) for x in p))
            if len(cur) == 3:
                tris.append(tuple(cur))
                cur = []
    return tris


def signed_volume(a, b, c):
    """三角形对原点的有向体积 ×6"""
    return (a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]))


def measure(path):
    tris = read_stl(path)
    if not tris:
        return None
    xs, ys, zs = [], [], []
    vol = 0.0
    for a, b, c in tris:
        vol += signed_volume(a, b, c) / 6.0
        for p in (a, b, c):
            xs.append(p[0]); ys.append(p[1]); zs.append(p[2])
    bb = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    return {
        'n': len(tris),
        'vol_mm3': abs(vol),
        'bb': bb,
        'g': abs(vol) / 1000.0 * PLA_DENSITY * SHELL_RATIO,
    }


def suggest(bb, group):
    """按尺寸和分组给建议切片参数。
    判定规则与 mechanical/gen_parts.py 的 slice_param() 保持一致：
    先看厚度再看面积 —— 薄板件的内部空间本来就被外壳占满了，
    填充分辨率对它意义不大，真正要防的是翘边，所以大面积的一律加 Brim。"""
    t = min(bb)
    big = max(bb) > 120.0
    if group == 'pretty':
        return ('0.16', '15%', '否', '外观件，层高小一点好看；不承重，15% 填充够用')
    if t <= 2.5:
        return ('0.20', '40%', 'Brim 8mm' if big else '否',
                '超薄件（≤2.5mm），填充给满才不容易掰断')
    if t <= 4.5:
        return ('0.20', '30%', 'Brim 8mm' if big else '否',
                '薄板件，切片时基本会被外壳填满；大面积要防翘边')
    if big:
        return ('0.20', '25%', 'Brim 8mm', '大件怕翘边，加一圈裙边；别用整面支撑')
    return ('0.20', '30%', '否', '结构件受力，填充别低于 25%')


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = []
    for group, d, tag in (('struct', STL_DIR, '结构'), ('pretty', STL_PRETTY, '外观')):
        if not os.path.isdir(d):
            continue
        for fn in sorted(os.listdir(d)):
            if not fn.lower().endswith('.stl'):
                continue
            key = fn[:-4]
            info = measure(os.path.join(d, fn))
            if info is None:
                continue
            lay, inf, sup, note = suggest(info['bb'], group)
            qty = QTY.get(key, 1)
            spare = key in SPARE
            batch = BATCH.get(key, 4 if group == 'pretty' else 9)
            rows.append({
                'file': fn,
                'key': key,
                'name': NAMES.get(key, key),
                'group': tag,
                'batch': batch,
                'qty': qty,
                'spare': spare,
                'x': info['bb'][0], 'y': info['bb'][1], 'z': info['bb'][2],
                'vol': info['vol_mm3'],
                'g': info['g'],
                'tri': info['n'],
                'layer': lay, 'infill': inf, 'support': sup, 'note': note,
            })

    # 按批次排序：打印顺序 = 装配顺序
    rows.sort(key=lambda r: (r['batch'], r['key']))

    if not rows:
        print('没找到 STL，检查路径：%s / %s' % (STL_DIR, STL_PRETTY))
        return 1

    # 总重按"实际要打多少个"算（乘 qty），和图纸集口径一致
    tot_g = sum(r['g'] * r['qty'] for r in rows)
    tot_vol = sum(r['vol'] * r['qty'] for r in rows)
    tot_pcs = sum(r['qty'] for r in rows)

    # ---- CSV ----
    csv_path = os.path.join(OUT, '打印清单.csv')
    with open(csv_path, 'w', encoding='utf-8-sig', newline='') as f:
        f.write('批次,文件,中文名,数量,建议多打一份,尺寸X,尺寸Y,尺寸Z,'
                '实体体积mm3,单件重g,小计g,层高mm,填充%,支撑,备注\n')
        for r in rows:
            f.write('%s,%s,%s,%d,%s,%.1f,%.1f,%.1f,%.0f,%.1f,%.1f,%s,%s,%s,%s\n' % (
                BATCH_NAME.get(r['batch'], '其他'), r['file'], r['name'], r['qty'],
                '是' if r['spare'] else '',
                r['x'], r['y'], r['z'], r['vol'], r['g'], r['g'] * r['qty'],
                r['layer'], r['infill'], r['support'], r['note']))

    # ---- Markdown ----
    md = []
    md.append('# 3D 打印清单（自动生成）\n')
    md.append('> 由 `tools/print_bill.py` 扫描 `mechanical/stl` 与 `mechanical/stl_pretty` 生成，')
    md.append('> 名称 / 数量 / 批次直接从 `gen_parts.py`、`gen_shell.py` 抓取，不会和图纸集对不上。')
    md.append('> 重量按 PLA 密度 1.24 g/cm³ × 用料系数 %.2f 估算，**打完请实称**。\n'
              % SHELL_RATIO)
    md.append('共 **%d 个文件 / %d 件**，实体总体积 %.0f mm³，估算总重 **%.0f g**。\n'
              % (len(rows), tot_pcs, tot_vol, tot_g))
    md.append('## 按批次\n')
    md.append('| 批次 | 文件数 | 件数 | 估算重量 | 什么时候打 |')
    md.append('|---|---|---|---|---|')
    when = {1: '现在就打，装完车能跑', 2: '底盘装好就打',
            3: '车能跑之后', 4: '最后打，不影响功能'}
    for b in sorted(set(r['batch'] for r in rows)):
        sub = [r for r in rows if r['batch'] == b]
        md.append('| %s | %d | %d | %.0f g | %s |'
                  % (BATCH_NAME.get(b, '其他'), len(sub),
                     sum(r['qty'] for r in sub),
                     sum(r['g'] * r['qty'] for r in sub), when.get(b, '')))
    md.append('')

    for tag in ('结构', '外观'):
        sub = [r for r in rows if r['group'] == tag]
        if not sub:
            continue
        sg = sum(r['g'] * r['qty'] for r in sub)
        md.append('\n## %s件（%d 个文件 / %d 件，约 %.0f g）\n'
                  % (tag, len(sub), sum(r['qty'] for r in sub), sg))
        md.append('| 文件 | 名称 | 数量 | 尺寸 X×Y×Z (mm) | 单件重 | 层高 | 填充 | 支撑 |')
        md.append('|---|---|---|---|---|---|---|---|')
        for r in sub:
            spare = ' ⚠多打一份' if r['spare'] else ''
            md.append('| `%s` | %s%s | ×%d | %.0f × %.0f × %.0f | %.1f g | %s | %s | %s |'
                      % (r['file'], r['name'], spare, r['qty'],
                         r['x'], r['y'], r['z'],
                         r['g'], r['layer'], r['infill'], r['support']))
    md_path = os.path.join(OUT, '打印清单.md')
    open(md_path, 'w', encoding='utf-8').write('\n'.join(md) + '\n')

    print('文件数：%d / 件数：%d' % (len(rows), tot_pcs))
    print('总体积：%.0f mm3' % tot_vol)
    print('估算总重：%.0f g（已乘数量）' % tot_g)
    for b in sorted(set(r['batch'] for r in rows)):
        sub = [r for r in rows if r['batch'] == b]
        print('  %-16s %2d 文件  %2d 件  %5.0f g'
              % (BATCH_NAME.get(b, '其他'), len(sub), sum(r['qty'] for r in sub),
                 sum(r['g'] * r['qty'] for r in sub)))
    print('CSV：%s' % csv_path)
    print('MD ：%s' % md_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
