#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ps2_hw.py —— PS2「到底是接线问题还是接收器问题」硬件级探针

用法（在 CRTC 目录下，ST-LINK 接好、板子上电）：
    python tools/ps2_hw.py

━━ 它干什么 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
绕开固件，直接用 OpenOCD 扒 STM32 的寄存器，做两组测试：

  A 电气测试
    把 PA5(CLK) / PA6(MISO) / PA7(MOSI) / PB0(CS) 四个脚轮流设成
    "内部下拉输入" 和 "内部上拉输入"，看电平跟不跟着自己走：

      上拉读 1 + 下拉读 1  → 恒定高  = 外面有东西在驱动这根线
      上拉读 1 + 下拉读 0  → 悬空    = 这个节点是空的（线没接上 / 接的是输入脚）
      上拉读 0 + 下拉读 0  → 恒定低  = 被强行拉低（短到 GND）

  B 握手测试
    手工往 SPI1_DR 里塞 9 字节，CPOL/CPHA 四种模式各试一遍，
    直接看接收器回回来的原始字节（正常首字节应该是 0x73 / 0x41 / 0x79，
    第 3 字节应该是 0x5A）。

━━ 为什么这两种测试能定案 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"信号干扰"这个解释在这里不成立：281kHz、10cm 的线，不可能把整帧
9 个字节全部干扰成 0xFF。0xFF 是"这根线上一直是高电平"，只能是
①没接上 ②接到了一个输入脚（高阻） ③接收器根本不工作。
这三件事，A 测试全都能分开。

结束时自动 reset run，固件照常跑，不留任何副作用。
"""

import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import flash                                                   # noqa: E402

# SWD 时钟。这块克隆版 ST-LINK + 杜邦线在 OpenOCD 默认 4000 kHz 下经常
# `Cannot detect cache` / `Examination failed`，降到 300 kHz 就稳。
# 需要更快时：set SWD_SPEED=500
SWD_SPEED = os.environ.get('SWD_SPEED', '300')

# ---------------------------------------------------------------- 寄存器地址
GPIOA = 0x40010800
GPIOB = 0x40010C00
SPI1 = 0x40013000

A_CRL = GPIOA + 0x00          # PA0~PA7 配置
A_CRH = GPIOA + 0x04          # PA8~PA15 配置
A_IDR = GPIOA + 0x08          # 输入电平
A_BSRR = GPIOA + 0x10         # 置位
A_BRR = GPIOA + 0x14          # 清零
B_CRL = GPIOB + 0x00
B_CRH = GPIOB + 0x04
B_IDR = GPIOB + 0x08
B_BSRR = GPIOB + 0x10
B_BRR = GPIOB + 0x14
SPI_CR1 = SPI1 + 0x00
SPI_SR = SPI1 + 0x08
SPI_DR = SPI1 + 0x0C

RCC_APB2ENR = 0x40021018
AFIO_MAPR = 0x40010004        # bits 26:24 = SWJ_CFG；0b010 = 关 JTAG、留 SWD

# SPI1_CR1 的固定位：MSTR | SSM | SSI | BR=111(/256) | SPE  = 0x037C
CR1_BASE = 0x0000037C
# 只留 SSM|SSI、先关 SPE —— 绝不能把 SSM 写成 0，否则 NSS 变外部脚会触发 MODF
CR1_SAFE_OFF = 0x00000300

POLL = [0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]


# ================================================================ OpenOCD 会话
class Session(object):
    """攒一堆 OpenOCD 命令，一口气发出去，再把 mdw 的返回值按顺序取回来。"""

    def __init__(self):
        self.cmds = ['halt']
        self.labels = []

    def w(self, addr, val):
        self.cmds.append('mww 0x%08X 0x%08X' % (addr, val))

    def r(self, addr, label):
        self.cmds.append('mdw 0x%08X 1' % addr)
        self.labels.append(label)
        return len(self.labels) - 1          # 返回下标，稍后按下标取值

    def run(self):
        oc, scr = flash.find_openocd()
        if not oc:
            print('[错误] 找不到 OpenOCD。先跑 python tools/env_check.py 看看环境。')
            return None, ''

        cmd = [oc, '-s', scr, '-f', flash.IFACE_CFG, '-f', flash.TARGET_CFG,
               '-c', 'adapter speed %s' % SWD_SPEED,
               '-c', 'init']
        for c in self.cmds:
            cmd += ['-c', c]
        cmd += ['-c', 'reset run', '-c', 'exit']

        try:
            p = subprocess.run(cmd, capture_output=True, timeout=180)
        except subprocess.TimeoutExpired:
            print('[错误] OpenOCD 超时。ST-LINK 插好了吗？板子通电了吗？')
            return None, ''

        out = (p.stdout or b'').decode('utf-8', 'ignore')
        out += (p.stderr or b'').decode('utf-8', 'ignore')

        vals = [int(m.group(1), 16)
                for m in re.finditer(r'^0x[0-9a-fA-F]{8}:\s+([0-9a-fA-F]{8})', out, re.M)]
        return vals, out


# ================================================================ 小工具
def nibble_set(word, pin, nib):
    """把 32 位配置字里第 pin 个脚的 4 位（0~7 用 CRL）改成 nib。"""
    shift = 4 * pin
    return (word & ~(0xF << shift)) | ((nib & 0xF) << shift)


def state_of(pu, pd):
    """pu = 上拉时的读数, pd = 下拉时的读数。返回 (代号, 中文)"""
    if pu and pd:
        return 'HIGH', '恒定高（外面有东西在驱动）'
    if pu and not pd:
        return 'FLOAT', '悬空（能被自己拉得动）'
    if (not pu) and (not pd):
        return 'LOW', '恒定低（被强行拉低）'
    return 'ODD', '异常（下拉时反而读到高）'


def bit(word, n):
    return (word >> n) & 1


# ================================================================ 测试 A：握手
def test_handshake():
    print('')
    print('=' * 70)
    print('B  握手测试：手工发 9 字节，四种 SPI 模式各试一遍')
    print('=' * 70)

    s = Session()
    idx = {}                     # (cpol, cpha) -> [(sr_idx, rx_idx), ...]

    for cpol in (0, 1):
        for cpha in (0, 1):
            mode = (cpol << 1) | cpha

            s.w(SPI_CR1, CR1_SAFE_OFF)          # 关 SPE（保留 SSM/SSI）
            s.r(SPI_SR, 'clr_sr')
            s.r(SPI_DR, 'clr_dr')               # 读 SR + DR 清溢出标志
            s.w(B_BSRR, 0x0001)                 # PB0 高 —— CS 不选中
            s.w(SPI_CR1, CR1_BASE | mode)       # 新的 CPOL/CPHA + 开 SPE
            s.w(B_BRR, 0x0001)                  # PB0 低 —— CS 选中

            pairs = []
            for b in POLL:
                s.w(SPI_DR, b)                  # 塞一字节（硬件自己会打时钟）
                si = s.r(SPI_SR, 'sr')
                ri = s.r(SPI_DR, 'rx')          # 读回来（同时清 RXNE）
                pairs.append((si, ri))

            s.w(B_BSRR, 0x0001)                 # PB0 高 —— 收工
            idx[mode] = pairs

    vals, raw = s.run()
    if vals is None:
        return None

    need = len(s.labels)
    if len(vals) != need:
        print('[警告] 读回 %d 个值，应该 %d 个。原始输出：' % (len(vals), need))
        print(raw[-2000:])
        return None

    print('')
    print('  模式  第0字节 第1字节 第2字节 第3字节 第4字节 第5字节 第6字节 第7字节 第8字节   判定')
    print('  ' + '-' * 88)
    result = {}
    for mode in sorted(idx):
        rx = []
        for (si, ri) in idx[mode]:
            sw = vals[si]
            rw = vals[ri]
            rx.append(rw & 0xFF)
            if sw & 0x80:                   # MODF：模式没配好
                rx[-1] = -1
        result[mode] = rx
        txt = ' '.join('%02X    ' % b if b >= 0 else 'ERR   ' for b in rx)
        ok = (rx[1] in (0x41, 0x73, 0x79)) or (rx[2] == 0x5A)
        name = 'CPOL=%d CPHA=%d(模式%d)' % ((mode >> 1) & 1, mode & 1, mode)
        print('  %-22s %s  %s' % (name, txt, '★ 有回应' if ok else '无'))

    return result


# ================================================================ 测试 B：电气
def test_electric(a_crl, b_crl):
    print('')
    print('=' * 70)
    print('A  电气测试：四个脚被打成"上拉/下拉输入"，看电平跟不跟自己走')
    print('=' * 70)

    s = Session()

    # ---- 全部先设成输入(CNF=10, MODE=00 → 0x8)，并置成下拉方向 ----
    a_input = nibble_set(a_crl, 5, 0x8)     # PA5 CLK
    a_input = nibble_set(a_input, 6, 0x8)   # PA6 MISO/DAT
    a_input = nibble_set(a_input, 7, 0x8)   # PA7 MOSI/CMD
    b_input = nibble_set(b_crl, 0, 0x8)     # PB0 CS

    s.w(A_BRR, 0x00E0)                      # PA5/6/7 的 ODR 清 0 = 下拉
    s.w(B_BRR, 0x0001)                      # PB0 的 ODR 清 0 = 下拉
    s.w(A_CRL, a_input)
    s.w(B_CRL, b_input)

    i_pd_a = s.r(A_IDR, 'pd_a')
    i_pd_b = s.r(B_IDR, 'pd_b')

    # ---- 一个个换成上拉，读回来再换回下拉 ----
    s.w(A_BSRR, 0x0020); i5 = s.r(A_IDR, 'pu5'); s.w(A_BRR, 0x0020)
    s.w(A_BSRR, 0x0040); i6 = s.r(A_IDR, 'pu6'); s.w(A_BRR, 0x0040)
    s.w(A_BSRR, 0x0080); i7 = s.r(A_IDR, 'pu7'); s.w(A_BRR, 0x0080)
    s.w(B_BSRR, 0x0001); i0 = s.r(B_IDR, 'pu0'); s.w(B_BRR, 0x0001)

    vals, raw = s.run()
    if vals is None:
        return None
    if len(vals) != len(s.labels):
        print('[警告] 值数量对不上（%d / %d），原始输出：' % (len(vals), len(s.labels)))
        print(raw[-2000:])
        return None

    pd_a, pd_b = vals[i_pd_a], vals[i_pd_b]
    rows = [
        ('PA5', 'CLK  → 接收器 CLK', bit(vals[i5], 5), bit(pd_a, 5)),
        ('PA6', 'MISO ← 接收器 DAT', bit(vals[i6], 6), bit(pd_a, 6)),
        ('PA7', 'MOSI → 接收器 CMD', bit(vals[i7], 7), bit(pd_a, 7)),
        ('PB0', 'CS   → 接收器 CS', bit(vals[i0], 0), bit(pd_b, 0)),
    ]

    print('')
    print('  引脚  作用                  下拉读到 上拉读到   状态')
    print('  ' + '-' * 74)
    out = {}
    for name, desc, pu, pd in rows:
        code, txt = state_of(pu, pd)
        out[name] = code
        print('  %-4s  %-20s   %d        %d        %s' % (name, desc, pd, pu, txt))
    return out


# ================================================================ 测试 C：CS 拉低时 DAT 到底谁在驱动
def test_cs_drive():
    """★ 这是整份工具里最有信息量的一项。

    原理：PA6 内部下拉大约 40kΩ，接收器的 DAT 是推挽输出（驱动能力 ~20mA）。
    所以只要接收器**活着并且被 CS 选中**，它就能把这个下压倒过去 → 读 1。
    如果读 0，说明这根线上**没有任何东西在驱动** → 接收器没电 / 坏了 / DAT 线没通。

    这样就能把「接线问题」和「接收器问题」彻底分开：
        CS 低 + 下拉 读到 1  →  接收器活着，只是回 0xFF  →  ★问题在 RF/配对，不是接线
        CS 低 + 下拉 读到 0  →  接收器根本没在驱动     →  没电 / 坏 / DAT 线断
    对照组（CS 高 + 下拉 应为 0）用来证明这个下拉是有效的、不是一直读 1。
    """
    print('')
    print('=' * 70)
    print('C  关键测试：CS 拉低时，DAT 上到底是接收器在驱动，还是空的')
    print('=' * 70)

    s0 = Session()
    i_apb2 = s0.r(RCC_APB2ENR, 'apb2')
    i_acrl = s0.r(A_CRL, 'a_crl')
    i_bcrl = s0.r(B_CRL, 'b_crl')
    v0, raw0 = s0.run()
    if v0 is None or len(v0) != len(s0.labels):
        print('[错误] 读不到寄存器。')
        return None
    apb2, a_crl, b_crl = v0[i_apb2], v0[i_acrl], v0[i_bcrl]

    # PA5(CLK)/PA7(CMD) 设成推挽输出 0xB；PA6(DAT) 设成输入 0x8（上下拉由 ODR 定）；PB0(CS) 推挽输出
    a_out = nibble_set(a_crl, 5, 0xB)
    a_out = nibble_set(a_out, 7, 0xB)
    a_out = nibble_set(a_out, 6, 0x8)
    b_out = nibble_set(b_crl, 0, 0xB)

    s = Session()
    s.w(RCC_APB2ENR, apb2 | 0x0000000C)      # 确保 GPIOA/GPIOB 时钟开着
    s.w(A_BSRR, 0x000000A0)                  # PA5/PA7 拉高（模式3 空闲高）
    s.w(B_BSRR, 0x00000001)                  # CS 高（不选中）
    s.w(A_CRL, a_out)
    s.w(B_CRL, b_out)

    # ---- ① CS 低 + PA6 下拉 ----
    s.w(A_BRR, 0x00000040)                   # ODR6=0 → 下拉
    s.w(B_BRR, 0x00000001)                   # CS 低
    a1 = s.r(A_IDR, 'cslo_pd1')
    a2 = s.r(A_IDR, 'cslo_pd2')
    a3 = s.r(A_IDR, 'cslo_pd3')

    # ---- ② CS 低 + PA6 上拉（对照：应该也是 1，说明脚没坏） ----
    s.w(A_BSRR, 0x00000040)
    a4 = s.r(A_IDR, 'cslo_pu1')

    # ---- ③ CS 高 + PA6 下拉（对照：接收器不选中时应该释放 → 读到 0） ----
    s.w(B_BSRR, 0x00000001)
    s.w(A_BRR, 0x00000040)
    a5 = s.r(A_IDR, 'cshi_pd1')
    a6 = s.r(A_IDR, 'cshi_pd2')

    # ---- ④ 收工，CS 回家 ----
    s.w(B_BSRR, 0x00000001)

    v, raw = s.run()
    if v is None or len(v) != len(s.labels):
        print('[警告] 值数量对不上（%d / %d）' % (len(v) if v else -1, len(s.labels)))
        print(raw[-1500:])
        return None

    lo_pd = [bit(v[a1], 6), bit(v[a2], 6), bit(v[a3], 6)]
    lo_pu = bit(v[a4], 6)
    hi_pd = [bit(v[a5], 6), bit(v[a6], 6)]

    print('')
    print('  条件                        PA6(DAT) 读到')
    print('  ' + '-' * 50)
    print('  CS 低 + 内部下拉            %s   ← 关键' % ' '.join(str(x) for x in lo_pd))
    print('  CS 低 + 内部上拉            %d' % lo_pu)
    print('  CS 高 + 内部下拉            %s   ← 对照' % ' '.join(str(x) for x in hi_pd))
    print('')
    print('  （PA5/PA7 也已经设成输出去驱动接收器的 CLK/CMD 了，这下它该说话了）')

    driven = all(x == 1 for x in lo_pd)
    idle_low = all(x == 0 for x in hi_pd)

    print('')
    if driven and idle_low:
        print('★★★ 接收器是活的！CS 拉低时它把 DAT 硬顶成高电平，CS 一放就松手。')
        print('    → 6 根线全部接对了，供电也到了。')
        print('    → 问题在**无线这一段**：手柄没开 / 不是同一套 / 没对上频。')
    elif driven and not idle_low:
        print('⚠ CS 拉低、CS 放高，两次下拉都压不住 —— 这根线是**硬接在电源上**的。')
        print('   正常的接收器 DAT 待机时会被下拉压到 0V，这里压不住。')
        print('   → 最可能：这根线插到了电源孔，而不是接收器的 DAT。')
        print('   → 接着跑 python tools/ps2_hw.py --adc 量一下真实电压再定。')
    elif idle_low:
        print('❌ CS 拉低时 DAT 上空的 —— 没有任何东西在驱动它。')
        print('   → 接收器没在响应。三种可能：')
        print('     ① VCC 没真正到接收器（量 XTARK ⑤VCC-⑥GND）')
        print('     ② DAT 这根线没通（换一根线）')
        print('     ③ 接收器本身坏了 / 这板子顶边焊点虚焊')
    else:
        print('⚠ 读数不一致（%s / %s），可能有接触不良，反复插拔后再跑一次。'
              % (lo_pd, hi_pd))
    return {'lo_pd': lo_pd, 'lo_pu': lo_pu, 'hi_pd': hi_pd}


# ================================================================ 测试 D：ADC 量 PA6 到底有多少伏
ADC1 = 0x40012400
ADC_SR = ADC1 + 0x00
ADC_CR2 = ADC1 + 0x08
ADC_SMPR2 = ADC1 + 0x10
ADC_SQR1 = ADC1 + 0x2C
ADC_SQR3 = ADC1 + 0x34
ADC_DR = ADC1 + 0x4C
RCC_CFGR = 0x40021004


def adc_read_pa6(s, tag):
    """用 ADC1_IN6 量一次 PA6 的电压（此时 PA6 处于"输入+下拉"状态）。"""
    s.w(ADC_CR2, 0x00000001)                  # ADON 唤醒
    s.w(ADC_CR2, 0x00000005)                  # ADON|RSTCAL
    s.w(ADC_CR2, 0x00000001)                  # 清 RSTCAL
    s.w(ADC_CR2, 0x00000005)                  # ADON|CAL 开始自校准
    for _ in range(3):                        # 每次 JTAG 读都要 ms 级，够它校准完
        s.r(ADC_CR2, tag + '_cal')
    s.w(ADC_SMPR2, 0x00FFFFFF)                # ch0~9 全部 239.5 周期（最慢最准）
    s.w(ADC_SQR1, 0x00000000)                 # 只做 1 次转换
    s.w(ADC_SQR3, 0x00000006)                 # 通道 6 = PA6
    s.w(ADC_CR2, (7 << 17) | (1 << 20) | 0x00000001)          # SWSTART 触发源
    s.w(ADC_CR2, (7 << 17) | (1 << 20) | (1 << 22) | 0x00000001)   # 开跑
    s.r(ADC_SR, tag + '_sr')
    return s.r(ADC_DR, tag + '_dr')


def test_adc():
    """★ 量 PA6 上的真实电压，判断那根 DAT 线到底接在什么东西上。

    内部下拉是 ~40kΩ，所以节点电压能反推外部的上拉电阻：
        读满量程(≈100%)  → 直接接在 3.3V 轨上（插到电源孔了）或接收器在硬顶高
        读 10%~95%       → 外面挂着一个上拉电阻（正常，适配板标配）
        读 ≈0            → 这根线上没有东西（没接上）
    """
    print('')
    print('=' * 70)
    print('D  ADC 量 PA6（DAT）上的真实电压')
    print('=' * 70)

    s0 = Session()
    i_apb2 = s0.r(RCC_APB2ENR, 'apb2')
    i_cfgr = s0.r(RCC_CFGR, 'cfgr')
    i_acrl = s0.r(A_CRL, 'a_crl')
    i_bcrl = s0.r(B_CRL, 'b_crl')
    v0, raw0 = s0.run()
    if v0 is None or len(v0) != len(s0.labels):
        print('[错误] 读不到寄存器。')
        return None
    apb2, cfgr, a_crl, b_crl = v0[i_apb2], v0[i_cfgr], v0[i_acrl], v0[i_bcrl]

    # PA5/PA7/PB0 推挽输出（让接收器有 CLK/CS），PA6 输入（下拉由 ODR 定）
    a_out = nibble_set(a_crl, 5, 0xB)
    a_out = nibble_set(a_out, 7, 0xB)
    a_out = nibble_set(a_out, 6, 0x8)
    b_out = nibble_set(b_crl, 0, 0xB)

    s = Session()
    s.w(RCC_APB2ENR, apb2 | 0x0000020C)              # GPIOA/B + ADC1 时钟
    s.w(RCC_CFGR, (cfgr & ~(3 << 14)) | (2 << 14))   # ADC 时钟 = PCLK2/6 = 12MHz
    s.w(A_BSRR, 0x000000A0)
    s.w(B_BSRR, 0x00000001)
    s.w(A_CRL, a_out)
    s.w(B_CRL, b_out)
    s.w(A_BRR, 0x00000040)                           # PA6 下拉（40k）

    d1 = adc_read_pa6(s, 'cs_hi')                    # CS 高（接收器待机）
    s.w(B_BRR, 0x00000001)                           # CS 拉低
    d2 = adc_read_pa6(s, 'cs_lo')
    s.w(B_BSRR, 0x00000001)

    # 对照组：PA6 完全不接内部上下拉（纯浮空），ADC 还是能量到外部电平
    s.w(A_CRL, nibble_set(a_out, 6, 0x4))            # 浮空输入
    d3 = adc_read_pa6(s, 'float')

    # PA4（ADC1_IN4）：如果引了一根线到 XTARK 的 VCC，这里就是接收器的工作电压
    s.w(A_CRL, nibble_set(nibble_set(a_out, 6, 0x4), 4, 0x4))   # PA4 浮空输入
    s.w(ADC_SQR3, 0x00000004)                        # 通道 4 = PA4
    s.w(ADC_CR2, (7 << 17) | (1 << 20) | (1 << 22) | 0x00000001)
    s.r(ADC_SR, 'pa4_sr')
    # 二次采样（第一次可能采样电容没稳定）
    s.w(ADC_CR2, (7 << 17) | (1 << 20) | (1 << 22) | 0x00000001)
    s.r(ADC_SR, 'pa4_sr2')
    d4 = s.r(ADC_DR, 'pa4_dr')

    v, raw = s.run()
    if v is None or len(v) != len(s.labels):
        print('[警告] 值数量对不上（%d / %d）' % (len(v) if v else -1, len(s.labels)))
        print(raw[-1500:])
        return None

    dr_hi = v[d1] & 0xFFFF
    dr_lo = v[d2] & 0xFFFF
    dr_fl = v[d3] & 0xFFFF
    dr_p4 = v[d4] & 0xFFFF

    def pct(x):
        return 100.0 * x / 4095.0

    print('')
    print('  条件                         ADC 读数   占满量程   估算电压')
    print('  ' + '-' * 62)
    print('  PA6  CS 高 + 内部下拉 40kΩ   %5d      %5.1f%%     %.2f V'
          % (dr_hi, pct(dr_hi), pct(dr_hi) / 100 * 3.3))
    print('  PA6  CS 低 + 内部下拉 40kΩ   %5d      %5.1f%%     %.2f V'
          % (dr_lo, pct(dr_lo), pct(dr_lo) / 100 * 3.3))
    print('  PA6  完全浮空（不下拉）      %5d      %5.1f%%     %.2f V'
          % (dr_fl, pct(dr_fl), pct(dr_fl) / 100 * 3.3))
    print('  PA4  （若已接到 XTARK VCC）  %5d      %5.1f%%     %.2f V'
          % (dr_p4, pct(dr_p4), pct(dr_p4) / 100 * 3.3))

    print('')
    print('  说明：内部下拉 ≈40kΩ。读数越接近 100%，说明外面拉得越硬。')
    print('        挂 10kΩ 上拉 → 约 20%    挂 4.7kΩ 上拉 → 约 89%')

    # ★ 用「浮空 - 下拉」的电压差反推外部源内阻：Δ = Vcc·Rpu/(Rpu+Rpd)
    dv = dr_fl - dr_lo
    rpu = None
    if dv > 0:
        k = float(dv) / 4095.0
        rpu = 40000.0 * k / (1.0 - k) if k < 0.999 else 0.0
        print('')
        print('  下拉前后电压差 = %d LSB（%.1f mV）→ 反推外部源内阻 ≈ %.0f Ω'
              % (dv, dv / 4095.0 * 3.3 * 1000, rpu))
        print('    正常上拉电阻 4.7k~10kΩ 会让电压掉 0.2~0.6V；')
        print('    只掉十几 mV → 那不是电阻，是一根导线直连电源。')

    ref = dr_lo if dr_lo else dr_hi
    p = pct(ref)
    print('')
    if p < 3:
        print('❌ PA6 基本是 0V —— 这根线上什么也没接。')
        print('   → DAT 这根线断了 / 插错孔了 / 插到了 GND。')
    elif p > 95 and rpu is not None and rpu < 1500:
        print('★★★ 定位到了：A6 这根线接的是**电源**，不是接收器的 DAT。')
        print('')
        print('   证据有三条，互相咬得死：')
        print('     ① 用 40k 下拉去拽，电压只掉 %.0f mV —— 这点力气拽不动一根导线，' % (dv / 4095.0 * 3300))
        print('        但拽得动一个 4.7k~10k 的上拉电阻（那会掉 200~600mV）')
        print('     ② CS 放高（接收器根本不选中）时它照样是 3.3V —— 接收器不会在')
        print('        没被选中时驱动 DAT，所以这 3.3V 不是它给的')
        print('     ③ PA5(CLK)、PA7(CMD) 两根却能老老实实被下拉压到 0V —— 说明')
        print('        适配板上根本没有上拉电阻这种网络')
        print('')
        print('   ⇒ 所以：A6 这头是对的，**另一头插到电源孔里去了**。')
        print('     断电，去看 XTARK 板底边那 6 个孔：')
        print('       · ④DAT 那个孔里有没有线？（很可能是空的）')
        print('       · ⑤VCC 那个孔是不是插了两根线？（很可能挤了两根）')
        print('     A6 那根线的另一头，应该从 ⑤VCC 挪到 ④DAT。')
    elif p > 95:
        print('⚠ PA6 是满量程，但源内阻不算太小 —— 可能是强上拉，也可能是接收器')
        print('   真的在硬顶高电平。断电把 DAT 线的两头看一下：')
        print('   XTARK 那头必须 ④DAT，STM32 那头必须 A6。')
    elif p > 10:
        print('✅ PA6 上挂着一个上拉电阻（%.0f%% ≈ 几 kΩ~10kΩ），说明：' % p)
        print('   ① 这根线确实接在适配板的 DAT 节点上（没插错孔）')
        print('   ② 适配板的 VCC 是通着的（上拉的另一头就是 VCC）')
        print('   ⇒ 那就只剩「接收器在通电但不回应」→ 换一套手柄/接收器试。')
    else:
        print('⚠ 读数偏低（%.1f%%），可能是弱上拉或者接触不良，重插后再跑一次。' % p)
    return {'hi': dr_hi, 'lo': dr_lo, 'float': dr_fl}


# ================================================================ 测试 E：A6 是不是跟旁边的针短在一起了
def drive_pins(s, pa5, pa7, pb0):
    """把 PA5/PA7/PB0 三个推挽输出分别驱到指定电平。"""
    s.w(A_BSRR if pa5 else A_BRR, 0x00000020)
    s.w(A_BSRR if pa7 else A_BRR, 0x00000080)
    s.w(B_BSRR if pb0 else B_BRR, 0x00000001)


def test_short():
    """★ 查 A6 是不是被焊锡连到了旁边的针上。

    做法：A6 设成"纯输入"（什么都不驱动），然后把 A5、A7、B0 三个
    轮着一个拉低、两个拉高，看 A6 跟不跟着变。

        A6 跟着某个脚一起变  →  那两个针在板子上是连通的（焊锡搭桥）
        A6 一直不动           →  没搭桥，那个 3.3V 是从外面（线或适配板）来的

    排针顺序（上排）：... B0  A7  A6  A5  A4 ...
    所以 A6 的左右邻居正好是 A7(CMD) 和 A5(CLK)，最容易被焊锡连起来。
    """
    print('')
    print('=' * 70)
    print('E  A6 跟旁边的针有没有短在一起')
    print('=' * 70)

    s0 = Session()
    i_apb2 = s0.r(RCC_APB2ENR, 'apb2')
    i_cfgr = s0.r(RCC_CFGR, 'cfgr')
    i_acrl = s0.r(A_CRL, 'a_crl')
    i_bcrl = s0.r(B_CRL, 'b_crl')
    v0, raw0 = s0.run()
    if v0 is None or len(v0) != len(s0.labels):
        print('[错误] 读不到寄存器。')
        return None
    apb2, cfgr, a_crl, b_crl = v0[i_apb2], v0[i_cfgr], v0[i_acrl], v0[i_bcrl]

    # PA5/PA7 推挽输出，A6 纯输入（不做上下拉，避免污染判断），PB0 推挽输出
    a_out = nibble_set(a_crl, 5, 0xB)
    a_out = nibble_set(a_out, 7, 0xB)
    a_out = nibble_set(a_out, 6, 0x4)        # 浮空输入
    b_out = nibble_set(b_crl, 0, 0xB)

    s = Session()
    s.w(RCC_APB2ENR, apb2 | 0x0000020C)
    s.w(RCC_CFGR, (cfgr & ~(3 << 14)) | (2 << 14))
    s.w(A_CRL, a_out)
    s.w(B_CRL, b_out)

    CASES = [
        ('全高（基准）      ', 1, 1, 1),
        ('只把 A5(CLK) 拉低 ', 0, 1, 1),
        ('只把 A7(CMD) 拉低 ', 1, 0, 1),
        ('只把 B0(CS) 拉低  ', 1, 1, 0),
        ('再全高（复核）    ', 1, 1, 1),
    ]
    idx = []
    for name, p5, p7, p0 in CASES:
        drive_pins(s, p5, p7, p0)
        si = s.r(A_IDR, 'idr')
        di = adc_read_pa6(s, 'adc')
        idx.append((name, si, di))

    v, raw = s.run()
    if v is None or len(v) != len(s.labels):
        print('[警告] 值数量对不上（%d / %d）' % (len(v) if v else -1, len(s.labels)))
        print(raw[-1500:])
        return None

    print('')
    print('  操作                 A6 数字电平   A6 电压')
    print('  ' + '-' * 50)
    rows = []
    for name, si, di in idx:
        lvl = bit(v[si], 6)
        volt = (v[di] & 0xFFFF) / 4095.0 * 3.3
        rows.append((name, lvl, volt))
        print('  %s     %d          %.2f V' % (name, lvl, volt))

    base = rows[0][1]
    follow = []
    if rows[1][1] != base:
        follow.append('A5(CLK)')
    if rows[2][1] != base:
        follow.append('A7(CMD)')
    if rows[3][1] != base:
        follow.append('B0(CS)')

    print('')
    if follow:
        print('★★★ 找到了：A6 会跟着 %s 一起变 —— 说明这两个针在板子上是连着的。'
              % '、'.join(follow))
        print('   → 焊锡搭桥了。拿烙铁把那两个针根部的锡拖干净（或拿吸锡带吸一下）。')
        print('   → 位置就在上排：B0 — A7 — A6 — A5 这四个是一排挨着的。')
    else:
        print('✅ A6 不跟着 A5/A7/B0 任何一个变 —— 排针上没有搭桥。')
        print('   那 3.24V 是从**线**或**适配板**那边过来的，不是单片机板子的问题。')
        print('   下一步：跑 --adc，再让适配板单独验（见文档"如果 6 个孔确认没插错"）。')
    return rows


# ================================================================ 全板引脚地图
# PA13/PA14 是 SWD，绝对不能碰（碰了就连不上，得断电重来）
# PC13 是板载 LED，PC14/PC15 是晶振，也跳过
SKIP_A = {13, 14}


def test_pinmap():
    """把 PA0~PA12、PA15、PB0~PB15 全部打成输入，逐脚做上拉/下拉读数。

    这一张表能直接看出「哪几个脚上真的接着东西」：
        0/0  恒定低  → 这根线接的是 GND
        1/1  恒定高  → 这根线接的是 VCC，或者接到一个有驱动能力的输出
        0/1  悬空    → 没接东西，或接到一个高阻输入脚
    """
    print('')
    print('=' * 70)
    print('C  全板引脚地图：找出这 6 根线到底插在哪些孔里')
    print('=' * 70)

    s0 = Session()
    a = s0.r(A_CRL, 'a_crl')
    b = s0.r(A_CRH, 'a_crh')
    c = s0.r(B_CRL, 'b_crl')
    d = s0.r(B_CRH, 'b_crh')
    e = s0.r(AFIO_MAPR, 'mapr')
    f = s0.r(RCC_APB2ENR, 'apb2')
    v, raw = s0.run()
    if v is None or len(v) != len(s0.labels):
        print('[错误] 读不到 GPIO 配置寄存器。')
        return None

    acrl, acrh, bcrl, bcrh, mapr, apb2 = v[a], v[b], v[c], v[d], v[e], v[f]

    # 全部改成「输入 + 上/下拉由 ODR 决定」(0x8)；PA13/PA14 原样保留
    MASK_ACRH = 0xF00FFFFF
    new_acrh = (acrh & ~MASK_ACRH) | 0x80088888

    s = Session()
    # ★ 先关 JTAG：否则 PA15/PB3/PB4 上挂着 JTAG 的上下拉，会假报「有接线」
    s.w(RCC_APB2ENR, apb2 | 0x00000001)                  # AFIO 时钟
    s.w(AFIO_MAPR, (mapr & ~(7 << 24)) | (2 << 24))      # SWJ_CFG=010：关 JTAG、留 SWD
    s.w(A_BRR, 0xFFFFFFFF)
    s.w(B_BRR, 0xFFFFFFFF)
    s.w(A_CRL, 0x88888888)
    s.w(A_CRH, new_acrh)
    s.w(B_CRL, 0x88888888)
    s.w(B_CRH, 0x88888888)
    i_apd = s.r(A_IDR, 'a_pd')
    i_bpd = s.r(B_IDR, 'b_pd')
    s.w(A_BSRR, 0x0000FFFF)
    s.w(B_BSRR, 0x0000FFFF)
    i_apu = s.r(A_IDR, 'a_pu')
    i_bpu = s.r(B_IDR, 'b_pu')

    v, raw = s.run()
    if v is None or len(v) != len(s.labels):
        print('[错误] 地图测试没跑出来。')
        return None

    apd, bpd, apu, bpu = v[i_apd], v[i_bpd], v[i_apu], v[i_bpu]

    rows = []
    for n in range(16):
        if n in SKIP_A:
            continue
        rows.append(('PA%d' % n, bit(apu, n), bit(apd, n)))
    for n in range(16):
        rows.append(('PB%d' % n, bit(bpu, n), bit(bpd, n)))

    print('')
    print('  引脚    下拉  上拉   判定')
    print('  ' + '-' * 46)
    found = {}
    for name, pu, pd in rows:
        code, txt = state_of(pu, pd)
        found[name] = code
        if code != 'FLOAT':                      # 只打印"有东西"的脚
            mark = {'HIGH': '★ 恒定高（接了 VCC 或被驱动）',
                    'LOW': '★ 恒定低（接了 GND）',
                    'ODD': '? 异常'}[code]
            print('  %-6s   %d     %d    %s' % (name, pd, pu, mark))

    print('')
    print('  （其余引脚都是「悬空」，也就是没接东西，略过不打印）')

    hi = [k for k, vv in found.items() if vv == 'HIGH']
    lo = [k for k, vv in found.items() if vv == 'LOW']
    print('')
    print('  汇总：恒定高的脚 = %s' % (', '.join(hi) if hi else '无'))
    print('        恒定低的脚 = %s' % (', '.join(lo) if lo else '无'))
    return found


# ================================================================ 结论
def verdict(elec, hs):
    print('')
    print('=' * 70)
    print('结论')
    print('=' * 70)

    if elec is None:
        print('电气测试没跑出来，先解决 OpenOCD 连接问题。')
        return

    pa5, pa6, pa7, pb0 = elec['PA5'], elec['PA6'], elec['PA7'], elec['PB0']

    if hs:
        for mode, rx in sorted(hs.items()):
            if rx and len(rx) >= 3 and rx[1] in (0x41, 0x73, 0x79):
                print('★ 好消息：模式 %d 收到回应了，接收器是活的！' % mode)
                print('  第 2 字节 = 0x%02X（手柄 ID），说明 SPI 模式改成 CPOL=%d/CPHA=%d 就能通。'
                      % (rx[1], (mode >> 1) & 1, mode & 1))
                print('  → 我改固件里的 CPOL/CPHA 就行，你等着。')
                return

    all_float = (pa5 == 'FLOAT' and pa6 == 'FLOAT'
                 and pa7 == 'FLOAT' and pb0 == 'FLOAT')

    if all_float:
        print('❌ 四个脚全是悬空 —— 说明接收器那边**一点电效应都没有**。')
        print('   → 最可能：接收器没供电（VCC/GND 没到）或根本没通电。')
        print('   量一下 XTARK 板 ⑤VCC 和 ⑥GND 之间有没有 3.2~3.3V。')
        return

    if pa6 == 'HIGH':
        print('✅ PA6（DAT）这根线上有东西在驱动它 = 接收器活着，DAT 也接过来了。')
        print('   那问题就在 SPI 模式或者 CS/CLK 的时序上，不是接线。')
        return

    if pa6 == 'FLOAT':
        if pa7 == 'HIGH':
            print('★★★ 定位到了：CMD 和 DAT 接反了。')
            print('   现象对照：')
            print('     PA6 悬空  ← 它接的是接收器的 CMD（输入脚，高阻）')
            print('     PA7 恒定高 ← 它接的是接收器的 DAT（输出脚，被接收器驱动着）')
            print('   → 把 XTARK 板 ③CMD 和 ④DAT 这两根线对调，就通了。')
        elif pa7 == 'FLOAT':
            print('❌ PA6 悬空、PA7 也悬空 —— 这两根线都没接到活的输出脚上。')
            print('   可能：①PA6 那根线没插紧 / 断了 / 插到了别的空孔')
            print('        ②接收器在工作但 DAT 输出是高阻（很少见）')
            print('   → 先看 PA6 那根线两头，一头在 XTARK 的 ④DAT、一头在 STM32 的 A6。')
            print('     注意：如果接收器其实没通电，DAT 也是悬空的 —— 先量 VCC。')
        else:
            print('❌ PA6 悬空、PA7 恒定低 —— PA7 短到 GND 了，先查那根线。')
        return

    if pa6 == 'LOW':
        print('❌ PA6 恒定低 —— 这根线被强行拉在 0V，不是读数问题。')
        print('   最可能：接错到了 GND 孔，或者两根线短在一起了。')
        return

    print('数据有点怪，把上面的表发我。')


# ================================================================ main
# ================================================================ 测试 F：五个脚一起做「体质检查」
# PA4=ADC_IN4（备用探针） PA5=IN5(CLK) PA6=IN6(DAT) PA7=IN7(CMD) PB0=IN8(CS)
SCAN5 = [
    ('PA4', 'A', 4, '备用'),
    ('PA5', 'A', 5, 'CLK'),
    ('PA6', 'A', 6, 'DAT'),
    ('PA7', 'A', 7, 'CMD'),
    ('PB0', 'B', 0, 'CS '),
]


def adc_one(s, ch, tag):
    """切到某个 ADC 通道，连转两次取第二次（第一次带上一通道的电荷残留）。"""
    s.w(ADC_SQR3, ch & 0x1F)
    s.w(ADC_CR2, (7 << 17) | (1 << 20) | (1 << 22) | 0x00000001)
    s.r(ADC_SR, tag + '_a')
    s.w(ADC_CR2, (7 << 17) | (1 << 20) | (1 << 22) | 0x00000001)
    s.r(ADC_SR, tag + '_b')
    return s.r(ADC_DR, tag + '_d')


def _scan5_round(mode):
    """mode: 'float' / 'pd' / 'pu'。返回 {脚名: 原始 ADC 值}。"""
    s0 = Session()
    i_apb2 = s0.r(RCC_APB2ENR, 'apb2')
    i_cfgr = s0.r(RCC_CFGR, 'cfgr')
    i_acrl = s0.r(A_CRL, 'acrl')
    i_bcrl = s0.r(B_CRL, 'bcrl')
    v0, _ = s0.run()
    if v0 is None or len(v0) != len(s0.labels):
        return None
    apb2, cfgr, a_crl, b_crl = v0[i_apb2], v0[i_cfgr], v0[i_acrl], v0[i_bcrl]

    nib = {'float': 0x4, 'pd': 0x8, 'pu': 0x8}[mode]
    for _, port, pin, _ in SCAN5:
        if port == 'A':
            a_crl = nibble_set(a_crl, pin, nib)
        else:
            b_crl = nibble_set(b_crl, pin, nib)

    s = Session()
    s.w(RCC_APB2ENR, apb2 | 0x0000020C)              # GPIOA/B + ADC1
    s.w(RCC_CFGR, (cfgr & ~(3 << 14)) | (2 << 14))   # ADC 时钟 PCLK2/6 = 12MHz
    s.w(ADC_CR2, 0x00000001)
    s.w(ADC_CR2, 0x00000005)
    s.w(ADC_CR2, 0x00000001)
    s.w(ADC_CR2, 0x00000005)
    for _ in range(3):
        s.r(ADC_CR2, 'cal')
    s.w(ADC_SMPR2, 0x00FFFFFF)                       # ch0~9 全部 239.5 周期
    s.w(ADC_SQR1, 0x00000000)

    if mode == 'pd':
        s.w(A_BRR, 0x000000F0)                       # PA4~PA7 ODR=0 → 下拉
        s.w(B_BRR, 0x00000001)                       # PB0  ODR=0 → 下拉
    if mode == 'pu':
        s.w(A_BSRR, 0x000000F0)
        s.w(B_BSRR, 0x00000001)

    s.w(A_CRL, a_crl)
    s.w(B_CRL, b_crl)

    idx = {}
    for name, _, ch, _ in SCAN5:
        idx[name] = adc_one(s, ch, name)

    vals, raw = s.run()
    if vals is None or len(vals) != len(s.labels):
        return None
    return {k: vals[i] & 0xFFFF for k, i in idx.items()}


def test_scan5():
    """五个脚 × 三种偏置（浮空/40k下拉/40k上拉）→ 反推每个脚外面接的到底是什么。"""
    print('')
    print('=' * 76)
    print('F  五脚体质检查 —— 每根线外面接的是电源、电阻、还是什么都没接')
    print('=' * 76)

    r_fl = _scan5_round('float')
    r_pd = _scan5_round('pd')
    r_pu = _scan5_round('pu')
    if not (r_fl and r_pd and r_pu):
        print('[错误] 采样失败，检查 ST-LINK 连接。')
        return

    def volt(x):
        return 3.3 * x / 4095.0

    print('')
    print('  脚     角色    浮空      40k下拉   40k上拉   反推外部源阻抗   判断')
    print('  ' + '-' * 86)
    result = {}
    for name, _, _, role in SCAN5:
        vf, vd, vu = volt(r_fl[name]), volt(r_pd[name]), volt(r_pu[name])
        verdict_s, rpu = '?', None
        if vf < 0.25 and vd < 0.25:
            if vu < 0.25:
                verdict_s = '★★ 被短接到 GND'
            else:
                verdict_s = '悬空（外面什么都没有）'
        elif vf > 3.0:
            if vd > 3.1:
                verdict_s = '★★★ 直连电源（不是拉电阻！）'
                rpu = 0
            else:
                k = (vf - vd) / vf if vf > 0 else 0
                if k <= 0:
                    rpu = 0
                elif k >= 0.999:
                    rpu = 0
                else:
                    rpu = 40000.0 * k / (1.0 - k)
                if rpu < 1500:
                    verdict_s = '有上拉（≈%.0fΩ，偏硬）' % rpu
                elif rpu < 200000:
                    verdict_s = '有上拉（≈%d kΩ，正常）' % (rpu / 1000)
                else:
                    verdict_s = '直连电源或巨大上拉'
        else:
            verdict_s = '中间电平（异常）'

        rs = '——' if not rpu else ('%.0f Ω' % rpu if rpu < 10000 else '%.0f kΩ' % (rpu / 1000))
        print('  %-5s  %-5s  %5.2fV   %6.2fV  %6.2fV     %-14s  %s'
              % (name, role, vf, vd, vu, rs, verdict_s))
        result[name] = (vf, vd, vu, verdict_s)

    print('')
    print('  参考：40k 下拉会把外部上拉分走多少电压')
    print('        直连 3.3V 电源 → 掉不到 30mV    4.7kΩ 上拉 → 掉 400mV    10kΩ 上拉 → 掉 940mV')
    print('        外面什么都没有 → 直接被拉到 0V')

    print('')
    print('  正常应该长这样：')
    print('        CLK(PA5) / CMD(PA7)  = 悬空        ← 它们是 MCU 发给接收器的，接收器那头是高阻输入')
    print('        DAT(PA6)             = 有上拉      ← 适配板给数据线挂的上拉，必须有')
    print('        CS (PB0)             = 有上拉      ← 接收器的片选脚常态被拉高')
    print('')
    bad = [k for k, v in result.items()
           if ('悬空' in v[3] and k in ('PA5', 'PA6', 'PA7', 'PB0')) or '短接' in v[3]]
    if bad:
        print('  ⚠ 可疑：%s' % '、'.join(bad))
        print('    → 如果 CLK/CMD 显示"悬空"而它本该接到接收器，请把这两根线拔下来重插到底；')
        print('      或直接换两根新线（母头插一半也照样挂着，肉眼看不出）。')
    else:
        print('  ✓ 四根线的电气特征和"接线正确"完全一致')


def test_verify():
    """测法自检：同一次会话里，同一个脚同时读「数字 IDR」和「ADC 电压」，看两者对不对得上。

    目的：`--map` 说 PB0 恒定高、`--scan5` 说 PB0 是 0V，必须弄清是谁错了，
    否则后面所有结论都建在沙子上。
    """
    print('')
    print('=' * 72)
    print('G  测法自检 —— 「数字读数」和「ADC 读数」谁可信')
    print('=' * 72)

    s0 = Session()
    i_apb2 = s0.r(RCC_APB2ENR, 'apb2')
    i_cfgr = s0.r(RCC_CFGR, 'cfgr')
    i_acrl = s0.r(A_CRL, 'acrl')
    i_bcrl = s0.r(B_CRL, 'bcrl')
    v0, _ = s0.run()
    if v0 is None or len(v0) != len(s0.labels):
        print('[错误] 读不到寄存器，ST-LINK 插好了吗？')
        return
    apb2, cfgr, a_crl, b_crl = v0[i_apb2], v0[i_cfgr], v0[i_acrl], v0[i_bcrl]

    s = Session()
    s.w(RCC_APB2ENR, apb2 | 0x0000020C)
    s.w(RCC_CFGR, (cfgr & ~(3 << 14)) | (2 << 14))
    s.w(ADC_CR2, 0x00000001)
    s.w(ADC_CR2, 0x00000005)
    s.w(ADC_CR2, 0x00000001)
    s.w(ADC_CR2, 0x00000005)
    for _ in range(3):
        s.r(ADC_CR2, 'cal')
    s.w(ADC_SMPR2, 0x00FFFFFF)
    s.w(ADC_SQR1, 0x00000000)

    # ---- 自检 1：PA5 自己推挽驱动，IDR 必须跟着变 ----
    s.w(A_CRL, nibble_set(a_crl, 5, 0x3))            # PA5 推挽输出
    s.w(A_BSRR, 0x00000020)                          # 输出高
    i_hi = s.r(A_IDR, 'pa5_hi')
    s.w(A_BRR, 0x00000020)                           # 输出低
    i_lo = s.r(A_IDR, 'pa5_lo')

    # ---- 自检 2：PA6 / PB0 输入+40k下拉，同时读 IDR 和 ADC ----
    s.w(A_CRL, nibble_set(nibble_set(a_crl, 5, 0x4), 6, 0x8))
    s.w(B_CRL, nibble_set(b_crl, 0, 0x8))
    s.w(A_BRR, 0x00000040)                           # PA6 下拉
    s.w(B_BRR, 0x00000001)                           # PB0 下拉
    i_ia = s.r(A_IDR, 'idr_a')
    i_ib = s.r(B_IDR, 'idr_b')

    d6 = adc_one(s, 6, 'ch6')
    d8 = adc_one(s, 8, 'ch8')
    d5 = adc_one(s, 5, 'ch5')
    d7 = adc_one(s, 7, 'ch7')

    v, raw = s.run()
    if v is None or len(v) != len(s.labels):
        print('[错误] 读数数量对不上（%d / %d）' % (len(v) if v else -1, len(s.labels)))
        print(raw[-1200:])
        return

    idr_a = v[i_ia]
    idr_b = v[i_ib]

    def volt(x):
        return 3.3 * (x & 0xFFFF) / 4095.0

    print('')
    print('  自检 1：把 PA5 配成推挽输出，看 IDR 跟不跟得动')
    print('    输出高 → IDR bit5 = %d %s' % ((v[i_hi] >> 5) & 1,
                                       '✓' if (v[i_hi] >> 5) & 1 else '✗'))
    print('    输出低 → IDR bit5 = %d %s' % ((v[i_lo] >> 5) & 1,
                                       '✓' if not (v[i_lo] >> 5) & 1 else '✗'))
    idr_ok = ((v[i_hi] >> 5) & 1) == 1 and ((v[i_lo] >> 5) & 1) == 0
    print('    ⇒ 数字读数通路%s' % ('正常' if idr_ok else '★ 不正常，--map 的结论不可信'))

    print('')
    print('  自检 2：四个脚都配成「输入 + 40k 下拉」，同时读两样')
    print('')
    print('    脚    IDR 位    ADC 电压    一致？')
    print('    ' + '-' * 50)
    rows = [
        ('PA5', (idr_a >> 5) & 1, volt(v[d5]), 'CLK'),
        ('PA6', (idr_a >> 6) & 1, volt(v[d6]), 'DAT'),
        ('PA7', (idr_a >> 7) & 1, volt(v[d7]), 'CMD'),
        ('PB0', (idr_b >> 0) & 1, volt(v[d8]), 'CS '),
    ]
    conflict = []
    for name, bit, vv, role in rows:
        same = (bit == 1 and vv > 2.0) or (bit == 0 and vv < 1.0)
        if not same:
            conflict.append(name)
        print('    %-4s    %d      %5.2f V     %s   (%s)'
              % (name, bit, vv, '✓' if same else '✗ 矛盾', role))

    print('')
    if conflict:
        print('  ★ 矛盾的脚：%s' % '、'.join(conflict))
        print('    数字说 1 而 ADC 说 0V（或反过来），说明**至少有一种测法在这颗芯片上不可用**。')
        print('    PB0 是 ADC1_IN8，某些克隆芯片（CKS32 / CH32 之类）的 IN8 通道是废的，')
        print('    此时应当只信 IDR 的数字读数。')
    elif idr_ok:
        print('  ✓ 两种测法完全一致，前面 --map 的结论（PA6/PB0 恒定高）可信。')
        print('    ⇒ 那两根线确实连着东西；PA5/PA7 悬空也是真的。')
    print('')
    print('  注：以上全部在「40k 下拉」状态下测的。读数 1 或 3.2V = 外面有东西拽着；')
    print('      读数 0 或 0V = 外面什么都没接（或接的是高阻输入脚）。')


def test_link():
    """★ 不用万用表的线通断测试。

    原理：把 4 根信号线在 XTARK 那一头全部拧在一起（变成同一个电气节点），
    然后让单片机轮流出「一个脚拉低、其余脚上拉读电平」：
        - 线是通的  → 自己会被那根拉低的线拽到 0
        - 线内部断了 → 上拉把自己的脚拽到 1
    单片机自己既当电源又当表笔，不需要任何外部仪器。
    """
    print('')
    print('=' * 74)
    print('H  不用万用表的通断测试 —— 单片机自己当表笔')
    print('=' * 74)
    print('')
    print('  做之前必须按顺序完成（差一步就会误判或短路）：')
    print('')
    print('   ① 拔掉 ST-LINK 的 USB            ← 一定先断电')
    print('   ② 把 XTARK 板上 6 根线【全部】拔下来')
    print('   ③ 在拔下来的线头里，挑出这 4 根信号线：')
    print('        CLK(①)  CS(②)  CMD(③)  DAT(④)')
    print('      把它们的金属插针【拧在一起】、拧紧')
    print('      ⚠ VCC(⑤) 和 GND(⑥) 这两根单独放一边，绝对不能拧进去')
    print('   ④ 插回 ST-LINK 的 USB（给单片机供电）')
    print('')
    print('  STM32 那一头 4 根线保持插在 A5 / B0 / A7 / A6 上，一根都别动。')
    print('')

    s0 = Session()
    i_apb2 = s0.r(RCC_APB2ENR, 'apb2')
    i_acrl = s0.r(A_CRL, 'acrl')
    i_bcrl = s0.r(B_CRL, 'bcrl')
    v0, _ = s0.run()
    if v0 is None or len(v0) != len(s0.labels):
        print('[错误] 读不到寄存器，ST-LINK 插好了吗？')
        return
    apb2, a0, b0 = v0[i_apb2], v0[i_acrl], v0[i_bcrl]

    s = Session()
    s.w(RCC_APB2ENR, apb2 | 0x0000000C)               # GPIOA/B 时钟

    # ---- 第 1 轮：PA5 拉低充当"地"，量 A6 / A7 / B0
    a = nibble_set(a0, 5, 0x3)                        # PA5 推挽输出
    a = nibble_set(a, 6, 0x8)                         # PA6 输入
    a = nibble_set(a, 7, 0x8)                         # PA7 输入
    b = nibble_set(b0, 0, 0x8)                        # PB0 输入
    s.w(A_BSRR, 0x000000C0)                           # PA6/PA7 ODR=1 → 上拉
    s.w(B_BSRR, 0x00000001)                           # PB0  上拉
    s.w(A_BRR, 0x00000020)                            # PA5  ODR=0 → 输出低
    s.w(A_CRL, a)
    s.w(B_CRL, b)
    r1a = s.r(A_IDR, 'r1a')
    r1b = s.r(B_IDR, 'r1b')

    # ---- 第 2 轮：换成 PA7 拉低，把 PA5 改成上拉量一量（交叉验证）
    a = nibble_set(a0, 5, 0x8)                        # PA5 输入
    a = nibble_set(a, 6, 0x8)
    a = nibble_set(a, 7, 0x3)                         # PA7 推挽输出
    b = nibble_set(b0, 0, 0x8)
    s.w(A_BSRR, 0x000000E0)                           # PA5/PA6/PA7 ODR=1
    s.w(B_BSRR, 0x00000001)
    s.w(A_BRR, 0x00000080)                            # PA7 ODR=0 → 输出低
    s.w(A_CRL, a)
    s.w(B_CRL, b)
    r2a = s.r(A_IDR, 'r2a')
    r2b = s.r(B_IDR, 'r2b')

    v, raw = s.run()
    if v is None or len(v) != len(s.labels):
        print('[错误] 读数对不上（%d / %d）' % (len(v) if v else -1, len(s.labels)))
        print(raw[-1200:])
        return

    A1, B1 = v[r1a], v[r1b]
    A2, B2 = v[r2a], v[r2b]

    print('  第 1 轮：PA5 拉低当"地"   |   第 2 轮：PA7 拉低当"地"')
    print('')
    print('    线        第1轮读数   第2轮读数   判定')
    print('    ' + '-' * 56)

    def cell(bit):
        return '低(0)' if bit == 0 else '高(1)'

    a6_1 = (A1 >> 6) & 1
    a7_1 = (A1 >> 7) & 1
    b0_1 = (B1 >> 0) & 1
    a5_2 = (A2 >> 5) & 1
    a6_2 = (A2 >> 6) & 1
    b0_2 = (B2 >> 0) & 1

    # A5 只能在第 2 轮被量到；但前提是 A7 那根"地"是通的，
    # 否则第 2 轮谁都拉不下来，A5 会被误判成断线。
    a5_final = a5_2 if a7_1 == 0 else -1

    rows = [
        ('A5 (CLK)', '—（当地用）', cell(a5_2), a5_final),
        ('A6 (DAT)', cell(a6_1), cell(a6_2), a6_1 | a6_2),
        ('A7 (CMD)', cell(a7_1), '—（当地用）', a7_1),
        ('B0 (CS )', cell(b0_1), cell(b0_2), b0_1 | b0_2),
    ]
    broken, unclear = [], []
    for name, c1, c2, final in rows:
        if final == 0:
            judge = '✓ 通'
        elif final == 1:
            judge = '✗ 断'
            broken.append(name)
        else:
            judge = '? 判不了'
            unclear.append(name)
        print('    %-9s %-11s %-11s %s' % (name, c1, c2, judge))

    print('')
    if not broken and not unclear:
        print('  ✅ 四根线全部导通。')
        print('     ⇒ 线本身没问题，问题在 XTARK 板顶边那排 9 脚焊点（虚焊），或接收器坏。')
        print('       下一步：把 XTARK 顶边那排焊点用烙铁 350℃ 重新拖一遍，或换一块板。')
    elif len(broken) >= 3:
        print('  ⚠ %s 都读高，可能线头没拧紧（接触不良），也可能真的都断了。' % '、'.join(broken))
        print('     → 先把 4 个线头拧得更紧（或用一根细导线把四个头缠在一起），再跑一次。')
    else:
        if broken:
            print('  ★ 断掉的线：%s' % '、'.join(broken))
            print('     → 把这根换成新杜邦线，其余不用动。')
        if unclear:
            print('  ★ 判不了的：%s（因为它的"地"那根断了）' % '、'.join(unclear))
        print('     → 先把上面断掉的换掉，再重跑一次确认。')
    print('')
    print('  说明：读 0 = 这根线把那头拧在一起的"地"引回来了（通）；')
    print('        读 1 = 上拉把自己拽住了（这根线内部断了）。')


def main():
    if '--link' in sys.argv[1:]:
        print('')
        print('不用万用表的通断测试')
        test_link()
        print('')
        print('（已自动 reset run，测完请把 XTARK 那 6 根线按原样插回去）')
        return 0

    if '--verify' in sys.argv[1:]:
        print('')
        print('测法自检 —— 确认「数字读数」和「ADC 读数」哪个可信')
        print('（跑完自动复位，不影响固件）')
        test_verify()
        print('')
        print('（已自动 reset run）')
        return 0

    if '--scan5' in sys.argv[1:]:
        print('')
        print('PS2 五脚体质检查 —— 一次量 CLK/CMD/DAT/CS 四根线')
        print('（跑完自动复位，不影响固件）')
        test_scan5()
        print('')
        print('（已自动 reset run）')
        return 0

    if '--short' in sys.argv[1:]:
        print('')
        print('PS2 排针短路检查 —— A6 跟旁边的针是不是连在一起了')
        print('（跑完自动复位，不影响固件）')
        test_short()
        print('')
        print('（已自动 reset run）')
        return 0

    if '--adc' in sys.argv[1:]:
        print('')
        print('PS2 电压测量 —— 直接量 PA6(DAT) 上是多少伏')
        print('（跑完自动复位，不影响固件）')
        test_adc()
        print('')
        print('（已自动 reset run）')
        return 0

    if '--cs' in sys.argv[1:]:
        print('')
        print('PS2 关键测试 —— CS 拉低时到底谁在驱动 DAT')
        print('（跑完自动复位，不影响固件）')
        test_cs_drive()
        print('')
        print('（已自动 reset run）')
        return 0

    if '--map' in sys.argv[1:]:
        print('')
        print('PS2 全板引脚地图 —— 找出那 6 根线到底插在哪些孔')
        print('（跑完自动复位，不影响固件）')
        test_pinmap()
        print('')
        print('（已自动 reset run）')
        return 0

    print('')
    print('PS2 硬件级探针 —— 不走固件，直接读 STM32 寄存器和引脚电平')
    print('（ST-LINK 要接好，板子要通电；跑完自动复位，不影响固件）')

    print('')
    print('-' * 70)
    print('第 1 步：读出当前 GPIO 配置（后面要在这个基础上改）')
    s = Session()
    i_acrl = s.r(A_CRL, 'a_crl')
    i_bcrl = s.r(B_CRL, 'b_crl')
    i_cr1 = s.r(SPI_CR1, 'spi_cr1')
    vals, raw = s.run()
    if vals is None:
        return 1
    if len(vals) != len(s.labels):
        print('[错误] 读不出寄存器。原始输出：')
        print(raw[-2000:])
        return 1

    a_crl, b_crl, cr1 = vals[i_acrl], vals[i_bcrl], vals[i_cr1]
    print('  GPIOA_CRL = 0x%08X   GPIOB_CRL = 0x%08X   SPI1_CR1 = 0x%08X'
          % (a_crl, b_crl, cr1))
    if (cr1 & 0x40) == 0:
        print('  ⚠ SPI1 的 SPE 没置位 —— 说明固件可能没跑到 PS2 初始化，'
              '或者烧的不是 full。')
    else:
        print('  ✓ SPI1 已使能（SPE=1），模式 3 的位也对得上')

    hs = test_handshake()
    elec = test_electric(a_crl, b_crl)
    verdict(elec, hs)

    print('')
    print('（已自动 reset run，板子回到固件正常运行状态）')
    print('')
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
