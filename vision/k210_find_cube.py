# -*- coding: utf-8 -*-
"""
k210_find_cube.py —— CRTC2026 视觉模块（MaixPy / K210）

功能：识别场地上的 白色能量单元 / 黄色能量核心，把最大色块的中心坐标、
      尺寸、面积通过串口按定长帧发给 STM32。

帧格式（与 firmware/Core/Inc/vision.h 严格对应）：
    AA 55 cls xL xH yL yH wL wH hL hH areaL areaH chk
    cls: 1=白块(能量单元)  2=黄块(能量核心)  0=没找到
    chk : 从 cls 到 areaH 全部字节依次异或，取低 8 位

接线（K210 Dock / K210 Bit 都一样）：
    K210 IO7(TX)  -> STM32 PA10 (USART1 RX)
    K210 IO6(RX)  -> STM32 PA9  (USART1 TX)   [可只接一根，单向也够用]
    GND           -> 共地（必须！不共地会收到乱码）
    K210 5V/3V3   -> 从 5V 降压模块取（别直接挂在电机电源上）

【新手调试顺序 —— 不要跳步】
  1. 先跑 CALIB_MODE = 1，用 MaixPy IDE 的「工具 -> 机器视觉 -> 阈值编辑器」
     对着实际方块抠阈值，把打印出来的 LAB 阈值抄到下面的 YELLOW_TH / WHITE_TH。
     场地灯一换就要重抠，这是视觉 90% 的坑。
  2. 关掉 CALIB_MODE，打开串口助手看 x/y 是否合理（画面中心是 160,120）。
  3. 再接到 STM32 上，看 app.c 打出来的 "vis cls=..." 行。
  4. 最后才调 config.h 里的 VISION_TURN_KP（先给 0.001，车反应不够再加）。

作者：机器人总动员战队  2026
"""

import sensor
import image
import time
import lcd
from machine import UART
from fpioa_manager import fm

# ============================ 可调参数 ============================
CALIB_MODE   = 0        # 1 = 标定模式（只打印，不发串口）
UART_BAUD    = 115200
IMG_W, IMG_H = 320, 240
MIN_AREA     = 400      # 小于这个像素面积的色块当作噪点丢掉
MIN_WH       = 12       # 宽或高小于这么多像素也不要

# 这两个阈值必须现场标定！下面只是初始值，几乎肯定不对
YELLOW_TH = (50, 90, -20, 40, 25, 75)     # 黄色能量核心 (Lmin,Lmax,Amin,Amax,Bmin,Bmax)
WHITE_TH  = (65, 100, -12, 12, -12, 12)   # 白色能量单元
# ==================================================================


def init_uart():
    fm.register(7, fm.fpioa.UART1_TX, force=True)
    fm.register(6, fm.fpioa.UART1_RX, force=True)
    return UART(UART.UART1, UART_BAUD, 8, 0, 1, timeout=1000, read_buf_len=4096)


def make_frame(cls, x, y, w, h, area):
    x = int(max(-32768, min(32767, x)))
    y = int(max(-32768, min(32767, y)))
    b = bytearray(14)
    b[0] = 0xAA
    b[1] = 0x55
    b[2] = cls & 0xFF
    b[3] = x & 0xFF
    b[4] = (x >> 8) & 0xFF
    b[5] = y & 0xFF
    b[6] = (y >> 8) & 0xFF
    b[7] = w & 0xFF
    b[8] = (w >> 8) & 0xFF
    b[9] = h & 0xFF
    b[10] = (h >> 8) & 0xFF
    b[11] = area & 0xFF
    b[12] = (area >> 8) & 0xFF
    chk = 0
    for i in range(2, 13):
        chk ^= b[i]
    b[13] = chk & 0xFF
    return bytes(b)


def pick_best(blobs):
    """在若干色块里挑一个最可信的：面积优先，其次更靠近画面下方（离车近）"""
    best = None
    best_score = -1
    for b in blobs:
        if b.area() < MIN_AREA:
            continue
        if b.w() < MIN_WH or b.h() < MIN_WH:
            continue
        # 打分：面积越大越好；越靠画面下方（y 越大）说明越近，优先
        score = b.area() + b.cy() * 20
        if score > best_score:
            best_score = score
            best = b
    return best


def main():
    uart = init_uart()

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)      # 320x240
    sensor.skip_frames(time=2000)
    # 关自动增益/白平衡：不然灯一闪阈值就漂
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    try:
        sensor.set_vflip(False)
        sensor.set_hmirror(False)
    except Exception:
        pass

    lcd.init()
    clock = time.clock()

    while True:
        clock.tick()
        img = sensor.snapshot()

        blobs_y = img.find_blobs([YELLOW_TH], area_threshold=MIN_AREA,
                                 merge=True, margin=8)
        blobs_w = img.find_blobs([WHITE_TH], area_threshold=MIN_AREA,
                                 merge=True, margin=8)

        by = pick_best(blobs_y)
        bw = pick_best(blobs_w)

        cls, tx, ty, tw, th, tarea = 0, 0, 0, 0, 0, 0

        # 黄块得分高（4 分），同样可信时优先黄块
        if by is not None and (bw is None or by.area() * 2 >= bw.area()):
            cls, tx, ty, tw, th, tarea = 2, by.cx(), by.cy(), by.w(), by.h(), by.area()
            img.draw_rectangle(by.rect(), color=(255, 255, 0))
            img.draw_cross(by.cx(), by.cy(), color=(255, 255, 0))
        elif bw is not None:
            cls, tx, ty, tw, th, tarea = 1, bw.cx(), bw.cy(), bw.w(), bw.h(), bw.area()
            img.draw_rectangle(bw.rect(), color=(255, 255, 255))
            img.draw_cross(bw.cx(), bw.cy(), color=(255, 255, 255))

        # 画面中心画个十字，方便看偏差
        img.draw_cross(IMG_W // 2, IMG_H // 2, color=(0, 255, 0))
        lcd.display(img)

        if CALIB_MODE:
            print("fps=%.1f cls=%d x=%d y=%d w=%d h=%d area=%d" %
                  (clock.fps(), cls, tx, ty, tw, th, tarea))
            # 标定辅助：把鼠标/手指对着方块，打印中心像素的 LAB 值
            print("  center LAB =", img.get_pixel(IMG_W // 2, IMG_H // 2))
        else:
            uart.write(make_frame(cls, tx, ty, tw, th, tarea))
            time.sleep_ms(10)     # 50Hz 发送，别把 STM32 串口中断打爆


if __name__ == "__main__":
    main()
