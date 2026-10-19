#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
serial_watch.py —— 盯 CRTC 固件的调试串口（115200 8N1）

用法：
    python tools/serial_watch.py            # 自动找 COM 口
    python tools/serial_watch.py COM5       # 指定 COM 口
    Ctrl+C 退出

会自动标出：
    [PS2-RAW] 行：第 3 字节是不是 5A（5A = 接收器已回话）
    [MODE] 行：手柄切模式
    V= 行：电池电压和按键
"""
import sys, time, re

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺 pyserial：python -m pip install pyserial -i https://mirrors.aliyun.com/pypi/simple/")
    sys.exit(1)

BAUD = 115200

def pick_port():
    if len(sys.argv) > 1:
        return sys.argv[1]
    ports = list(list_ports.comports())
    if not ports:
        print("没找到任何 COM 口。检查：")
        print("  1. USB-TTL 模块插到电脑了吗")
        print("  2. 设备管理器里有没有出现 COM 口（CH340/CP2102）")
        print("  3. 有黄色感叹号 = 缺驱动，装 CH340 驱动")
        return None
    if len(ports) == 1:
        return ports[0].device
    print("找到多个串口，选一个重新运行：")
    for p in ports:
        print("   ", p.device, "|", p.description)
    return None

def main():
    port = pick_port()
    if not port:
        return 1

    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except Exception as e:
        print("打不开 %s：%s" % (port, e))
        return 1

    print("已连接 %s @ %d 8N1   （Ctrl+C 退出）" % (port, BAUD))
    print("=" * 60)

    buf = b""
    t0 = time.time()
    try:
        while True:
            chunk = ser.read(256)
            if not chunk:
                continue
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    s = line.decode("utf-8", errors="replace").rstrip("\r")
                except Exception:
                    s = repr(line)
                if not s:
                    continue
                ts = time.time() - t0
                tag = ""
                if "PS2-RAW" in s:
                    m = re.search(r"PS2-RAW\] ((?:[0-9A-F]{2} ?){9})", s)
                    if m:
                        bts = m.group(1).split()
                        if len(bts) >= 3 and bts[2] == "5A":
                            tag = "  <<< 5A 出现！接收器回话了"
                        elif all(b == "FF" for b in bts):
                            tag = "  <<< 全 FF，接收器没回话"
                        else:
                            tag = "  <<< 有变化！记录下来"
                print("[%7.2f] %s%s" % (ts, s, tag))
    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        ser.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
