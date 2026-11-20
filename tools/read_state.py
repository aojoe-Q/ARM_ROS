#!/usr/bin/env python3
"""Read-only USB protocol v2 STATE monitor. Never writes to the port.

python3 tools/read_state.py --port /dev/ttyACM0 --duration 10 [--csv out.csv]
Frame layout follows src/dm_mc02_hardware/include/dm_mc02_hardware/arm_protocol.hpp.
"""
import argparse
import csv
import struct
import sys
import time

import serial

SOF, VERSION, TYPE_STATE = 0xA5, 0x02, 0x01
HEADER = struct.Struct("<BBBBHI")
STATE = struct.Struct("<IBBHBBH7H7i7i7i7i")
STATE_FRAME = HEADER.size + STATE.size + 2
NAMES = ["J1", "J2", "J3", "J4", "J5", "J6", "Grip"]


def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
    return crc


def frames(buf, stats):
    """Yield decoded STATE payloads, consuming bytes from buf."""
    while True:
        start = buf.find(bytes([SOF, VERSION]))
        if start < 0:
            del buf[:-1]
            return
        del buf[:start]
        if len(buf) < HEADER.size:
            return
        _, _, ftype, _, length, _ = HEADER.unpack_from(buf)
        total = HEADER.size + length + 2
        if length > 512:
            del buf[:1]
            continue
        if len(buf) < total:
            return
        frame = bytes(buf[:total])
        if struct.unpack_from("<H", frame, total - 2)[0] != crc16(frame[:-2]):
            stats["crc_err"] += 1
            del buf[:1]
            continue
        del buf[:total]
        if ftype == TYPE_STATE and total == STATE_FRAME:
            yield STATE.unpack_from(frame, HEADER.size)
        else:
            stats["other"] += 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--duration", type=float, default=10.0)
    p.add_argument("--csv")
    args = p.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    writer = None
    if args.csv:
        f = open(args.csv, "w", newline="")
        writer = csv.writer(f)
        writer.writerow(["host_t", "mcu_ms", "mode", "sys", "fault", "flags", "feedback"]
                        + [f"{n}_age_ms" for n in NAMES] + [f"{n}_rad" for n in NAMES])

    buf, stats = bytearray(), {"state": 0, "crc_err": 0, "other": 0}
    t0 = last_print = time.monotonic()
    try:
        while time.monotonic() - t0 < args.duration:
            buf += ser.read(4096)
            for s in frames(buf, stats):
                stats["state"] += 1
                mcu_ms, mode, sys_state, fault, flags, fb = s[0], s[1], s[2], s[3], s[4], s[5]
                ages = s[7:14]
                pos = [v / 1e6 for v in s[14:21]]
                if writer:
                    writer.writerow([f"{time.monotonic() - t0:.3f}", mcu_ms, mode, sys_state,
                                     f"0x{fault:04x}", f"0x{flags:02x}", f"0x{fb:02x}", *ages,
                                     *[f"{v:.6f}" for v in pos]])
                now = time.monotonic()
                if now - last_print >= 0.5:
                    last_print = now
                    print(f"mode={mode} sys={sys_state} fault=0x{fault:04x} flags=0x{flags:02x} "
                          f"feedback=0x{fb:02x} ready={'yes' if flags & 1 else 'no'}")
                    print("  pos rad: " + "  ".join(f"{n}={v:+.4f}" for n, v in zip(NAMES, pos)))
                    print("  age ms : " + "  ".join(f"{n}={a}" for n, a in zip(NAMES, ages)))
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if writer:
            f.close()
    elapsed = time.monotonic() - t0
    print(f"\nSTATE frames={stats['state']} ({stats['state'] / elapsed:.1f} Hz) "
          f"crc_err={stats['crc_err']} other_frames={stats['other']}")
    return 0 if stats["state"] else 1


if __name__ == "__main__":
    sys.exit(main())
