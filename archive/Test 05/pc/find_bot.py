#!/usr/bin/env python3
"""Find the ESP32-S3 bot's IP address over WiFi -- no USB cable needed.

How it works: the robot firmware (robot_firmware.py, UDP port 4210) streams
telemetry back to whichever address last sent it a command. Broadcasting a
harmless COAST command (0.0, 0.0 RPM -- the robot does not move) to the
whole subnet makes any bot on the network start replying with its 24-byte
telemetry packets, which reveals its IP.

Usage:
    python3 find_bot.py                      # broadcast to 255.255.255.255
    python3 find_bot.py 192.168.50.255       # targeted subnet broadcast
                                             # (try this if the general
                                             # broadcast finds nothing)

Stdlib only. Safe: the command sent is (0.0, 0.0) = coast. Stop with Ctrl-C.
"""

import socket
import struct
import sys
import time

UDP_PORT = 4210                 # must match robot_firmware.py / udp_protocol.py
TELEMETRY_SIZE = 24             # '<ffiiff'
DISCOVER_SECONDS = 5.0


def main():
    bcast = sys.argv[1] if len(sys.argv) > 1 else "255.255.255.255"
    coast = struct.pack("<ff", 0.0, 0.0)     # coast command, 8 bytes

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.25)

    print("broadcasting coast commands to %s:%d for %.0f s..."
          % (bcast, UDP_PORT, DISCOVER_SECONDS))
    found = {}
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < DISCOVER_SECONDS:
            sock.sendto(coast, (bcast, UDP_PORT))
            try:
                while True:
                    data, addr = sock.recvfrom(64)
                    if len(data) == TELEMETRY_SIZE:
                        l_rpm, r_rpm = struct.unpack("<ff", data[:8])
                        found[addr[0]] = (l_rpm, r_rpm)
            except socket.timeout:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if found:
        for ip, (l_rpm, r_rpm) in sorted(found.items()):
            print("  BOT FOUND at %-15s (alive: L=%.1f RPM R=%.1f RPM)"
                  % (ip, l_rpm, r_rpm))
        print("\nuse it with:  python3 test_suite.py --test 0 --ip %s"
              % sorted(found)[0])
        return 0
    print("  no bot replied. Checklist:")
    print("  - bot powered and running robot_firmware.py (PID firmware)?")
    print("  - PC and bot on the SAME 2.4 GHz network?")
    print("  - try the targeted subnet broadcast, e.g.: "
          "python3 find_bot.py 192.168.50.255")
    print("  - last resort: read the boot-time IP over serial")
    print("    (mpremote connect COM13 repl) or the router's DHCP client list")
    return 1


if __name__ == "__main__":
    sys.exit(main())
