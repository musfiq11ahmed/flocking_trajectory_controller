#!/usr/bin/env python3
"""PC-side WiFi trigger for the ESP32-S3 open-loop motor test.

Talks to firmware/motor_test_esp32.py (deployed on the ESP32 as main.py) over
UDP port 4211 -- no USB cable needed after the one-time firmware deploy.

Usage:
    python3 motor_test_pc.py --ip 192.168.1.50     # run the duty sweep
    python3 motor_test_pc.py --discover            # find the bot (broadcast PING)
    python3 motor_test_pc.py --ip 192.168.1.50 --ping   # link check only

Standard library only (socket, argparse, sys, time), like the rest of pc/.

Protocol (ASCII datagrams, port 4211):
    PC -> ESP32 : PING / RUN / STOP
    ESP32 -> PC : PONG ip=<ip> / sweep result lines ... END / OK / ERR / BUSY

How to read the sweep output (same logic as motor_test.py):
  - RPM rises clearly with duty (~40 RPM @ 0.40 -> 100+ RPM @ 1.00)
      -> hardware GOOD; any motion problem is upstream (WiFi/PID path).
  - No motion at ANY duty -> DRV8833 power path: VM missing, STBY not at
    3V3, no common GND, miswired AOUT/BOUT, dead driver/motor.
  - SLOW/INTERMITTENT motion -> marginal VM supply (DRV8833 undervoltage
    lockout ~2.5-2.7 V): power VM direct from battery/5-6 V with short
    thick wires, 100-470 uF cap at the driver, motor power off breadboard.
  - Wheel turns but ~0 edges -> encoder wiring/power issue (this alone
    would NOT stop the wheels; fix motion first, encoders second).
"""

import argparse
import socket
import sys
import time

UDP_PORT = 4211
PING_RETRIES = 3
PING_TIMEOUT_S = 1.0
# Sweep duration on the ESP32: 2 motors x (3 fwd + 1 rev) x (1.0 s spin +
# 0.3 s settle) = ~10.4 s. The receive timeout adds generous margin.
DEFAULT_RECV_TIMEOUT_S = 45.0

READING_GUIDE = """\
--- how to read the result -----------------------------------------------
RPM rises clearly with duty      -> hardware GOOD (bug is upstream: WiFi/PID)
no motion at ANY duty            -> DRV8833 power path (VM, STBY, GND, wiring)
slow/intermittent motion         -> marginal VM supply (undervoltage lockout)
wheel turns but ~0 edges         -> encoder wiring/power (fix motion first)
---------------------------------------------------------------------------"""


def send_cmd(sock, cmd, ip, port):
    sock.sendto(cmd.encode(), (ip, port))


def ping(sock, ip, port, verbose=True):
    """Send PING, wait for PONG. Returns the reported IP string or None."""
    for attempt in range(1, PING_RETRIES + 1):
        send_cmd(sock, "PING", ip, port)
        sock.settimeout(PING_TIMEOUT_S)
        try:
            data, _ = sock.recvfrom(256)
        except socket.timeout:
            if verbose:
                print("  PING attempt %d/%d: no reply" % (attempt, PING_RETRIES))
            continue
        text = data.decode(errors="replace").strip()
        if text.startswith("PONG"):
            return text
        if verbose:
            print("  unexpected reply: %r" % text)
    return None


def discover(port, listen_s=3.0, bcast="255.255.255.255"):
    """Broadcast PING and collect PONGs. Returns a list of (ip, pong_text)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.2)
    found = {}
    deadline = time.monotonic() + listen_s
    try:
        sock.sendto(b"PING", (bcast, port))
        while time.monotonic() < deadline:
            try:
                data, addr = sock.recvfrom(256)
            except socket.timeout:
                continue
            text = data.decode(errors="replace").strip()
            if text.startswith("PONG"):
                found[addr[0]] = text
    finally:
        sock.close()
    return sorted(found.items())


def run_sweep(sock, ip, port, recv_timeout_s):
    """Send RUN and print result lines until END or timeout."""
    send_cmd(sock, "RUN", ip, port)
    print("sweep running on the bot (~11 s)... (Ctrl-C sends STOP)")
    sock.settimeout(2.0)
    deadline = time.monotonic() + recv_timeout_s
    last_rx = time.monotonic()
    try:
        while time.monotonic() < deadline:
            try:
                data, _ = sock.recvfrom(512)
            except socket.timeout:
                if time.monotonic() - last_rx > recv_timeout_s:
                    break
                continue
            last_rx = time.monotonic()
            text = data.decode(errors="replace").strip()
            if text == "END":
                print("--- sweep finished, motors coasted ---")
                return True
            print(text)
    except KeyboardInterrupt:
        print("\nCtrl-C -> sending STOP")
        for _ in range(3):
            send_cmd(sock, "STOP", ip, port)
            time.sleep(0.1)
        return False
    print("WARNING: timed out waiting for END "
          "(link loss? bot rebooted? try --ping)")
    return False


def main():
    parser = argparse.ArgumentParser(
        description="WiFi trigger for the ESP32-S3 open-loop motor test "
                    "(firmware/motor_test_esp32.py, UDP port %d)" % UDP_PORT)
    parser.add_argument("--ip", help="robot IP address (use --discover to find it)")
    parser.add_argument("--port", type=int, default=UDP_PORT,
                        help="UDP port on the robot (default %(default)s)")
    parser.add_argument("--discover", action="store_true",
                        help="broadcast PING to find the bot, then exit")
    parser.add_argument("--bcast", default="255.255.255.255",
                        help="broadcast address for --discover "
                             "(default %(default)s; try e.g. 192.168.1.255)")
    parser.add_argument("--ping", action="store_true",
                        help="link check only, do not run the sweep")
    parser.add_argument("--yes", action="store_true",
                        help="skip the 'wheels off the ground' confirmation")
    parser.add_argument("--timeout", type=float, default=DEFAULT_RECV_TIMEOUT_S,
                        help="overall receive timeout in s (default %(default)s)")
    args = parser.parse_args()

    if args.discover:
        print("broadcasting PING on %s:%d for 3 s..." % (args.bcast, args.port))
        found = discover(args.port, bcast=args.bcast)
        if found:
            for ip, pong in found:
                print("  found bot at %-15s (%s)" % (ip, pong))
        else:
            print("  no reply -- check the bot is powered, on the same 2.4 GHz "
                  "network, and running motor_test_esp32.py; or try "
                  "--bcast <your subnet>.255")
        return 0 if found else 1

    if not args.ip:
        parser.error("--ip is required (or use --discover to find the bot)")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        pong = ping(sock, args.ip, args.port)
        if pong is None:
            print("FAIL: no reply from %s:%d -- bot unreachable "
                  "(wrong IP? bot not running motor_test_esp32.py? "
                  "PC and bot on the same 2.4 GHz network?)"
                  % (args.ip, args.port))
            return 1
        print("link OK: %s" % pong)
        if args.ping:
            return 0

        if not args.yes:
            answer = input("Lift the wheels OFF the ground, then press Enter "
                           "(or Ctrl-C to abort): ")
            del answer

        ok = run_sweep(sock, args.ip, args.port, args.timeout)
        print(READING_GUIDE)
        return 0 if ok else 1
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
