"""
motor_test_esp32.py -- WiFi-triggered version of motor_test.py (ESP32-S3, MicroPython)
=====================================================================================
Same open-loop duty-sweep diagnostic as motor_test.py, but triggered and
observed over WiFi/UDP instead of mpremote over the USB cable.

Why the sweep still runs ON the ESP32:
  The timing-critical part (1 s PWM steps + encoder Phase-A edge counting)
  must not go over UDP -- WiFi jitter would corrupt the very measurement
  this test exists to make. WiFi carries only three tiny ASCII commands and
  the result lines:

  Commands (PC -> ESP32, UDP port 4211, ASCII):
    PING  -> replies "PONG ip=<ip>"          (link check / discovery)
    RUN   -> runs the full duty sweep once, streaming every result line
             back to the sender as it is produced; ends with "END"
    STOP  -> aborts a running sweep within ~20 ms and coasts everything

Deploy (one time, over USB or Thonny -- afterwards no cable is needed):
    mpremote connect COM13 fs cp motor_test_esp32.py :main.py
    mpremote connect COM13 reset
  This TEMPORARILY replaces robot_firmware.py as main.py (only one program
  can own the PWM pins at a time). When finished, copy robot_firmware.py
  back to :main.py and reset.

PC side:
    python3 pc/motor_test_pc.py --ip 192.168.1.50
    python3 pc/motor_test_pc.py --discover      # find the bot via broadcast

Safety: the sweep is self-terminating (bounded 1 s spins, then coast) even
if the PC disappears mid-run; STOP coasts within one 20 ms poll slice.

PIN MAP (authoritative -- identical to motor_test.py / robot_firmware.py):
  GPIO 4/5 -> DRV8833 AIN1/AIN2 (left),  GPIO 6/7 -> BIN1/BIN2 (right)
  GPIO 1   <- left encoder Phase A,      GPIO 9   <- right encoder Phase A
  DRV8833 STBY is hardwired to 3V3 -- this script must NOT drive it.
  Forbidden pins 0, 3, 43, 44, 46 are intentionally not referenced.
"""

import time

import network
import usocket as socket
from machine import PWM, Pin

# ============================ TUNABLES (edit me) ============================
WIFI_SSID = "ASUS_1E_NIRO_2.4G"          # same as robot_firmware.py
WIFI_PASSWORD = "niro@2026"
WIFI_CONNECT_TIMEOUT_MS = 15000

UDP_PORT = 4211                          # 4210 stays reserved for the PID firmware

PWM_FREQ_HZ = 20000
DUTIES = (0.40, 0.70, 1.00)              # duty sweep: speed MUST rise with duty
SPIN_MS = 1000
SETTLE_MS = 300
POLL_MS = 20                             # STOP responsiveness during a spin
# ========================== END OF TUNABLES =================================

# name, forward IN pin, reverse IN pin, encoder Phase A pin
MOTORS = (
    ("LEFT ", 4, 5, 1),
    ("RIGHT", 6, 7, 9),
)

_edges = [0]
_ip = "?.?.?.?"


def _count(pin):
    _edges[0] += 1


def coast_all():
    """Every DRV8833 IN pin at duty 0 (outputs float). Safe to call anytime."""
    for _, fwd, rev, _ in MOTORS:
        PWM(Pin(fwd, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
        PWM(Pin(rev, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)


def wifi_connect():
    """Station-mode connect with timeout + retry; prints the IP on success."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    attempt = 0
    while True:
        attempt += 1
        print('[WiFi] connecting to "%s" (attempt %d)' % (WIFI_SSID, attempt))
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        t0 = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > WIFI_CONNECT_TIMEOUT_MS:
                print("[WiFi] connect timed out, retrying...")
                break
            time.sleep_ms(250)
        else:
            break
    ip = wlan.ifconfig()[0]
    print("[WiFi] connected, IP =", ip)
    print("[UDP ] listening on port", UDP_PORT)
    return ip


def _sleep_poll(sock, ms):
    """Sleep in POLL_MS slices while watching for STOP/other datagrams.

    Returns True if a STOP arrived (caller must coast and abort).
    PINGs are answered so the link check keeps working mid-sweep; anything
    else gets BUSY.
    """
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < ms:
        try:
            data, addr = sock.recvfrom(64)
        except OSError:
            data = None
        if data is not None:
            cmd = data.strip().upper()
            if cmd == b"STOP":
                return True
            elif cmd == b"PING":
                try:
                    sock.sendto(("PONG ip=" + _ip).encode(), addr)
                except OSError:
                    pass
            else:
                try:
                    sock.sendto(b"BUSY sweep in progress (send STOP to abort)",
                                addr)
                except OSError:
                    pass
        time.sleep_ms(POLL_MS)
    return False


def spin(sock, reply, label, pin_on, pin_off, pin_enc_a, duty_frac):
    """One 1 s open-loop spin + edge count. Returns True if STOP aborted it."""
    pwm_on = PWM(Pin(pin_on, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
    pwm_off = PWM(Pin(pin_off, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
    enc = Pin(pin_enc_a, Pin.IN)
    enc.irq(handler=_count, trigger=Pin.IRQ_RISING)
    _edges[0] = 0
    pwm_on.duty_u16(int(duty_frac * 65535))
    stopped = _sleep_poll(sock, SPIN_MS)
    pwm_on.duty_u16(0)         # coast
    pwm_off.duty_u16(0)
    eps = _edges[0] * 1000.0 / SPIN_MS
    reply("  %s duty=%.2f -> %5d edges (%5.0f edges/s ~ %3.0f RPM)"
          % (label, duty_frac, _edges[0], eps, eps / 30.0))
    enc.irq(handler=None)
    return stopped


def run_sweep(sock, remote):
    """The exact motor_test.py sequence, streaming every line to `remote`."""
    def reply(line):
        print(line)                       # still visible on serial if attached
        try:
            sock.sendto(line.encode(), remote)
        except OSError:
            pass                          # WiFi hiccup: drop a line, keep going

    reply("BEGIN motor_test_esp32: open-loop duty sweep, 1 s per step")
    reply("wheels OFF the ground; watch whether speed rises with duty")
    aborted = False
    for name, fwd, rev, enc_a in MOTORS:
        for d in DUTIES:
            reply("%s forward duty %.2f:" % (name, d))
            if spin(sock, reply, "fwd", fwd, rev, enc_a, d):
                aborted = True
                break
            if _sleep_poll(sock, SETTLE_MS):
                aborted = True
                break
        if aborted:
            break
        reply("%s reverse duty 1.00:" % name)
        if spin(sock, reply, "rev", rev, fwd, enc_a, 1.00):
            aborted = True
            break
        if _sleep_poll(sock, SETTLE_MS):
            aborted = True
            break
    coast_all()
    if aborted:
        reply("ABORTED by STOP -- all outputs coasted")
    else:
        reply("done -- all outputs coasted")
    # END marker, sent 3x so one lost datagram can't hang the PC side.
    for _ in range(3):
        try:
            sock.sendto(b"END", remote)
        except OSError:
            pass
        time.sleep_ms(50)


def main():
    global _ip
    print("=== motor_test_esp32: WiFi-triggered open-loop motor test ===")
    coast_all()                           # start coasted, whatever reset left
    _ip = wifi_connect()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except (OSError, AttributeError):
        pass
    s.bind(("0.0.0.0", UDP_PORT))
    s.setblocking(False)
    print("[UDP ] listening on 0.0.0.0:%d -- commands: PING / RUN / STOP"
          % UDP_PORT)

    while True:
        try:
            data, addr = s.recvfrom(64)
        except OSError:
            time.sleep_ms(POLL_MS)        # nothing arrived this slice
            continue
        cmd = data.strip().upper()
        if cmd == b"PING":
            try:
                s.sendto(("PONG ip=" + _ip).encode(), addr)
            except OSError:
                pass
        elif cmd == b"RUN":
            run_sweep(s, addr)            # blocking ~11 s; STOP still polled
        elif cmd == b"STOP":
            coast_all()
            try:
                s.sendto(b"OK coasted", addr)
            except OSError:
                pass
        else:
            try:
                s.sendto(b"ERR unknown command (use PING / RUN / STOP)", addr)
            except OSError:
                pass


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[STOP] KeyboardInterrupt -- coasting motors")
    except Exception as exc:
        print("[FAULT]", exc, "-- coasting motors")
    finally:
        coast_all()
