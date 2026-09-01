#!/usr/bin/env python3
"""rc_bridge.py

Reads a standard Linux joystick device (/dev/input/js0, the kernel's own
struct js_event protocol) and forwards all four Mode 2 stick axes --
throttle/roll/pitch/yaw -- to control_server.py's /api/stick.

This is the "ART TECH GAME" RC dongle (Novatek Microelectronics, a
different/simpler transmitter than an earlier one tried on this project --
see History.md): it declares a real HID joystick usage page, so the
kernel's generic hid/usbhid/joydev drivers create a normal calibrated
/dev/input/js0 -- no protocol reverse-engineering needed, unlike the
first dongle tried (whose raw HID report had no public docs and turned
out not to carry a usable signed pitch value at all).

Axis mapping was confirmed live, moving one stick at a time and watching
which axis number changed:
    axis 0 = roll, axis 1 = pitch, axis 2 = throttle, axis 5 = yaw
    axis 3 = a 2-position switch (32767/-32767) -- used here as arm/disarm.
The kernel joystick driver already calibrates raw ADC readings to the
standard -32767..32767 range, so no per-unit min/center/max calibration
is needed here (unlike the earlier hidraw-based approach).

Setup, once per USB attach (see README -- modprobe only takes one module
name at a time, so these can't be combined into one call):
    sudo modprobe hid-generic
    sudo modprobe usbhid
    sudo modprobe joydev
    sudo chmod 666 /dev/input/js0

Run with control_server.py already up (plain localhost HTTP, no ROS2
import needed here):
    python3 rc_bridge.py [control_server_port]
"""
import struct
import sys
import threading
import time
import urllib.request

DEV = '/dev/input/js0'
CONTROL_SERVER_PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
SEND_HZ = 20
JS_EVENT_FORMAT = 'IhBB'  # time(u32), value(s16), type(u8), number(u8)
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FORMAT)
JS_EVENT_AXIS = 0x02
JS_EVENT_BUTTON = 0x01

AXIS_ROLL, AXIS_PITCH, AXIS_THROTTLE, AXIS_YAW = 0, 1, 2, 5
SWITCH_AXIS = 3


def normalize(raw):
    """Kernel joydev already calibrates to -32767..32767 -- just scale."""
    return max(-1.0, min(1.0, raw / 32767.0))


RECONNECT_POLL_S = 1.0


def _read_events(state, lock):
    """Runs in its own thread, forever: waits for /dev/input/js0 to exist
    (not plugged in yet, or unplugged mid-session), reads events from it
    once it does, and goes back to waiting the moment a read fails (USB
    unplugged) -- so plugging the transmitter in at any point during a
    session, or unplugging and replugging it, just works without
    restarting this script. state['connected'] reflects the CURRENT
    state, checked by the send loop in main() every tick, not just at
    startup.

    read() on a joystick device blocks until the NEXT event, so a
    resting/untouched stick would otherwise starve the send loop below
    entirely -- reading and sending must be independent threads, not
    "send right after each read returns"."""
    switch_high = None  # tri-state: None until the first switch event arrives
    while True:
        try:
            f = open(DEV, 'rb', buffering=0)
        except OSError:
            with lock:
                state['connected'] = False
            time.sleep(RECONNECT_POLL_S)
            continue

        with lock:
            state['connected'] = True
        print(f'{DEV} connected')
        try:
            while True:
                data = f.read(JS_EVENT_SIZE)
                _, value, ev_type, number = struct.unpack(JS_EVENT_FORMAT, data)
                kind = ev_type & 0x7f  # strip the JS_EVENT_INIT flag bit
                if kind == JS_EVENT_AXIS and number in state['axes']:
                    with lock:
                        state['axes'][number] = value
                elif kind == JS_EVENT_AXIS and number == SWITCH_AXIS:
                    high = value > 0
                    if high != switch_high:
                        switch_high = high
                        cmd = 'arm' if high else 'disarm'
                        try:
                            urllib.request.urlopen(
                                f'http://localhost:{CONTROL_SERVER_PORT}/api/manual?cmd={cmd}',
                                timeout=0.2)
                        except Exception as e:
                            print(f'{cmd} post failed: {e}')
        except OSError:
            pass  # device unplugged mid-read -- fall through to reconnect
        finally:
            f.close()
            with lock:
                state['connected'] = False
            print(f'{DEV} disconnected -- waiting for reconnect')


def main():
    state = {
        'connected': False,
        'axes': {AXIS_ROLL: 0, AXIS_PITCH: 0, AXIS_THROTTLE: 0, AXIS_YAW: 0},
    }
    lock = threading.Lock()
    threading.Thread(target=_read_events, args=(state, lock), daemon=True).start()

    while True:
        time.sleep(1.0 / SEND_HZ)
        with lock:
            connected = state['connected']
            axes = dict(state['axes'])
        if not connected:
            # Post nothing -- control_server.py's own RC_PRESENT_TIMEOUT_S
            # naturally expires and the browser falls back to its virtual
            # sticks, rather than this script claiming hw=1 presence for
            # a device that isn't actually there.
            continue
        roll = normalize(axes[AXIS_ROLL])
        # Real bug found live: pitch-forward and throttle-up both
        # commanded the opposite motion in a real flight test. The axis
        # IDENTITY (which raw axis number is which stick) was confirmed
        # correct live -- moving just one stick at a time only ever
        # changed its own axis -- so this is a pure sign flip, not a
        # mis-mapped axis.
        pitch = -normalize(axes[AXIS_PITCH])
        throttle = -normalize(axes[AXIS_THROTTLE])
        yaw = normalize(axes[AXIS_YAW])
        url = (f'http://localhost:{CONTROL_SERVER_PORT}/api/stick'
               f'?throttle={throttle:.3f}&roll={roll:.3f}'
               f'&pitch={pitch:.3f}&yaw={yaw:.3f}&hw=1')
        try:
            urllib.request.urlopen(url, timeout=0.2)
        except Exception as e:
            print(f'stick post failed: {e}')


if __name__ == '__main__':
    main()
