"""RC-override PWM mapping and MAVROS arm/mode helpers, shared between
web_viewer/control_server.py (manual flight) and controller_node.py
(autonomous mission control) -- both drive the vehicle through the same
/mavros/rc/override channel, since ArduPilot exposes no MAVLink XY
position/velocity injection path for this airframe in any flight mode
(see docs/superpowers/specs/2026-09-02-mission-stack-mavros-port-design.md).

Extracted from control_server.py, verified live there during Phase 1 --
this is not new/unverified logic, just given a second caller.
"""
# RC override PWM: 1500 centre, +/-500 at full stick deflection, the
# standard 1000-2000 us band ArduPilot's RCn_MIN/MAX default to.
RC_CENTER = 1500
RC_SPAN = 500
# Throttle's own PWM range: caps at 1900, not 2000 (RC_CENTER + RC_SPAN),
# matching a real Mode 2 transmitter's ratcheted throttle gimbal.
THROTTLE_MID = 1450
THROTTLE_SPAN = 450
# ArduPlane custom mode numbers (ArduPlane/mode.h).
MODE_QHOVER = 18
MODE_QLAND = 20
MAV_CMD_DO_SET_MODE = 176
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1.0


def resolve_stick(stick, stick_time, now, timeout):
    """Per-axis dead-man's-switch: an axis whose last update is older
    than `timeout` reads as 0.0 regardless of its last commanded value."""
    return {
        name: (0.0 if now - stick_time[name] > timeout else stick[name])
        for name in stick
    }


def pwm(value, scale, invert=False):
    """A [-1, 1] command axis -> RC_CENTER +/- RC_SPAN PWM."""
    v = max(-1.0, min(1.0, value * scale))
    return int(round(RC_CENTER + (-v if invert else v) * RC_SPAN))


def pwm_throttle(value, scale):
    """A [-1, 1] throttle command -> THROTTLE_MID +/- THROTTLE_SPAN PWM."""
    v = max(-1.0, min(1.0, value * scale))
    return int(round(THROTTLE_MID + v * THROTTLE_SPAN))


def arm(client, value):
    """Request arm (value=True) or disarm (value=False) over an already-
    constructed mavros_msgs.srv.CommandBool rclpy client
    (create_client(CommandBool, '/mavros/cmd/arming'))."""
    from mavros_msgs.srv import CommandBool
    req = CommandBool.Request()
    req.value = value
    client.call_async(req)


def set_mode(client, mode):
    """Request a custom-mode switch (e.g. MODE_QHOVER) over an already-
    constructed mavros_msgs.srv.CommandLong rclpy client
    (create_client(CommandLong, '/mavros/cmd/command')). Goes through
    COMMAND_LONG/DO_SET_MODE rather than /mavros/set_mode -- this
    airframe heartbeats as MAV_TYPE_VTOL_TILTROTOR (21), which is absent
    from MAVROS's ArduPilot mode tables, so /mavros/set_mode always
    returns mode_sent=False for it (verified live in Phase 1)."""
    from mavros_msgs.srv import CommandLong
    req = CommandLong.Request()
    req.command = MAV_CMD_DO_SET_MODE
    req.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
    req.param2 = float(mode)
    client.call_async(req)
