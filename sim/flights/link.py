import json
import math
import os
import time

from pymavlink import mavutil


class Vehicle:
    """MAVLink link to PX4 SIH: RC sticks through MANUAL_CONTROL, failures through MAV_CMD_INJECT_FAILURE."""

    def __init__(self, period=0.04):
        self.m = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.m.wait_heartbeat(timeout=30)
        self.sys, self.comp = self.m.target_system, self.m.target_component
        self.period = period
        self.boot_ms = 0
        self.height = 0.0
        self.roll = self.pitch = 0.0
        self.mode = ''
        self.armed = False
        self.marks = []

    def pump(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            t = msg.get_type()
            if t == 'ATTITUDE':
                self.boot_ms = msg.time_boot_ms
                self.roll, self.pitch = math.degrees(msg.roll), math.degrees(msg.pitch)
            elif t == 'LOCAL_POSITION_NED':
                self.height = -msg.z
            elif t == 'HEARTBEAT' and msg.get_srcComponent() == 1:
                self.mode = mavutil.mode_string_v10(msg)
                self.armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif t == 'STATUSTEXT':
                print('[%.1f] %s' % (self.boot_ms / 1000, msg.text), flush=True)

    def sticks(self, x=0, y=0, z=500, r=0, t=1.0, each=None):
        """Hold the sticks for t seconds (x, y, r in -1000..1000, throttle z in 0..1000).
        Returns the maximum tilt [deg] and height [m] seen."""
        end = time.time() + t
        tilt = height = 0.0
        while time.time() < end:
            self.m.mav.manual_control_send(self.sys, x, y, z, r, 0)
            self.m.mav.heartbeat_send(6, 8, 0, 0, 0)
            if each:
                each()
            self.pump()
            tilt = max(tilt, math.hypot(self.roll, self.pitch))
            height = max(height, self.height)
            time.sleep(self.period)
        return tilt, height

    def phase(self, name, **kw):
        tilt, height = self.sticks(**kw)
        print('%-38s max tilt %6.1f deg | max height %5.2f m | now %5.2f m | %s armed=%s'
              % (name, tilt, height, self.height, self.mode, self.armed), flush=True)

    def set_mode(self, name):
        self.m.set_mode(name)

    def command(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        self.m.mav.command_long_send(self.sys, self.comp, cmd, 0, *p)

    def arm(self):
        self.command(400, 1)

    def motor_off(self, motor, off=True):
        self.command(420, 101, 1 if off else 0, motor)

    def param(self, name, value):
        self.m.mav.param_set_send(self.sys, self.comp, name.encode(), float(value),
                                  mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def mark(self, name):
        self.pump()
        self.marks.append((name, self.boot_ms))

    def save_marks(self):
        with open(os.environ.get('CTRLHEX_MARKS', 'marks.json'), 'w') as f:
            json.dump(self.marks, f)
