import sys

import numpy as np

from common import load

# Usage: python3 landing.py log.ulg ...   Max tilt from the motor stop to disarm, and how the flight ended
for fn in sys.argv[1:]:
    u, d = load(fn)
    a = d['vehicle_attitude']
    q0, q1, q2, q3 = (a['q[%d]' % i] for i in range(4))
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q1 ** 2 + q2 ** 2), -1, 1)))
    fd = d['failure_detector_status']
    t_fail = fd['timestamp'][np.argmax(fd['motor_failure_mask'] != 0)]
    k = a['timestamp'] >= t_fail
    msgs = [m.message.strip() for m in u.logged_messages
            if any(s in m.message for s in ('Landing detected', 'Disarmed', 'Attitude failure', 'Ground contact'))]
    print('%-28s max tilt after the stop %6.1f deg | %s' % (fn.split('/')[-1], tilt[k].max(), '; '.join(msgs)))
