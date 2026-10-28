import json
import sys

import numpy as np

from common import attitude, load

# Usage: python3 maneuvers.py <log.ulg> <marks.json>
# Per phase: roll/pitch set-point and measured (last 0.8 s), max tilt error, height and yaw change, motor range
u, d = load(sys.argv[1])
marks = json.load(open(sys.argv[2]))
names = [n for n, _ in marks]
times = [t / 1000 for _, t in marks]

ta, (r, p, y) = attitude(d)
ts, (rs, ps, _) = attitude(d, 'vehicle_attitude_setpoint', 'q_d')
lp = d['vehicle_local_position']
tl = lp['timestamp'] / 1e6
am = d['actuator_motors']
tm = am['timestamp'] / 1e6
M = np.stack([am['control[%d]' % i] for i in range(6)], 1)

print('%-22s %7s %7s %7s %7s %6s %6s %6s %7s  %s' % ('phase', 'roll_sp', 'roll', 'pit_sp', 'pitch', 'errmax',
                                                      'dz', 'vxy', 'yaw_drf', 'motors min/max'))
for i in range(len(marks) - 1):
    n, t0, t1 = names[i], times[i], times[i + 1]
    if n in ('climb', 'land', 'restore'):
        continue
    k = (ta >= t0) & (ta < t1)
    ks = (ts >= t0) & (ts < t1)
    kl = (tl >= t0) & (tl < t1)
    km = (tm >= t0) & (tm < t1)
    tail = (ta >= t1 - 0.8) & (ta < t1)
    tails = (ts >= t1 - 0.8) & (ts < t1)
    err = np.max(np.hypot(rs[ks] - np.interp(ts[ks], ta, r), ps[ks] - np.interp(ts[ks], ta, p)))
    z = lp['z'][kl]
    vxy = np.hypot(lp['vx'][kl], lp['vy'][kl])
    mm = M[km]
    print('%-22s %7.1f %7.1f %7.1f %7.1f %6.1f %6.2f %6.2f %7.1f  %s' % (
        n, rs[tails].mean(), r[tail].mean(), ps[tails].mean(), p[tail].mean(), err, z.max() - z.min(), vxy.max(),
        y[k][-1] - y[k][0], ' '.join('%.2f/%.2f' % (mm[:, j].min(), mm[:, j].max()) for j in range(6))))

cas = d.get('control_allocator_status')
if cas is not None:
    tc = cas['timestamp'] / 1e6
    for lab, t0, t1 in [('nominal', times[names.index('nominal:roll_right')], times[names.index('inject')]),
                        ('failed', times[names.index('inject')] + 1, times[names.index('restore')])]:
        k = (tc >= t0) & (tc < t1)
        un = np.stack([cas['unallocated_torque[%d]' % j][k] for j in range(3)], 1)
        print('unallocated torque %-7s max |roll| %.3f |pitch| %.3f |yaw| %.3f' % (lab, *np.abs(un).max(0)))

fd = d['failure_detector_status']
mask = int(fd['motor_failure_mask'].max())
print('motor failure mask 0x%x' % mask)
for m in u.logged_messages:
    if 'otor' in m.message:
        print('  %s' % m.message.strip())
