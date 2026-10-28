import sys

import numpy as np

from common import attitude, load

# Usage: python3 restore.py [-v] log.ulg ...
# Attitude, rate, height and speed excursions in the 3 s after each motor stop and each restore
verbose = '-v' in sys.argv
for fn in [a for a in sys.argv[1:] if a != '-v']:
    u, d = load(fn)
    t0 = u.start_timestamp
    a, av, lp, am = d['vehicle_attitude'], d['vehicle_angular_velocity'], d['vehicle_local_position'], d['actuator_motors']
    _, (roll, pitch, yaw) = attitude(d)
    fd = d['failure_detector_status']
    v = fd['motor_failure_mask'].astype(int)
    ev = [(fd['timestamp'][k + 1], 'fails' if v[k + 1] else 'back ') for k in np.nonzero(np.diff(v))[0]]
    print('\n=== %s' % fn.split('/')[-1])
    for m in u.logged_messages:
        if any(s in m.message for s in ('otor', 'estor')):
            print('   %7.2f  %s' % ((m.timestamp - t0) / 1e6, m.message.strip()))
    for te, what in ev:
        w = (a['timestamp'] >= te) & (a['timestamp'] < te + 3e6)
        b = (a['timestamp'] >= te - 1e6) & (a['timestamp'] < te)
        wr = (av['timestamp'] >= te) & (av['timestamp'] < te + 3e6)
        wl = (lp['timestamp'] >= te) & (lp['timestamp'] < te + 3e6)
        z0 = np.interp(te, lp['timestamp'], lp['z'])
        y0 = np.interp(te, a['timestamp'], yaw)
        print('   motor %s at %6.2f s: roll %5.1f  pitch %5.1f  yaw %5.1f deg | rates p %4.0f q %4.0f r %4.0f deg/s'
              ' | height %+5.2f/%+5.2f m | hor speed %.2f m/s' % (
                  what, (te - t0) / 1e6,
                  np.max(np.abs(roll[w] - roll[b].mean())), np.max(np.abs(pitch[w] - pitch[b].mean())),
                  np.max(np.abs(yaw[w] - y0)),
                  *(np.degrees(np.max(np.abs(av['xyz[%d]' % i][wr]))) for i in range(3)),
                  -(np.max(lp['z'][wl]) - z0), -(np.min(lp['z'][wl]) - z0),
                  np.max(np.hypot(lp['vx'][wl], lp['vy'][wl]))))
        if verbose and what == 'back ':
            print('      t     roll  pitch   yaw   | motors 1..6')
            for t in np.arange(te - 0.2e6, te + 2.2e6, 0.1e6):
                k = np.searchsorted(a['timestamp'], t)
                j = min(np.searchsorted(am['timestamp'], t), len(am['timestamp']) - 1)
                mo = ' '.join('%5.2f' % am['control[%d]' % i][j] for i in range(6))
                print('   %6.2f  %5.1f  %5.1f  %5.1f  | %s' % ((t - te) / 1e6, roll[k], pitch[k], yaw[k] - y0, mo))
