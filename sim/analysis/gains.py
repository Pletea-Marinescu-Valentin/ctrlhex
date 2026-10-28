import json
import os
import sys

import numpy as np

from common import attitude, load

# Usage: python3 gains.py <dir> tag1 tag2 ...   (reads <dir>/<tag>.ulg and <dir>/<tag>.marks.json)


def step_metrics(t, sp, x, t0, t1):
    """Rise time 10-90 %, overshoot %, settling time (10 % band) and RMS error after 1 s, for a step at t0."""
    k = (t >= t0) & (t < t1)
    tt, s, y = t[k] - t0, sp[k], x[k]
    base = np.median(x[(t >= t0 - 0.5) & (t < t0)])
    amp = np.median(s[tt > 1.0]) - base
    if abs(amp) < 1e-3:
        return np.nan, np.nan, np.nan, np.nan
    n = (y - base) / amp
    rise = tt[np.argmax(n >= 0.9)] - tt[np.argmax(n >= 0.1)]
    over = max(0.0, (np.max(n) - 1.0) * 100)
    outside = np.where(np.abs(n - 1.0) > 0.1)[0]
    settle = tt[outside[-1]] if len(outside) else 0.0
    err = np.sqrt(np.mean((s - y)[tt > 1.0] ** 2))
    return rise, over, settle, err


folder = sys.argv[1]
rows = []
for tag in sys.argv[2:]:
    u, d = load(os.path.join(folder, tag + '.ulg'))
    mt = {k: v / 1000.0 for k, v in json.load(open(os.path.join(folder, tag + '.marks.json')))}
    av, rsp, lp, am = d['vehicle_angular_velocity'], d['vehicle_rates_setpoint'], d['vehicle_local_position'], d['actuator_motors']
    ta, (r, p, _) = attitude(d)
    ts, (rs, ps, _) = attitude(d, 'vehicle_attitude_setpoint', 'q_d')
    rs_i, ps_i = np.interp(ta, ts, rs), np.interp(ta, ts, ps)
    roll = step_metrics(ta, rs_i, r, mt['roll_step'], mt['roll_release'])
    pitch = step_metrics(ta, ps_i, p, mt['pitch_step'], mt['pitch_release'])
    tw, tr = av['timestamp'] / 1e6, rsp['timestamp'] / 1e6
    yaw = step_metrics(tw, np.interp(tw, tr, np.degrees(rsp['yaw'])), np.degrees(av['xyz[2]']), mt['yaw_step'], mt['yaw_release'])

    tl = lp['timestamp'] / 1e6
    before = (tl >= mt['wind_on'] - 1) & (tl < mt['wind_on'])
    x0, y0 = np.median(lp['x'][before]), np.median(lp['y'][before])
    kw = (tl >= mt['wind_on']) & (tl < mt['wind_off'] + 4)
    wind_dev = np.max(np.hypot(lp['x'][kw] - x0, lp['y'][kw] - y0))
    kws = (tl >= mt['wind_off'] - 3) & (tl < mt['wind_off'])
    wind_end = np.max(np.hypot(lp['x'][kws] - x0, lp['y'][kws] - y0))

    kf = (ta >= mt['fail']) & (ta < mt['fail'] + 4)
    fail_att = np.max(np.hypot(r[kf] - rs_i[kf], p[kf] - ps_i[kf]))
    kf2 = (tl >= mt['fail']) & (tl < mt['fail'] + 6)
    fail_z = np.max(np.abs(lp['z'][kf2] - np.median(lp['z'][(tl >= mt['fail'] - 1) & (tl < mt['fail'])])))
    fail_xy = np.max(np.hypot(lp['x'][kf2] - lp['x'][kf2][0], lp['y'][kf2] - lp['y'][kf2][0]))
    froll = step_metrics(ta, rs_i, r, mt['fail_roll_step'], mt['fail_roll_release'])
    tm = am['timestamp'] / 1e6
    M = np.stack([am['control[%d]' % i] for i in range(6)], 1)
    sat = np.mean(np.any(M[(tm >= mt['fail']) & (tm < mt['end'])] > 0.98, axis=1)) * 100
    rows.append((tag, roll, pitch, yaw, wind_dev, wind_end, fail_att, fail_z, fail_xy, froll, sat))

print('%-22s | %-26s | %-26s | %-18s | %-15s | %-24s | %-20s | %s' % (
    'run', 'roll step: rise/over/settle', 'pitch step: rise/over/settle', 'yaw rate: rise/over',
    'wind: max/end', 'motor 1 off: att/dz/dxy', 'roll step on 5 motors', 'sat%'))
for tag, ro, pi, ya, wd, we, fa, fz, fxy, fr, sat in rows:
    print('%-22s | %5.2fs %5.1f%% %5.2fs     | %5.2fs %5.1f%% %5.2fs     | %5.2fs %5.1f%%     | %4.2fm %4.2fm     '
          '| %5.1fdeg %4.2fm %4.2fm   | %5.2fs %5.1f%% %4.2fs | %4.1f'
          % (tag, ro[0], ro[1], ro[2], pi[0], pi[1], pi[2], ya[0], ya[1], wd, we, fa, fz, fxy, fr[0], fr[1], fr[2], sat))
