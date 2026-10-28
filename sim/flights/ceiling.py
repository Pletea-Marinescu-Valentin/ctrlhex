import argparse

from link import Vehicle

# Stabilized mode (throttle sets the thrust directly): the height ceiling, or the forced geofence alone
ap = argparse.ArgumentParser()
ap.add_argument('test', choices=['ceiling', 'geofence'])
ap.add_argument('--armed-wait', type=float, default=0.0, help='time armed on the ground before take-off [s]')
a = ap.parse_args()
v = Vehicle()

v.phase('wait', t=2)
v.set_mode('STABILIZED'); v.phase('stabilized', z=0, t=1)
v.arm(); v.phase('arm', z=0, t=1)
if a.armed_wait > 0:
    v.phase('armed on the ground, throttle 0', z=0, t=a.armed_wait)
if a.test == 'ceiling':
    v.phase('throttle 70% for 10 s', z=700, t=10)
    v.phase('throttle 100% for 6 s', z=1000, t=6)
    v.phase('throttle 100% + full forward 4 s', z=1000, x=1000, t=4)
    v.phase('throttle 40% (descend) 4 s', z=400, t=4)
    v.phase('throttle 100% again 5 s', z=1000, t=5)
    v.phase('throttle 0 (land) 8 s', z=0, t=8)
else:
    v.phase('throttle 100% 6 s', z=1000, t=6)
    v.phase('sticks + full throttle during Land 6 s', z=1000, x=700, y=-700, t=6)
    v.phase('mode switch flips during Land 6 s', z=1000, t=6, each=lambda: v.set_mode('STABILIZED'))
    v.phase('wait 6 s', z=500, t=6)
