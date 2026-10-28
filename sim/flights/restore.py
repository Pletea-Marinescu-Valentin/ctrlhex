import sys

from link import Vehicle

# A motor stops and comes back. argv: motor (1-based), test:
#   alt    Altitude mode
#   stab   Stabilized, linear throttle under the ceiling
#   again  Altitude mode, the motor fails again in the middle of its restore
motor = int(sys.argv[1]) if len(sys.argv) > 1 else 2
test = sys.argv[2] if len(sys.argv) > 2 else 'alt'
v = Vehicle(period=0.02)

v.phase('wait', t=2)
if test == 'stab':
    v.set_mode('STABILIZED'); v.phase('stabilized', z=0, t=1)
    v.arm(); v.phase('arm', z=0, t=1)
    v.phase('throttle 35% up to the ceiling', z=350, t=7)
    v.motor_off(motor); v.phase('motor %d off' % motor, z=350, t=6)
    v.motor_off(motor, False); v.phase('motor back, first 1.5 s', z=350, t=1.5)
    v.phase('motor back, 1.5-4 s', z=350, t=2.5)
    v.phase('throttle 15% descend', z=150, t=6)
    v.phase('throttle 0', z=0, t=4)
else:
    v.set_mode('ALTCTL'); v.phase('altitude mode', t=1)
    v.arm(); v.phase('arm', z=0, t=1)
    v.phase('climb', z=800, t=5)
    v.phase('hover', t=3)
    v.motor_off(motor); v.phase('motor %d off' % motor, t=6)
    if test == 'again':
        v.motor_off(motor, False); v.phase('motor back for 0.7 s', t=0.7)
        v.motor_off(motor); v.phase('motor off again', t=4)
    v.motor_off(motor, False); v.phase('motor back, first 1.5 s', t=1.5)
    v.phase('motor back, 1.5-4 s', t=2.5)
    v.phase('descend and land', z=0, t=14)
