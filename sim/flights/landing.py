import sys

from link import Vehicle

# Landing with motor 1 stopped ('keep') or restored before the landing ('restore')
how = sys.argv[1] if len(sys.argv) > 1 else 'keep'
v = Vehicle()

v.phase('wait', t=2)
v.set_mode('POSCTL'); v.phase('position mode', t=1)
v.arm(); v.phase('arm', z=0, t=1)
v.phase('climb', z=800, t=3)
v.phase('hover', t=3)
v.motor_off(1); v.phase('motor 1 off', t=5)
if how == 'restore':
    v.motor_off(1, False); v.phase('motor 1 restored', t=3)
v.phase('descend and land', z=0, t=14)
