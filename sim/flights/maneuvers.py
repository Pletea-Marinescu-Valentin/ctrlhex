from link import Vehicle

# Identical manoeuvres before and after motor 1 is stopped: Altitude mode with 60% stick, then Position mode
v = Vehicle()


def maneuvers(tag):
    for name, x, y in [('roll_right', 0, 600), ('roll_left', 0, -600), ('pitch_fwd', 600, 0),
                       ('pitch_back', -600, 0), ('diag', 500, 500), ('diag2', -500, -500)]:
        v.mark(tag + ':' + name); v.sticks(x=x, y=y, t=2.0)
        v.mark(tag + ':hold'); v.sticks(t=2.0)


v.sticks(t=2)
v.set_mode('ALTCTL'); v.sticks(t=1)
v.arm(); v.sticks(z=0, t=1)
v.mark('climb'); v.sticks(z=800, t=5)
v.mark('hover'); v.sticks(t=3)
maneuvers('nominal')
v.mark('inject'); v.motor_off(1)
v.sticks(t=4)
maneuvers('failed')
v.set_mode('POSCTL'); v.sticks(t=2)
v.mark('failed:pos_fwd'); v.sticks(x=1000, t=4)
v.mark('failed:pos_stop'); v.sticks(t=4)
v.mark('failed:pos_right'); v.sticks(y=1000, t=4)
v.mark('failed:pos_stop2'); v.sticks(t=4)
v.mark('failed:yaw'); v.sticks(r=600, t=3)
v.mark('failed:yaw_stop'); v.sticks(t=3)
v.mark('restore'); v.motor_off(1, False)
v.sticks(t=3)
v.mark('land'); v.command(21)
v.sticks(t=60)
v.mark('end')
v.save_marks()
