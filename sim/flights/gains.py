from link import Vehicle

# Same sequence for every gain set: attitude steps, yaw-rate step, 4 m/s wind, motor 1 off, roll step on 5 motors
v = Vehicle()

v.sticks(t=2)
v.set_mode('ALTCTL'); v.sticks(t=1)
v.arm(); v.sticks(z=0, t=1)
v.sticks(z=800, t=3); v.sticks(t=4)
v.mark('roll_step'); v.sticks(y=500, t=2.5); v.mark('roll_release'); v.sticks(t=3.5)
v.mark('pitch_step'); v.sticks(x=500, t=2.5); v.mark('pitch_release'); v.sticks(t=3.5)
v.mark('yaw_step'); v.sticks(r=500, t=2.5); v.mark('yaw_release'); v.sticks(t=3.5)
v.set_mode('POSCTL'); v.sticks(t=4)
v.mark('wind_on'); v.param('SIH_WIND_N', 4.0); v.sticks(t=10)
v.mark('wind_off'); v.param('SIH_WIND_N', 0.0); v.sticks(t=8)
v.mark('fail'); v.motor_off(1); v.sticks(t=8)
v.mark('fail_roll_step'); v.sticks(y=500, t=2.5); v.mark('fail_roll_release'); v.sticks(t=3.5)
v.mark('end')
v.set_mode('LAND'); v.sticks(t=10)
v.save_marks()
