"""Where the follower's tip is, live: move the arm by hand and watch x, y, z and the gripper's pitch. Each
line is one read of the six servos and the forward kinematics, both in the C library (so101_arm_where), as
a board with a display would run them; examples/where.c is the same without Python.

The torque goes off at the start so the joints turn by hand: hold the arm if it stands raised with its
torque on. Nothing moves by itself; Ctrl-C ends it. SO101_PORT=sim tries it without an arm."""
import time

from so101ik import Arm

arm = Arm(name="where")      # the port, the calibration and its offsets found as for a move
try:
    arm.check()              # every servo answers and holds the calibration
    arm.torque_off()
    print("the tip in mm (x forward, y to the arm's left, z up) and the gripper's pitch in degrees (-90 down)")
    while True:
        x, y, z, pitch = arm.where()
        print(f"\rx {1000 * x:6.1f}   y {1000 * y:6.1f}   z {1000 * z:6.1f}   pitch {pitch:6.1f} ", end="", flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    print()
finally:
    arm.close()
