import devices
import util

robot = Robot
debug_logger = util.DebugLogger(default_interval=2000)
drive_motor_left = devices.Motor(robot, debug_logger, "6_16799360815822931500", "b")
drive_motor_right = devices.Motor(robot, debug_logger, "6_16799360815822931500", "a")

def autonomous_setup():
    drive_motor_left.set_velocity(1)
    drive_motor_right.set_velocity(1)
def autonomous_main():
    pass
def teleop_setup():
    pass
def teleop_main():
    pass
