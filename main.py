import devices
import math
import mock_robot
import util

HUB_TO_WHEEL_GEAR_RATIO = 84 / 38 # check this
DRIVE_MOTOR_RATIO = 70
DRIVE_MOTOR_TPR = 64
DRIVE_WHEEL_SPAN = 1 # need an actual measurement
DRIVE_WHEEL_RADIUS = util.inches_to_meters(4)

debug_logger = util.DebugLogger(default_interval=1)
#robot = mock_robot.MockRobot(debug_logger, {"koalabear": 1, "servocontroller": 0})
robot = Robot
drive_wheel_left = devices.Wheel(
    debug_logger,
    devices.Motor(robot, debug_logger, "6_16799360815822931500", "b")
        .set_invert(False)
        .set_pid(None, None, None),
    DRIVE_WHEEL_RADIUS,
    DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
    )
drive_wheel_right = devices.Wheel(
    debug_logger,
    devices.Motor(robot, debug_logger, "6_16799360815822931500", "a")
        .set_invert(False)
        .set_pid(None, None, None),
    DRIVE_WHEEL_RADIUS,
    DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
    )

def linear_move(dist):
    init_dist = 0
    def linear_move_action(setup):
        nonlocal init_dist
        if setup:
            init_dist = drive_wheel_left.get_distance()
            #print(f"linear_move_action init_dist = {init_dist}")
            drive_wheel_left.set_velocity(math.copysign(1, dist))
            drive_wheel_right.set_velocity(math.copysign(1, dist))
        else:
            delta_dist = drive_wheel_left.get_distance() - init_dist
            #debug_logger.print(f"linear_move_action delta_dist = {delta_dist}")
            return dist * delta_dist >= 0 and abs(delta_dist) > abs(dist)
    return linear_move_action
def turn(ccw_angle):
    init_dist = 0
    goal_dist = ccw_angle * DRIVE_WHEEL_SPAN / 2
    def turn_action(setup):
        nonlocal init_dist
        if setup:
            init_dist = drive_wheel_right.get_distance()
            #print(f"turn_action init_dist = {init_dist} goal_dist = {goal_dist}")
            drive_wheel_left.set_velocity(-math.copysign(1, goal_dist))
            drive_wheel_right.set_velocity(math.copysign(1, goal_dist))
        else:
            delta_dist = drive_wheel_right.get_distance() - init_dist
            #debug_logger.print(f"turn_action delta_dist = {delta_dist}")
            return goal_dist * delta_dist >= 0 and abs(delta_dist) > abs(goal_dist)
    return turn_action
def nop(is_done):
    def nop_action(setup):
        return is_done
    return nop_action

actions = [nop(True)]

def autonomous_setup():
    for _ in range(4):
        actions.append(linear_move(1))
        actions.append(turn(-math.pi / 2))
def autonomous_main():
    if actions and actions[0](False):
        actions.pop(0)
        if actions:
            print("Completed action, setting up next one.")
            actions[0](True)
        else:
            print("Done!")
    debug_logger.tick()
def teleop_setup():
    pass
def teleop_main():
    pass
