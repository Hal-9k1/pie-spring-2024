import devices
import math
import mock_robot
import util

HUB_TO_WHEEL_GEAR_RATIO = 84 / 36
DRIVE_MOTOR_RATIO = 30
DRIVE_MOTOR_TPR = 64
DRIVE_WHEEL_SPAN = 0.258
DRIVE_WHEEL_RADIUS = util.inches_to_meters(4)
ARM_MOTOR_RATIO = 30
ARM_MOTOR_TPR = 64
ARM_LENGTH = util.inches_to_meters(14)

debug_logger = util.DebugLogger(default_interval=2000)
initialized = False
robot = None
drive_wheel_left = None
drive_wheel_right = None
arm = None

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
    print(goal_dist)
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
def arm_height(height):
    init_height = 0
    goal_delta_height = 0
    def arm_height_action(setup):
        nonlocal init_height
        nonlocal goal_delta_height
        if setup:
            init_height = arm.get_height()
            goal_delta_height = height - init_height
            arm.set_velocity(math.copysign(1, delta_height))
        else:
            delta_height = arm.get_height() - init_height
            return (goal_delta_height * delta_height >= 0
                and abs(delta_height) > abs(goal_delta_height))
    return arm_height_action
def nop(is_done):
    def nop_action(setup):
        return is_done
    return nop_action
def halt():
    def halt_action(setup):
        if setup:
            drive_wheel_left.set_velocity(0)
            drive_wheel_right.set_velocity(0)
        return True
    return halt_action

actions = []

def initialize():
    global initialized
    global robot
    global drive_wheel_left
    global drive_wheel_right
    global arm
    if not initialized:
        initialized = True
        #robot = mock_robot.MockRobot(debug_logger, {"koalabear": 1, "servocontroller": 0})
        robot = Robot
        drive_wheel_left = devices.Wheel(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_10833107448071795766", "b")
                .set_invert(True)
                .set_pid(None, None, None),
            DRIVE_WHEEL_RADIUS,
            DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
            )
        drive_wheel_right = devices.Wheel(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_10833107448071795766", "a")
                .set_invert(False)
                .set_pid(None, None, None),
            DRIVE_WHEEL_RADIUS,
            DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
            )
        arm = devices.Arm(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_8847060420572259627", "b")
                .set_invert(False)
                .set_pid(None, None, None),
            ARM_LENGTH,
            ARM_MOTOR_TPR * ARM_MOTOR_RATIO,
            math.pi / 2
            )
def autonomous_setup():
    initialize()
    actions.append(nop(True))
    up = False
    #for _ in range(4):
    #    actions.append(linear_move(1))
    #    up = not up
    #    actions.append(arm_height((ARM_HEIGHT / 2) if up else 0))
    #    actions.append(turn(-math.pi / 2))
    #actions.append(halt())
    actions.append(linear_move(1))
    actions.append(halt())
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
    initialize()
def tank_drive_teleop_main():
    drive_wheel_left.set_velocity(Gamepad.get_value("joystick_left_y"))
    drive_wheel_right.set_velocity(Gamepad.get_value("joystick_right_y"))
def drive_turn_teleop_main():
    drive = Gamepad.get_value("joystick_left_y")
    turn = Gamepad.get_value("joystick_right_x")
    left_velocity = drive + turn
    right_velocity = drive - turn
    max_abs_velocity = max(1.0, abs(left_velocity), abs(right_velocity))
    drive_wheel_left.set_velocity(left_velocity / max_abs_velocity)
    drive_wheel_right.set_velocity(right_velocity / max_abs_velocity)
def arm_teleop_main():
    arm_vel_dir = ((1 if Gamepad.get_value("r_bumper") else 0)
        - (1 if Gamepad.get_value("r_trigger") else 0))
    arm_pos = arm.get_normalized_position()
    # 1 / (x^2 - 2x + 5/2) + 1/3
    arm.set_velocity(arm_vel_dir * arm.is_velocity_safe(arm_vel_dir)
        / (arm_pos * (arm_pos - 2) + 5/2) + 1/3)
def teleop_main():
    tank_drive_teleop_main()
    #drive_turn_teleop_main()
    arm_teleop_main()
    debug_logger.tick()

