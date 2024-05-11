import devices
import math
import mock_robot
import robot_executor
import util

HUB_TO_WHEEL_GEAR_RATIO = 84 / 36 # gear further from motor / gear closer to motor
DRIVE_MOTOR_RATIO = 70
DRIVE_MOTOR_TPR = 16 # ticks per rev. "hey isn't this supposed to be 64" IT DOESN'T WORK WITH 64
DRIVE_WHEEL_SPAN = 0.258
DRIVE_WHEEL_RADIUS = util.inches_to_meters(2) # meters.
HUB_TO_ARM_GEAR_RATIO = 84 / 36
ARM_MOTOR_RATIO = 70
ARM_MOTOR_TPR = 16
ARM_LENGTH = util.inches_to_meters(10.5) # meters.
HUB_TO_HAND_GEAR_RATIO = (48 / 36) * (36 / 12) # 36 to 48 common axle with 12 to 36
HAND_MOTOR_RATIO = 70
HAND_MOTOR_TPR = 16
HAND_CENTER_OFFSET = 0.04 # meters.
HAND_OPEN_WIDTH = 0.15 # meters.
HAND_LENGTH = util.inches_to_meters(6.5) # meters.
HAND_STRUGGLE_DURATION = 0 # seconds. 0 to disable
HAND_START_OPEN = True
USE_MOCK_ROBOT = False
ARM_RANGE_LIMIT = 0 #math.pi / 3 # radians. 0 to disable
ARM_USE_EASING = False # only effective with ARM_RANGE_LIMIT

debug_logger = util.DebugLogger(default_interval=2000)
initialized = False
is_dawn_environment = None
robot = None
drive_wheel_left = None
drive_wheel_right = None
arm = None
hand = None
l_bumper_edge = None
a_button_edge = None
disconnected = False

def initialize():
    global initialized, robot, drive_wheel_left, drive_wheel_right, arm, hand, executor
    global is_dawn_environment
    if not initialized:
        initialized = True
        try:
            Robot
        except NameError:
            is_dawn_environment = False
        else:
            is_dawn_environment = True
        if not is_dawn_environment and not USE_MOCK_ROBOT:
            print("No Robot detected; forcing USE_MOCK_ROBOT.")
        robot = (mock_robot.MockRobot(debug_logger, {"koalabear": 2, "servocontroller": 0}, 1,
            5000) if USE_MOCK_ROBOT or not is_dawn_environment else Robot)
        drive_wheel_left = devices.Wheel(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_5011048539317462848", "a")
                .set_invert(False)
                .set_pid(None, None, None),
            DRIVE_WHEEL_RADIUS,
            DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
            )
        drive_wheel_right = devices.Wheel(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_5011048539317462848", "b")
                .set_invert(True)
                .set_pid(None, None, None),
            DRIVE_WHEEL_RADIUS,
            DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
            )
        arm = devices.Arm(
            debug_logger,
            devices.PidMotor(robot, debug_logger, "6_6925048702386060390", "b")
                .set_invert(False)
                .set_pid(0.05, 0.035, -0.01),
            ARM_LENGTH,
            ARM_MOTOR_TPR * ARM_MOTOR_RATIO * HUB_TO_ARM_GEAR_RATIO,
            ARM_RANGE_LIMIT or math.pi * 2
            )
        hand = devices.Hand(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_6925048702386060390", "a")
                .set_invert(True)
                .set_pid(None, None, None),
            HAND_MOTOR_TPR * HAND_MOTOR_RATIO * HUB_TO_HAND_GEAR_RATIO,
            HAND_OPEN_WIDTH,
            HAND_CENTER_OFFSET,
            HAND_LENGTH,
            HAND_STRUGGLE_DURATION,
            HAND_START_OPEN
            )
        executor = robot_executor.ActionExecutor(debug_logger, robot, DRIVE_WHEEL_SPAN,
            drive_wheel_left, drive_wheel_right, arm, hand)
def common_main():
    debug_logger.tick()
    #debug_logger.print("TPS: " + str(debug_logger.get_ticks_per_second()), interval=64000)
@_PREP_ENTRY_POINT
def autonomous_setup():
    initialize()
    #up = False
    #for _ in range(4):
    #    executor.linear_move(1)
    #    up = not up
    #    executor.arm_height((ARM_LENGTH / 2) if up else 0)
    #    executor.toggle_hand()
    #    executor.turn(-math.pi / 2)
    #executor.linear_move(DRIVE_WHEEL_RADIUS * 2 * math.pi)
    executor.linear_move(0.9)
    executor.halt()
    #executor.status()
@_PREP_ENTRY_POINT
def autonomous_main():
    executor.tick()
    common_main()
@_PREP_ENTRY_POINT
def teleop_setup():
    global l_bumper_edge
    global a_button_edge
    initialize()
    l_bumper_edge = util.FlagEdgeDetector(lambda: Gamepad.get_value("l_bumper"))
    a_button_edge = util.FlagEdgeDetector(lambda: Gamepad.get_value("button_a"))
def tank_drive_teleop_main():
    # down is positive on the logitech gamepads for REASONS
    drive_wheel_left.set_velocity(-Gamepad.get_value("joystick_left_y"))
    drive_wheel_right.set_velocity(-Gamepad.get_value("joystick_right_y"))
def drive_turn_teleop_main():
    drive = -Gamepad.get_value("joystick_left_y")
    # right is positive
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
    if ARM_RANGE_LIMIT:
        if ARM_USE_EASING:
            arm.set_velocity(arm_vel_dir * (arm.is_velocity_safe(arm_vel_dir)
                / (arm_pos * (arm_pos - 2) + 5/2) + 1/3))
        else:
            arm.set_velocity(arm_vel_dir * arm.is_velocity_safe(arm_vel_dir))
    else:
        arm.set_velocity(arm_vel_dir)
def hand_teleop_main():
    if l_bumper_edge.test():
        hand.toggle_state()
    hand.tick()
def disconnect_teleop_main():
    global disconnected
    if y_button_edge.test():
        disconnected = not disconnected
    if disconnected:
        debug_logger.print("DISCONNECT DETECTED - TAP Y TO RESOLVE")
        arm.set_velocity(0)
        drive_wheel_left.set_velocity(0)
        drive_wheel_right.set_velocity(0)
    return disconnected
@_PREP_ENTRY_POINT
def teleop_main():
    if a_button_edge.test():
        print("re-initializing")
        global HAND_CENTER_OFFSET
        HAND_CENTER_OFFSET = 0 if HAND_CENTER_OFFSET else 0.04
        initialize()
    if disconnect_teleop_main():
        return
    #tank_drive_teleop_main()
    drive_turn_teleop_main()
    arm_teleop_main()
    hand_teleop_main()
    common_main()
