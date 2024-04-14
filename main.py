import devices
import math
import mock_robot
import robot_executor
import util

HUB_TO_WHEEL_GEAR_RATIO = 84 / 36
DRIVE_MOTOR_RATIO = 70
DRIVE_MOTOR_TPR = 16 # "hey isn't this supposed to be 64" IT DOESN'T WORK WITH 64
DRIVE_WHEEL_SPAN = 0.258
DRIVE_WHEEL_RADIUS = util.inches_to_meters(2)
HUB_TO_ARM_GEAR_RATIO = 84 / 36
ARM_MOTOR_RATIO = 70
ARM_MOTOR_TPR = 16
ARM_LENGTH = util.inches_to_meters(10.5)
HUB_TO_HAND_GEAR_RATIO = 36 / 12
HAND_MOTOR_RATIO = 70
HAND_MOTOR_TPR = 16
USE_MOCK_ROBOT = False

debug_logger = util.DebugLogger(default_interval=1)
initialized = False
robot = None
drive_wheel_left = None
drive_wheel_right = None
arm = None
hand = None
hand_toggled = False

def initialize():
    global initialized
    global robot
    global drive_wheel_left
    global drive_wheel_right
    global arm
    global hand
    global executor
    if not initialized:
        initialized = True
        robot = (mock_robot.MockRobot(debug_logger, {"koalabear": 2, "servocontroller": 0}, 5000)
            if USE_MOCK_ROBOT else Robot)
        drive_wheel_left = devices.Wheel(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_10833107448071795766", "a") # maybe swap + invert both
                .set_invert(False)
                .set_pid(None, None, None),
            DRIVE_WHEEL_RADIUS,
            DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
            )
        drive_wheel_right = devices.Wheel(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_10833107448071795766", "b")
                .set_invert(True)
                .set_pid(None, None, None),
            DRIVE_WHEEL_RADIUS,
            DRIVE_MOTOR_TPR * DRIVE_MOTOR_RATIO * HUB_TO_WHEEL_GEAR_RATIO
            )
        arm = devices.Arm(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_8847060420572259627", "b")
                .set_invert(True)
                .set_pid(None, None, None),
            ARM_LENGTH,
            ARM_MOTOR_TPR * ARM_MOTOR_RATIO * HUB_TO_ARM_GEAR_RATIO,
            math.pi / 6
            )
        hand = devices.Hand(
            debug_logger,
            devices.Motor(robot, debug_logger, "6_8847060420572259627", "a")
                .set_invert(True)
                .set_pid(None, None, None),
            HAND_MOTOR_TPR * HAND_MOTOR_RATIO * HUB_TO_HAND_GEAR_RATIO,
            math.pi * 80 / 180,
            math.pi / 36,
            start_open = True
            )
        executor = robot_executor.ActionExecutor(debug_logger, robot, DRIVE_WHEEL_SPAN,
            drive_wheel_left, drive_wheel_right, arm, hand)
def autonomous_setup():
    initialize()
    executor.nop(True)
    #up = False
    #for _ in range(4):
    #    actions.append(linear_move(1))
    #    up = not up
    #    executor.arm_height((ARM_HEIGHT / 2) if up else 0)
    #    executor.turn(-math.pi / 2)
    #executor.halt()
    #executor.linear_move(1)
    #executor.halt()
    #executor.toggle_hand()
    #executor.sleep(500)
    #executor.turn(-math.pi / 2)
    #executor.halt()
    #executor.linear_move(DRIVE_WHEEL_RADIUS * 2 * math.pi)
    executor.turn(-math.pi / 2)
    executor.halt()
    executor.status()
def autonomous_main():
    executor.tick()
    debug_logger.tick()
def teleop_setup():
    initialize()
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
    #arm.set_velocity(arm_vel_dir * (arm.is_velocity_safe(arm_vel_dir)
    #    / (arm_pos * (arm_pos - 2) + 5/2) + 1/3))
    arm.set_velocity(arm_vel_dir * arm.is_velocity_safe(arm_vel_dir))
def hand_teleop_main():
    global hand_toggled
    bumper_down = Gamepad.get_value("l_bumper")
    if not hand_toggled and bumper_down:
        hand.toggle_state()
    hand_toggled = bumper_down
    hand.tick()
def teleop_main():
    tank_drive_teleop_main()
    #drive_turn_teleop_main()
    arm_teleop_main()
    hand_teleop_main()
    debug_logger.tick()
