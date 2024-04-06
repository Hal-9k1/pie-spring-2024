import math

class Motor:
    """Wraps a KoalaBear-controlled motor."""
    def __init__(self, robot, debug_logger, controller_id, motor):
        self._controller = controller_id
        self._motor = motor
        self._robot = robot
        self._debug_logger = debug_logger
        self._is_inverted = False
    def set_invert(self, invert):
        self._set("invert", invert)
        self._is_inverted = invert
        return self
    def set_deadband(self, deadband):
        self._set("deadband", deadband)
        return self
    def set_pid(self, p, i, d):
        if not (p or i or d):
            self._set("pid_enabled", False)
            return self
        self._set("pid_enabled", True)
        if p:
            self._set("pid_kp", p)
        if i:
            self._set("pid_ki", i)
        if d:
            self._set("pid_kd", d)
        return self
    def set_velocity(self, velocity):
        self._set("velocity", velocity)
        return self
    def get_velocity(self):
        return self._get("velocity")
    def get_encoder(self):
        return self._get("enc") * (-1 if self._is_inverted else 1)
    def reset_encoder(self):
        self._set("enc", 0)
    def _set(self, key, value):
        self._robot.set_value(self._controller, f"{key}_{self._motor}", value)
    def _get(self, key):
        return self._robot.get_value(self._controller, f"{key}_{self._motor}")
    
class MotorPair(Motor):
    def __init__(self, robot, debug_logger, controller_id, motor_suffix,
        paired_controller_id, paired_motor_suffix, paired_motor_inverted):
        super().__init__(robot, debug_logger, controller_id, motor_suffix)
        self._paired_motor = Motor(robot, debug_logger, paired_controller_id,
            paired_motor_suffix).set_invert(paired_motor_inverted)
        self._inverted = False
    def set_invert(self, invert):
        if invert != self._inverted:
            self._inverted = invert
            super().set_invert(not self._get("invert"))
            self._paired_motor.set_invert(not self._paired_motor.get("invert"))
        return self
    def set_deadband(deadband):
        super().set_deadband(self, deadband)
        self._paired_motor.set_deadband(deadband)
        return self
    def set_pid(self, p, i, d):
        super().set_pid(p, i, d)
        self._paired_motor.set_pid(p, i, d)
        return self
    def set_velocity(self, velocity):
        super().set_velocity(velocity)
        self._paired_motor.set_velocity(velocity)
        return self

class Wheel:
    """Encapsulates a Motor attached to a wheel that can calculate distance travelled given the
    motor's ticks per rotation and the wheel's radius."""
    def __init__(self, debug_logger, motor, radius, ticks_per_rotation):
        self._motor = motor
        self._radius = radius
        self._ticks_per_rot = ticks_per_rotation
        self._debug_logger = debug_logger
    def get_distance(self):
        """Interpreting the motor as being attached to a wheel, converts the encoder readout of the
        motor to a distance traveled by the wheel."""
        return self._motor.get_encoder() / self._ticks_per_rot * 2 * math.pi * self._radius
    def set_velocity(self, velocity):
        """Sets the velocity of the underlying motor."""
        self._motor.set_velocity(velocity)

class Arm:
    """Alternatively uses these variables to calculate the height of an arm attached to the
    motor."""
    def __init__(self, debug_logger, motor, radius, ticks_per_rotation, max_angle):
        self._motor = motor
        self._radius = radius
        self._ticks_per_rot = ticks_per_rotation
        self._debug_logger = debug_logger
        self._max_angle = max_angle
    def get_height(self):
        """Interpreting the motor as being attached to an arm, converts the encoder readout of the
        motor to the vertical position of the arm's tip relative to the motor."""
        return (math.sin(self._motor.get_encoder() / self._ticks_per_rot * 2 * math.pi)
            * self._radius)
    def set_velocity(self, velocity):
        """Sets the velocity of the underlying motor."""
        self._motor.set_velocity(velocity)

class Servo:
    def __init__(self, robot, controller, servo):
        self._controller = controller
        self._servo = servo
        self._robot = robot
    def set_position(self, position):
        self._robot.set_value(self._controller, "servo" + self._servo, position)
