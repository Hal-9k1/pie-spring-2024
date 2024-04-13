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
    def get_angle(self, ticks_per_rot):
        return self.get_encoder() / ticks_per_rot * 2 * math.pi
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
        return self._motor.get_angle(self._ticks_per_rot) * self._radius
    def set_velocity(self, velocity):
        """Sets the velocity of the underlying motor."""
        self._motor.set_velocity(velocity)

class Arm:
    """Encapsulates a Motor attached to an arm that can calculate the height of the arm's end
    relative to the motor and detect out-of-bounds movement given the motor's ticks per rotation,
    the arm's length, and the maximum angle."""
    def __init__(self, debug_logger, motor, length, ticks_per_rotation, max_angle):
        self._motor = motor
        self._length = length
        self._ticks_per_rot = ticks_per_rotation
        self._debug_logger = debug_logger
        self._max_angle = max_angle
    def get_height(self):
        """Interpreting the motor as being attached to an arm, converts the encoder readout of the
        motor to the vertical position of the arm's tip relative to the motor."""
        return math.sin(self._motor.get_angle(self._ticks_per_rot)) * self._length
    def set_velocity(self, velocity):
        """Sets the velocity of the underlying motor."""
        self._motor.set_velocity(velocity)
    def get_normalized_position(self):
        """Returns a number in the range [0, 1] where 0 is linearly mapped to an encoder position of
        0 and 1 is linearly mapped to the encoder position corrosponding to the arm's maximum
        angle."""
        return self._motor.get_angle(self._ticks_per_rot) / self._max_angle
    def is_velocity_safe(self, velocity):
        """Checks if the arm is currently within its defined safe bounds. If in of bounds, returns
        True. Otherwise returns whether the given velocity is in the right direction to return the
        arm to its safe bounds."""
        angle = self._motor.get_angle(self._ticks_per_rot)
        if angle > self._max_angle:
            return velocity < 0
        elif angle < 0:
            return velocity > 0
        else:
            return True

class Hand:
    def __init__(self, debug_logger, motor, ticks_per_rotation, max_angle):
        self._debug_logger = debug_logger
        self._motor = motor
        self._ticks_per_rot = ticks_per_rotation
        self._open_angle = max_angle
        self._close_angle = 0
        self._state = False
        self._finished = True
    def set_state(self, state):
        self._state = state
        self._motor.set_velocity(1 if state else -1)
        self._finished = False
    def tick(self):
        if not self._finished:
            angle = self._motor.get_angle(self._ticks_per_rot)
            if (self._state and angle > self._open_angle) or angle < self._close_angle:
                self._finished = True
                self._motor.set_velocity(0)
                return True
        return False
class Servo:
    def __init__(self, robot, controller, servo):
        self._controller = controller
        self._servo = servo
        self._robot = robot
    def set_position(self, position):
        self._robot.set_value(self._controller, "servo" + self._servo, position)
