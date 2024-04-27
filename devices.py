import math
import time

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

class PidMotor(Motor):
    def __init__(self, robot, debug_logger, controller_id, motor):
        super().__init__(robot, debug_logger, controller_id, motor)
        super().set_pid(None, None, None)
        self._clear_samples()
        self._max_samples = 200
        self._held_position = None
        self._last_timestamp = None
        self._coeffs = None
    def set_velocity(self, velocity):
        self._held_position = None # force clear on next hold_position call
        super().set_velocity(velocity)
        return self
    def set_pid(self, p, i, d):
        if not p and not i and not d:
            self._coeffs = None
        else:
            self._coeffs = (p, i, d)
        return self
    def set_position(self, pos):
        self._clear_samples()
        self._held_position = pos
        return self
    def hold_position(self):
        if not self._coeffs:
            raise Exception("PID coefficients not set.")
        self._record_sample()
        super().set_velocity(self._calc_proportional(self._held_position)
            + self._calc_integral(self._held_position) + self._calc_derivative())
        return self
    def _record_sample(self):
        self._enc_samples[self._cur_sample] = self.get_encoder()
        timestamp = time.time()
        self._time_samples[self._cur_sample] = timestamp - (self._last_timestamp or 0)
        self._last_timestamp = timestamp
        self._cur_sample += 1
        if self._cur_sample == self._max_samples:
            self._cur_sample = 0
    def _clear_samples(self):
        self._enc_samples = []
        self._time_samples = []
        self._cur_sample = 0
    def _calc_proportional(self):
        return self._coeffs[0] * (self.get_encoder() - self._held_position)
    def _calc_integral(self):
        return self._coeffs[1] * sum((enc - self._held_position) * time for (enc, time)
            in zip(self._enc_samples, self._time_samples))
    def _calc_derviative(self):
        return (self._coeffs[2]
            * (self._enc_samples[self._cur_sample] - self._enc_samples[self._cur_sample - 1])
            / (self._time_samples[self._cur_sample] - self._time_samples[self._cur_sample - 1]))
    
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
    def __init__(self, debug_logger, motor, length, ticks_per_rotation, max_height):
        self._motor = motor
        self._length = length
        self._ticks_per_rot = ticks_per_rotation
        self._debug_logger = debug_logger
        self._max_height = max_height
        self._zero_height = 0 # bootstrap for get_height
        self._zero_height = self.get_height()
    def get_height(self):
        """Interpreting the motor as being attached to an arm, converts the encoder readout of the
        motor to the vertical position of the arm's tip relative to the motor."""
        return (math.sin(self._motor.get_angle(self._ticks_per_rot)) * self._length
            - self._zero_height)
    def set_velocity(self, velocity):
        """Sets the velocity of the underlying motor."""
        if not velocity and self._motor.get_velocity():
            #self._motor.set_position(self._motor.get_encoder())
            pass
        elif not velocity:
            #self._motor.hold_position()
            pass
        self._motor.set_velocity(velocity)
    def get_normalized_position(self):
        """Returns a number in the range [0, 1] where 0 is linearly mapped to an encoder position of
        0 and 1 is linearly mapped to the encoder position corrosponding to the arm's maximum
        angle."""
        return self.get_height() / (self._max_height - self._zero_height)
    def is_velocity_safe(self, velocity):
        """Checks if the arm is currently within its defined safe bounds. If in of bounds, returns
        True. Otherwise returns whether the given velocity is in the right direction to return the
        arm to its safe bounds."""
        height = self.get_height()
        if height > self._max_height:
            return velocity < 0
        elif height < 0:
            return velocity > 0
        else:
            return True

class Hand:
    _MAX_HISTORY_LENGTH = 40000
    _STRUGGLE_THRESHOLD = 0.02
    def __init__(self, debug_logger, motor, ticks_per_rotation, max_width, hand_length,
            struggle_duration, start_open = True):
        # disable struggle checking if struggle_duration == 0
        self._debug_logger = debug_logger
        self._motor = motor
        self._ticks_per_rot = ticks_per_rotation
        self._hand_length = hand_length
        self._struggle_duration = struggle_duration
        init_width = self._get_width()
        if start_open:
            self._open_width = init_width
            self._close_width = init_width - max_width
        else:
            self._open_width = init_width + max_width
            self._close_width = init_width
        self._state = start_open
        self._width_history = [(0, None)] * self._MAX_HISTORY_LENGTH
        self._hist_pos = 0
        self._finished = True
    def toggle_state(self):
        self._state = not self._state
        self._motor.set_velocity(self._get_hand_speed() * (1 if self._state else -1))
        self._finished = False
    def tick(self):
        if not self._finished:
            width = self._get_width()
            reached_end = (self._state and width < self._open_width) or (not self._state and
                width > self._close_width)
            # don't check if struggle duration is 0
            if self._struggle_duration:
                lookbehind = self._get_hist_lookbehind()
                struggling = (lookbehind and abs(lookbehind - self._get_width())
                    < self._STRUGGLE_THRESHOLD)
            else:
                struggling = False
            if reached_end or struggling:
                self._finished = True
                self._motor.set_velocity(0)
                return True
        return False
    def _get_width(self):
        return math.sin(self._motor.get_angle(self._ticks_per_rot)) * self._hand_length
    def _get_hand_speed(self):
        return 0.5
    def _get_hist_time(self, i):
        return self._width_history[i * 2]
    def _get_hist_width(self, i):
        return self._width_history[i * 2 + 1]
    def _set_hist_time(self, i, x):
        self._width_history[i * 2] = x
    def _set_hist_width(self, i, x):
        self._width_history[i * 2 + 1] = x
    def _record_history(self):
        self._set_hist_time(self._hist_pos, time.time())
        self._set_hist_width(self._hist_pos, self._get_width())
        self._hist_pos = (self._hist_pos + 1) % self._MAX_HISTORY_LENGTH
    def _get_hist_lookbehind(self):
        min_idx = self._hist_pos
        max_idx = self._hist_pos + self._MAX_HISTORY_LENGTH - 1
        goal_time = time.time() - self._struggle_duration
        while True:
            mid_idx = math.floor((min_idx + max_idx) / 2)
            hist_entry = self._width_history[mid_idx % self._MAX_HISTORY_LENGTH]
            if hist_entry[0] < goal_time:
                min_idx = mid_idx + 1
            elif hist_entry[0] > goal_time:
                max_idx = mid_idx - 1
            if min_idx == max_idx:
                return hist_entry[1]

class Servo:
    def __init__(self, robot, controller, servo):
        self._controller = controller
        self._servo = servo
        self._robot = robot
    def set_position(self, position):
        self._robot.set_value(self._controller, "servo" + self._servo, position)
