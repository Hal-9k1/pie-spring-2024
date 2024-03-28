import math

class Motor:
    """Wraps a KoalaBear-controlled motor."""
    __slots__ = "_controller", "_motor", "_robot", "_debug_logger", "_is_inverted"
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
        self._robot.set_value(self._controller, key + "_" + self._motor, value)
    def _get(self, key):
        return self._robot.get_value(self._controller, key + "_" + self._motor)
    
class MotorPair(Motor):
    __slots__ = "_paired_motor"
    def __init__(self, robot, debug_logger, controller_id, motor_suffix,
        paired_controller_id, paired_motor_suffix):
        super().__init__(robot, debug_logger, controller_id, motor_suffix)
        self._paired_motor = Motor(robot, debug_logger, paired_controller_id,
            paired_motor_suffix) #.set_invert(True)
    def set_invert(self, invert):
        super().set_invert(invert)
        #self._paired_motor.set_invert(not invert)
        self._paired_motor.set_invert(invert)
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
        #self._debug_logger.print(f"MotorPair velocity: {velocity}")
        super().set_velocity(velocity)
        self._paired_motor.set_velocity(velocity)
        return self

class Servo:
    __slots__ = "_controller", "_servo", "_robot"
    def __init__(self, robot, controller, servo):
        self._controller = controller
        self._servo = servo
        self._robot = robot
    def set_position(self, position):
        self._robot.set_value(self._controller, "servo" + self._servo, position)
