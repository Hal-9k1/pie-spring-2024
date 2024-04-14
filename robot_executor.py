import time

class ActionExecutor:
    def __init__(self, debug_logger, robot, drive_wheel_span, drive_wheel_left, drive_wheel_right,
            arm, hand):
        self._debug_logger = debug_logger
        self._robot = robot
        self._drive_wheel_span = drive_wheel_span
        self._drive_wheel_left = drive_wheel_left
        self._drive_wheel_right = drive_wheel_right
        self._arm = arm
        self._hand = hand
        self._actions = []
    def linear_move(self, dist):
        init_dist = 0
        def linear_move_action(setup):
            nonlocal init_dist
            if setup:
                init_dist = self._drive_wheel_left.get_distance()
                #print(f"linear_move_action init_dist = {init_dist}")
                self._drive_wheel_left.set_velocity(math.copysign(1, dist))
                self._drive_wheel_right.set_velocity(math.copysign(1, dist))
            else:
                delta_dist = self._drive_wheel_left.get_distance() - init_dist
                #debug_logger.print(f"linear_move_action delta_dist = {delta_dist}")
                return dist * delta_dist >= 0 and abs(delta_dist) > abs(dist)
        self._actions.append(linear_move_action)
    def turn(self, ccw_angle):
        init_dist = 0
        goal_dist = ccw_angle * self._drive_wheel_span / 2
        print(goal_dist)
        def turn_action(setup):
            nonlocal init_dist
            if setup:
                init_dist = self._drive_wheel_right.get_distance()
                #print(f"turn_action init_dist = {init_dist} goal_dist = {goal_dist}")
                self._drive_wheel_left.set_velocity(-math.copysign(1, goal_dist))
                self._drive_wheel_right.set_velocity(math.copysign(1, goal_dist))
            else:
                delta_dist = self._drive_wheel_right.get_distance() - init_dist
                #debug_logger.print(f"turn_action delta_dist = {delta_dist}")
                return goal_dist * delta_dist >= 0 and abs(delta_dist) > abs(goal_dist)
        self._actions.append(turn_action)
    def arm_height(self, height):
        init_height = 0
        goal_delta_height = 0
        def arm_height_action(setup):
            nonlocal init_height
            nonlocal goal_delta_height
            if setup:
                init_height = self._arm.get_height()
                goal_delta_height = height - init_height
                self._arm.set_velocity(math.copysign(1, delta_height))
            else:
                delta_height = self._arm.get_height() - init_height
                return (goal_delta_height * delta_height >= 0
                    and abs(delta_height) > abs(goal_delta_height))
        self._actions.append(arm_height_action)
    def toggle_hand(self):
        def toggle_hand_action(setup):
            if setup:
                self._hand.toggle_state()
            else:
                return self._hand.tick()
        self._actions.append(toggle_hand_action)
    def nop(self, is_done):
        def nop_action(setup):
            return is_done
        self._actions.append(nop_action)
    def sleep(self, duration_ms):
        start = 0
        def sleep_action(setup):
            nonlocal start
            if setup:
                start = time.time()
            else:
                return ((time.time() - start) * 1000) >= duration_ms
        self._actions.append(sleep_action)
    def halt(self):
        def halt_action(setup):
            if setup:
                self._drive_wheel_left.set_velocity(0)
                self._drive_wheel_right.set_velocity(0)
            return True
        self._actions.append(halt_action)
    def status(self):
        def status_action(setup):
            if setup:
                print(f"Left wheel: {self._drive_wheel_left._motor.get_encoder()}\n" +
                      f"Right wheel: {self._drive_wheel_right._motor.get_encoder()}\n" +
                      f"Arm: {self._arm._motor.get_encoder()}\n" +
                      f"Hand: {self._hand._motor.get_encoder()}\n")
            return True
        self._actions.append(status_action)
    def tick(self):
        if self._actions and self._actions[0](False):
            self._actions.pop(0)
            if self._actions:
                print("Completed action, setting up next one.")
                self._actions[0](True)
            else:
                print("Done!")
