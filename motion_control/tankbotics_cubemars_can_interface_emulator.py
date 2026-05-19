import time

LOG_PERIOD = 0.05

class can_motor_interface_emulator:
    def __init__(self, joint_model, motor_id):
        self.motor_id = motor_id
        self.kp = 300.0
        self.kd = 0.5
        self.orientation = 1
        self.last_position_feedback = 0.0
        self.joint_model = 2 if joint_model == "aka10" else 1
        self.last_log = {}
        self.log_period = LOG_PERIOD

    def _should_log(self, method_name):
        now = time.time()
        if method_name not in self.last_log or now - self.last_log[method_name] >= self.log_period:
            self.last_log[method_name] = now
            return True
        return False

    def _log(self, method_name, extra_info=""):
        if self._should_log(method_name):
            model_name = "aka10" if self.joint_model == 2 else "ak70"
            print(f"VIRTUAL_CAN_EMULATOR | motor_{self.motor_id} | {model_name} | {method_name}{extra_info}")

    def _fake_status(self, position):
        return [position, 0.0, 0.0, 30.0, 0, "no error"]

    def enable_motor(self):
        self._log("enable_motor")
        return True

    def disable_motor(self):
        self._log("disable_motor")
        return True

    def set_zero_torque_and_get_status(self):
        self._log("set_zero_torque_and_get_status")
        self.last_position_feedback = 0.0
        return self._fake_status(0.0)

    def set_zero_torque_and_get_position(self):
        status = self.set_zero_torque_and_get_status()
        return status[0] if status else None

    def set_origin(self):
        self._log("set_origin")

    def send_position_controll_command_arbitrary(self, position):
        p_des = position * self.orientation
        self._log("send_position_controll_command_arbitrary", f" p_des={p_des:.3f}")
        status = self._fake_status(position)
        return status

    def send_position_controll_command(self, position):
        if abs(position - self.last_position_feedback) > 8:
            self._log("send_position_controll_command", " REJECTED too far")
            return [0.0, 0.0, 0.0, 25.0, 0, "position too far away"]
        status = self.send_position_controll_command_arbitrary(position)
        self.last_position_feedback = position
        return status

    def send_velocity_controll_command(self, v_des, kd):
        v_des = v_des * self.orientation
        self._log("send_velocity_controll_command", f" v_des={v_des:.3f}")
        return self._fake_status(self.last_position_feedback)

    def send_torque_controll_command(self, t_ff):
        t_ff = t_ff * self.orientation
        self._log("send_torque_controll_command", f" t_ff={t_ff:.3f}")
        return self._fake_status(self.last_position_feedback)

    def send_current_brake_command(self, current):
        if self.joint_model != 2:
            return False
        self._log("send_current_brake_command", f" current={current:.3f}")
        return self._fake_status(self.last_position_feedback)

    def send_position_velocity_command(self, position_deg, speed_erpm, accel_erpm2):
        position_deg = position_deg * self.orientation
        self._log("send_position_velocity_command", f" pos={position_deg:.3f}")
        status = self._fake_status(position_deg)
        self.last_position_feedback = position_deg
        return status

    def send_position_loop_command(self, position_deg):
        if self.joint_model != 2:
            return False
        position_deg = position_deg * self.orientation
        self._log("send_position_loop_command", f" pos={position_deg:.3f}")
        status = self._fake_status(position_deg)
        self.last_position_feedback = position_deg
        return status