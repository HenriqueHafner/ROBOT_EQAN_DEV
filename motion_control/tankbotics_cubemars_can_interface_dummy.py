class can_motor_interface:
    def __init__(self, joint_model, motor_id):
        self.motor_id = motor_id
        self.kp = 5.0
        self.kd = 0.5
        self.orientation = 1
        self.last_position_feedback = 0.0
        if joint_model == "aka10":
            self.joint_model = 2
        elif joint_model == "ak70":
            self.joint_model = 1
        else:
            print("joint_model not specified: ", joint_model)

    def enable_motor(self):
        return [self.last_position_feedback, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def disable_motor(self):
        return [self.last_position_feedback, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def set_zero_torque_and_get_status(self):
        return [self.last_position_feedback, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def set_zero_torque_and_get_position(self):
        return 0.0

    def set_origin(self):
        pass

    def send_position_controll_command_arbitrary(self, position):
        self.last_position_feedback = position
        return [position, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def send_position_controll_command(self, position):
        self.last_position_feedback = position
        return [position, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def send_velocity_controll_command(self, v_des, kd):
        return [0.0, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def send_torque_controll_command(self, t_ff):
        return [0.0, 0.0, 0.0, 25.0, 0, "Dummy mode"]
    
    def send_current_brake_command(self, current):
        return [0.0, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def send_position_velocity_command(self, position_deg, speed_erpm, accel_erpm2):
        return [0.0, 0.0, 0.0, 25.0, 0, "Dummy mode"]

    def send_position_loop_command(self, position_deg):
        return [0.0, 0.0, 0.0, 25.0, 0, "Dummy mode"]