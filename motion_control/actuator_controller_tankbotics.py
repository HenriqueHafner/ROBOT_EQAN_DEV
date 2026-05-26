import time

from motion_control import acceleration_planner

class actuator_controller:
    aproximation_velocity  = 1.0
    aproximation_acceleration = 1.0
    def __init__(self, model_name, identifier, node_client = None):
        self.model = model_name
        self.identifier = identifier
        self.name = "joint_"+str(identifier)
        self.interface = None
        self.node = node_client
        self.print_orders = True
        self.last_print_timestamp = time.time()

        self.setup_done = False
        self.home_position = None
        self.last_position_feedback = 0.0
        self.last_current_feedback = 0.0
        self.last_temperature_feedback = 0.0
        self.target_position = None
        self.target_torque = None
        self.last_target_position = None
        self.target_current_break = None
        self.rest_offset = 0.0

        self.manuever_status = 0
        self.trajectory = [None]
        self.trajectory_step_index = 0
        self.trajectory_size = 1

        self.motion_planner = acceleration_planner.position_aproximation_planner(self.identifier)
        self.motion_planner.set_params(max_velocity=36, max_acceleration=18, iteration_period=1/60)

    def interface_set(self, interface_reference):
        self.interface = interface_reference

    def set_resting_position(self):
        feedback_value = self.interface.set_zero_torque_and_get_position()
        time.sleep(0.1)
        feedback_value = self.interface.set_zero_torque_and_get_position()
        self.home_position = feedback_value
        self.target_position = feedback_value
        self.last_position_feedback = feedback_value
        print("Resting position set for ", self.identifier, " value: ", feedback_value)
        self.setup_done = True
        return True

    def set_position_controll_target(self, target):
        self.target_position = target

    def set_relative_position_controll_target(self, target):
        self.target_position = target + self.home_position

    def send_position_controll_target(self):
        self.print_target_position()
        feedback = self.interface.send_position_controll_command(self.target_position)
        self.last_target_position = self.target_position
        if feedback is not False:
            print(feedback)
            if feedback[0]:
                self.last_position_feedback = feedback[0]
        if self.node:
            self.node.write_socket_float(self.name, self.target_position)
        return True

    def print_target_position(self):
        current_timestamp = time.time()
        delta_time = current_timestamp - self.last_print_timestamp
        if self.print_orders and delta_time >= 0.5:
            formatted_position = f"{self.target_position:+.3f}"
            self.last_print_timestamp = current_timestamp
        return True

    def _manuever_advance_position_target(self):
        if self.trajectory_step_index < self.trajectory_size:
            self.target_position = self.trajectory[self.trajectory_step_index]
            self.trajectory_step_index += 1
            return True
        else:
            self.manuever_status = 1
            return False
        
    def set_torque_controll_target(self, target):
        self.target_torque = target

    def send_torque_controll_target(self):
        feedback = self.interface.send_torque_controll_command(self.target_torque)
        self.last_target_position = self.target_position
        if feedback is not False:
            if feedback[0]:
                self.last_position_feedback = feedback[0]
        if self.node:
            self.node.write_socket_float(self.name, self.target_position)
        return True

    def set_velocity_controll_target(self, target):
        self.target_velocity = target

    def send_velocity_controll_target(self):
        feedback = self.interface.send_velocity_controll_command(self.target_velocity, kd=1)
        self.last_target_position = self.target_position
        if feedback is not False:
            if feedback[0]:
                self.last_position_feedback = feedback[0]
        if self.node:
            self.node.write_socket_float(self.name, self.target_position)
        return True
    
    def set_position_velocity_controll_target(self, target_position, target_velocity=20000, target_accel=30000):
        self.target_position = target_position
        self.target_velocity = target_velocity
        self.target_accel = target_accel

    def send_position_velocity_controll_target(self):
        feedback = self.interface.send_position_velocity_command(self.target_position, self.aproximation_velocity, self.aproximation_acceleration)
        self.last_target_position = self.target_position
        if feedback:
            if feedback[0]:
                self.last_position_feedback = feedback[0]
                self.last_current_feedback = feedback[2]
                self.last_temperature_feedback = feedback[3]
        if self.node:
            self.node.write_socket_float(self.name, self.target_position)
        return True

    def set_current_break_controll_target(self, target_current_break=1):
        self.target_current_break = target_current_break

    def send_current_break_controll_target(self):
        feedback = self.interface.send_current_brake_command(self.target_current_break)
        self.last_target_position = self.target_position
        if feedback is not False:
            if feedback[0]:
                self.last_position_feedback = feedback[0]
        if self.node:
            self.node.write_socket_float(self.name, self.target_position)
        return True
    
    def set_origin_position(self):
        self.interface.set_origin()

    def manuever_iterate(self):
        if self.manuever_status == 0:
            return True
        elif self.manuever_status == 1:
            self.send_position_controll_target()
        elif self.manuever_status == 2:
            self._manuever_advance_position_target()
            self.send_position_controll_target()

    def define_motion_positions(self, absolute_destination, manuever_time, start_position = None,  start_manuever = True):
        if start_position is None:
            start_position = self.last_position_feedback
        positions = self.motion_planner.define_positions(start_position, absolute_destination, manuever_time)
        self.trajectory = positions
        self.trajectory_size = len(positions)
        self.trajectory_step_index = 0
        if start_manuever:
            self.manuever_status = 2
        return True