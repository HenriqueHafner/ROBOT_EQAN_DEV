import math

from motion_control import actuator_controller_tankbotics
from motion_control import tankbotics_cubemars_can_interface_emulator
from motion_control import tankbotics_cubemars_can_interface
from tankbotics_ros2_manager.intra_process_communication import node_client

class crawler_rpc_controller:
    maximum_position_in_radians = 35000.0
    maximum_electrical_rpm = 10000
    maximum_acceleration = 300000
    maximum_position_step_in_radians = math.radians(5.0)
    linear_gain = 1.0
    rotational_gain = 1.0
    maximum_offset_in_radians = math.radians(5.0)

    def __init__(self, use_emulator=True):
        self.use_emulator = use_emulator
        if self.use_emulator:
            print("CRAWLER_RPC_CONTROLLER: running in emulator mode")
        self.node_client = node_client.client()
        self.L_R_target_position_in_radians = 0.0
        self.L_F_target_position_in_radians = 0.0
        self.R_R_target_position_in_radians = 0.0
        self.R_F_target_position_in_radians = 0.0
        self.L_R_rest_offset_in_radians = 0.0
        self.L_F_rest_offset_in_radians = 0.0
        self.R_R_rest_offset_in_radians = 0.0
        self.R_F_rest_offset_in_radians = 0.0
        self.L_R_actuator = None
        self.L_F_actuator = None
        self.R_R_actuator = None
        self.R_F_actuator = None
        self.setup_all_actuators()

    def setup_all_actuators(self):
        self.L_R_actuator = self.create_and_configure_actuator(model_name="aka10", motor_id=1, orientation=1)
        self.L_F_actuator = self.create_and_configure_actuator(model_name="aka10", motor_id=2, orientation=1)
        self.R_R_actuator = self.create_and_configure_actuator(model_name="aka10", motor_id=3, orientation=-1)
        self.R_F_actuator = self.create_and_configure_actuator(model_name="aka10", motor_id=4, orientation=-1)

    def create_and_configure_actuator(self, model_name, motor_id, orientation):
        if self.use_emulator:
            interface = tankbotics_cubemars_can_interface_emulator.can_motor_interface_emulator(model_name, motor_id)
        else:
            interface = tankbotics_cubemars_can_interface.can_motor_interface(model_name, motor_id)
        actuator = actuator_controller_tankbotics.actuator_controller(model_name, motor_id, self.node_client)
        actuator.interface_set(interface)
        interface.orientation = orientation
        interface.enable_motor()
        interface.set_origin()
        actuator.set_resting_position()
        interface.proportional_gain = 50.0
        interface.derivative_gain = 3.0
        return actuator

    def enable_all_motors(self):
        self.L_R_actuator.interface.enable_motor()
        self.L_F_actuator.interface.enable_motor()
        self.R_R_actuator.interface.enable_motor()
        self.R_F_actuator.interface.enable_motor()

    def disable_all_motors(self):
        self.L_R_actuator.interface.disable_motor()
        self.L_F_actuator.interface.disable_motor()
        self.R_R_actuator.interface.disable_motor()
        self.R_F_actuator.interface.disable_motor()

    def set_differential_position_targets(self, linear_velocity, rotational_velocity):
        linear_component = linear_velocity * self.linear_gain
        rotational_component = rotational_velocity * self.rotational_gain
        left_value = linear_component + rotational_component
        right_value = linear_component - rotational_component
        maximum_value = max(abs(left_value), abs(right_value), 1.0)
        left_value = left_value / maximum_value
        right_value = right_value / maximum_value
        left_delta = left_value * self.maximum_position_step_in_radians
        right_delta = right_value * self.maximum_position_step_in_radians
        self.L_R_target_position_in_radians += left_delta
        self.L_F_target_position_in_radians += left_delta
        self.R_R_target_position_in_radians += right_delta
        self.R_F_target_position_in_radians += right_delta

    def send_position_targets(self):
        self.L_R_actuator.set_position_velocity_controll_target(self.L_R_target_position_in_radians, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.L_F_actuator.set_position_velocity_controll_target(self.L_F_target_position_in_radians, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.R_R_actuator.set_position_velocity_controll_target(self.R_R_target_position_in_radians, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.R_F_actuator.set_position_velocity_controll_target(self.R_F_target_position_in_radians, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.L_R_actuator.send_position_velocity_controll_target()
        self.L_F_actuator.send_position_velocity_controll_target()
        self.R_R_actuator.send_position_velocity_controll_target()
        self.R_F_actuator.send_position_velocity_controll_target()
        self.node_client.set_float("L_R_target_position", self.L_R_target_position_in_radians)
        self.node_client.set_float("L_F_target_position", self.L_F_target_position_in_radians)
        self.node_client.set_float("R_R_target_position", self.R_R_target_position_in_radians)
        self.node_client.set_float("R_F_target_position", self.R_F_target_position_in_radians)

    def modify_offset_hold_position(self, wheel_name, delta):
        if abs(delta) >= self.maximum_offset_in_radians:
            return
        if wheel_name == "left_rear":
            if abs(self.L_R_rest_offset_in_radians) >= self.maximum_offset_in_radians:
                return
            self.L_R_target_position_in_radians += delta
            self.L_R_rest_offset_in_radians += delta
        elif wheel_name == "left_front":
            if abs(self.L_F_rest_offset_in_radians) >= self.maximum_offset_in_radians:
                return
            self.L_F_target_position_in_radians += delta
            self.L_F_rest_offset_in_radians += delta
        elif wheel_name == "right_rear":
            if abs(self.R_R_rest_offset_in_radians) >= self.maximum_offset_in_radians:
                return
            self.R_R_target_position_in_radians += delta
            self.R_R_rest_offset_in_radians += delta
        elif wheel_name == "right_front":
            if abs(self.R_F_rest_offset_in_radians) >= self.maximum_offset_in_radians:
                return
            self.R_F_target_position_in_radians += delta
            self.R_F_rest_offset_in_radians += delta
        self.send_position_targets()
        self.node_client.set_float("L_R_rest_offset", self.L_R_rest_offset_in_radians)
        self.node_client.set_float("L_F_rest_offset", self.L_F_rest_offset_in_radians)
        self.node_client.set_float("R_R_rest_offset", self.R_R_rest_offset_in_radians)
        self.node_client.set_float("R_F_rest_offset", self.R_F_rest_offset_in_radians)