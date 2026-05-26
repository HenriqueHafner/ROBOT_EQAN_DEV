import math

from motion_control import actuator_controller_tankbotics
from motion_control import tankbotics_cubemars_can_interface_emulator
from motion_control import tankbotics_cubemars_can_interface

class crawler_rpc_controller:
    maximum_position = 35000.0
    maximum_electrical_rpm = 10000
    maximum_acceleration = 300000
    maximum_position_step = math.radians(5.0)
    linear_gain = 1.0
    rotational_gain = 1.0
    maximum_offset = math.radians(5.0)
    L_R_rest_offset = 0.0
    L_F_rest_offset = 0.0
    R_R_rest_offset = 0.0
    R_F_rest_offset = 0.0

    def __init__(self, use_emulator=True):
        self.use_emulator = use_emulator
        if self.use_emulator:
            print("CRAWLER_RPC_CONTROLLER: running in emulator mode")
        self.L_R = self.create_and_configure_actuator(model_name="aka10", motor_id=1, orientation=1)
        self.L_R.offset_hold_position = 0.0
        self.L_F = self.create_and_configure_actuator(model_name="aka10", motor_id=2, orientation=1)
        self.L_F.offset_hold_position = 0.0
        self.R_R = self.create_and_configure_actuator(model_name="aka10", motor_id=3, orientation=-1)
        self.R_R.offset_hold_position = 0.0
        self.R_F = self.create_and_configure_actuator(model_name="aka10", motor_id=4, orientation=-1)
        self.R_F.offset_hold_position = 0.0

    def create_and_configure_actuator(self, model_name, motor_id, orientation):
        if self.use_emulator:
            interface = tankbotics_cubemars_can_interface_emulator.can_motor_interface_emulator(joint_model=model_name, motor_id=motor_id)
        else:
            interface = tankbotics_cubemars_can_interface.can_motor_interface(joint_model=model_name, motor_id=motor_id)
        actuator = actuator_controller_tankbotics.actuator_controller(model_name=model_name, identifier=motor_id)
        actuator.interface_set(interface)
        interface.orientation = orientation
        interface.enable_motor()
        interface.set_origin()
        actuator.set_resting_position()
        interface.proportional_gain = 50.0
        interface.derivative_gain = 3.0
        return actuator

    def enable_all_motors(self):
        self.L_R.interface.enable_motor()
        self.L_F.interface.enable_motor()
        self.R_R.interface.enable_motor()
        self.R_F.interface.enable_motor()

    def disable_all_motors(self):
        self.L_R.interface.disable_motor()
        self.L_F.interface.disable_motor()
        self.R_R.interface.disable_motor()
        self.R_F.interface.disable_motor()

    def calculate_differential_position_targets(self, linear_velocity, rotational_velocity):
        linear_component = linear_velocity * self.linear_gain
        rotational_component = rotational_velocity * self.rotational_gain
        left_value = linear_component + rotational_component
        right_value = linear_component - rotational_component
        maximum_value = max(abs(left_value), abs(right_value), 1.0)
        left_value = left_value / maximum_value
        right_value = right_value / maximum_value
        left_delta = left_value * self.maximum_position_step
        right_delta = right_value * self.maximum_position_step
        self.L_R.target_position += left_delta
        self.L_F.target_position += left_delta
        self.R_R.target_position += right_delta
        self.R_F.target_position += right_delta

    def set_position_targets_in_controllers(self):
        self.L_R.set_position_velocity_controll_target(self.L_R.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.L_F.set_position_velocity_controll_target(self.L_F.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.R_R.set_position_velocity_controll_target(self.R_R.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.R_F.set_position_velocity_controll_target(self.R_F.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)

    def send_position_targets_to_interface(self):
        self.L_R.send_position_velocity_controll_target()
        self.L_F.send_position_velocity_controll_target()
        self.R_R.send_position_velocity_controll_target()
        self.R_F.send_position_velocity_controll_target()


    def modify_offset_hold_position(self, wheel_name, delta):
        if abs(delta) >= self.maximum_offset:
            print("Delta value exceeds maximum offset limit.")
            return
        if wheel_name == "left_rear":
            wheel = self.L_R
        elif wheel_name == "left_front":
            wheel = self.L_F
        elif wheel_name == "right_rear":
            wheel = self.R_R
        elif wheel_name == "right_front":
            wheel = self.R_F
        else:
            print("Invalid wheel name provided.")
            return
        if abs(wheel.offset_hold_position) >= self.maximum_offset:
            print("Current offset hold position exceeds maximum offset limit.")
            return
        wheel.target_position += delta
        wheel.offset_hold_position += delta
        self.set_position_targets_in_controllers()
        self.send_position_targets_to_interface()


if __name__ == "__main__":
    crawler_rpc_controller_instance = crawler_rpc_controller(use_emulator=True)
    crawler_rpc_controller_instance.calculate_differential_position_targets(1.2, 0.2)
    crawler_rpc_controller_instance.modify_offset_hold_position("left_rear", 0.5)
    crawler_rpc_controller_instance.set_position_targets_in_controllers()
    crawler_rpc_controller_instance.send_position_targets_to_interface()
    input()