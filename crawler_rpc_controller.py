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
    class crawler_actuator:
        maximum_target_position = 8 * 2 * math.pi
        limit_position_to_move_origin = 8 * 2 * math.pi - math.pi
        target_position = 0.0
        rest_offset = 0.0
        driver_position = 0.0
        driver_target_position = 0.0
        virtual_origin_position = 0.0
        controller = object

    def __init__(self, use_emulator=True, node_client=None):
        self.use_emulator = use_emulator
        if self.use_emulator:
            print("CRAWLER_RPC_CONTROLLER: running in emulator mode")
        self.node_client = node_client
        self.L_R = self.crawler_actuator()
        self.L_F = self.crawler_actuator()
        self.R_R = self.crawler_actuator()
        self.R_F = self.crawler_actuator()
        self.actuators_list = [self.L_R, self.L_F, self.R_R, self.R_F] # type: list[crawler_rpc_controller.rawler_actuator]
        self.setup_all_actuators()

    def setup_all_actuators(self):
        self.L_R.controller = self.create_and_configure_actuator(model_name="aka10", motor_id=1, orientation=1)
        self.L_F.controller = self.create_and_configure_actuator(model_name="aka10", motor_id=2, orientation=1)
        self.R_R.controller = self.create_and_configure_actuator(model_name="aka10", motor_id=3, orientation=-1)
        self.R_F.controller = self.create_and_configure_actuator(model_name="aka10", motor_id=4, orientation=-1)

    def create_and_configure_actuator(self, model_name, motor_id, orientation):
        if self.use_emulator:
            interface = tankbotics_cubemars_can_interface_emulator.can_motor_interface_emulator(joint_model=model_name, motor_id=motor_id)
        else:
            interface = tankbotics_cubemars_can_interface.can_motor_interface(joint_model=model_name, motor_id=motor_id)
        controller = actuator_controller_tankbotics.actuator_controller(model_name=model_name, identifier=motor_id)
        controller.interface_set(interface)
        interface.orientation = orientation
        interface.enable_motor()
        interface.set_origin()
        controller.set_resting_position()
        interface.proportional_gain = 50.0
        interface.derivative_gain = 3.0
        return controller

    def enable_all_motors(self):
        self.L_R.controller.interface.enable_motor()
        self.L_F.controller.interface.enable_motor()
        self.R_R.controller.interface.enable_motor()
        self.R_F.controller.interface.enable_motor()

    def disable_all_motors(self):
        self.L_R.controller.interface.disable_motor()
        self.L_F.controller.interface.disable_motor()
        self.R_R.controller.interface.disable_motor()
        self.R_F.controller.interface.disable_motor()

    def set_differential_position_targets(self, linear_velocity, rotational_velocity):
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
        self.handle_cubemars_position_limitation()

    def handle_cubemars_position_limitation(self):
        #deve ser modificada para assegurar permanencia de target position delta
        #nao está funcionando
        return True
        for actuator in self.actuators_list:
            if abs(actuator.driver_target_position) > actuator.limit_position_to_move_origin:
                actuator.virtual_origin_position = actuator.driver_position
                self.driver_position = 0.0
                # reset origin here
            return True
        else:
            return False

    def send_position_targets(self):
        self.L_R.controller.set_position_velocity_controll_target(self.L_R.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.L_F.controller.set_position_velocity_controll_target(self.L_F.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.R_R.controller.set_position_velocity_controll_target(self.R_R.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)
        self.R_F.controller.set_position_velocity_controll_target(self.R_F.target_position, self.maximum_electrical_rpm, self.maximum_acceleration)
        feedback = self.L_R.controller.send_position_velocity_controll_target()
        self.L_R.driver_position = feedback
        feedback = self.L_F.controller.send_position_velocity_controll_target()
        self.L_F.driver_position = feedback
        feedback = self.R_R.controller.send_position_velocity_controll_target()
        self.R_R.driver_position = feedback
        feedback = self.R_F.controller.send_position_velocity_controll_target()
        self.R_F.driver_position = feedback
        if self.node_client is not None:
            self.node_client.set_float("L_R_target_position", self.L_R.target_position)
            self.node_client.set_float("L_F_target_position", self.L_F.target_position)
            self.node_client.set_float("R_R_target_position", self.R_R.target_position)
            self.node_client.set_float("R_F_target_position", self.R_F.target_position)


    def modify_offset_hold_position(self, wheel_name, delta):
        if abs(delta) >= self.maximum_offset:
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
            return
        if abs(wheel.rest_offset) >= self.maximum_offset:
            return
        wheel.target_position += delta
        wheel.rest_offset += delta
        self.send_position_targets()
        if self.node_client is not None:
            self.node_client.set_float("L_R_rest_offset", self.L_R.rest_offset)
            self.node_client.set_float("L_F_rest_offset", self.L_F.rest_offset)
            self.node_client.set_float("R_R_rest_offset", self.R_R.rest_offset)
            self.node_client.set_float("R_F_rest_offset", self.R_F.rest_offset)

if __name__ == "__main__":
    crawler_rpc_controller_instance = crawler_rpc_controller(use_emulator=True)
    crawler_rpc_controller_instance.set_differential_position_targets(1.2, 0.2)
    crawler_rpc_controller_instance.send_position_targets()
    input()