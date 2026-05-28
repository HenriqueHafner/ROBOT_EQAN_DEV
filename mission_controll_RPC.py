import sys
import time

from crawler_rpc_controller import crawler_rpc_controller
from tankbotics_ros2_manager.intra_process_communication.node_client import client


args = sys.argv
if "-emulate_can" in args:
    should_use_emulator = True
else:
    should_use_emulator = False

#### implementar cache de leitura com timeout para evitar leituras bloqueantes e lentas do socket, e também para evitar leituras desnecessárias quando os dados não mudaram. O cache deve ser invalidado quando uma escrita for feita no servidor, ou após um timeout de validade dos dados. ####
#### atualizar dependencia de submodulo do node_client para a versão mais recente, que inclui o método async _send_packet, e atualizar o método send_server para usar esse método async, e também para invalidar o cache de leitura quando uma escrita for feita. ####

class mission_controller:
    def __init__(self, use_emulator_crawler=False):
        self.crawler = crawler_rpc_controller(use_emulator=use_emulator_crawler)
        self.node_client = client()
        self.current_state = 1
        self.keep_iterating = True

    def run(self):
        self.crawler.enable_all_motors()
        print("MISSION_CONTROLLER started - 60Hz loop active")
        target_period = 1.0 / 60.0
        last_time = time.monotonic()
        try:
            while self.keep_iterating:
                current_time = time.monotonic()
                delta = current_time - last_time
                if delta >= target_period:
                    self.iterate()
                    last_time = current_time
                    print(delta)
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        finally:
            self.shutdown()

    def iterate(self):
        if self.current_state == 1: # actuators active state
            self.read_gamepad_and_set_velocity()
            self.crawler.send_position_targets_to_interface()
            self.publish_actuators_status()
            if self.node_client.connected == False and self.node_client.connected_before == True:
                self.current_state = 0 # set to Idle state to avoid motions unexpected behavior
                print("Mission controll set to Idle dues to node_client.connected == False.")
        elif self.current_state == 0: # idle state
            self.crawler.send_position_targets_to_interface()
            self.publish_actuators_status()
            # Adicionar comando que mantém os motores desligados?
        elif self.current_state == 2: # emergencia
            self.keep_iterating = False
            self.crawler.disable_all_motors()
        
    def axis_filter(self, raw_value):
        deadzone = 0.2
        if abs(raw_value) <= deadzone:
            return 0.0
        sign = 1.0 if raw_value > 0 else -1.0
        normalized = (abs(raw_value) - deadzone) / (1.0 - deadzone)
        return sign * normalized

    def read_gamepad_and_set_velocity(self):
        # floats_from_server = self.node_client.read_multiple_floats(["s_axis_2", "s_axis_5"]) # x_axis_i for xbox controller
        floats_from_server = self.node_client.read_multiple_floats(["x_axis_1", "s_axis_2"]) # x_axis_i for xbox controller
        if floats_from_server[0] is None or floats_from_server[1] is None:
            return
        raw_linear = floats_from_server[0]
        raw_rotational = floats_from_server[1]
        linear_velocity = self.axis_filter(raw_linear)
        rotational_velocity = self.axis_filter(raw_rotational)
        self.crawler.calculate_differential_position_targets(linear_velocity, rotational_velocity)
        self.crawler.set_position_targets_in_controllers()

    def publish_actuators_status(self):
        for (actuator_instance, name) in self.crawler.acutators_named_list:
            self.node_client.set_float(f"{name}_target_position", actuator_instance.target_position)
            self.node_client.set_float(f"{name}_position", actuator_instance.last_position_feedback)
            self.node_client.set_float(f"{name}_rest_offset", actuator_instance.rest_offset)
            self.node_client.set_float(f"{name}_current", actuator_instance.last_current_feedback)
            self.node_client.set_float(f"{name}_temperature", actuator_instance.last_temperature_feedback)

    def shutdown(self):
        self.keep_iterating = False
        self.crawler.disable_all_motors()
        print("MISSION_CONTROLLER shutdown completed")


if __name__ == "__main__":
    mission = mission_controller(use_emulator_crawler = should_use_emulator)
    mission.run()