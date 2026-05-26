import time

from crawler_rpc_controller import crawler_rpc_controller
from tankbotics_ros2_manager.intra_process_communication.node_client import client

class mission_controller:
    def __init__(self, use_emulator=True):
        self.crawler = crawler_rpc_controller(use_emulator=use_emulator)
        self.node_client = client()
        self.current_state = "IDLE"
        self.is_running = True

    def run(self):
        self.crawler.enable_all_motors()
        print("MISSION_CONTROLLER started - 60Hz loop active")
        target_period = 1.0 / 60.0
        last_time = time.monotonic()
        try:
            while self.is_running:
                current_time = time.monotonic()
                delta = current_time - last_time
                if delta >= target_period:
                    self.iterate()
                    last_time = current_time
                time.sleep(0.001)
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        finally:
            self.shutdown()

    def iterate(self):
        if self.current_state == "ACTIVE":
            self.read_gamepad_and_set_velocity()
            self.crawler.send_position_targets()
            self.publish_telemetry()

    def axis_filter(self, raw_value):
        deadzone = 0.2
        if abs(raw_value) <= deadzone:
            return 0.0
        sign = 1.0 if raw_value > 0 else -1.0
        normalized = (abs(raw_value) - deadzone) / (1.0 - deadzone)
        return sign * normalized

    def read_gamepad_and_set_velocity(self):
        raw_linear = self.node_client.read_float("axis_name_1") or 0.0
        raw_rotational = self.node_client.read_float("axis_name_2") or 0.0
        linear_velocity = self.axis_filter(raw_linear)
        rotational_velocity = self.axis_filter(raw_rotational)
        self.crawler.set_differential_position_targets(linear_velocity, rotational_velocity)

    def publish_telemetry(self):
        wheels = {
            "L_R": self.crawler.L_R,
            "L_F": self.crawler.L_F,
            "R_R": self.crawler.R_R,
            "R_F": self.crawler.R_F
        }
        for name, wheel in wheels.items():
            self.node_client.set_float(f"{name}_target_position", wheel.target_position)
            self.node_client.set_float(f"{name}_position", wheel.last_position_feedback)
            self.node_client.set_float(f"{name}_rest_offset", wheel.rest_offset)

    def shutdown(self):
        self.is_running = False
        self.crawler.disable_all_motors()
        print("MISSION_CONTROLLER shutdown completed")


if __name__ == "__main__":
    mission = mission_controller(use_emulator=True)
    mission.run()