import platform
import time

import sys
import os
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if os.path.exists(parent_dir) and os.path.isdir(parent_dir):
    if parent_dir not in sys.path:
        sys.path.append(parent_dir)
from intra_process_communication import node_client

node = node_client.client()


class gamepad_handler:
    def __init__(self):
        self.timestamp_bind_last = time.monotonic()
        self.timestamp_state_last = time.monotonic()
        self.joystick_online = False
        self.os_name = None
        self.os_version = None
        self.gamepad_instance = None
        self.gamepad_name = ""
        self.idle_print_time = 0.0
        self.print_status = False
        self.get_system_info()

    def get_system_info(self):
        self.os_name = str(platform.system()).lower()
        self.os_version = str(platform.version()).lower()
        print("Ready to bind joystick through", self.os_name, self.os_version, "kernel.")

    def loop_routine(self):
        if self.joystick_online and self.gamepad_instance:
            try:
                self.state, update_flag = self.gamepad_instance.get_state()
                if update_flag:
                    if self.print_status:
                        self.timestamp_state_last = time.monotonic()
                    if node:
                        for i, (name, value) in enumerate(self.gamepad_instance.get_named_state()):
                            if i == 0:  # posição 0 = sempre botões
                                byte_data = bools_to_3bytes(value)
                                node.send_server(name, byte_data)
                            else:
                                node.set_float(name, value)

                    return True
                else:
                    print("Joystick connection lost, forcing fail state.")
                    self.joystick_online = False
                    self.fail_safe_mode()
            except Exception as e:
                self.joystick_online = False
                self.gamepad_instance = None
                self.fail_safe_mode()
                print(f"{type(e).__name__}: {str(e)}")
                return False
        else:
            self.binding_handler()
        return False

    def setup_controller_by_name(self, gamepad_name=None):
        if gamepad_name:
            self.gamepad_name = str(gamepad_name).lower()

        gamepad_instance = None

        if self.gamepad_name == "xbox360_wireless":
            if self.os_name in ("windows", "linux"):
                import gamepad_xbox360_wireless
                gamepad_instance = gamepad_xbox360_wireless.get_gamepad_interface_xbox360()

        elif "logitech_hotas" in self.gamepad_name:
            name_to_bind = None
            if "stick" in self.gamepad_name:
                name_to_bind = "stick"
            elif "throttle" in self.gamepad_name:
                name_to_bind = "throttle"
            if self.os_name in ("windows", "linux"):
                print(self.os_name)
                import gamepad_logitech_hotas_usb_windows
                gamepad_instance = gamepad_logitech_hotas_usb_windows.get_gamepad_interface_hotas(name_to_bind)

        if gamepad_instance:
            self.gamepad_instance = gamepad_instance
            self.joystick_online = self.gamepad_instance.online_satus
            if self.joystick_online:
                print("joystick binded.")
                return True
            else:
                print("Failed to setup a online gamepad.")
        else:
            print("Unable to find a setup implemented for:", self.gamepad_name)

    def binding_handler(self):
        timestamp_bind_current = time.monotonic()
        delta_time = abs(timestamp_bind_current - self.timestamp_bind_last)
        if delta_time >= 0.5:
            self.setup_controller_by_name()
            self.timestamp_bind_last = timestamp_bind_current

    def fail_safe_mode(self):
        # Pendente implementação
        return True

def bools_to_3bytes(bools_list):
    if len(bools_list) > 24:
        raise ValueError("bools_list larger than 24 elements, unable to pack in to 3 bytes.")
    value = 0
    for i, bit in enumerate(bools_list):
        if bit:
            value |= 1 << i
    return value.to_bytes(3, byteorder='little')