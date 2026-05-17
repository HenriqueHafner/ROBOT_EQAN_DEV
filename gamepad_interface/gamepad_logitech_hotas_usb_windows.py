import pygame
import time
import sys

def get_gamepad_interface_hotas(name_to_bind):
    pygame.init()
    pygame.joystick.init()
    device_count = pygame.joystick.get_count()
    device_index_to_bind = -1
    gamepad_hotas_stick_instance = None
    if device_count == 0:
        print("No devices detected.")
    for device_index in range(device_count):
        joystick_instance = pygame.joystick.Joystick(device_index)
        name = joystick_instance.get_name()
        guid = joystick_instance.get_guid()
        print(name)
        print(guid)
        if str(name_to_bind).lower() in str(name).lower():
            device_index_to_bind = device_index
            break
    if device_index_to_bind >= 0:
        joystick_instance.init()
        suffix = "s" if "stick" in str(name_to_bind).lower() else "t"
        gamepad_hotas_stick_instance = gamepad(device_index_to_bind, suffix)
        print("Binding to device number: ", device_index_to_bind)
    else:
        print("No matching device found with key: ", name_to_bind)
    return gamepad_hotas_stick_instance


class gamepad:
    def __init__(self, device_number, suffix):
        self.device_number = device_number
        self.suffix = suffix
        self.joystick_instance = pygame.joystick.Joystick(device_number)
        self.joystick_instance.init()
        self.axis_count = self.joystick_instance.get_numaxes()
        self.button_count = self.joystick_instance.get_numbuttons()
        self.hat_count = self.joystick_instance.get_numhats()

        self.state_name = self._create_state_name_list()
        self.state = [0] * 9
        self.counter_updates = 0
        self.online_satus = True
        self.update_state()

    def _create_state_name_list(self):
        names = [None] * 9
        names[0] = f"{self.suffix}_buttons"
        for i in range(1, 7):
            names[i] = f"{self.suffix}_axis_{i}"
        names[7] = f"{self.suffix}_pov"
        names[8] = f"{self.suffix}_package_counter"
        return names

    def update_state(self):
        pygame.event.pump()
        self.translate_state()
        self.counter_updates += 1

    def get_state(self):
        self.update_state()
        return self.state, self.online_satus

    def translate_state(self):
        buttons_bits_list = [self.joystick_instance.get_button(i) for i in range(self.button_count)]
        self.state[0] = buttons_bits_list
        axis_values = [self.joystick_instance.get_axis(i) for i in range(self.axis_count)]
        for i in range(len(axis_values)):
            self.state[i + 1] = axis_values[i]
        self.state[7] = -1
        self.state[8] = self.counter_updates

    def get_named_state(self):
        return [(self.state_name[i], self.state[i]) for i in range(len(self.state)-2)]

    def get_stringfied_state(self):
        stringfied_status = ""
        buttons_stringfied = ''.join(str(b) for b in self.state[0])
        stringfied_status += buttons_stringfied + " "
        for index in range(1, 7):
            axis_float = self.state[index]
            stringfied_status += self.format_trunc_4dec(axis_float) + " "
        stringfied_status += str(self.state[7])
        stringfied_status += ' ' + str(self.state[8])
        return stringfied_status

    def print_state(self, on_line_0 = False):
        if on_line_0:
            sys.stdout.write("\033[0;0H")
            sys.stdout.write("\033[K")
            sys.stdout.write("### hotas_stick: " + self.get_stringfied_state()+"\n")
            sys.stdout.flush()
        else:
            sys.stdout.write("\033[1A")
            sys.stdout.write("\033[K")
            sys.stdout.write("### hotas_stick: " + self.get_stringfied_state()+"\n")
            sys.stdout.flush()

    def format_trunc_4dec(self, float_value):
        truncated = int(abs(float_value) * 1000) / 1000
        formatted = f"{truncated:0.3f}"
        sign = '+' if float_value >= 0 else '-'
        return f"{sign}{formatted}"


if __name__ == "__main__":
    hotas_stick_instance = get_gamepad_interface_hotas("stick")
    if hotas_stick_instance:
        update_timestamp = time.monotonic()
        keep_loop = True
        while keep_loop:
            delta_time = time.monotonic() - update_timestamp
            if delta_time >= 0.017:
                update_timestamp = time.monotonic()
                hotas_stick_instance.update_state()
                hotas_stick_instance.print_state()