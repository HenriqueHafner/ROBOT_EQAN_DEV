# Gamepad Interface Package

## Objective

This package provides a **unified, clean and maintainable** interface for reading modern USB game controllers (Logitech HOTAS Stick/Throttle and Xbox 360 Wireless) on Windows and Linux.

It uses **pygame** (SDL2 backend) to ensure reliable real-time polling, avoiding the legacy limitations of WinMM/XInput.

The architecture is designed to work seamlessly with the **intra-process communication client** (`node_client`), allowing the gamepad data to be published as named variables (axes as floats, buttons as packed 3-byte data).

> **Important:**  
> An `intra_process_communication` server **must be running** before executing any gamepad client.  
> The server is part of the `tankbotics_ros2_manager` dependency.


## Example Usage
**test_gamepad_node_xbox.py**

```python
import time
import gamepad_node

LOOP_PERIOD = 0.035

if __name__ == "__main__":
    gamepad_name_1 = "gamepad_logitech_hotas_sidestick_usb_windows"
    gamepad_handler_instance_1 = gamepad_node.gamepad_handler()
    gamepad_handler_instance_1.setup_controller_by_name(gamepad_name_1)

    gamepad_name_2 = "gamepad_logitech_hotas_throttle_usb_windows"
    gamepad_handler_instance_2 = gamepad_node.gamepad_handler()
    gamepad_handler_instance_2.setup_controller_by_name(gamepad_name_2)
    gamepads = [gamepad_handler_instance_1, gamepad_handler_instance_2]

    
    period_delay_antecipation = LOOP_PERIOD / 20
    time_stamp = time.monotonic()

    while True:
        current_time = time.monotonic()
        delta_time = current_time - time_stamp
        if delta_time >= LOOP_PERIOD:
            for gamepad in gamepads:
                gamepad.loop_routine()
            time_stamp = current_time
        else:
            time.sleep(max(0.001, LOOP_PERIOD - delta_time - period_delay_antecipation))
```
If the controller is connected, the gamepad data will be published in server. It should be visible printed in the server's terminal.
## Data Published to Node

For each connected gamepad the following variables are published:

- `buttons` → 3-byte packed data
- `axis_i ` → normalized floats (-1.0 to 1.0)

---

