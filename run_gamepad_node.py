import os
import sys
import time
from tankbotics_ros2_manager.gamepad_interface import gamepad_node
module_folder = os.path.dirname(gamepad_node.__file__)
sys.path.insert(0, module_folder)

gamepad_node = gamepad_node.gamepad_handler()
gamepad_node.setup_controller_by_name("xbox360_wireless")
# gamepad_node.setup_controller_by_name("logitech_hotas_stick")

try:
    while True:
        gamepad_node.loop_routine()
        time.sleep(0.04)
except Exception as e:
    print(e)
    input()
except KeyboardInterrupt:
    pass
