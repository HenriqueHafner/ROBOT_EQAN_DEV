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