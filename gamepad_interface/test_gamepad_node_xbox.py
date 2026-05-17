import time
import gamepad_node

LOOP_PERIOD = 0.035

if __name__ == "__main__":
    gamepad_name = "xbox360_wireless"
    gamepad_handler_instance = gamepad_node.gamepad_handler()
    gamepad_handler_instance.setup_controller_by_name(gamepad_name)
    gamepads = [gamepad_handler_instance]
    
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