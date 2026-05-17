import pygame # type: ignore
import time
import math

class GamepadConnectionError(Exception):
    """Custom exception for lost or failed gamepad connections."""
    pass

class Gamepad:
    def __init__(self, joystick_id=0, deadzone=0.1, max_velocity=0.25):
        pygame.init()
        pygame.joystick.init()

        self.deadzone = deadzone
        self.max_velocity = max_velocity
        self.joystick = None
        self.connected = False
        self.last_update_time = 0.0
        self.timeout_seconds = 2.0  # Max delay before triggering connection error

        # Internal states
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.gimbal_x = 0.0
        self.gimbal_y = 0.0

        # Try to connect
        self.connect(joystick_id)

    # ============================================================
    # Connection Management
    # ============================================================
    def connect(self, joystick_id):
        """Try to connect to a joystick device."""
        if pygame.joystick.get_count() == 0:
            raise GamepadConnectionError("No gamepad detected. Please connect a controller.")

        self.joystick = pygame.joystick.Joystick(joystick_id)
        self.joystick.init()
        self.connected = True

        self.num_buttons = self.joystick.get_numbuttons()
        self.num_axes = self.joystick.get_numaxes()

        self.toggle_states = [False] * self.num_buttons
        self.button_previous_states = [False] * self.num_buttons

        print(f"Gamepad connected: {self.joystick.get_name()}")

    def reconnect_if_needed(self):
        """Attempt reconnection if the controller is unplugged or inactive."""
        if not self.connected:
            try:
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    self.connect(0)
            except Exception as e:
                raise GamepadConnectionError(f"Reconnection failed: {e}")

    # ============================================================
    # Input Filtering
    # ============================================================
    def apply_deadzone(self, value):
        """Filters small joystick noise using a deadzone threshold."""
        if abs(value) < self.deadzone:
            return 0.0
        return round(value, 3)

    def normalize_axis(self, axis_value):
        """Keeps joystick axis within -1.0 to 1.0"""
        return max(-1.0, min(1.0, axis_value))

    # ============================================================
    # Update Cycle
    # ============================================================
    def update(self):
        """Updates the joystick state, handling connection safety."""
        try:
            pygame.event.pump()

            if pygame.joystick.get_count() == 0:
                self.connected = False
                raise GamepadConnectionError("Gamepad disconnected.")

            self.last_update_time = time.time()

            for i in range(self.num_buttons):
                current = self.joystick.get_button(i)
                if current and not self.button_previous_states[i]:
                    self.toggle_states[i] = not self.toggle_states[i]
                self.button_previous_states[i] = current

            # Update gimbals and velocities
            self.update_gimbals_and_velocity()

        except Exception as e:
            self.connected = False
            raise GamepadConnectionError(f"Gamepad update failed: {e}")

    # ============================================================
    # Gimbal and Velocity Logic
    # ============================================================
    def update_gimbals_and_velocity(self):
        """Reads joystick axes and computes velocity from direction and intensity."""
        if not self.connected or self.num_axes < 2:
            self.gimbal_x = 0.0
            self.gimbal_y = 0.0
            self.velocity_x = 0.0
            self.velocity_y = 0.0
            return

        raw_x = self.joystick.get_axis(0)
        raw_y = self.joystick.get_axis(1)

        # Apply deadzone and normalize
        self.gimbal_x = self.apply_deadzone(self.normalize_axis(raw_x))
        self.gimbal_y = self.apply_deadzone(self.normalize_axis(raw_y))

        # Convert joystick direction to velocity
        self.velocity_x = self.gimbal_x * self.max_velocity
        self.velocity_y = -self.gimbal_y * self.max_velocity  # Invert Y for natural up/down motion

    # ============================================================
    # Data Accessors
    # ============================================================
    def get_gimbals(self):
        """Returns gimbal values (axes) already processed by deadzone."""
        if not self.connected:
            return [0.0, 0.0]
        return [self.gimbal_x, self.gimbal_y]

    def get_velocity(self):
        """Returns current velocity vector computed from joystick direction."""
        if not self.connected:
            return (0.0, 0.0)
        return (self.velocity_x, self.velocity_y)

    def get_buttons(self):
        """Return a list of all button states."""
        if not self.connected:
            return [False] * self.num_buttons
        return [self.joystick.get_button(i) for i in range(self.num_buttons)]

    def get_button_state(self, index):
        """Return the state of a specific button."""
        if not self.connected:
            return False
        return self.joystick.get_button(index)

    def close(self):
        """Safely closes pygame and joystick resources."""
        pygame.quit()
        self.connected = False
        print("Gamepad closed safely.")


# ============================================================
# Simple test execution
# ============================================================
if __name__ == "__main__":
    try:
        gamepad = Gamepad(max_velocity=0.3)
        while True:
            gamepad.update()
            vx, vy = gamepad.get_velocity()
            buttons = gamepad.get_buttons()
            print(f"Gimbals: {gamepad.get_gimbals()} | Velocity: ({vx:.2f}, {vy:.2f}) | Buttons: {buttons}")
            time.sleep(0.1)
    except GamepadConnectionError as e:
        print(e)
    except KeyboardInterrupt:
        print("🧠 User interruption. Closing...")
    finally:
        if 'gamepad' in locals():
            gamepad.close()
