# de uma forma geral diversas funções fazem coisas demais.
# o script é grande por misturar funcionalidades diferentes.
# mission control apenas deve executar ordens de controle de missão, nunca lógica de controle ou processos que estão contidos em um componente.

# Um exemplo seria a orderm "correr" que executa uma rotina de um pacote menor que tem "calcular posição dos pés".
# O controle da missão sabe que deve correr mas nao precisa definir este comportamento, por outro lado o corredor sabe como mover os pés mas nao sabe se deve correr.

# Iterate não deve possuir lógica de controle, ele deve apenas chamar funções de comportamento de alto nível.

# tudo que é inerente ao cubemars deve estar dentro de tankbotics_cubemars_can_interface.py, como por exemplo o problema de limitação de encoder.

# TankDifferentialController TankMockupController tem a mesma função aparentemente.
# deve ser escrito em snake_case e ter seu próprio pacote crawler_rpc_controller.py, sendo importado aqui.

# time.sleep deve ser evitado.
# time.sleep chamado em alguma rotida dentro de iterate() é proibido.
# setup_actuators deve ser um processo de tankbotics_cubemars_can_interface.py

from collections import deque
import threading
import time

from motion_control import actuator_controller_tankbotics
from motion_control import tankbotics_cubemars_can_interface
from intra_process_comumnication.intra_process_comunicator_client import client 

# ============================================================
# CONFIGURAÇÃO DE TESTE
# ============================================================

USE_DUMMY_CAN = False

if USE_DUMMY_CAN:
    print("Using dummy CAN interface for offline testing.")
    from motion_control import tankbotics_cubemars_can_interface_emulator as tankbotics_cubemars_can_interface
else:
    from motion_control import tankbotics_cubemars_can_interface

# ============================================================
# CONFIGURAÇÃO DE REDE
# ============================================================

IP_BASE_SERVER = '192.168.0.100'

# ============================================================
# DIFFERENTIAL DRIVE CONTROLLER
# ============================================================

class TankDifferentialController:
    def __init__(self):
        self.max_position_step_deg = 5.0
        self.linear_gain = 1.0
        self.angular_gain = 1.0

        self.left_delta_deg = 0.0
        self.right_delta_deg = 0.0

    def get_position_deg(self, linear_velocity, angular_velocity):

        linear = linear_velocity * self.linear_gain
        angular = angular_velocity * self.angular_gain

        left = linear + angular
        right = linear - angular

        max_val = max(abs(left), abs(right), 1.0)

        left /= max_val
        right /= max_val

        self.left_delta_deg = left * self.max_position_step_deg
        self.right_delta_deg = right * self.max_position_step_deg

        return self.left_delta_deg, self.right_delta_deg

# ============================================================
# HARDWARE LAYER
# ============================================================

class TankMockupController:

    def __init__(self):

        self.actuators = []

        self.drive_controller = TankDifferentialController()

        self.max_position_deg = 35000.0

        self.erpm_max = 10000
        self.accel_max = 300000

        self.left_position = 0.0
        self.right_position = 0.0

        # ====================================================
        # TRIM DAS RODAS DA FRENTE
        # Motor 2 = frente esquerda
        # Motor 4 = frente direita
        # ====================================================

        self.front_left_trim_offset = 0.0
        self.front_right_trim_offset = 0.0

        self.trim_step_deg = 0.5
        self.max_trim_deg = 5.0

        self.setup_actuators()

    def _setup_actuator(self, model, motor_id, kp, kd, orientation=1):

        actuator = actuator_controller_tankbotics.actuator_controller(model, motor_id)

        interface = tankbotics_cubemars_can_interface.can_motor_interface(model, motor_id)

        actuator.interface_set(interface)

        actuator.interface.orientation = orientation

        actuator.interface.enable_motor()
        actuator.interface.set_origin()

        time.sleep(0.6)

        actuator.set_resting_position()

        actuator.interface.kp = kp
        actuator.interface.kd = kd

        self.actuators.append(actuator)

        print(f"Actuator {motor_id} ({model}) configured.")

        return actuator

    def setup_actuators(self):

        # LEFT REAR
        self._setup_actuator(
            "aka10",
            1,
            kp=50.0,
            kd=3.0,
            orientation=1
        )

        # LEFT FRONT
        self._setup_actuator(
            "aka10",
            2,
            kp=50.0,
            kd=3.0,
            orientation=1
        )

        # RIGHT REAR
        self._setup_actuator(
            "aka10",
            3,
            kp=50.0,
            kd=3.0,
            orientation=-1
        )

        # RIGHT FRONT
        self._setup_actuator(
            "aka10",
            4,
            kp=50.0,
            kd=3.0,
            orientation=-1
        )

    def reset_motor_origin(self):

        print("Manual origin reset")

        self.left_position = 0.0
        self.right_position = 0.0

        self.front_left_trim_offset = 0.0
        self.front_right_trim_offset = 0.0

        for motor in self.actuators:
            motor.set_origin_position()

        time.sleep(0.3)

    # ========================================================
    # TRIM CONTROL
    # ========================================================

    def add_front_left_trim(self, delta):

        new_trim = self.front_left_trim_offset + delta

        new_trim = max(
            -self.max_trim_deg,
            min(self.max_trim_deg, new_trim)
        )

        self.front_left_trim_offset = new_trim

        print(f"\n[TRIM] Front LEFT: {self.front_left_trim_offset:+.1f} deg")

    def add_front_right_trim(self, delta):

        new_trim = self.front_right_trim_offset + delta

        new_trim = max(
            -self.max_trim_deg,
            min(self.max_trim_deg, new_trim)
        )

        self.front_right_trim_offset = new_trim

        print(f"\n[TRIM] Front RIGHT: {self.front_right_trim_offset:+.1f} deg")

    # ========================================================
    # HOLD POSITION
    # ========================================================

    def send_hold_position(self):

        left_rear_target = self.left_position
        left_front_target = self.left_position + self.front_left_trim_offset

        right_rear_target = self.right_position
        right_front_target = self.right_position + self.front_right_trim_offset

        # LEFT REAR -> MOTOR 1
        self.actuators[0].set_position_velocity_controll_target(
            left_rear_target,
            self.erpm_max,
            self.accel_max
        )

        # LEFT FRONT -> MOTOR 2
        self.actuators[1].set_position_velocity_controll_target(
            left_front_target,
            self.erpm_max,
            self.accel_max
        )

        # RIGHT REAR -> MOTOR 3
        self.actuators[2].set_position_velocity_controll_target(
            right_rear_target,
            self.erpm_max,
            self.accel_max
        )

        # RIGHT FRONT -> MOTOR 4
        self.actuators[3].set_position_velocity_controll_target(
            right_front_target,
            self.erpm_max,
            self.accel_max
        )

        for motor in self.actuators:
            motor.send_position_velocity_controll_target()

    # ========================================================
    # MAIN MOTOR TARGETS
    # ========================================================

    def parse_and_send_position_targets(self):

        d_left = self.drive_controller.left_delta_deg
        d_right = self.drive_controller.right_delta_deg

        left_next = self.left_position + d_left
        right_next = self.right_position + d_right

        left_limit = abs(left_next) > self.max_position_deg
        right_limit = abs(right_next) > self.max_position_deg

        if left_limit or right_limit:

            print("Position limit reached — blocking movement")

            left_next = self.left_position
            right_next = self.right_position

        self.left_position = left_next
        self.right_position = right_next

        # ====================================================
        # TARGETS
        # ====================================================

        left_rear_target = self.left_position
        left_front_target = self.left_position + self.front_left_trim_offset

        right_rear_target = self.right_position
        right_front_target = self.right_position + self.front_right_trim_offset

        # MOTOR 1 -> LEFT REAR
        self.actuators[0].set_position_velocity_controll_target(
            left_rear_target,
            self.erpm_max,
            self.accel_max
        )

        # MOTOR 2 -> LEFT FRONT
        self.actuators[1].set_position_velocity_controll_target(
            left_front_target,
            self.erpm_max,
            self.accel_max
        )

        # MOTOR 3 -> RIGHT REAR
        self.actuators[2].set_position_velocity_controll_target(
            right_rear_target,
            self.erpm_max,
            self.accel_max
        )

        # MOTOR 4 -> RIGHT FRONT
        self.actuators[3].set_position_velocity_controll_target(
            right_front_target,
            self.erpm_max,
            self.accel_max
        )

        for motor in self.actuators:
            motor.send_position_velocity_controll_target()

    # ========================================================
    # STOP
    # ========================================================

    def stop_all(self):
        self.send_hold_position()

    def shutdown_all(self):

        for actuator in self.actuators:
            actuator.interface.disable_motor()

        print("All actuators shut down.")

        self.actuators[0].interface.shutdown_bus()

    
# ============================================================
# MISSION CONTROLLER
# ============================================================

class MissionController:

    def __init__(self):

        self.state = "IDLE"

        self.tank_controller = None

        self.max_velocity = 1.0

        self.tcp_node = client(IP_BASE_SERVER)

        self.last_iteration_timestamp = time.monotonic()

        self.target_period = 1.0 / 60.0

        self.running = True

        self.plot_max_len = 300 
        self.plot_time = deque(maxlen=self.plot_max_len) 
        self.plot_currents = [deque(maxlen=self.plot_max_len) for _ in range(4)] 
        self.start_time_real = time.time()
        self.tank_controller = None

        # ====================================================
        # SHARED MEMORY
        # ====================================================

        self.data_lock = threading.Lock()

        self.shared_gy = 0.0
        self.shared_yaw = 0.0

        self.shared_btn_0 = 0.0
        self.shared_btn_1 = 0.0
        self.shared_btn_5 = 0.0

        self.shared_btn_33 = 0.0
        self.shared_btn_34 = 0.0
        self.shared_btn_35 = 0.0

        # ====================================================
        # NEW TRIM BUTTONS
        # ====================================================

        self.shared_btn_05 = 0.0
        self.shared_btn_06 = 0.0
        self.shared_btn_07 = 0.0
        self.shared_btn_08 = 0.0

        self.prev_btn_05 = 0.0
        self.prev_btn_06 = 0.0
        self.prev_btn_07 = 0.0
        self.prev_btn_08 = 0.0

        self.network_ok = False

    # ========================================================
    # SETUP
    # ========================================================

    def setup(self):

        self.tank_controller = TankMockupController()

        print("Setup completo.")

    # ========================================================
    # TCP THREAD
    # ========================================================

    def tcp_network_worker(self):

        telemetry_counter = 0

        while self.running:

            if not getattr(self.tcp_node, 'connected', False):

                with self.data_lock:
                    self.network_ok = False

                print("\n[REDE] Sem conexão. Tentando reconectar...")

                time.sleep(0.5)

                try:
                    self.tcp_node = client(IP_BASE_SERVER)
                except Exception:
                    pass

                continue

            try:

                gy_raw = self.tcp_node.read_float("axis_2s")
                yaw_raw = self.tcp_node.read_float("axis_4s")

                btn_0_raw = self.tcp_node.read_float("button_trigger")
                btn_1_raw = self.tcp_node.read_float("button_off")
                btn_5_raw = self.tcp_node.read_float("button_reset")

                btn_33_raw = self.tcp_node.read_float("button_33")
                btn_34_raw = self.tcp_node.read_float("button_34")
                btn_35_raw = self.tcp_node.read_float("button_35")

                # ============================================
                # NEW TRIM BUTTONS
                # ============================================

                btn_05_raw = self.tcp_node.read_float("button_05")
                btn_06_raw = self.tcp_node.read_float("button_06")
                btn_07_raw = self.tcp_node.read_float("button_07")
                btn_08_raw = self.tcp_node.read_float("button_08")

                with self.data_lock:

                    self.shared_gy = gy_raw if gy_raw is not None else 0.0
                    self.shared_yaw = yaw_raw if yaw_raw is not None else 0.0

                    self.shared_btn_0 = btn_0_raw if btn_0_raw is not None else 0.0
                    self.shared_btn_1 = btn_1_raw if btn_1_raw is not None else 0.0
                    self.shared_btn_5 = btn_5_raw if btn_5_raw is not None else 0.0

                    self.shared_btn_33 = btn_33_raw if btn_33_raw is not None else 0.0
                    self.shared_btn_34 = btn_34_raw if btn_34_raw is not None else 0.0
                    self.shared_btn_35 = btn_35_raw if btn_35_raw is not None else 0.0

                    self.shared_btn_05 = btn_05_raw if btn_05_raw is not None else 0.0
                    self.shared_btn_06 = btn_06_raw if btn_06_raw is not None else 0.0
                    self.shared_btn_07 = btn_07_raw if btn_07_raw is not None else 0.0
                    self.shared_btn_08 = btn_08_raw if btn_08_raw is not None else 0.0

                    self.network_ok = (gy_raw is not None)

                    if gy_raw is None:
                        self.tcp_node.connected = False

                telemetry_counter += 1

                if telemetry_counter >= 5 and self.tank_controller:

                    status_val = (
                        1.0 if self.state == "ACTIVE"
                        else (0.0 if self.state == "IDLE" else -1.0)
                    )

                    # --- TELEMETRIA BÁSICA ORIGINAL ---
                    self.tcp_node.set_float("L_pos", self.tank_controller.left_position)
                    self.tcp_node.set_float("R_pos", self.tank_controller.right_position)
                    self.tcp_node.set_float("status", status_val)

                    # --- TELEMETRIA AVANÇADA (Os 4 Motores) ---
                    if len(self.tank_controller.actuators) >= 4:
                        
                        # Função auxiliar para extrair dados sem quebrar (getattr)
                        def get_motor_data(idx):
                            m = self.tank_controller.actuators[idx]
                            return (
                                getattr(m, 'last_current_feedback', 0.0),
                                getattr(m, 'last_position_feedback', 0.0),
                                getattr(m, 'last_velocity_feedback', 0.0),
                                getattr(m, 'last_temperature_feedback', 0.0)
                            )
                        
                        # Varre os 4 motores (índices 0 a 3)
                        for i in range(4):
                            c, p, v, t = get_motor_data(i)
                            
                            # Gera o prefixo dinamicamente (M1, M2, M3, M4)
                            prefix = f"M{i+1}" 
                            
                            self.tcp_node.set_float(f"{prefix}_curr", c)
                            self.tcp_node.set_float(f"{prefix}_pos", p)
                            self.tcp_node.set_float(f"{prefix}_vel", v)
                            self.tcp_node.set_float(f"{prefix}_temp", t)

                    telemetry_counter = 0

                time.sleep(0.01)

            except Exception:

                with self.data_lock:
                    self.network_ok = False

                self.tcp_node.connected = False

                time.sleep(0.5)

    # ========================================================
    # MAIN LOOP
    # ========================================================

    def iterate(self):

        with self.data_lock:

            net_ok = self.network_ok

            gy = self.shared_gy
            yaw = self.shared_yaw

            btn_0 = self.shared_btn_0
            btn_1 = self.shared_btn_1
            btn_5 = self.shared_btn_5

            btn_33 = self.shared_btn_33
            btn_34 = self.shared_btn_34
            btn_35 = self.shared_btn_35

            btn_05 = self.shared_btn_05
            btn_06 = self.shared_btn_06
            btn_07 = self.shared_btn_07
            btn_08 = self.shared_btn_08

        # ====================================================
        # RECONECT
        # ====================================================
        if self.state == "RECONECT":

            self.tank_controller.send_hold_position()

            if net_ok:

                print("\n[INFO] Rede reconectada!")
                print("Pressione o gatilho para rearmar.")

                self.state = "IDLE"

            return

        # ====================================================
        # IDLE
        # ====================================================

        if self.state == "IDLE":

            self.tank_controller.stop_all()

            if btn_0 > 0.5 and net_ok:

                self.state = "ACTIVE"

                print("\n[INFO] Robô ATIVO!")

            return

        # ====================================================
        # ACTIVE
        # ====================================================

        if self.state == "ACTIVE":

            if not net_ok:

                print("\n[ALERTA] Rede perdida!")

                self.state = "RECONECT"

                return

            if btn_1 > 0.5:

                self.state = "TO_HOME"

                return

            if btn_5 > 0.5:
                self.tank_controller.reset_motor_origin()

            # =================================================
            # SPEED SELECTOR
            # =================================================

            if btn_33 > 0.5:
                self.tank_controller.drive_controller.max_position_step_deg = 2.0

            elif btn_34 > 0.5:
                self.tank_controller.drive_controller.max_position_step_deg = 4.0

            elif btn_35 > 0.5:
                self.tank_controller.drive_controller.max_position_step_deg = 6.0

            # =================================================
            # FRONT WHEEL TRIM
            # =================================================

            trim_step = self.tank_controller.trim_step_deg

            # LEFT FRONT +
            if btn_05 > 0.5 and self.prev_btn_05 <= 0.5:
                self.tank_controller.add_front_left_trim(+trim_step)

            # LEFT FRONT -
            if btn_06 > 0.5 and self.prev_btn_06 <= 0.5:
                self.tank_controller.add_front_left_trim(-trim_step)

            # RIGHT FRONT +
            if btn_07 > 0.5 and self.prev_btn_07 <= 0.5:
                self.tank_controller.add_front_right_trim(+trim_step)

            # RIGHT FRONT -
            if btn_08 > 0.5 and self.prev_btn_08 <= 0.5:
                self.tank_controller.add_front_right_trim(-trim_step)

            self.prev_btn_05 = btn_05
            self.prev_btn_06 = btn_06
            self.prev_btn_07 = btn_07
            self.prev_btn_08 = btn_08

            # =================================================
            # DEADZONE
            # =================================================

            deadzone = 0.1

            gy_safe = 0.0 if abs(gy) < deadzone else gy
            yaw_safe = 0.0 if abs(yaw) < deadzone else yaw

            # =================================================
            # KINEMATICS
            # =================================================

            linear_v = gy_safe * self.max_velocity

            angular_v = -yaw_safe * self.max_velocity * 0.5

            self.tank_controller.drive_controller.get_position_deg(
                linear_velocity=linear_v,
                angular_velocity=angular_v
            )

            self.tank_controller.parse_and_send_position_targets()

            print(
                f"\rL: {self.tank_controller.left_position:+.1f}° "
                f"R: {self.tank_controller.right_position:+.1f}° "
                f"FL Trim: {self.tank_controller.front_left_trim_offset:+.1f}° "
                f"FR Trim: {self.tank_controller.front_right_trim_offset:+.1f}°"
                .ljust(100),
                end=""
            )

        # ====================================================
        # SHUTDOWN
        # ====================================================

        elif self.state == "TO_HOME":

            self.tank_controller.stop_all()

            self.state = "SHUTDOWN"

    # ========================================================
    # RUN
    # ========================================================

    def run(self, run_period_seconds=3600.0):

        t_network = threading.Thread(
            target=self.tcp_network_worker
        )

        t_network.start()

        try:

            start_time = time.monotonic()

            while (
                time.monotonic() - start_time < run_period_seconds
                and self.state != "SHUTDOWN"
            ):

                current_time = time.monotonic()

                if (
                    current_time - self.last_iteration_timestamp
                    >= self.target_period
                ):

                    self.iterate()

                    self.last_iteration_timestamp = current_time

                else:

                    time.sleep(
                        max(
                            0.001,
                            self.target_period -
                            (
                                current_time -
                                self.last_iteration_timestamp
                            )
                        )
                    )

        except KeyboardInterrupt:

            print("\nParada de emergência.")

            self.tank_controller.stop_all()

        finally:

            self.running = False

            t_network.join()

            self.tank_controller.shutdown_all()

# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":

    mission = MissionController() 
    mission.setup()

    # Inicia a thread do robô
    robot_thread = threading.Thread(target=mission.run)
    robot_thread.start()

    print("\nIniciando interface gráfica...")
    print(">>> Pressione CTRL+C no terminal OU feche a janela do gráfico para sair <<<\n")
    


    

   