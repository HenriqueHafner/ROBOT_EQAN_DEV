import time
import threading
import math

from motion_control import actuator_controller_tankbotics
from motion_control import tankbotics_cubemars_can_interface
from intra_process_comumnication.intra_process_comunicator_client import client 

# Configuração de Teste
USE_DUMMY_CAN = False
if USE_DUMMY_CAN:
    print("Using dummy CAN interface for offline testing.")
    from motion_control import tankbotics_cubemars_can_interface_dummy as tankbotics_cubemars_can_interface
else:
    from motion_control import tankbotics_cubemars_can_interface

# ============================================================
# CONFIGURAÇÃO DE REDE
# ============================================================
# IP do Computador/Servidor central rodando o hub de variáveis TCP
IP_BASE_SERVER = '192.168.0.100' 

# ============================================================
# Differential Drive Controller (MANTIDO)
# ============================================================
class TankDifferentialController:
    def __init__(self):
        self.max_position_step_deg = 5.0
        self.linear_gain = 1.0
        self.angular_gain = 1.0
        self.left_delta_deg = 0.0
        self.right_delta_deg = 0.0
        self.last_good_gy = 0.0
        self.last_good_yaw = 0.0

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
# Hardware Layer (MANTIDO)
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
        self._setup_actuator("aka10", 1, kp=50.0, kd=3.0, orientation=1)
        self._setup_actuator("aka10", 2, kp=50.0, kd=3.0, orientation=1)
        self._setup_actuator("aka10", 3, kp=50.0, kd=3.0, orientation=-1)
        self._setup_actuator("aka10", 4, kp=50.0, kd=3.0, orientation=-1)

    def reset_motor_origin(self):
        print("Manual origin reset")
        self.left_position = 0.0
        self.right_position = 0.0
        for motor in self.actuators:
            motor.set_origin_position()
        time.sleep(0.3)

    def send_hold_position(self):
        # Mantém exatamente a posição atual
        for i in [0, 1]:
            self.actuators[i].set_position_velocity_controll_target(
                self.left_position, self.erpm_max, self.accel_max
            )
        for i in [2, 3]:
            self.actuators[i].set_position_velocity_controll_target(
                self.right_position, self.erpm_max, self.accel_max
            )

        for motor in self.actuators:
            motor.send_position_velocity_controll_target()

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
        
        self.actuators[0].set_position_velocity_controll_target(self.left_position, self.erpm_max, self.accel_max)
        self.actuators[1].set_position_velocity_controll_target(self.left_position, self.erpm_max, self.accel_max)
        self.actuators[2].set_position_velocity_controll_target(self.right_position, self.erpm_max, self.accel_max)
        self.actuators[3].set_position_velocity_controll_target(self.right_position, self.erpm_max, self.accel_max)

        for motor in self.actuators:
            motor.send_position_velocity_controll_target()

        # self.send_hold_position()

    def stop_all(self):
        self.send_hold_position()

    def shutdown_all(self):
        for actuator in self.actuators:
            actuator.interface.disable_motor()
        print("All actuators shut down.")
        self.actuators[0].interface.shutdown_bus()

# ============================================================
# Mission Controller 
# ============================================================

class MissionController:
    def __init__(self):
        self.state = "IDLE"
        self.tank_controller = None
        self.max_velocity = 1.0
        self.network_loss_timestamp = None

        # Cliente TCP Único
        self.tcp_node = client(IP_BASE_SERVER)

        self.last_iteration_timestamp = time.monotonic()
        self.target_period = 1.0 / 60.0 # 60Hz cravados para os motores
        self.running = True
        
        # ==========================================
        # BUFFER DE RAM (Comunicação entre as Threads)
        # ==========================================
        self.data_lock = threading.Lock()
        self.shared_gy = 0.0
        self.shared_yaw = 0.0
        self.shared_btn_0 = 0.0
        self.shared_btn_1 = 0.0
        self.shared_btn_5 = 0.0

        # --- NOVAS VARIÁVEIS DA CHAVE SELETORA ---
        self.shared_btn_33 = 0.0
        self.shared_btn_34 = 0.0
        self.shared_btn_35 = 0.0

        self.network_ok = False 

    def setup(self):
        self.tank_controller = TankMockupController()
        print("Setup completo. Iniciando Arquitetura Assíncrona TCP (Motores Desacoplados da Rede)...")

    # --------------------------------------------------------
    # THREAD EXCLUSIVA DE REDE (Roda 100% isolada dos motores)
    # --------------------------------------------------------
    def tcp_network_worker(self):
        # Esta thread faz leitura e envio usando o mesmo socket, 
        # garantindo que não haja colisão de portas.
        telemetry_counter = 0
        
        while self.running:
            # =======================================================
            # O PORTEIRO (Reconnect de 5s fora da via expressa)
            # =======================================================
            if not getattr(self.tcp_node, 'connected', False):
                with self.data_lock:
                    self.network_ok = False
                             
                print(f"\n[REDE] Sem conexão. Tentando reconectar...")
                time.sleep(0.5) # Dorme em paz por 5s para não estrangular a CPU
                
                try:
                    # Tenta recriar o socket de rede do zero
                    self.tcp_node = client(IP_BASE_SERVER) 
                    #self.state == "IDLE"
                except Exception:
                    pass
                continue # Pula de volta pro início do while, testando o porteiro de novo

            # =======================================================
            # SUA ABORDAGEM EXATA DE ALTA PERFORMANCE
            # =======================================================
            try:
                # 1. LEITURA (Prioridade Alta)
                gy_raw = self.tcp_node.read_float("axis_2s")
                yaw_raw = self.tcp_node.read_float("axis_4s")
                btn_0_raw = self.tcp_node.read_float("button_trigger")
                btn_1_raw = self.tcp_node.read_float("button_off")
                btn_5_raw = self.tcp_node.read_float("button_reset")
                
                # --- Lendo a chave seletora do Throttle ---
                btn_33_raw = self.tcp_node.read_float("button_33")
                btn_34_raw = self.tcp_node.read_float("button_34")
                btn_35_raw = self.tcp_node.read_float("button_35")

                # Salva na RAM de forma segura usando o Lock
                with self.data_lock:
                    self.shared_gy = gy_raw if gy_raw is not None else 0.0
                    self.shared_yaw = yaw_raw if yaw_raw is not None else 0.0
                    self.shared_btn_0 = btn_0_raw if btn_0_raw is not None else 0.0
                    self.shared_btn_1 = btn_1_raw if btn_1_raw is not None else 0.0
                    self.shared_btn_5 = btn_5_raw if btn_5_raw is not None else 0.0
                    
                    # --- Salvando a chave seletora ---
                    self.shared_btn_33 = btn_33_raw if btn_33_raw is not None else 0.0
                    self.shared_btn_34 = btn_34_raw if btn_34_raw is not None else 0.0
                    self.shared_btn_35 = btn_35_raw if btn_35_raw is not None else 0.0
                    
                    self.network_ok = (gy_raw is not None)

                    # --- O GATILHO DA RECONEXÃO ---
                    # Se veio None, a rede morreu silenciosamente. 
                    # Forçamos a variável da biblioteca para acionar o Porteiro de 5s lá em cima!
                    if gy_raw is None:
                        self.tcp_node.connected = False

                # 2. ESCRITA DE TELEMETRIA (Prioridade Baixa)
                # Envia telemetria apenas a cada 5 ciclos para economizar banda
                telemetry_counter += 1
                if telemetry_counter >= 5 and self.tank_controller:
                    status_val = 1.0 if self.state == "ACTIVE" else (0.0 if self.state == "IDLE" else -1.0)
                    self.tcp_node.set_float("L_pos", self.tank_controller.left_position)
                    self.tcp_node.set_float("R_pos", self.tank_controller.right_position)
                    self.tcp_node.set_float("status", status_val)
                    telemetry_counter = 0

                # Pequeno respiro para não saturar a porta TCP
                time.sleep(0.01) 

            except Exception as e:
                with self.data_lock:
                    self.network_ok = False
                
                # Se o socket der erro no sistema operacional (cabo solto), força a reconexão
                self.tcp_node.connected = False 
                time.sleep(0.5)

    # --------------------------------------------------------
    # LOOP PRINCIPAL: O CÉREBRO DOS MOTORES (Blindado a 60Hz)
    # --------------------------------------------------------

    def iterate(self):
        with self.data_lock:
            net_ok = self.network_ok
            gy = self.shared_gy
            yaw = self.shared_yaw
            btn_0 = self.shared_btn_0
            btn_1 = self.shared_btn_1
            btn_5 = self.shared_btn_5

            # --- Puxando a chave da RAM ---
            btn_33 = self.shared_btn_33
            btn_34 = self.shared_btn_34
            btn_35 = self.shared_btn_35

        # ==========================================
        # MÁQUINA DE ESTADOS DO ROBÔ
        # ==========================================

        # 1. NOVO ESTADO: Quarentena de Rede (RECONECT)
        if self.state == "RECONECT":
            # Mantém as rodas travadas fisicamente enquanto estiver aqui
            self.tank_controller.send_hold_position()
            
            # Se a rede voltar, ele NÃO volta a andar. Ele vai para o IDLE.
            if net_ok:
                print("\n[INFO] Rede reconectada! Modo IDLE ativado.")
                print("Pressione o GATILHO (btn_0) para retomar o movimento.")
                self.state = "IDLE"
            
            return # Trava a execução do loop aqui dentro.

        # 2. Estado de Espera (Aguardando Gatilho)
        if self.state == "IDLE":
            self.tank_controller.stop_all()
            
            # Só permite armar o robô se a rede estiver OK e o gatilho for puxado
            if btn_0 > 0.5 and net_ok: 
                self.state = "ACTIVE"
                print("\n[INFO] Robô ATIVO! Iniciando cinemática...")
            return

        # 3. Operação Normal
        if self.state == "ACTIVE":
            
            # O GATILHO DE QUEDA: Se perder a rede enquanto anda, foge para a quarentena
            if not net_ok:
                print("\n[ALERTA] Conexão TCP perdida! Entrando em modo RECONECT.")
                self.state = "RECONECT"
                return

            if btn_1 > 0.5:
                self.state = "TO_HOME"
                return

            if btn_5 > 0.5:
                self.tank_controller.reset_motor_origin()

            # ==========================================
            # SELETOR DE ACELERAÇÃO FÍSICA
            # ==========================================
            # Altera diretamente a variável dentro do Drive Controller
            if btn_33 > 0.5:
                self.tank_controller.drive_controller.max_position_step_deg = 2.0
            elif btn_34 > 0.5:
                self.tank_controller.drive_controller.max_position_step_deg = 4.0
            elif btn_35 > 0.5:
                self.tank_controller.drive_controller.max_position_step_deg = 6.0

            # Filtro de Deadzone (5%)
            deadzone = 0.05
            gy_safe = 0.0 if abs(gy) < deadzone else gy
            yaw_safe = 0.0 if abs(yaw) < deadzone else yaw

            # Cinemática
            linear_v = gy_safe * self.max_velocity
            angular_v = -yaw_safe * self.max_velocity * 0.5

            self.tank_controller.drive_controller.get_position_deg(linear_velocity=linear_v, angular_velocity=angular_v)
            self.tank_controller.parse_and_send_position_targets()
            
            print(f"\rMotores -> L: {self.tank_controller.left_position:+.1f}° | R: {self.tank_controller.right_position:+.1f}°".ljust(60), end="")

        # 4. Encerramento
        elif self.state == "TO_HOME":
            self.tank_controller.stop_all()
            self.state = "SHUTDOWN"

    def run(self, run_period_seconds=3600.0):
        # Inicia a Thread única de Rede
        t_network = threading.Thread(target=self.tcp_network_worker)
        t_network.start()

        try:
            start_time = time.monotonic()
            while (time.monotonic() - start_time < run_period_seconds and self.state != "SHUTDOWN"):
                current_time = time.monotonic()
                if current_time - self.last_iteration_timestamp >= self.target_period:
                    self.iterate()
                    self.last_iteration_timestamp = current_time
                else:
                    time.sleep(max(0.001, self.target_period - (current_time - self.last_iteration_timestamp)))

        except KeyboardInterrupt:
            print("\nParada de emergência.")
            self.tank_controller.stop_all()

        finally:
            self.running = False
            t_network.join()
            self.tank_controller.shutdown_all()

if __name__ == "__main__":
    mission = MissionController()
    mission.setup()
    mission.run()