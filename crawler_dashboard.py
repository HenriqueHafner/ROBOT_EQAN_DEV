import time
import threading
import math
import random
from collections import deque
import pandas as pd

import customtkinter as ctk
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

try:
    from intra_process_comumnication.intra_process_comunicator_client import client
except ImportError:
    client = None

# ==========================================
# CONFIGURAÇÕES
# ==========================================
SIMULATION_MODE = True
IP_BASE_SERVER = '192.168.0.236'
FREQ_HZ = 40
HISTORY_SECONDS = 30
MAX_POINTS = FREQ_HZ * HISTORY_SECONDS

# --- THRESHOLDS DE ALERTA ---
# Ajuste esses valores conforme as especificações dos seus motores
CURRENT_WARN  = 20.0   # [A] — amarelo: atenção
CURRENT_CRIT  = 30.0   # [A] — vermelho: risco de queimar encoder
TEMP_WARN     = 45.0  # [°C] — amarelo
TEMP_CRIT     = 58.0  # [°C] — vermelho: dano ao encoder

# Torque simulado (substitua pelas leituras reais do seu robô, ex: via corrente × Kt)
KT = 0.35  # [Nm/A] — constante de torque dos motores (ajuste para o seu modelo)

# ==========================================
# HELPERS
# ==========================================
def get_severity(value, warn, crit):
    """Retorna 'ok', 'warn' ou 'crit' com base nos thresholds."""
    if value >= crit:
        return 'crit'
    if value >= warn:
        return 'warn'
    return 'ok'

SEVERITY_COLOR = {
    'ok':   '#44ff44',
    'warn': '#ffaa00',
    'crit': '#ff3333',
}

SEVERITY_LABEL = {
    'ok':   'OK',
    'warn': 'ATENÇÃO',
    'crit': 'CRÍTICO',
}

# ==========================================
# VISUAL
# ==========================================
ctk.set_appearance_mode("dark")
ctk.set_default_color_theme("green")

MOTOR_COLORS = ['#ff4444', '#44ff44', '#4488ff', '#ffaa00']


class MotorStatusCard(ctk.CTkFrame):
    """Card compacto com indicador de saúde para um motor individual."""

    def __init__(self, master, motor_name, **kwargs):
        super().__init__(master, corner_radius=8, **kwargs)
        self.motor_name = motor_name

        self.lbl_name = ctk.CTkLabel(self, text=motor_name, font=("Roboto", 13, "bold"))
        self.lbl_name.grid(row=0, column=0, columnspan=2, padx=8, pady=(6, 2))

        ctk.CTkLabel(self, text="I:", font=("Roboto", 11)).grid(row=1, column=0, padx=(8, 2), sticky="e")
        self.lbl_current = ctk.CTkLabel(self, text="--", font=("Roboto", 11))
        self.lbl_current.grid(row=1, column=1, padx=(0, 8), sticky="w")

        ctk.CTkLabel(self, text="T:", font=("Roboto", 11)).grid(row=2, column=0, padx=(8, 2), sticky="e")
        self.lbl_temp = ctk.CTkLabel(self, text="--", font=("Roboto", 11))
        self.lbl_temp.grid(row=2, column=1, padx=(0, 8), sticky="w")

        ctk.CTkLabel(self, text="τ:", font=("Roboto", 11)).grid(row=3, column=0, padx=(8, 2), sticky="e")
        self.lbl_torque = ctk.CTkLabel(self, text="--", font=("Roboto", 11))
        self.lbl_torque.grid(row=3, column=1, padx=(0, 8), sticky="w")

        self.lbl_status = ctk.CTkLabel(
            self, text="●  OK",
            font=("Roboto", 12, "bold"),
            text_color="#44ff44",
        )
        self.lbl_status.grid(row=4, column=0, columnspan=2, pady=(4, 6))

    def update(self, current, temp, torque):
        sev_c = get_severity(current, CURRENT_WARN, CURRENT_CRIT)
        sev_t = get_severity(temp,    TEMP_WARN,    TEMP_CRIT)
        worst = 'crit' if 'crit' in (sev_c, sev_t) else ('warn' if 'warn' in (sev_c, sev_t) else 'ok')

        self.lbl_current.configure(text=f"{current:5.2f} A", text_color=SEVERITY_COLOR[sev_c])
        self.lbl_temp.configure(   text=f"{temp:5.1f} °C", text_color=SEVERITY_COLOR[sev_t])
        self.lbl_torque.configure( text=f"{torque:5.3f} Nm")
        self.lbl_status.configure(
            text=f"●  {SEVERITY_LABEL[worst]}",
            text_color=SEVERITY_COLOR[worst],
        )
        return worst


class GroundStationApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("Ground Station — Crawler Telemetry v2")
        self.geometry("1400x900")

        self.running   = True
        self.start_time = time.time()
        self._lock     = threading.Lock()  # protege os buffers compartilhados

        # Posições das esteiras
        self.l_pos = 0.0
        self.r_pos = 0.0

        # Buffers de dados
        self.plot_time         = deque(maxlen=MAX_POINTS)
        self.plot_currents     = [deque(maxlen=MAX_POINTS) for _ in range(4)]
        self.plot_temperatures = [deque(maxlen=MAX_POINTS) for _ in range(4)]
        self.plot_torques      = [deque(maxlen=MAX_POINTS) for _ in range(4)]

        # Log de eventos (timestamp, mensagem)
        self.event_log = []

        # Estado de alerta anterior (para não logar o mesmo evento repetido)
        self._prev_severity = ['ok'] * 4

        self.setup_ui()

        self.net_thread = threading.Thread(target=self.network_worker, daemon=True)
        self.net_thread.start()

        self.update_gui()

    # ==========================================
    # 1. INTERFACE
    # ==========================================
    def setup_ui(self):
        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0, weight=1)

        # --- PAINEL SUPERIOR ---
        top = ctk.CTkFrame(self, height=55, corner_radius=10)
        top.grid(row=0, column=0, padx=20, pady=(20, 8), sticky="ew")
        top.grid_columnconfigure(1, weight=1)

        self.lbl_status = ctk.CTkLabel(
            top,
            text="AGUARDANDO DADOS...",
            font=("Roboto", 18, "bold"),
        )
        self.lbl_status.grid(row=0, column=1, pady=14)

        # Indicador global (semáforo)
        self.lbl_global_indicator = ctk.CTkLabel(top, text="●", font=("Roboto", 28, "bold"), text_color="#44ff44")
        self.lbl_global_indicator.grid(row=0, column=0, padx=(16, 0))

        # --- ÁREA CENTRAL: cards de motor + gráficos ---
        center = ctk.CTkFrame(self, corner_radius=0, fg_color="transparent")
        center.grid(row=1, column=0, padx=0, pady=0, sticky="nsew")
        center.grid_rowconfigure(0, weight=1)
        center.grid_columnconfigure(1, weight=1)

        # Coluna esquerda: cards dos 4 motores + log de eventos
        left_panel = ctk.CTkFrame(center, width=180, corner_radius=10)
        left_panel.grid(row=0, column=0, padx=(20, 8), pady=8, sticky="ns")

        ctk.CTkLabel(left_panel, text="STATUS DOS MOTORES", font=("Roboto", 12, "bold")).pack(pady=(10, 4))

        self.motor_cards = []
        for i in range(4):
            card = MotorStatusCard(left_panel, f"Motor M{i+1}")
            card.pack(padx=10, pady=4, fill="x")
            self.motor_cards.append(card)

        # --- Log de Eventos ---
        ctk.CTkLabel(left_panel, text="LOG DE EVENTOS", font=("Roboto", 11, "bold")).pack(pady=(14, 2))
        self.log_box = ctk.CTkTextbox(left_panel, height=180, font=("Courier", 10), state="disabled")
        self.log_box.pack(padx=10, pady=(0, 10), fill="both", expand=True)

        # Coluna direita: gráficos
        graph_frame = ctk.CTkFrame(center, corner_radius=10)
        graph_frame.grid(row=0, column=1, padx=(0, 20), pady=8, sticky="nsew")

        plt.style.use('dark_background')
        self.fig, self.axs = plt.subplots(3, 2, figsize=(11, 8), facecolor='#2b2b2b')
        self.fig.subplots_adjust(hspace=0.42, wspace=0.22, bottom=0.06, top=0.94, left=0.07, right=0.97)

        panel_info = [
            (0, 0, "Corrente M1/M2 (A)",    "A"),
            (0, 1, "Temperatura M1/M2 (°C)", "°C"),
            (1, 0, "Corrente M3/M4 (A)",    "A"),
            (1, 1, "Temperatura M3/M4 (°C)", "°C"),
            (2, 0, "Torque M1/M2 (Nm)",     "Nm"),
            (2, 1, "Torque M3/M4 (Nm)",     "Nm"),
        ]

        for r, c, title, unit in panel_info:
            ax = self.axs[r, c]
            ax.set_title(title, color='white', weight='bold', fontsize=10)
            ax.set_facecolor('#1e1e1e')
            ax.grid(True, alpha=0.15, color='#555555')
            ax.tick_params(colors='#aaaaaa', labelsize=8)
            ax.set_ylabel(unit, color='#aaaaaa', fontsize=8)

        # Linhas dos dados
        line_config = [
            ('c1',  0, 0, MOTOR_COLORS[0], "M1"),
            ('c2',  0, 0, MOTOR_COLORS[1], "M2"),
            ('t1',  0, 1, MOTOR_COLORS[0], "M1"),
            ('t2',  0, 1, MOTOR_COLORS[1], "M2"),
            ('c3',  1, 0, MOTOR_COLORS[2], "M3"),
            ('c4',  1, 0, MOTOR_COLORS[3], "M4"),
            ('t3',  1, 1, MOTOR_COLORS[2], "M3"),
            ('t4',  1, 1, MOTOR_COLORS[3], "M4"),
            ('tq1', 2, 0, MOTOR_COLORS[0], "M1"),
            ('tq2', 2, 0, MOTOR_COLORS[1], "M2"),
            ('tq3', 2, 1, MOTOR_COLORS[2], "M3"),
            ('tq4', 2, 1, MOTOR_COLORS[3], "M4"),
        ]

        self.lines = {}
        for key, r, c, color, label in line_config:
            self.lines[key] = self.axs[r, c].plot([], [], color=color, label=label, lw=1.4)[0]

        for ax in self.axs.flat:
            ax.legend(loc="upper left", fontsize=7, framealpha=0.5)

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, padx=8, pady=8)

        # --- PAINEL INFERIOR ---
        bottom = ctk.CTkFrame(self, height=50, corner_radius=10, fg_color="transparent")
        bottom.grid(row=2, column=0, padx=20, pady=(4, 16), sticky="ew")

        self.btn_export = ctk.CTkButton(
            bottom, text="⬇  Exportar XLSX",
            font=("Roboto", 13, "bold"),
            command=self.save_excel,
        )
        self.btn_export.pack(side="right")

        # Thresholds na parte inferior (referência rápida)
        info = (
            f"Thresholds:  "
            f"Corrente  warn={CURRENT_WARN}A  crit={CURRENT_CRIT}A  |  "
            f"Temp  warn={TEMP_WARN}°C  crit={TEMP_CRIT}°C  |  "
            f"Kt={KT} Nm/A"
        )
        ctk.CTkLabel(bottom, text=info, font=("Roboto", 10), text_color="#888888").pack(side="left", padx=4)

    # ==========================================
    # 2. REDE / SIMULAÇÃO
    # ==========================================
    def network_worker(self):
        while self.running:
            current_t = time.time() - self.start_time

            if SIMULATION_MODE:
                l_pos = current_t * 0.42 + random.uniform(-0.02, 0.02)
                r_pos = current_t * 0.40 + random.uniform(-0.02, 0.02)

                currents = []
                temps    = []
                torques  = []

                for i in range(4):
                    # Simula um pico de corrente no motor 2 após 15s (esteira sendo esticada)
                    base_current = 4.0 + math.sin(current_t * 1.5 + i) * 2
                    if i == 1 and 15 < current_t < 22:
                        base_current += 3.5 * math.sin((current_t - 15) * 1.1)

                    c = base_current + random.uniform(-0.3, 0.3)
                    t = min(65.0, 28.0 + current_t * (0.15 + i * 0.03)) + random.uniform(-0.1, 0.1)
                    tq = max(0.0, c * KT)

                    currents.append(c)
                    temps.append(t)
                    torques.append(tq)

            else:
                # --- MODO REAL ---
                l_pos = 0.0
                r_pos = 0.0
                currents = [0.0] * 4
                temps    = [0.0] * 4
                torques  = [0.0] * 4
 
                if not getattr(self, 'tcp_node', None) or not getattr(self.tcp_node, 'connected', False):
                    try:
                        self.tcp_node = client(IP_BASE_SERVER)
                    except Exception:
                        pass
                else:
                    lv = self.tcp_node.read_float("L_pos")
                    rv = self.tcp_node.read_float("R_pos")
                    l_pos = lv if lv is not None else 0.0
                    r_pos = rv if rv is not None else 0.0

                    for i in range(4):
                        prefix = f"M{i+1}"
                        c  = self.tcp_node.read_float(f"{prefix}_curr")
                        t  = self.tcp_node.read_float(f"{prefix}_temp")
                        tq = self.tcp_node.read_float(f"{prefix}_torque")  # leitura direta OU use c * KT
                        currents[i] = c  if c  is not None else 0.0
                        temps[i]    = t  if t  is not None else 0.0
                        # Se o robô não envia torque diretamente, calcule:
                        torques[i]  = (tq if tq is not None else currents[i] * KT)

            # --- Escreve nos buffers com lock ---
            with self._lock:
                self.l_pos = l_pos
                self.r_pos = r_pos
                self.plot_time.append(current_t)
                for i in range(4):
                    self.plot_currents[i].append(currents[i])
                    self.plot_temperatures[i].append(temps[i])
                    self.plot_torques[i].append(torques[i])

            time.sleep(1.0 / FREQ_HZ)

    # ==========================================
    # 3. ATUALIZAÇÃO DA TELA (10 Hz)
    # ==========================================
    def update_gui(self):
        if not self.running:
            return

        with self._lock:
            if not self.plot_time:
                self.after(33, self.update_gui)
                return

            t_data   = list(self.plot_time)
            currents = [list(b) for b in self.plot_currents]
            temps    = [list(b) for b in self.plot_temperatures]
            torques  = [list(b) for b in self.plot_torques]
            l_pos    = self.l_pos
            r_pos    = self.r_pos

        n = min(len(t_data), *[len(b) for b in currents + temps + torques])
        if n == 0:
            self.after(33, self.update_gui)
            return

        t_slice  = t_data[:n]
        current_t = t_slice[-1]
        conn_txt  = "SIMULAÇÃO" if SIMULATION_MODE else "CONECTADO"

        # --- Atualiza cards de motor e coleta severidade global ---
        worst_global = 'ok'
        for i in range(4):
            c  = currents[i][n - 1]
            te = temps[i][n - 1]
            tq = torques[i][n - 1]
            worst = self.motor_cards[i].update(c, te, tq)

            # Log de eventos quando a severidade piora
            if worst != self._prev_severity[i]:
                if worst in ('warn', 'crit'):
                    msg = (
                        f"[{current_t:7.1f}s] M{i+1} {SEVERITY_LABEL[worst]}: "
                        f"I={c:.2f}A T={te:.1f}°C τ={tq:.3f}Nm\n"
                    )
                    self.event_log.append(msg)
                    self._append_log(msg)
                elif self._prev_severity[i] in ('warn', 'crit'):
                    msg = f"[{current_t:7.1f}s] M{i+1} voltou ao normal\n"
                    self.event_log.append(msg)
                    self._append_log(msg)
                self._prev_severity[i] = worst

            if worst == 'crit':
                worst_global = 'crit'
            elif worst == 'warn' and worst_global == 'ok':
                worst_global = 'warn'

        # --- Indicador global e status ---
        self.lbl_global_indicator.configure(text_color=SEVERITY_COLOR[worst_global])
        status_str = (
            f"L: {l_pos:6.2f} rad  |  R: {r_pos:6.2f} rad  "
            f"[{SEVERITY_LABEL[worst_global]}  •  {conn_txt}]"
        )
        self.lbl_status.configure(text=status_str, text_color=SEVERITY_COLOR[worst_global])

        # --- Atualiza as linhas dos gráficos ---
        mapping = {
            'c1':  currents[0], 'c2':  currents[1],
            'c3':  currents[2], 'c4':  currents[3],
            't1':  temps[0],    't2':  temps[1],
            't3':  temps[2],    't4':  temps[3],
            'tq1': torques[0],  'tq2': torques[1],
            'tq3': torques[2],  'tq4': torques[3],
        }
        for key, data in mapping.items():
            self.lines[key].set_data(t_slice, data[:n])

        for ax in self.axs.flat:
            ax.set_xlim(max(0, current_t - HISTORY_SECONDS), max(HISTORY_SECONDS, current_t))
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)

        self.canvas.draw_idle()
        self.after(33, self.update_gui)

    def _append_log(self, msg):
        """Insere uma linha no log de eventos (thread-safe via after())."""
        self.log_box.configure(state="normal")
        self.log_box.insert("end", msg)
        self.log_box.see("end")
        self.log_box.configure(state="disabled")

    # ==========================================
    # 4. EXPORTAÇÃO EXCEL
    # ==========================================
    def save_excel(self):
        self.btn_export.configure(state="disabled", text="Gerando...")
        self.update()

        try:
            with self._lock:
                n = min(len(self.plot_time), *[len(b) for b in self.plot_currents + self.plot_temperatures + self.plot_torques])
                data = {'Tempo_s': list(self.plot_time)[:n]}
                for i in range(4):
                    m = f"M{i+1}"
                    data[f'{m}_Corrente_A']   = list(self.plot_currents[i])[:n]
                    data[f'{m}_Temp_C']        = list(self.plot_temperatures[i])[:n]
                    data[f'{m}_Torque_Nm']     = list(self.plot_torques[i])[:n]

            df = pd.DataFrame(data)

            # Colunas de severidade (útil para análise pós-missão)
            for i in range(4):
                m = f"M{i+1}"
                df[f'{m}_Sev_Corrente'] = df[f'{m}_Corrente_A'].apply(
                    lambda v: get_severity(v, CURRENT_WARN, CURRENT_CRIT))
                df[f'{m}_Sev_Temp'] = df[f'{m}_Temp_C'].apply(
                    lambda v: get_severity(v, TEMP_WARN, TEMP_CRIT))

            filename = f"telemetria_crawler_{int(time.time())}.xlsx"
            with pd.ExcelWriter(filename, engine='openpyxl') as writer:
                df.to_excel(writer, sheet_name='Telemetria', index=False)

                # Aba extra com o log de eventos
                if self.event_log:
                    log_df = pd.DataFrame({'Evento': self.event_log})
                    log_df.to_excel(writer, sheet_name='Log_Eventos', index=False)

            print(f"[SUCESSO] Dados salvos em {filename}")

        except Exception as e:
            print(f"[ERRO] {e}")

        finally:
            self.btn_export.configure(state="normal", text="⬇  Exportar XLSX")

    def destroy(self):
        self.running = False
        super().destroy()


if __name__ == "__main__":
    app = GroundStationApp()
    app.mainloop()