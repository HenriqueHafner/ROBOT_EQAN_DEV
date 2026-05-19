import struct
import time
import math


import can # type: ignore
bus = can.interface.Bus(bustype="socketcan", channel="can0", bitrate=500000)

class can_motor_interface:
    def __init__(self, joint_model, motor_id):
        global bus
        self.bus = bus
        self.motor_id = motor_id
        self.kp = 300.0
        self.kd = 0.5
        self.orientation = 1
        self.last_position_feedback = 0.0
        if joint_model == "aka10":
            self.joint_model = 2
        elif joint_model == "ak70":
            self.joint_model = 1
        else:
            print("joint_model not specified: ", joint_model)

    def enable_motor(self):
        if self.joint_model == 1:
            return enable_ak70(self.bus, self.motor_id)
        elif self.joint_model == 2:
            return enable_aka10(self.bus, self.motor_id)

    def disable_motor(self):
        if self.joint_model == 1:
            return disable_ak70(self.bus, self.motor_id)
        elif self.joint_model == 2:
            return disable_aka10(self.bus, self.motor_id)

    def set_zero_torque_and_get_status(self):
        if self.joint_model == 1:
            status = send_motor_cmd_ak70(self.bus, self.motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0)
        elif self.joint_model == 2:
            status = send_motor_cmd_aka10(self.bus, self.motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0)
        if status is None:
            print("feedback status is None")
            return None
        status[0] = status[0] * self.orientation
        status[1] = status[1] * self.orientation
        return status

    def set_zero_torque_and_get_position(self):
        status = self.set_zero_torque_and_get_status()
        if status:
            return status[0]
        else:
            return None

    def set_origin(self):
        if self.joint_model == 1:
            set_origin_ak70(self.bus, self.motor_id)
        elif self.joint_model == 2:
            set_origin_aka10(self.bus, self.motor_id)  

    def send_position_controll_command_arbitrary(self, position):
        p_des = position * self.orientation
        v_des = 0.0
        t_ff = 0.0
        if self.joint_model == 1:
            status = send_motor_cmd_ak70(self.bus, self.motor_id, p_des, v_des, self.kp, self.kd, t_ff)
        elif self.joint_model == 2:
            status = send_motor_cmd_aka10(self.bus, self.motor_id, p_des, v_des, self.kp, self.kd, t_ff)
        if status is None:
            print("feedback status is None")
            return False
        status[0] = status[0] * self.orientation
        status[1] = status[1] * self.orientation
        return status

    def send_position_controll_command(self, position):
        if abs(position-self.last_position_feedback) > 8:
            return [0.0, 0.0, 0.0, 25.0, 0, "position too far away"]
        p_des = position * self.orientation
        status = self.send_position_controll_command_arbitrary(position)
        return status

    def send_velocity_controll_command(self, v_des, kd):
        v_des = v_des * self.orientation
        p_des = 0.0
        t_ff = 0.0
        kp = 0.0
        if self.joint_model == 1:
            status = send_motor_cmd_ak70(self.bus, self.motor_id, p_des, v_des, kp, kd, t_ff)
        elif self.joint_model == 2:
            status = send_motor_cmd_aka10(self.bus, self.motor_id, p_des, v_des, kp, kd, t_ff)
        if status is None:
            print("feedback status is None")
            return False
        status[0] = status[0] * self.orientation
        status[1] = status[1] * self.orientation
        return status

    def send_torque_controll_command(self, t_ff):
        t_ff = t_ff * self.orientation
        p_des = 0.0
        v_des = 0.0
        kp = 0.0
        kd = 0.0
        if self.joint_model == 1:
            status = send_motor_cmd_ak70(self.bus, self.motor_id, p_des, v_des, kp, kd, t_ff)
        elif self.joint_model == 2:
            status = send_motor_cmd_aka10(self.bus, self.motor_id, p_des, v_des, kp, kd, t_ff)
        if status is None:
            print("feedback status is None")
            return False
        status[0] = status[0] * self.orientation
        status[1] = status[1] * self.orientation
        return status
    
    def send_current_brake_command(self, current):
        if self.joint_model == 2:
            status = set_current_brake(self.bus, self.motor_id, current)
        elif self.joint_model == 1:
            return False
        if status is None:
            print("feedback status is None")
            return False
        status[0] = status[0] * self.orientation
        status[1] = status[1] * self.orientation
        return status
    
    def send_position_velocity_command(self, position_deg, speed_erpm, accel_erpm2):
        position_deg = position_deg * self.orientation
        if self.joint_model == 2:
            status = set_position_velocity(self.bus, self.motor_id, position_deg, speed_erpm, accel_erpm2)
        elif self.joint_model == 1:
            return False
        if status is None:
            print("feedback status is None")
            return False
        status[0] = status[0] * self.orientation
        status[1] = status[1] * self.orientation
        return status
    
    def send_position_loop_command(self, position_deg):
        if self.joint_model == 2:
            status = set_position_loop(self.bus, self.motor_id, position_deg)
        elif self.joint_model == 1:
            return False
        if status is None:
            print("feedback status is None")
            return False
        status[0] = status[0] * self.orientation
        status[1] = status[1] * self.orientation
        return status

def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    x_clamped = max(min(x, x_max), x_min)
    return int((x_clamped - x_min) * ((1 << bits) / span))

def uint_to_float(x_int, x_min, x_max, bits):
    span = x_max - x_min
    return float(x_int) * span / ((1 << bits) - 1) + x_min

ERROR_CODES = {
    0: "Sem falhas",
    1: "Sobreaquecimento do motor",
    2: "Sobrecorrente",
    3: "Sobrevoltagem",
    4: "Subvoltagem",
    5: "Falha no encoder",
    6: "Sobreaquecimento do MOSFET",
    7: "Motor travado"
}

def decode_motor_can_message_aka10(data):
    pos = struct.unpack(">h", data[0:2])[0] * 0.1
    speed = struct.unpack(">h", data[2:4])[0] * 10.0
    current = struct.unpack(">h", data[4:6])[0] * 0.01
    temp = struct.unpack("b", data[6:7])[0]
    error_code = data[7]
    error = ERROR_CODES.get(error_code, f"Erro desconhecido ({error_code})")
    return [math.radians(pos), speed, current, temp, error_code, error]

def pack_cmd_aka10(p_des, v_des, kp, kd, t_ff):
    p_min = -12.56
    p_max = 12.56
    v_min = -28.0
    v_max = 28.0
    t_min = -54.0
    t_max = 54.0
    kp_min = 0
    kp_max = 500.0
    kd_min = 0
    kd_max = 5.0
    p_int = float_to_uint(p_des, p_min, p_max, 16)
    v_int = float_to_uint(v_des, v_min, v_max, 12)
    kp_int = float_to_uint(kp, kp_min, kp_max, 12)
    kd_int = float_to_uint(kd, kd_min, kd_max, 12)
    t_int = float_to_uint(t_ff, t_min, t_max, 12)
    data = [
        (kp_int >> 4) & 0xFF,
        ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF),
        kd_int & 0xFF,
        (p_int >> 8) & 0xFF,
        p_int & 0xFF,
        (v_int >> 4) & 0xFF,
        ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
        t_int & 0xFF
    ]
    return data

def send_motor_cmd_aka10(can_bus, motor_id, p_des, v_des, kp, kd, t_ff):
    can_id = (8 << 8) | motor_id
    data = pack_cmd_aka10(p_des, v_des, kp, kd, t_ff)
    while not (can_bus.recv(timeout=0) is None):
        pass
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    try:
        can_bus.send(msg)
    except can.CanError:
        print("Erro ao enviar comando")
    msg = can_bus.recv()
    if msg.arbitration_id == (41 << 8) | motor_id:
        motor_data = decode_motor_can_message_aka10(msg.data)
        return motor_data
    return None

def enable_aka10(can_bus, motor_id):
    can_id = (5 << 8) | motor_id
    data = pack_cmd_aka10(0, 0, 0, 0, 0)
    msg = can.Message(arbitration_id=can_id, data=[0xFF]*7 + [0xFC], is_extended_id=True)
    can_bus.send(msg)
    return send_motor_cmd_aka10(can_bus, motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0)

def disable_aka10(can_bus, motor_id):
    return send_motor_cmd_aka10(can_bus, motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0)

def set_zero_torque_and_get_status_aka10(can_bus, motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0):
    return send_motor_cmd_aka10(can_bus, motor_id, p_des, v_des, kp, kd, t_ff)

def pack_cmd_ak70(p_des, v_des, kp, kd, torque):
    p_min = -12.5
    p_max = 12.5
    v_min = -50.0
    v_max = 50.0
    t_min = -25.0
    t_max = 25.0
    kp_min = 0
    kp_max = 500
    kd_min = 0
    kd_max = 5
    p_int = float_to_uint(p_des, p_min, p_max, 16)
    v_int = float_to_uint(v_des, v_min, v_max, 12)
    kp_int = float_to_uint(kp, kp_min, kp_max, 12)
    kd_int = float_to_uint(kd, kd_min, kd_max, 12)
    t_int = float_to_uint(torque, t_min, t_max, 12)
    return [
        (p_int >> 8) & 0xFF,
        p_int & 0xFF,
        (v_int >> 4) & 0xFF,
        ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF),
        kp_int & 0xFF,
        (kd_int >> 4) & 0xFF,
        ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF),
        t_int & 0xFF,
    ]

def send_motor_cmd_ak70(can_bus, motor_id, p_des, v_des, kp, kd, t_ff):
    data = pack_cmd_ak70(p_des, v_des, kp, kd, t_ff)
    while not (can_bus.recv(timeout=0) is None):
        pass
    msg = can.Message(arbitration_id=motor_id, data=data, is_extended_id=False)
    can_bus.send(msg)
    msg = can_bus.recv()
    if msg.arbitration_id == motor_id:
        motor_data = decode_motor_can_message_ak70(msg)
        return motor_data
    return None

def enable_ak70(can_bus, motor_id):
    msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFC], is_extended_id=False)
    can_bus.send(msg)
    time.sleep(0.01)
    recv_msg = can_bus.recv(timeout=0.1)
    if recv_msg and recv_msg.arbitration_id == motor_id:
        return decode_motor_can_message_ak70(recv_msg)
    return None

def disable_ak70(can_bus, motor_id):
    send_motor_cmd_ak70(can_bus, motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0)
    time.sleep(0.1)
    msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFD], is_extended_id=False)
    can_bus.send(msg)
    return None

def set_zero_position(can_bus, motor_id):
    msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFE], is_extended_id=False)
    can_bus.send(msg)
    return None

def decode_motor_can_message_ak70(msg):
    data = msg.data
    pos_int = (data[1] << 8) | data[2]
    vel_int = (data[3] << 4) | (data[4] >> 4)
    cur_int = ((data[4] & 0x0F) << 8) | data[5]
    temperature = data[6] - 40
    error_code = data[7]
    error = ERROR_CODES.get(error_code, f"Erro desconhecido ({error_code})")
    pos = uint_to_float(pos_int, -12.5, 12.5, 16)
    vel = uint_to_float(vel_int, -50.0, 50.0, 12)
    current = uint_to_float(cur_int, -25.0, 25.0, 12)
    return [pos, vel, current, temperature, error_code, error]

def set_zero_torque_and_get_status_ak70(can_bus, motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0):
    return send_motor_cmd_ak70(can_bus, motor_id, p_des, v_des, kp, kd, t_ff)

def set_origin_aka10(bus, motor_id, permanent=False):
    can_id = (5 << 8) | motor_id  # Control Mode ID 5
    origin_mode = 1 if permanent else 0
    data = [origin_mode]  # Apenas 1 byte
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    bus.send(msg)

def set_position_velocity(can_bus, motor_id, position_deg, speed_erpm, accel_erpm2):
    # Converte posição em graus para protocolo (1° = 10000 unidades)
    pos_int = int(position_deg * 10000)
    
    # Velocidade e aceleração já são int16 (escala direta conforme datasheet)
    spd_int = int(speed_erpm / 10)   # dividido por 10 (datasheet)
    acc_int = int(accel_erpm2 / 10)  # dividido por 10 (datasheet)

    # Empacotar em 8 bytes (big-endian)
    data = bytearray(8)
    struct.pack_into(">i", data, 0, pos_int)   # posição int32
    struct.pack_into(">h", data, 4, spd_int)   # velocidade int16
    struct.pack_into(">h", data, 6, acc_int)   # aceleração int16

    can_id = (6 << 8) | motor_id  # Control Mode ID 6
    while not (can_bus.recv(timeout=0) is None):
        pass
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    try:
        can_bus.send(msg)
    except can.CanError:
        print("Erro ao enviar comando")
    msg = can_bus.recv()
    if msg.arbitration_id == (41 << 8) | motor_id:
        motor_data = decode_motor_can_message_aka10(msg.data)
        return motor_data
    return None

def set_position_loop(can_bus, motor_id, position_deg):
    """
    Send Position Loop Mode command (CAN_PACKET_SET_POS)
    Equivalent to the C function comm_can_set_pos()

    Args:
        bus: python-can bus
        motor_id: CAN ID of the motor
        position_deg: desired position in degrees
    """

    # Convert to protocol units: 1° = 10000 units
    pos_int = int(position_deg * 10000.0)

    # Pack this int32 as big-endian (same as buffer_append_int32)
    data = struct.pack(">i", pos_int)

    # CAN ID = motor_id | (CAN_PACKET_SET_POS << 8)
    # CAN_PACKET_SET_POS = 4  (from CubeMars protocol)
    CAN_PACKET_SET_POS = 4
    can_id = motor_id | (CAN_PACKET_SET_POS << 8)
    while not (can_bus.recv(timeout=0) is None):
        pass
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)
    try:
        can_bus.send(msg)
    except can.CanError:
        print("Erro ao enviar comando")
    msg = can_bus.recv()
    if msg.arbitration_id == (41 << 8) | motor_id:
        motor_data = decode_motor_can_message_aka10(msg.data)
        return motor_data
    return None

def set_current_brake(can_bus, motor_id, current_a):
    # Clamp current within safe range (0–60 A)
    current_a = max(0.0, min(current_a, 60.0))

    # Convert current to protocol format: int32 = current * 1000
    current_int = int(current_a * 1000)

    # Pack as big-endian 4-byte signed integer
    data = struct.pack(">i", current_int)

    # Control Mode ID = 2 → Current Brake Mode
    can_id = (2 << 8) | motor_id

    # Create CAN message (extended frame, 8-byte header allowed)
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=True)

    while not (can_bus.recv(timeout=0) is None):
        pass
    can_bus.send(msg)

    if msg.arbitration_id == (41 << 8) | motor_id:
        motor_data = decode_motor_can_message_aka10(msg.data)
        return motor_data
    return None

def set_origin_ak70(bus, motor_id):
    send_motor_cmd_ak70(bus, motor_id, p_des=0.0, v_des=0.0, kp=0.0, kd=0.0, t_ff=0.0)
    time.sleep(0.1)
    msg = can.Message(arbitration_id=motor_id, data=[0xFF]*7 + [0xFE], is_extended_id=False)
    bus.send(msg)