import socket
import struct
import time
import random

class ExperimentalClient:

    def __init__(self, server_ip=None):
        self.server_port = 5007
        self.server_ip = server_ip if server_ip else '127.0.0.1'
        self.socket = None
        self.connected = False
        self.last_reconnect_attempt = 0.0
        self.reconnect_cooldown_seconds = 1.5
        self._connect()

    def _is_in_cooldown(self):
        elapsed = time.monotonic() - self.last_reconnect_attempt
        return elapsed < self.reconnect_cooldown_seconds

    def _mark_reconnect_attempt(self):
        self.last_reconnect_attempt = time.monotonic()

    def _close_socket(self):
        if self.socket:
            try:
                self.socket.close()
            except Exception:
                pass
        self.socket = None
        self.connected = False

    def _connect(self):
        if self.connected:
            return True
        if self._is_in_cooldown():
            return False

        self._mark_reconnect_attempt()
        self._close_socket()

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2.0)
            self.socket.connect((self.server_ip, self.server_port))
            self.socket.settimeout(1.0)
            self.connected = True
            return True
        except Exception:
            self.connected = False
            return False

    def _ensure_connection(self):
        if self.connected:
            return True
        return self._connect()

    def _recv_exactly(self, num_bytes):
        if not self.connected or not self.socket:
            return None
        data = b''
        try:
            while len(data) < num_bytes:
                chunk = self.socket.recv(num_bytes - len(data))
                if not chunk:
                    self.connected = False
                    return None
                data += chunk
            return data
        except Exception:
            self.connected = False
            return None

    def _send_frame(self, payload):
        if not self._ensure_connection():
            return False
        try:
            length = len(payload)
            frame = struct.pack('>I', length) + payload
            self.socket.sendall(frame)
            return True
        except Exception:
            self.connected = False
            return False

    def _receive_frame(self):
        if not self._ensure_connection():
            return None
        length_bytes = self._recv_exactly(4)
        if length_bytes is None:
            return None
        length = struct.unpack('>I', length_bytes)[0]
        if length == 0 or length > 1024 * 1024:
            self.connected = False
            return None
        payload = self._recv_exactly(length)
        if payload is None:
            return None
        return payload

    # Public API

    def send_float(self, name, value):
        name_bytes = name.encode('utf-8')
        if len(name_bytes) > 255:
            name_bytes = name_bytes[:255]
        value_bytes = struct.pack('>f', float(value))
        payload = (
            b'\x01'  # SET_FLOAT
            + struct.pack('B', len(name_bytes))
            + name_bytes
            + value_bytes
        )
        return self._send_frame(payload)

    def read_float(self, name):
        name_bytes = name.encode('utf-8')
        if len(name_bytes) > 255:
            name_bytes = name_bytes[:255]
        payload = (
            b'\x02'  # GET_FLOAT
            + struct.pack('B', len(name_bytes))
            + name_bytes
        )
        if not self._send_frame(payload):
            return None
        response = self._receive_frame()
        if response is None or len(response) < 1:
            return None
        if response[0] != 0x03:  # GET_FLOAT_RESPONSE
            return None
        if len(response) < 5:
            return None
        value_bytes = response[1:5]
        try:
            return struct.unpack('>f', value_bytes)[0]
        except Exception:
            return None

    def send_bytes(self, name, data):
        name_bytes = name.encode('utf-8')
        if len(name_bytes) > 255:
            name_bytes = name_bytes[:255]
        data_bytes = bytes(data)[:65535]
        payload = (
            b'\x04'  # SET_BYTES
            + struct.pack('B', len(name_bytes))
            + name_bytes
            + struct.pack('>H', len(data_bytes))
            + data_bytes
        )
        return self._send_frame(payload)

    def read_bytes(self, name):
        name_bytes = name.encode('utf-8')
        if len(name_bytes) > 255:
            name_bytes = name_bytes[:255]
        payload = (
            b'\x05'  # GET_BYTES
            + struct.pack('B', len(name_bytes))
            + name_bytes
        )
        if not self._send_frame(payload):
            return None
        response = self._receive_frame()
        if response is None or len(response) < 1:
            return None
        if response[0] != 0x06:  # GET_BYTES_RESPONSE
            return None
        if len(response) < 3:
            return None
        data_length = struct.unpack('>H', response[1:3])[0]
        if len(response) < 3 + data_length:
            return None
        return response[3 : 3 + data_length]

    # Compatibility aliases
    def set_float(self, name, value):
        return self.send_float(name, value)

    def write(self, name, value):
        if isinstance(value, float):
            return self.send_float(name, value)
        else:
            return self.send_bytes(name, value)

    def read(self, name):
        # Try float first, fallback to bytes
        result = self.read_float(name)
        if result is not None:
            return result
        return self.read_bytes(name)


if __name__ == "__main__":
    client = ExperimentalClient()
    client.send_float("test_speed", 42.5)
    value = client.read_float("test_speed")
    print("Read back:", value)
    client.send_bytes("test_message", b"hello experimental")
    print("Bytes:", client.read_bytes("test_message"))