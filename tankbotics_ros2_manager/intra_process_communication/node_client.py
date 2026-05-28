import socket
import struct
import time
import random

class client:
    def __init__(self, ip=None):
        self.server_port = 5007
        if ip:
            if self.server_reachable(ip, self.server_port):
                self.server_ip = ip
        else:
            print("Unable to reach server at ip: ", ip)
            print("Target server ip changed to localhost 127.0.0.1")
            self.server_ip = '127.0.0.1'
        self.socket = None
        self.connected_before = False
        self.connected = False
        self.last_reconnection_attempt = 0.0
        self.reconnection_cooldown = 2.0

        self.connect_to_server()

    def connection_handler(self):
        if self.connected:
            return True
        if self.connected_before:
            return self.connect_to_server()
        return False

    def in_reconnection_cooldown(self):
        cooldown_time = self.reconnection_cooldown + random.uniform(0.0, 1.0)
        if time.monotonic() - self.last_reconnection_attempt < cooldown_time:
            return True
        return False

    def connect_to_server(self):
        if self.in_reconnection_cooldown():
                    return False
        self.last_reconnection_attempt = time.monotonic()
        try:
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(1.0)
            self.socket.connect((self.server_ip, self.server_port))
            self.connected = True
            self.connected_before = True
            print("[rmock_client] connected to server")
            return True
        except Exception as e:
            self.connected = False
            print("[rmock_client] connection error:", e)
            return False

    def _recv_exactly(self, n):
        if not self.connected:
            return None
        data = b''
        while len(data) < n:
            try:
                chunk = self.socket.recv(n - len(data))
                if not chunk:
                    self.connected = False
                    return None
                data += chunk
            except Exception:
                self.connected = False
                return None
        return data

    def send_server(self, variable_name, data):
        if not self.connection_handler():
            return False
        name_bytes = variable_name.encode().ljust(32, b' ')
        data = bytes(data) if not isinstance(data, bytes) else data
        size = min(len(data), 255)
        packet = b'S' + name_bytes + struct.pack('B', size) + data[:size]
        self.socket.sendall(packet)
        return True

    def read_server(self, variable_name):
        if not self.connection_handler():
            return None

        try:
            name_bytes = variable_name.encode().ljust(32, b' ')
            self.socket.sendall(b'G' + name_bytes + struct.pack('B', 0) + b'')
            size_data = self._recv_exactly(1)
            if size_data is None:
                return None
            size = struct.unpack('B', size_data)[0]
            data = self._recv_exactly(size)
            return data
        except Exception as e:
            print("[rmock_client] read_server error:", e)
            self.connected = False
            return None

    def write(self, variable_name, value):
        if isinstance(value, float):
            data = struct.pack('f', float(value))
        elif isinstance(value, str):
            data = value.encode('utf-8')
        else:
            data = bytes(value) if hasattr(value, '__bytes__') else b''
        return self.send_server(variable_name, data)

    def read(self, variable_name):
        data = self.read_server(variable_name)
        if data is None or len(data) == 0:
            return None
        if len(data) == 4:
            try:
                return struct.unpack('f', data)[0]
            except:
                pass
        return data

    def set_float(self, variable_name, value):
        return self.write(variable_name, float(value))

    def read_float(self, variable_name):
        data = self.read_server(variable_name)
        if data and len(data) == 4:
            try:
                return struct.unpack('f', data)[0]
            except Exception as e:
                pass
        else:
            print(f"[rmock_client] read_float error: invalid data for variable '{variable_name}'")
        return None

    def read_bits(self, variable_name):
        data = self.read_server(variable_name)
        if data and len(data) == 4:
            try:
                return struct.unpack('i', data)[0]
            except:
                pass
        return None

    def modify_topic_property(self, variable_name: str, **kwargs):
        # This method is a placeholder and does not have an implementation in the client.
        # In a real implementation, this would send a command to the server to modify the topic properties.
        return False
    
    def server_reachable(self, server_ip, server_port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        try:
            connect_code = sock.connect_ex((server_ip, server_port))
            if connect_code == 0:
                return True
            else:
                return False
        except:
            return False
        finally:
            sock.close()

if __name__ == "__main__":
    node = client()
    node.set_float("test_float", 1.0)