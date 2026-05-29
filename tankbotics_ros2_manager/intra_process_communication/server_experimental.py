import socket
import struct
import time
import sys
import math
from collections import namedtuple

Topic = namedtuple('Topic', ['value', 'type_hint'])

class ExperimentalServer:

    def __init__(self, host=None):
        self.host = host if host else '0.0.0.0'
        self.port = 5007
        self.topics = {}
        self.connections = []
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(64)
        self.server_socket.setblocking(False)

        self.last_print_time = time.monotonic()
        self.print_interval = 1.0 / 30.0
        self.max_lines = 1

    def _get_or_create_topic(self, name):
        if name not in self.topics:
            self.topics[name] = Topic(value=b'\x00\x00\x00\x00', type_hint='float')
        return self.topics[name]

    def _accept_new_connections(self):
        try:
            conn, addr = self.server_socket.accept()
            conn.setblocking(False)
            self.connections.append(conn)
        except BlockingIOError:
            pass

    def _handle_one_connection(self, conn):
        try:
            length_bytes = conn.recv(4)
            if not length_bytes:
                self._close_connection(conn)
                return
            if len(length_bytes) < 4:
                return
            length = struct.unpack('>I', length_bytes)[0]
            if length == 0 or length > 2 * 1024 * 1024:
                self._close_connection(conn)
                return
            payload = self._recv_exactly_from_conn(conn, length)
            if payload is None:
                self._close_connection(conn)
                return
            self._process_payload(conn, payload)
        except BlockingIOError:
            pass
        except Exception:
            self._close_connection(conn)

    def _recv_exactly_from_conn(self, conn, num_bytes):
        data = b''
        try:
            while len(data) < num_bytes:
                chunk = conn.recv(num_bytes - len(data))
                if not chunk:
                    return None
                data += chunk
            return data
        except BlockingIOError:
            return None
        except Exception:
            return None

    def _process_payload(self, conn, payload):
        if len(payload) < 1:
            return
        command = payload[0]

        if command == 0x01:  # SET_FLOAT
            self._handle_set_float(payload)

        elif command == 0x02:  # GET_FLOAT
            self._handle_get_float(conn, payload)

        elif command == 0x04:  # SET_BYTES
            self._handle_set_bytes(payload)

        elif command == 0x05:  # GET_BYTES
            self._handle_get_bytes(conn, payload)

    def _handle_set_float(self, payload):
        if len(payload) < 2:
            return
        name_length = payload[1]
        if len(payload) < 2 + name_length + 4:
            return
        name = payload[2 : 2 + name_length].decode('utf-8', errors='ignore')
        value_bytes = payload[2 + name_length : 2 + name_length + 4]
        topic = self._get_or_create_topic(name)
        self.topics[name] = Topic(value=value_bytes, type_hint='float')

    def _handle_get_float(self, conn, payload):
        if len(payload) < 2:
            return
        name_length = payload[1]
        if len(payload) < 2 + name_length:
            return
        name = payload[2 : 2 + name_length].decode('utf-8', errors='ignore')
        topic = self._get_or_create_topic(name)
        response = b'\x03' + topic.value  # GET_FLOAT_RESPONSE
        try:
            length = len(response)
            conn.sendall(struct.pack('>I', length) + response)
        except Exception:
            self._close_connection(conn)

    def _handle_set_bytes(self, payload):
        if len(payload) < 2:
            return
        name_length = payload[1]
        if len(payload) < 2 + name_length + 2:
            return
        name = payload[2 : 2 + name_length].decode('utf-8', errors='ignore')
        data_length = struct.unpack('>H', payload[2 + name_length : 2 + name_length + 2])[0]
        start = 2 + name_length + 2
        if len(payload) < start + data_length:
            return
        value = payload[start : start + data_length]
        self.topics[name] = Topic(value=value, type_hint='bytes')

    def _handle_get_bytes(self, conn, payload):
        if len(payload) < 2:
            return
        name_length = payload[1]
        if len(payload) < 2 + name_length:
            return
        name = payload[2 : 2 + name_length].decode('utf-8', errors='ignore')
        topic = self._get_or_create_topic(name)
        value = topic.value
        response = b'\x06' + struct.pack('>H', len(value)) + value
        try:
            length = len(response)
            conn.sendall(struct.pack('>I', length) + response)
        except Exception:
            self._close_connection(conn)

    def _close_connection(self, conn):
        try:
            conn.close()
        except Exception:
            pass
        if conn in self.connections:
            self.connections.remove(conn)

    def _handle_connections(self):
        for conn in self.connections[:]:
            self._handle_one_connection(conn)

    def _format_float(self, value):
        if value == 0:
            return "+000.0000 e0"
        sign = "+" if value >= 0 else "-"
        abs_value = abs(value)
        exponent = max(0, int(math.floor(math.log10(abs_value))) - 2)
        scale = 10 ** exponent
        mantissa = value / scale
        truncated = math.trunc(mantissa * 10000) / 10000
        abs_mant = abs(truncated)
        integer_part = int(abs_mant)
        decimal_part = abs_mant - integer_part
        integer_str = f"{integer_part:03d}"
        decimal_str = f"{decimal_part:.4f}"[2:]
        return f"{sign}{integer_str}.{decimal_str} e{exponent}"

    def _print_state(self):
        now = time.monotonic()
        if now - self.last_print_time < self.print_interval:
            return
        self.last_print_time = now

        lines = []
        lines.append("topics:")

        for name, topic in sorted(self.topics.items()):
            short_name = name[:24].ljust(24)
            value = topic.value
            type_hint = getattr(topic, 'type_hint', 'float')

            if type_hint == 'float' and len(value) == 4:
                try:
                    fval = struct.unpack('>f', value)[0]
                    formatted = self._format_float(fval)
                except Exception:
                    formatted = "???"
            elif type_hint == 'bytes':
                if len(value) <= 16:
                    formatted = value.hex()
                else:
                    formatted = value[:16].hex() + "..."
            else:
                formatted = value.hex() if value else ""

            lines.append(f"  {short_name} : {formatted}")

        content = "\n".join(lines) + "\n"
        current_count = len(lines)
        self.max_lines = max(self.max_lines, current_count)
        while len(lines) < self.max_lines:
            lines.append("")

        sys.stdout.write("\033[H\033[J")
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()

    def run(self):
        while True:
            try:
                self._accept_new_connections()
                self._handle_connections()
                self._print_state()
            except KeyboardInterrupt:
                print("\nserver stopped")
                break
            except Exception:
                time.sleep(0.01)


if __name__ == "__main__":
    server = ExperimentalServer()
    server.run()