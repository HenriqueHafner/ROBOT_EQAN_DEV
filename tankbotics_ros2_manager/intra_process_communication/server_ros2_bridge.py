import socket
import struct
import time
import sys
import math

class Topic:
    def __init__(self, name: str, topic_id: int, value: bytes = b'\x00\x00\x00\x00', type_str: str = "float"):
        self.name = name
        self.id = topic_id
        self.value = value[:512]          # limite máximo
        self.type = type_str[:16]

class server:
    def __init__(self, ip=None):
        self.host = ip if ip else '0.0.0.0'
        self.port = 5007
        self.topics = {}                  # name -> Topic
        self.next_topic_id = 1000
        self.connections = []
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(32)
        self.server_socket.setblocking(False)
        print(f"[rmock_server] listening on {self.host}:{self.port}")

        self.print_last_timestamp = time.monotonic()
        self.loop_last_timestamp = time.monotonic()
        self.loop_counter = 0
        self.time_cost = 0.0
        self.max_line_count = 1
        self.print_interval = 1.0 / 30

    def _get_or_create_topic(self, name: str, default_type: str = "float", default_value: bytes = b'\x00\x00\x00\x00') -> Topic:
        """Substitui o antigo setdefault com lógica completa de criação."""
        if name not in self.topics:
            topic = Topic(name, self.next_topic_id, default_value, default_type)
            self.topics[name] = topic
            self.next_topic_id += 1
        return self.topics[name]

    def modify_topic_property(self, name: str, **kwargs):
        """Modifica qualquer propriedade do tópico (exceto 'id')."""
        topic = self._get_or_create_topic(name)
        for key, value in kwargs.items():
            if key != 'id' and hasattr(topic, key):
                setattr(topic, key, value)

    def accept_connections(self):
        try:
            conn, addr = self.server_socket.accept()
            conn.setblocking(False)
            if conn:
                self.connections.append(conn)
            print(f"[rmock_server] accepted connection from {addr}")
        except BlockingIOError:
            pass

    def handle_connections(self):
        for conn in self.connections[:]:
            try:
                command_byte = conn.recv(1)
                if not command_byte:
                    self._finish_connection(conn)
                    continue

                command = command_byte.decode(errors='ignore')

                name_bytes = conn.recv(32)
                if len(name_bytes) < 32:
                    self._finish_connection(conn)
                    continue
                var_name = name_bytes.decode(errors='ignore').strip()

                size_byte = conn.recv(1)
                if not size_byte:
                    self._finish_connection(conn)
                    continue
                payload_size = size_byte[0]

                payload = conn.recv(payload_size) if payload_size > 0 else b''

                if command == 'S':
                    self.set_bytes(var_name, payload)
                elif command == 'G':
                    data = self.get_bytes(var_name)
                    reply_size = min(len(data), 255)
                    reply_data = data[:reply_size]
                    conn.sendall(struct.pack('B', reply_size) + reply_data)
                elif command == 'M':
                    # implementar
                    # Exemplo de comando de modificação: 'M' + nome(32 bytes) + tipo(16 bytes) + valor(depende do tipo)
                    None
                else:
                    print("[rmock_server] unknown command:", command)

            except BlockingIOError:
                continue
            except Exception as e:
                print("[rmock_server] connection error:", e)
                self._finish_connection(conn)

    def _finish_connection(self,conn):
        try:
            conn.close()
        except:
            pass
        if conn in self.connections:
            self.connections.remove(conn)
        print("[rmock_server] closed connection")

    def set_bytes(self, name, value):
        if not isinstance(value, bytes):
            value = bytes(value) if hasattr(value, '__bytes__') else str(value).encode()
        value = value[:512]
        topic = self._get_or_create_topic(name)
        topic.value = value

    def get_bytes(self, name):
        topic = self._get_or_create_topic(name, default_type="float", default_value=b'\x00\x00\x00\x00')
        return topic.value

    def get_float(self, name):
        data = self.get_bytes(name)
        if len(data) == 4:
            try:
                return struct.unpack('f', data)[0]
            except:
                pass
        return 0.0

    def get_bits(self, name):
        data = self.get_bytes(name)
        if len(data) == 4:
            try:
                return struct.unpack('i', data)[0]
            except:
                pass
        return 0

    def formatFloat(self, value):
        isZero = value == 0
        if isZero:
            return "+000.0000 e0"
        signChar = "+" if value >= 0 else "-"
        absValue = abs(value)
        logValue = math.log10(absValue)
        floorLog = math.floor(logValue)
        exponent = max(0, floorLog - 2)
        scaleFactor = 10 ** exponent
        mantissa = value / scaleFactor
        decimalPlaces = 10000
        truncatedMantissa = math.trunc(mantissa * decimalPlaces) / decimalPlaces
        absMantissa = abs(truncatedMantissa)
        integerPart = int(absMantissa)
        decimalPart = absMantissa - integerPart
        strInteger = f"{integerPart:03d}"
        strDecimal = f"{decimalPart:.4f}"[2:]
        mantissaStr = strInteger + "." + strDecimal
        formattedStr = signChar + mantissaStr + " e" + str(exponent)
        return formattedStr

    def probe_time_cost(self):
        self.loop_counter += 1
        if self.loop_counter >= 1e5:
            time_now = time.monotonic()
            self.time_cost = (time_now-self.loop_last_timestamp)*10
            self.loop_last_timestamp = time_now
            self.loop_counter = 0

    def print_variables(self):
        current_timestamp = time.monotonic()
        if current_timestamp - self.print_last_timestamp >= self.print_interval:
            time_cost_line = [("timecost(us)", self.time_cost)]
            data = time_cost_line + list(self.variables.items())
            current_line_count = len(data)
            self.max_line_count = max(self.max_line_count, current_line_count)
            lines = []
            for name, value in data:
                truncated_name = name[:16].ljust(16)
                if isinstance(value, bytes):
                    length = len(value)
                    if length == 1:
                        formatted_value = f"{value[0]:3d}"
                    elif length in (2, 3):
                        int_val = int.from_bytes(value, 'big')
                        bits = length * 8
                        formatted_value = f"{int_val:0{bits}b}"
                    elif length == 4:
                        try:
                            fval = struct.unpack('f', value)[0]
                            formatted_value = self.formatFloat(fval)
                        except:
                            formatted_value = ' '.join(f'{b:02x}' for b in value)
                    else:
                        formatted_value = ' '.join(f'{b:02x}' for b in value) if value else '<empty>'
                elif isinstance(value, float):
                    formatted_value = self.formatFloat(value)
                else:
                    formatted_value = self.format_binary(value)
                line = f"{truncated_name} : {formatted_value}"
                lines.append(line)
            lines += [""] * (self.max_line_count - current_line_count)
            content_string = "\n".join(lines) + "\n" if lines else ""
            sys.stdout.write("\033[H")
            for _ in range(self.max_line_count):
                sys.stdout.write("\033[2K\033[1B")
            sys.stdout.write("\033[H")
            sys.stdout.write(content_string)
            sys.stdout.flush()
            self.print_last_timestamp = current_timestamp

    def loop(self):
        last_error_print = time.monotonic()
        try:
            while True:
                try:
                    self.accept_connections()
                    self.handle_connections()
                    self.print_variables()
                    self.probe_time_cost()
                except Exception as e:
                    current_time = time.monotonic()
                    if current_time - last_error_print >= 0.2:
                        print("[rmock_server] error in loop:", e)
                        last_error_print = current_time
                    sys.stdout.write("\033[2J\033[H")
                    sys.stdout.flush()
        except KeyboardInterrupt:
            print("\n[rmock_server] stopped by user (KeyboardInterrupt)")

if __name__ == "__main__":
    server_instance = server()
    server_instance.loop()