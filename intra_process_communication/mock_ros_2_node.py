from intra_process_communication.node_client import client

class mock_ros_node:
    def __init__(self):
        self.topics = {}
        self.connected = False
        self.client = idle_socket_client()

    def socket_connect(self, ip=None):
        try:
            client_instance = client(ip)
            self.client = client_instance
            self.connected = True
        except:
            print("Unable to connect to the socket server.")

    def write_socket_float(self, value_key, value):
        return self.client.set_float(value_key, value)

    def publish(self, value_key, value, msg_type=0):
        if msg_type == 0:
            return self.client.set_float(value_key, value)
        else:
            return False

    def get_topic_data(self, value_key, msg_type=0):
        if msg_type == 0:
            return self.client.read_float(value_key)
        else:
            return False

class idle_socket_client:
    None

mock_ros_node_instance = mock_ros_node()
