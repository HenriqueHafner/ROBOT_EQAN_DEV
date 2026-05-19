import time
import roslibpy #type: ignore
from node_client import client


# Run a ros2 bridge in the ros2 enviroment.
# Using the command: ros2 launch rosbridge_server rosbridge_websocket_launch.xml

class ros2_bridge:
    def __init__(self):
        self.ros2_ip = '127.0.0.1'
        self.ros2_port = 9090
        self.ipc_client = client(ip=self.ros2_ip)
        self.ros = None
        self.topic = None
        self.connect_to_ros2()

    def connect_to_ros2(self):
        if self.ros and self.ros.is_connected:
            return
        self.ros = roslibpy.Ros(host=self.ros2_ip, port=self.ros2_port)
        try:
            self.ros.connect()
            print(f"[ros2_bridge] connected to ros2 via rosbridge on {self.ros2_ip}:9090")
            self.topic = roslibpy.Topic(self.ros, '/ros2_mi', 'std_msgs/String')
            self.topic.advertise()
            print("[ros2_bridge] advertised topic /ros2_mi (std_msgs/String)")
        except Exception as e:
            print(f"[ros2_bridge] ros2 not reachable on {self.ros2_ip}:9090 -> {e}")
            self.ros = None

    def loop(self):
        while True:
            if self.ros and self.ros.is_connected:
                value = self.ipc_client.read("ros2_mi")
                if value is not None:
                    if isinstance(value, bytes):
                        value_str = value.decode(errors='ignore').strip()
                    else:
                        value_str = str(value).strip()

                    if value_str != "Empty" and value_str != "":
                        msg = roslibpy.Message({'data': value_str})
                        self.topic.publish(msg)
                        self.ipc_client.write("ros2_mi", "Empty")
                        print(f"[ros2_bridge] forwarded -> {value_str}")
            else:
                self.connect_to_ros2()

            time.sleep(0.1)

if __name__ == "__main__":
    bridge = ros2_bridge()
    bridge.loop()