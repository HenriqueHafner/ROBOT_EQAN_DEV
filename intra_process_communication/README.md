# Tankbotics Intra Process Communication

## Purpose

This package provides a lightweight and robust **Inter-Process Communication (IPC)** system via TCP.

Its main goal is to allow different processes (ROS2 nodes, simulators, test tools, scripts, etc.) to share variables in real time in a simple and reliable way.

- A central **server** (`server_ros2_bridge.py`) stores all variables and displays a live, clean view on the terminal.
- Any number of **clients** (`node_client.py`) can connect, read, and write variables with automatic reconnection.


## Simple Usage Example

### 1. Start the Server (first terminal)

python server_ros2_bridge.py

### 2. Test with Interactive Python Client (second terminal)

```bash
python -i node_client.py
```

Inside the Python REPL run:
```bash
>>> node.set_float("test_motor_speed", 123.45)
>>> node.set_float("test_voltage", 13.8)
>>> print(node.read_float("test_motor_speed"))
```


The variables appear and update in real time in the server window.
"""

### Supported Message / Variable Types

**Float (4 bytes)**  
Most common type for real-time values (speed, voltage, position, etc.).  
- **Write**: `node.set_float("var_name", 123.45)` or `node.write("var_name", 123.45)`  
- **Read**: `node.read_float("var_name")` or `node.read("var_name")` (auto-detects 4 bytes)  
- Server shows formatted scientific notation (`+123.4500 e0`)

**Raw Bytes (any length)** 
General purpose for strings, custom structs or longer data.  
- **Write**: `node.write("data", b'\x01\x02\x03')` or `node.write("text", "hello")`  
- **Read**: `node.read("data")` returns `bytes`  
- Server terminal shows hex  dump if value data length > 4 bytes
- Server terminal shows bits dump if value data length = 3 bytes


**String**  
Text messages (encoded as UTF-8 bytes).  
- **Write**: `node.write("message", "hello world")`  
- **Read**: `node.read("message")` returns `bytes` (decode with `.decode()` if needed)