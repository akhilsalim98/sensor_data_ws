import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import UInt16, Int16
from std_srvs.srv import SetBool
from example_interfaces.srv import SetInt32
import socket
import struct

class Sensor_Node(Node):
    def __init__(self):
        super().__init__('sensor_node')

        self.declare_parameter('interval', 1000)  # Interval set as 1000ms(1sec)

        self.supply_voltage_pub = self.create_publisher(UInt16, 'supply_voltage', 10)
        self.env_temp_pub = self.create_publisher(Int16, 'env_temp', 10)
        self.yaw_pub = self.create_publisher(Int16, 'yaw', 10)
        self.pitch_pub = self.create_publisher(Int16, 'pitch', 10)
        self.roll_pub = self.create_publisher(Int16, 'roll', 10)

        self.start_service = self.create_service(SetInt32, 'start_sensor', self.handle_start_sensor)
        self.stop_service = self.create_service(SetBool, 'stop_sensor', self.handle_stop_sensor)

        self.sensor_ip = '192.168.0.100'  # Sample IP for the time being
        self.sensor_port = 2000
        self.socket = None

        self.create_timer(1.0, self.initialize_sensor)

    def initialize_sensor(self):
        self.get_logger().info("Executing Sensor Initialization")
        self.send_start_command(self.get_parameter('interval').value)

    def connect_to_sensor(self):
        if self.socket is None:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.sensor_ip, self.sensor_port))
            except Exception as e:
                self.get_logger().error(f"Sesnsor connection failed : {e}")

    def send_start_command(self, interval):
        self.connect_to_sensor()
        if self.socket:
            command_id = '03'
            payload = struct.pack('<H', interval)
            message = self.build_message('#', command_id, payload)
            self.socket.sendall(message)
            self.receive_sensor_data()

    def send_stop_command(self):
        if self.socket:
            command_id = '09'
            message = self.build_message('#', command_id, b'')
            self.socket.sendall(message)
            self.socket.close()
            self.socket = None

    def receive_sensor_data(self):
        while self.socket:
            try:
                data = self.socket.recv(1024)
                if not data:
                    break
                self.process_sensor_data(data.decode())
            except Exception as e:
                self.get_logger().error(f"Error receiving data: {e}")
                break

    def process_sensor_data(self, message):
        if message.startswith('$'):
            command_id = message[1:3]
            if command_id == '11': 
                payload = bytes.fromhex(message[3:-4])
                self.decode_status_message(payload)

    def decode_status_message(self, payload):
        try:
            supply_voltage, env_temp, yaw, pitch, roll = struct.unpack('<Hhhhh', payload)

            self.supply_voltage_pub.publish(UInt16(data=supply_voltage))
            self.env_temp_pub.publish(Int16(data=env_temp))
            self.yaw_pub.publish(Int16(data=yaw))
            self.pitch_pub.publish(Int16(data=pitch))
            self.roll_pub.publish(Int16(data=roll))

            self.get_logger().info(f"Received Data: Voltage={supply_voltage}mV, Temp={env_temp}dC, Yaw={yaw}°, Pitch={pitch}°, Roll={roll}°")
        except Exception as e:
            self.get_logger().error(f"Decoding Error: {e}")

    def build_message(self, start_char, command_id, payload):
        payload_hex = payload.hex().upper()
        return f"{start_char}{command_id}{payload_hex}\r\n".encode()

    def handle_start_sensor(self, request, response):
        try:
            self.send_start_command(request.data)
            response.success = True
            response.message = f"Sensor started"
        except Exception as e:
            response.success = False
            response.message = f"Failed to start sensor: {e}"
        return response

    def handle_stop_sensor(self, request, response):
        try:
            self.send_stop_command()
            response.success = True
            response.message = "Sensor stopped"
        except Exception as e:
            response.success = False
            response.message = f"Failed to stop sensor: {e}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = Sensor_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
