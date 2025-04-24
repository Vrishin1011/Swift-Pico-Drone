import socket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler


# Change HOST to 'localhost' or '0.0.0.0' if testing locally
HOST = '192.168.1.4'  # Update this to your local machine's IP address
PORT = 2055  # Ensure this is consistent on both client and server


def euler_to_quaternion(roll, pitch, yaw):
    quaternion = quaternion_from_euler(roll, pitch, yaw)
    return quaternion


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.imu_pub = self.create_publisher(Imu, '/phone_imu', 10)
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)
        self.get_logger().info(f"Server listening on {HOST}:{PORT}")
        self.rate = self.create_rate(10)  # 10 Hz

    def start_server(self):
        client_socket, client_address = self.server_socket.accept()
        self.get_logger().info(f"Connected to client: {client_address}")
        client_socket.settimeout(5)

        while rclpy.ok():
            try:
                data = client_socket.recv(1024)
                if not data:
                    raise socket.timeout("No data received")
            except socket.timeout:
                self.get_logger().info("Timeout occurred. No data received.")
                break

            data1 = data.decode("utf-8").replace("\r", "").replace("\n", "")
            values = data1.split(',')

            try:
                values.remove('')  # Remove empty values
            except ValueError:
                pass

            if len(values) == 9:  # Ensure the expected number of values is received
                self.publish_imu(values)
            else:
                continue

        client_socket.close()
        self.server_socket.close()

    def publish_imu(self, values):
        imu_msg = Imu()
        imu_msg.header.frame_id = 'map'

        # Assigning angular velocity
        imu_msg.angular_velocity.x = float(values[0])
        imu_msg.angular_velocity.y = float(values[1])
        imu_msg.angular_velocity.z = float(values[2])

        # Assigning linear acceleration
        imu_msg.linear_acceleration.x = float(values[3])
        imu_msg.linear_acceleration.y = float(values[4])
        imu_msg.linear_acceleration.z = float(values[5])

        # Converting Euler angles to Quaternion
        w, x, y, z = euler_to_quaternion(float(values[6]), float(values[7]), float(values[8]))
        imu_msg.orientation.x = y
        imu_msg.orientation.y = z
        imu_msg.orientation.z = x
        imu_msg.orientation.w = w

        # Publish the IMU message
        self.imu_pub.publish(imu_msg)
        self.get_logger().info(f"Published IMU data: {values}")


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    node.start_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()