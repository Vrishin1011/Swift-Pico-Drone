import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QLabel, QPushButton
from PyQt5.QtCore import QTimer
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class IMUGUI(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'imu_gui')
        QMainWindow.__init__(self)

        # Subscribe to the IMU data topic
        self.imu_sub = self.create_subscription(Imu, '/phone_imu', self.imu_callback, 10)

        # Initialize the GUI
        self.init_ui()

        # Variables to store IMU data
        self.orientation = [0, 0, 0, 1]  # Quaternion (w, x, y, z)
        self.angular_velocity = [0, 0, 0]
        self.linear_acceleration = [0, 0, 0]
        self.display_euler = False  # Toggle flag for orientation display

        # Timer for ROS2 spin
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self.ros_spin)
        self.ros_timer.start(100)  # Spin ROS2 every 100ms

    def init_ui(self):
        self.setWindowTitle('IMU Data Display')
        self.setGeometry(100, 100, 400, 300)  # Set window size

        # Create a central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Create a vertical layout for the central widget
        self.layout = QVBoxLayout(central_widget)

        # Labels for displaying the IMU values
        self.label_orientation = QLabel('Orientation: (0, 0, 0, 1)', self)
        self.label_angular_velocity = QLabel('Angular Velocity: (0, 0, 0)', self)
        self.label_linear_acceleration = QLabel('Linear Acceleration: (0, 0, 0)', self)

        # Button to toggle between Euler and Quaternion
        self.toggle_button = QPushButton('Toggle to Euler', self)
        self.toggle_button.clicked.connect(self.toggle_orientation_display)

        # Add widgets to the layout
        self.layout.addWidget(self.label_orientation)
        self.layout.addWidget(self.label_angular_velocity)
        self.layout.addWidget(self.label_linear_acceleration)
        self.layout.addWidget(self.toggle_button)

        # Timer to update the display at regular intervals
        self.display_timer = QTimer(self)
        self.display_timer.timeout.connect(self.update_display)
        self.display_timer.start(100)  # Update every 100 ms

    def toggle_orientation_display(self):
        self.display_euler = not self.display_euler
        if self.display_euler:
            self.toggle_button.setText('Toggle to Quaternion')
        else:
            self.toggle_button.setText('Toggle to Euler')

    def imu_callback(self, msg):
        self.angular_velocity = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        self.linear_acceleration = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
        self.orientation = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]

    def update_display(self):
        # Update labels with actual IMU data
        self.label_angular_velocity.setText(f'Angular Velocity: {self.angular_velocity}')
        self.label_linear_acceleration.setText(f'Linear Acceleration: {self.linear_acceleration}')

        if self.display_euler:
            # Convert quaternion to Euler angles
            euler = euler_from_quaternion(self.orientation)
            self.label_orientation.setText(f'Orientation (Euler): {euler}')
        else:
            self.label_orientation.setText(f'Orientation (Quaternion): {self.orientation}')

        # Force the layout to update
        self.layout.update()

    def ros_spin(self):
        """Spin the ROS2 node once per timer tick."""
        rclpy.spin_once(self, timeout_sec=0.1)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)

    # Instantiate the GUI node
    imu_gui = IMUGUI()

    # Start the PyQt event loop
    imu_gui.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
