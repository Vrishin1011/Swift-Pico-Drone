#!/usr/bin/env python3

from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
import rclpy
from rclpy.node import Node


class Swift_Pico(Node):
    def __init__(self):
        super().__init__('pico_controller')  # initializing ros node with name pico_controller

        self.drone_position = [0.0, 0.0, 0.0]  # [x, y, z]
        self.setpoint = [2, 2, 19]  # Adjust to [2,2,19] as per the task

        self.cmd = SwiftMsgs()
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

        # PID coefficients for roll, pitch, throttle
        self.Kp = [0, 0, 0]
        self.Ki = [0, 0, 0]
        self.Kd = [0, 0, 0]

        # Variables for storing previous errors, error sum, and limits
        self.prev_error = [0, 0, 0]
        self.error_sum = [0, 0, 0]
        self.max_values = [2000, 2000, 2000]
        self.min_values = [1000, 1000, 1000]

        self.sample_time = 0.060  # 60ms

        # Publishers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)

        # Subscribers
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, "/throttle_pid", self.altitude_set_pid, 1)
        self.create_subscription(PIDTune, "/pitch_pid", self.pitch_set_pid, 1)
        self.create_subscription(PIDTune, "/roll_pid", self.roll_set_pid, 1)

        self.arm()

        # Creating a timer to run the PID function periodically
        self.create_timer(self.sample_time, self.pid)

    def disarm(self):
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        self.disarm()
        self.cmd.rc_roll = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_throttle = 1500
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def whycon_callback(self, msg):
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def altitude_set_pid(self, alt):
        self.Kp[2] = alt.kp * 0.03
        self.Ki[2] = alt.ki * 0.008
        self.Kd[2] = alt.kd * 0.6

    def pitch_set_pid(self, pitch):
        self.Kp[1] = pitch.kp * 0.03
        self.Ki[1] = pitch.ki * 0.008
        self.Kd[1] = pitch.kd * 0.6

    def roll_set_pid(self, roll):
        self.Kp[0] = roll.kp * 0.03
        self.Ki[0] = roll.ki * 0.008
        self.Kd[0] = roll.kd * 0.6

    def pid(self):
        # Compute errors for roll, pitch, and throttle
        error = [self.drone_position[i] - self.setpoint[i] for i in range(3)]
        change_in_error = [(error[i] - self.prev_error[i]) / self.sample_time for i in range(3)]
        self.error_sum = [self.error_sum[i] + error[i] * self.sample_time for i in range(3)]

        # PID output for roll, pitch, and throttle
        out_roll = self.Kp[0] * error[0] + self.Ki[0] * self.error_sum[0] + self.Kd[0] * change_in_error[0]
        out_pitch = self.Kp[1] * error[1] + self.Ki[1] * self.error_sum[1] + self.Kd[1] * change_in_error[1]
        out_throttle = self.Kp[2] * error[2] + self.Ki[2] * self.error_sum[2] + self.Kd[2] * change_in_error[2]

        # Apply corrections
        self.cmd.rc_roll = int(min(max(1500 + out_roll, self.min_values[0]), self.max_values[0]))
        self.cmd.rc_pitch = int(min(max(1500 + out_pitch, self.min_values[1]), self.max_values[1]))
        self.cmd.rc_throttle = int(min(max(1500 + out_throttle, self.min_values[2]), self.max_values[2]))

        # Update previous error
        self.prev_error = error

        # Publish the command
        self.command_pub.publish(self.cmd)

        # Publish PID errors for monitoring
        pid_error_msg = PIDError()
        pid_error_msg.roll_error = error[0]  # Correct field name from PIDError.msg
        pid_error_msg.pitch_error = error[1]  # Correct field name from PIDError.msg
        pid_error_msg.throttle_error = error[2]  # Correct field name from PIDError.msg
        pid_error_msg.yaw_error = 0.0  # Assuming yaw is not controlled at this point
        self.pid_error_pub.publish(pid_error_msg)

def main(args=None):
    rclpy.init(args=args)
    swift_pico = Swift_Pico()
    rclpy.spin(swift_pico)
    swift_pico.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
