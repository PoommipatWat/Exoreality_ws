import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

import numpy as np

class Omni_kinematics:
    def __init__(self, Distance, Radius, initial_theta=0) -> None:
        self.Distance = Distance
        self.initial_theta = initial_theta
        self.Radius = Radius

    def inverse_omni(self, vx, vy, w, max_rpm, filter=True): # max_rpm for 2.5 m/s at 41 mm self.Radius
        instance = np.array([[-np.sin(self.initial_theta), np.cos(self.initial_theta), self.Distance],
                            [-np.sin(np.pi/3 - self.initial_theta), -np.cos(np.pi/3 - self.initial_theta), self.Distance],
                            [np.sin(np.pi/3 + self.initial_theta), -np.cos(np.pi/3 + self.initial_theta), self.Distance]])
        v1, v2, v3 = instance @ np.array([vx, vy, w])

        # Convert linear velocities to angular velocities (radians per second)
        omega1 = v1 / self.Radius
        omega2 = v2 / self.Radius
        omega3 = v3 / self.Radius

        # Convert angular velocities to revolutions per minute (rpm)
        rpm1 = omega1 * 60 / (2 * np.pi)
        rpm2 = omega2 * 60 / (2 * np.pi)
        rpm3 = omega3 * 60 / (2 * np.pi)

        if filter:
            if abs(rpm1) > max_rpm or abs(rpm2) > max_rpm or abs(rpm3) > max_rpm:
                scale_factor = max_rpm / max(abs(rpm1), abs(rpm2), abs(rpm3))
                rpm1 *= scale_factor
                rpm2 *= scale_factor
                rpm3 *= scale_factor

        return rpm1, rpm2, rpm3

    def forward_omni(self, rpm1, rpm2, rpm3): # R in meters
        # Convert RPM to radians per second
        omega1 = rpm1 * 2 * np.pi / 60
        omega2 = rpm2 * 2 * np.pi / 60
        omega3 = rpm3 * 2 * np.pi / 60

        # Convert angular velocities to linear velocities
        v1 = omega1 * self.Radius
        v2 = omega2 * self.Radius
        v3 = omega3 * self.Radius

        instance = np.array([[-np.sin(self.initial_theta), np.cos(self.initial_theta), self.Distance],
                            [-np.sin(np.pi/3 - self.initial_theta), -np.cos(np.pi/3 - self.initial_theta), self.Distance],
                            [np.sin(np.pi/3 + self.initial_theta), -np.cos(np.pi/3 + self.initial_theta), self.Distance]])
        vx, vy, w = np.linalg.inv(instance) @ np.array([v1, v2, v3])
        return vx, vy, w

class Bringup(Node):
    def __init__(self):
        super().__init__('bringup')

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel/rpm', 10)
        self.publisher2_ = self.create_publisher(Twist, 'cmd_vel/vel', 10)

        self.timer_ = self.create_timer(0.01, self.timer_callback) # hz = 100
        
        self.subscriptions_ = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.subscriptions

        self.subscriptions2 = self.create_subscription(Twist, 'rpm', self.cmd_vel_callback2, 10)
        self.subscriptions2
        self.rpm = Twist()

        self.wheel_dimeter = 0.082
        self.wheel_distance = 0.1
        self.max_rpm = 400
        self.factor_rpm = 0.85

        self.omni = Omni_kinematics(0.10594374972, 0.041)

        self.cmd_vel = Twist()

        self.get_logger().info("Robot Bringup node has been started")

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg

    def cmd_vel_callback2(self, msg):
        vx, vy, w = self.omni.forward_omni(msg.linear.x, msg.linear.y, msg.linear.z)
        self.rpm.linear.x = vx
        self.rpm.linear.y = vy
        self.rpm.angular.z = w

    def timer_callback(self):
        rpm = self.omni.inverse_omni(self.cmd_vel.linear.x, self.cmd_vel.linear.y, self.cmd_vel.angular.z, self.max_rpm*self.factor_rpm, filter=True)
        rpm_msg = Twist()
        rpm_msg.linear.x = rpm[0]
        rpm_msg.linear.y = rpm[1]
        rpm_msg.angular.z = rpm[2]
        self.publisher_.publish(rpm_msg)

        vel_msg = Twist()
        vel_msg.linear.x = self.rpm.linear.x
        vel_msg.linear.y = self.rpm.linear.y
        vel_msg.angular.z = self.rpm.angular.z
        self.publisher2_.publish(vel_msg)
        

def main():
	rclpy.init()
	
	node = Bringup()
	rclpy.spin(node)
	
	rclpy.shutdown()
	
if __name__ == "__main__":
	main()