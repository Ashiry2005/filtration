#!/usr/bin python3.10
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class VehicleSim(Node):
    def __init__(self):
        super().__init__('vehicle_sim')

        self.subscription = self.create_subscription(Float32, '/cmd_vel', self.control_callback, 10)
        self.speed_pub = self.create_publisher(Float32, '/current_speed', 10)
        self.timer = self.create_timer(0.05, self.update)  # 20 Hz
        self.msg = Float32()

        self.speed = 0.0
        self.control = 0.0
        self.alpha = 0.15
        self.dt = 0.05

    def control_callback(self, msg):
        self.control = max(min(float(msg.data), 1.0), -1.0) # expected range: -1..1 for brake/throttle

    def update(self):
        acceleration = self.control * 5 - self.alpha * self.speed
        self.speed += acceleration * self.dt
        if self.speed < 0:
            self.speed = 0.0

        self.msg.data = self.speed * 3.6
        self.speed_pub.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
