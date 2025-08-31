import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time
import matplotlib.pyplot as plt


# ------------------------------
# PID Controller Class
# ------------------------------
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None

    def compute(self, target, current):
        error = target - current
        now = time.time()

        if self.prev_time is None:
            dt = 0.05  # assume 50ms for first loop
        else:
            dt = now - self.prev_time
        self.prev_time = now

        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = max(min(self.integral, 10.0), -10.0)

        # Derivative term
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # PID output
        u = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Clamp between [-1, 1]
        u = max(min(u, 1.0), -1.0)

        return u


# ------------------------------
# ROS2 Controller Node
# ------------------------------
class SpeedController(Node):
    def __init__(self):
        super().__init__('speed_controller')

        # Target speed (km/h)
        self.target_speed = 60.0  

        # Tune these for your task
        self.pid = PID(Kp=0.5, Ki=0.85, Kd=0.07)

        # Data logging for plotting
        self.start_time = time.time()
        self.time_log = []
        self.speed_log = []

        # Subscriber to current speed
        self.sub = self.create_subscription(
            Float32,
            '/current_speed',
            self.speed_callback,
            10
        )

        # Publisher to command velocity
        self.pub = self.create_publisher(Float32, '/cmd_vel', 10)

    def speed_callback(self, msg):
        current_speed = msg.data
        control_signal = self.pid.compute(self.target_speed, current_speed)

        # Publish throttle/brake
        cmd = Float32()
        cmd.data = control_signal
        self.pub.publish(cmd)

        # Log for plotting
        t = time.time() - self.start_time
        self.time_log.append(t)
        self.speed_log.append(current_speed)

        # Print for debugging
        self.get_logger().info(
            f"t={t:.2f}s | Target={self.target_speed:.2f} | "
            f"Speed={current_speed:.2f} | Control={control_signal:.2f}"
        )

    def plot_results(self):
        # Plot speed vs time
        plt.figure(figsize=(8,5))
        plt.plot(self.time_log, self.speed_log, label="Vehicle Speed")
        plt.axhline(self.target_speed, color='r', linestyle='--', label="Target Speed")
        plt.xlabel("Time (s)")
        plt.ylabel("Speed (km/h)")
        plt.title("Speed vs Time (PID Controller)")
        plt.grid(True)
        plt.legend()
        plt.show()


# ------------------------------
# Main
# ------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = SpeedController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # allow Ctrl+C to stop
    finally:
        node.plot_results()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
