import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import time

class SpeedPlotter(Node):
    def __init__(self):
        super().__init__('speed_plotter')
        self.subscription = self.create_subscription(
            Float32,
            'current_speed',  # topic published by vehicle_model
            self.listener_callback,
            10)
        
        self.speeds = []
        self.times = []
        self.start_time = time.time()
        self.target_speed = 60.0  # desired speed in km/h

        # enable interactive plotting
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-', label="Current Speed (km/h)")
        self.target_line, = self.ax.plot([], [], 'r--', label="Target Speed (60 km/h)")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (km/h)")
        self.ax.set_title("Vehicle Speed vs Time")
        self.ax.grid(True)
        self.ax.legend()

    def listener_callback(self, msg):
        t = time.time() - self.start_time
        self.speeds.append(msg.data)
        self.times.append(t)

        # update actual speed
        self.line.set_xdata(self.times)
        self.line.set_ydata(self.speeds)

        # update target line (constant 60 km/h)
        self.target_line.set_xdata(self.times)
        self.target_line.set_ydata([self.target_speed] * len(self.times))

        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = SpeedPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()

if __name__ == '__main__':
    main()
