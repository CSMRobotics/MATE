import rclpy
from rclpy.node import Node

from rov_interfaces.msg import JetsonNanoStatistics
from std_msgs.msg import Float32
from threading import Lock

import psutil

class ROV_Statistics(Node):
    def __init__(self) -> None:
        super().__init__(self, "rov_statistics")
        self.statistics_timer = self.create_timer(5, self.statistics)
        self.statistics_publisher = self.create_publisher(JetsonNanoStatistics, "rov_statistics", 2)
        self.power_draw_subscriber = self.create_subscription(Float32, "power_draw", self.power_draw, 2)
        self.power_draw_result = 0.0
        self.power_draw_mutex = Lock()

    def statistics(self):
        msg = JetsonNanoStatistics()

        self.power_draw_mutex.acquire()
        msg.power_draw = self.power_draw_result
        self.power_draw_mutex.release()
        
        msg.cpu_usage = psutil.cpu_percent(interval=None)

        freq = psutil.cpu_freq()
        msg.cur_cpu_freq = freq.current
        msg.min_cpu_freq = freq.min
        msg.max_cpu_freq = freq.max

        msg.memory_usage = psutil.virtual_memory().used

        # TODO: decode this crap into cpu core temps, package temps, and carrier board temps
        sensors = psutil.sensors_temperatures()
        for name, entries in sensors.items():
            print(name)
            for entry in entries:
                print("    %-20s %s °C (high = %s °C, critical = %s °C)" % (entry.label or name, entry.current, entry.high, entry.critical))
            print()

        self.statistics_publisher.publish(msg)
    
    def power_draw(self, msg:Float32):
        self.power_draw_result = msg

def main(args=None):
    rclpy.init(args=args)


if __name__ == '__main__':
    main()
