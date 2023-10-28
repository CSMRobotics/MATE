import rclpy
from rclpy.node import Node

from rov_interfaces.msg import JetsonNanoStatistics
from std_msgs.msg import Float32
from threading import Lock

import psutil
import subprocess
import json

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

        #Put sensor data in a JSON
        sensors = subprocess.run(['sensors', '-j'], stdout=subprocess.PIPE).stdout.decode('utf-8')
        sensor_list = json.loads(sensors)

        #CPU package temps
        msg.cpu_package_cur_temp = sensor_list["tmp102-i2c-0-49"]["temp1"]["temp1_input"]
        msg.cpu_package_max_temp = sensor_list["tmp102-i2c-0-49"]["temp1"]["temp1_max"]

        self.statistics_publisher.publish(msg)
    
    def power_draw(self, msg:Float32):
        self.power_draw_result = msg

def main(args=None):
    rclpy.init(args=args)


if __name__ == '__main__':
    main()
