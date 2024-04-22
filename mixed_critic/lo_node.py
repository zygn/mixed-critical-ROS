import rclpy 
import time
import threading
import numpy as np
from rclpy.node import Node 
from rclpy.timer import Rate
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

from mc_msgs.srv import Criticality

mode_str = ['LO', 'HI']

class SimpleDriving:

    def __init__(self) -> None:

        self._steering_angle = 0.0
        self._speed = 1.0
        self._desired_distance = 0.75

    def plan(self, scan_data: LaserScan):

        if not scan_data:
            return 0.0, 0.0
        
        scan = scan_data.ranges

        if len(scan) == 0:
            return 0.0, 0.0

        # Laser scan data is 270 degress, and angular resolution is 0.25 degrees
        #  = 1080 data points

        # Left side
        if scan[720] < self._desired_distance:
            self._steering_angle = -0.5
        # Right side
        elif scan[360] < self._desired_distance:
            self._steering_angle = 0.5
        else:
            self._steering_angle = 0.0

        return self._speed, self._steering_angle
    
class FgPlanner:
    BUBBLE_RADIUS = 160
    PREPROCESS_CONV_SIZE = 100  # PREPROCESS_consecutive_SIZE
    BEST_POINT_CONV_SIZE = 80
    MAX_LIDAR_DIST = 3000000
    STRAIGHTS_STEERING_ANGLE = np.pi / 18  # 10 degrees

    def __init__(self):
        self.robot_scale = 0.3302
        self.radians_per_elem = None
        self.STRAIGHTS_SPEED = 8.0
        self.CORNERS_SPEED = 4.0

    def preprocess_lidar(self, ranges):

        self.radians_per_elem = ((3/2) * np.pi) / len(ranges)
        proc_ranges = np.array(ranges[180:-180])
        proc_ranges = np.convolve(proc_ranges, np.ones(self.PREPROCESS_CONV_SIZE), 'same') / self.PREPROCESS_CONV_SIZE
        proc_ranges = np.clip(proc_ranges, 0, self.MAX_LIDAR_DIST)
        return proc_ranges

    def find_max_gap(self, free_space_ranges):

        masked = np.ma.masked_where(free_space_ranges == 0, free_space_ranges)
        slices = np.ma.notmasked_contiguous(masked)
        max_len = slices[0].stop - slices[0].start
        chosen_slice = slices[0]
        for sl in slices[1:]:
            sl_len = sl.stop - sl.start
            if sl_len > max_len:
                max_len = sl_len
                chosen_slice = sl
        return chosen_slice.start, chosen_slice.stop

    def find_best_point(self, start_i, end_i, ranges):

        averaged_max_gap = np.convolve(ranges[start_i:end_i], np.ones(self.BEST_POINT_CONV_SIZE),
                                       'same') / self.BEST_POINT_CONV_SIZE
        return averaged_max_gap.argmax() + start_i

    def get_angle(self, range_index, range_len):

        lidar_angle = (range_index - (range_len / 2)) * self.radians_per_elem
        steering_angle = lidar_angle / 2

        return steering_angle

    def plan(self, scan_data: LaserScan, odom_data=None):
        if not scan_data:
            return 0.0, 0.0
        
        ranges = scan_data.ranges
        proc_ranges = self.preprocess_lidar(ranges)
        closest = proc_ranges.argmin()

        min_index = closest - self.BUBBLE_RADIUS
        max_index = closest + self.BUBBLE_RADIUS
        if min_index < 0: min_index = 0
        if max_index >= len(proc_ranges): max_index = len(proc_ranges) - 1
        proc_ranges[min_index:max_index] = 0

        gap_start, gap_end = self.find_max_gap(proc_ranges)

        best = self.find_best_point(gap_start, gap_end, proc_ranges)

        steering_angle = self.get_angle(best, len(proc_ranges))
        if abs(steering_angle) > self.STRAIGHTS_STEERING_ANGLE:
            speed = self.CORNERS_SPEED
        else:
            speed = self.STRAIGHTS_SPEED
        # print('Steering angle in degrees: {}'.format((steering_angle / (np.pi / 2)) * 90))
        # print(f"Speed: {speed}")
        return speed, steering_angle

class LoNode(Node):

    def __init__(self):
        super().__init__("lo_node")

        scan_topic = "/scan"
        odom_topic = "/odom"
        drive_topic = "/drive"

        self.mode = 0

        self.hi_mode_hz = 5
        self.hi_mode_sec = 1/self.hi_mode_hz
        self.lo_mode_hz = 10
        self.lo_mode_sec = 1/self.lo_mode_hz

        
        self.data = {
            "scan": None,
            "odom": None,
        }

        self._scan_sub = self.create_subscription(msg_type=LaserScan, topic=scan_topic, callback=self.scan_callback, qos_profile=qos_profile_sensor_data)
        self._odom_sub = self.create_subscription(msg_type=Odometry, topic=odom_topic, callback=self.odom_callback, qos_profile=qos_profile_sensor_data)
        self._drive_pub = self.create_publisher(msg_type=AckermannDriveStamped, topic=drive_topic, qos_profile=qos_profile_system_default)
        
        self._mode_sub = self.create_service(Criticality, 'criticality', self.mode_change)

        self.timestamp = self.get_clock().now()
        self.drive_planner = FgPlanner()

    def ignoring_by_clock(self) -> bool:
        now_clock = self.get_clock_sec()
        last_tick = self.get_clock_sec(self.timestamp)
        if self.mode == 0:
            if now_clock - last_tick >= self.lo_mode_sec:
                return True 
            else:
                return False
        elif self.mode == 1:
            if now_clock - last_tick >= self.hi_mode_sec:
                return True 
            else:
                return False

    def mode_change(self, request: Criticality.Request, response: Criticality.Response):
        self.mode = request.mode
        response.result = True
        return response
    
    def scan_callback(self, msg: LaserScan):
        self.data["scan"] = msg
    
    def odom_callback(self, msg: Odometry):
        self.data["odom"] = msg
    
    def drive_publish(self):
        t0 = self.get_clock_sec()

        speed, steer = self.drive_planner.plan(self.data["scan"])

        ackermann_data = AckermannDriveStamped()
        ackermann_data.drive.speed = speed
        ackermann_data.drive.steering_angle = steer
        self._drive_pub.publish(ackermann_data)
        t1 = self.get_clock_sec()
        self.get_logger().info(f"{t1-t0}")
    
    def get_clock_sec(self, time=None):
        if not time:
            clock = self.get_clock().now().seconds_nanoseconds()
        else:
            clock = time.seconds_nanoseconds()
        return clock[0] + (clock[1]*10**-9)
    

    def pub(self):
        # self.get_logger().info("LO Node try to publish")
        if not self.ignoring_by_clock():
            return 
        self.drive_publish()
        end_tick = self.get_clock().now()
        duration = self.get_clock_sec(end_tick) - self.get_clock_sec(self.timestamp)
        self.get_logger().info(f"LO-Criticality Node published (Current mode: {mode_str[self.mode]}, Duration: {round(duration, 4)})")
        self.timestamp = end_tick
        

    

def main():
    rclpy.init()
    node = LoNode()
    while rclpy.ok():
        node.pub()
        if node.mode == 1:
            rclpy.spin_once(node, timeout_sec=node.hi_mode_sec)
        else:
            rclpy.spin_once(node, timeout_sec=node.lo_mode_sec)
        
    
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()