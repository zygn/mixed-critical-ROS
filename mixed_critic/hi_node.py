import rclpy 
from rclpy.node import Node 
from rclpy.qos import qos_profile_system_default

import cv2
import time

from ackermann_msgs.msg import AckermannDriveStamped
from mc_msgs.srv import Criticality

mode_str = ['LO', 'HI']

class SimpleObstacleDetection():

    def __init__(self) -> None:
        self.cap = cv2.VideoCapture(0)

        if self.cap.isOpened():
            self.cam_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            self.cam_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            self.centerline = self.cam_width / 2

        self.tracker = cv2.legacy.TrackerBoosting.create()

    def plan(self):

        ret, frame = self.cap.read()
        if ret:

            ok, bbox = self.tracker.update(frame)
            if not ok:
                return 0.0, 0.0 

            if bbox[0] > self.centerline:
                return 1.0, 1.0
            else:
                return 1.0, -1.0

        else:
            return 0.0, 0.0 



class HiNode(Node):

    def __init__(self):
        super().__init__("hi_node")

        drive_topic = "/drive"

        self.hi_mode_hz = 1/5
        self.lo_mode_hz = 1/2
        self.mode = 0
        
        self.data = {
            "obstacle_bbox": None 
        }

        self._drive_pub = self.create_publisher(msg_type=AckermannDriveStamped, topic=drive_topic, qos_profile=qos_profile_system_default)
        self._mode_sub = self.create_service(Criticality, 'criticality', self.mode_change)
        self.timestamp = self.get_clock().now()
        self.drive_planner = SimpleObstacleDetection()

    def mode_change(self, request: Criticality.Request, response: Criticality.Response):
        self.mode = request.mode
        response.result = True
        return response
    
    def get_clock_sec(self, time=None):
        if not time:
            clock = self.get_clock().now().seconds_nanoseconds()
        else:
            clock = time.seconds_nanoseconds()
        return clock[0] + (clock[1]*10**-9)

    def drive_publish(self):
        t0 = self.get_clock_sec()
        speed, steer = self.drive_planner.plan()

        ackermann_data = AckermannDriveStamped()
        ackermann_data.drive.speed = speed
        ackermann_data.drive.steering_angle = steer
        self._drive_pub.publish(ackermann_data)
        t1 = self.get_clock_sec()
        self.get_logger().info(f"{t1-t0}")

    def pub(self):
        # self.get_logger().info("HI Node try to publish")
        
        self.drive_publish()
        end_tick = self.get_clock().now()
        duration = self.get_clock_sec(end_tick) - self.get_clock_sec(self.timestamp)
        self.get_logger().info(f"HI-Criticality Node published (Current mode: {mode_str[self.mode]}, Duration: {round(duration, 4)})")
        self.timestamp = end_tick
    


def main():
    rclpy.init()
    node = HiNode()

    while rclpy.ok():
        node.pub()
        if node.mode == 1:
            rclpy.spin_once(node, timeout_sec=node.hi_mode_hz)
        else:
            rclpy.spin_once(node, timeout_sec=node.lo_mode_hz)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    

