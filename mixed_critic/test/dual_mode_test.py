import rclpy 
from rclpy.node import Node 

from std_msgs.msg import String, Int8
from example_interfaces.srv import SetBool
import random

"""
SpinOnce 같은 함수로 Timer 흉내
모드에 따른 주기 변화를 꾀함

while not rospy.is_shutdown():
    ...
    rospy.spinOnce()
    if mode == 1:
        rospy.sleep(...)
    else:
        rospy.sleep(...)
"""
rclpy.init()
mode = False

class DualMode(Node):
    def __init__(self):
        super().__init__('dual_mode_test')
        
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.mode_sub = self.create_service(SetBool, 'mode_set', self.mode_change)
        # self.mode_sub = self.create_subscription(Int8, 'mode_broadcast', self.mode_change, 10)
        self.i = 0

    def mode_change(self, request: SetBool.Request, response: SetBool.Response):
        global mode 
        mode = request.data
        response.message = f"Mode Changed to {mode}"
        response.success = True
        return response

    def pub(self):
        self.get_logger().info(f"Node Published {self.i} / Mode: {mode}")
        self.i += 1


def main():
    node = DualMode()
    global mode

    while rclpy.ok():
        node.pub()
        if not mode:
            rclpy.spin_once(node, timeout_sec=1.0)
        else:
            rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    


    
