import rclpy
from rclpy.node import Node 


from std_msgs.msg import String, Int8
from example_interfaces.srv import SetBool


class Scheduler:

    def __init__(self) -> None:
        self.mode = False # false: LO, true: HI
        self.timer = {
            'laser',
        }