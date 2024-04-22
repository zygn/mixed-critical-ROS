import rclpy 
from rclpy.node import Node 

from mc_msgs.srv import Criticality


class Broadcaster(Node):

    def __init__(self):
        super().__init__("broadcast_node")

