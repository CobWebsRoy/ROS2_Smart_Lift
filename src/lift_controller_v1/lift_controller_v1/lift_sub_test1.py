#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from lift_interfaces.msg import LiftRequest
from lift_interfaces.msg import LiftStatus
#from turtlesim.msg import Pose

class LiftRequestSub(Node):
	
	def __init__(self):
		super().__init__("lift_request_sub_node")
		self.get_logger().info("lift_request_sub_node started ... ")
		
		self.create_subscription(
			LiftRequest, "/lift_request", self.lift_request_callback, 10)
		
	def lift_request_callback(self, msg: LiftRequest):
		self.get_logger().info(str(msg))

class LiftStatusSub(Node):
	
	def __init__(self):
		super().__init__("lift_status_sub_node")
		self.get_logger().info("lift_status_sub_node started ... ")
		
		self.create_subscription(
			LiftStatus, "/lift_status", self.lift_status_callback, 10)
		
	def lift_status_callback(self, msg: LiftStatus):
		self.get_logger().info(str(msg))

def main(args=None):
	rclpy.init(args=args)
	
	node = LiftStatusSub()
	#node = LiftRequestSub()
	rclpy.spin(node)
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
