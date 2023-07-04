#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
#from geometry_msgs.msg import Twist
from std_msgs.msg import Int64
from lift_interfaces.msg import LiftRequest
from lift_interfaces.msg import LiftStatus

class LiftRequestPub(Node):
	
	def __init__(self):
		super().__init__("lift_request_pub_test")
		self.get_logger().info("lift_request_pub_test started...")
		
		self.lift_pub_ = self.create_publisher(LiftRequest, "/lift_request", 10)
		
		self.timer = self.create_timer(1.0, self.send_lift_request)
	
	def send_lift_request(self):

		self.msg = LiftRequest()
		self.msg.lift_id = 1
		self.msg.robot_id = 2
		self.msg.robot_current_floor = 3
		self.msg.robot_destination_floor = 4
		self.msg.robot_status = 2
		
		self.lift_pub_.publish(self.msg)
	

class LiftStatusPub(Node):
	
	def __init__(self):
		super().__init__("lift_status_pub_test")
		self.get_logger().info("lift_status_pub_test started...")
		
		self.lift_pub_ = self.create_publisher(LiftStatus, "/lift_status", 10)
		
		self.timer = self.create_timer(1.0, self.send_lift_status)
	
	def send_lift_status(self):

		self.msg = LiftStatus()
		self.msg.lift_id = 1
		self.msg.robot_id = 2
		self.msg.lift_current_floor = 3
		self.msg.robot_action = 2
		
		self.lift_pub_.publish(self.msg)

def main(args=None):
	rclpy.init(args=args)
	
	node = LiftStatusPub()
	#node = LiftRequestPub()
	rclpy.spin(node)
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
