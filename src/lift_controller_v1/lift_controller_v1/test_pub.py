#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TestPubNode(Node):
	
	def __init__(self):
		super().__init__("test_pub_node")
		self.get_logger().info("test_pub_node started...")
		
		self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
		
		self.timer = self.create_timer(1.0, self.send_velocity_callback)
	
	def send_velocity_callback(self):
		msg = Twist()
		msg.linear.x = 2.0
		msg.angular.z = 1.0
		
		self.cmd_vel_pub_.publish(msg)

def main(args=None):
	rclpy.init(args=args)
	
	node = TestPubNode()
	rclpy.spin(node)
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()
