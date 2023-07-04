#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from lift_interfaces.msg import LiftRequests
from lift_interfaces.msg import LiftStates

#This robot_control node will simulate a robot:
#1. send LiftRequest to lift_control to call for lift
#2. receive LiftStatus to move into lift
#3. send LiftRequest to lift_control to indicate inside lift
#4. receive LiftStatus to move out of lift
#5. send LiftRequest to lift_control to indicate out of lift


#SUBSCRIPTION NODE FOR LIFT STATUS ------
class LiftStatesSub(Node):
	
	def __init__(self):
		super().__init__("lift_states_sub_node")
		self.get_logger().info("Robot subscribing to LiftStates ...")
		
		self.create_subscription(LiftStates, "/lift_states", self.lift_states_callback, 10)
		
	def lift_states_callback(self, msg: LiftStates):
		
		#Checks if Lift ID = 1 and Robot ID = 1
		if msg.lift_id == 1 and msg.robot_id == 1:
			self.get_logger().info("Lift on floor " + str(msg.lift_current_floor) + ", calling for robot action " + str(msg.robot_action))
		
		
		#Lift ID != 1 or Robot ID != 1
		else:
			self.get_logger().info("Lift ID or Robot ID invalid")
			
			
#PUBLISHER NODE FOR LIFT REQUESTS ------
class LiftRequestsPub(Node):
	
	def __init__(self):
		super().__init__("lift_requests_pub_node")
		self.get_logger().info("Robot publishing to LiftRequests ... ")
		
		self.lift_pub_ = self.create_publisher(LiftRequests, "/lift_requests", 10)
		
		#self.timer = self.create_timer(1.0, self.send_lift_requests)
	
	def send_lift_requests(self):

		self.msg = LiftRequests()
		self.msg.lift_id = 1
		self.msg.robot_id = 1
		self.msg.robot_current_floor = 2
		self.msg.robot_destination_floor = 3
		self.msg.robot_status = 4
		
		self.lift_pub_.publish(self.msg)
		

#MAIN LOOP ------
def main(args=None):
	rclpy.init(args=args)
	
	
	LiftStatesSubNode = LiftStatesSub()
	#LiftRequestsPubNode = LiftRequestPub()
	
	try:
		while rclpy.ok():
			rclpy.spin(LiftStatesSubNode)
			#rclpy.spin(LiftRequestsPubNode)
	except KeyboardInterrupt:
		pass

	LiftStatesSubNode.destroy_node()
	#LiftRequestsPubNode.destroy_node()
	
	rclpy.shutdown()

if __name__ == '__main__':
	main()

