#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from lift_interfaces.msg import LiftRequests
from lift_interfaces.msg import LiftStates
from lift_interfaces.msg import ButtonPress
from lift_interfaces.msg import LiftFloor

#This lift_control node will do the following:
#1. recieve LiftRequest from robot_control
#2. call for lift
#3. send LiftStatus to robot_control to get robot to move in
#4. recieve LiftRequest from robot_control that robot is in lift
#5. send lift to destination lift
#6. send LiftStatus to robot_control to get robot to move out


#SUBSCRIPTION NODE FOR LIFT REQUESTS ------
class LiftRequestsSub(Node):
	
	def __init__(self):
		super().__init__("lift_requests_sub_node")
		self.get_logger().info("Lift subscribing to LiftRequests ... ")
		
		self.create_subscription(LiftRequests, "/lift_requests", self.lift_requests_callback, 10)
	
	def lift_requests_callback(self, msg: LiftRequests):
		
		#Checks if Lift ID = 1 and Robot ID = 1
		if msg.lift_id == 1 and msg.robot_id == 1:
			
			#DEBUGGING ------
			#self.get_logger().info("Robot on floor " + str(msg.robot_current_floor) + " requesting to go to floor " + str(msg.robot_destination_floor) + ", current status: " + str(msg.robot_status))

			robot_current_floor = int(msg.robot_current_floor)
			robot_destination_floor = int(msg.robot_destination_floor)
			robot_status = int(msg.robot_status)
		
		#Lift ID != 1 or Robot ID != 1
		else:
			self.get_logger().info("Lift ID or Robot ID invalid")
			
		#Robot is outside calling for lift
		if robot_status == 0:
			
			#instructs lift to go to robot_current_floor
			self.get_logger().info("Press button for floor " + str(robot_current_floor))
			self.button_press_pub_ = self.create_publisher(ButtonPress, "/button_to_press", 10)
			self.msg = ButtonPress()
			self.msg.button_press = robot_current_floor
			self.msg.door_open_close = 1
			self.button_press_pub_.publish(self.msg)
			
		#Robot is inside lift
		if robot_status == 2:
			
			#instructs lift to go to robot_destination_floor
			self.get_logger().info("Press button for floor " + str(robot_destination_floor) + " and release door open button")
			self.button_press_pub_ = self.create_publisher(ButtonPress, "/button_to_press", 10)
			self.msg = ButtonPress()
			self.msg.button_press = robot_destination_floor
			self.msg.door_open_close = 0
			self.button_press_pub_.publish(self.msg)
		
		#Robot is RELEASING lift
		if robot_status == 1:
			
			self.get_logger().info("Robot has released lift, releasing all buttons.")
			self.button_press_pub_ = self.create_publisher(ButtonPress, "/button_to_press", 10)
			self.msg = ButtonPress()
			self.msg.door_open_close = 0
			self.button_press_pub_.publish(self.msg)

#PUBLISHER NODE FOR LIFT STATUS ------
class LiftStatesPub(Node):
	
	def __init__(self):
		super().__init__("lift_states_pub_node")
		self.get_logger().info("Lift publishing to LiftStates ... ")
		
		self.lift_pub_ = self.create_publisher(LiftStates, "/lift_states", 10)

		self.msg = LiftStates()
		self.msg.lift_id = 1
		self.msg.robot_id = 1
		self.msg.lift_current_floor = lift_current_floor
		self.msg.robot_action = 2
		
		self.lift_pub_.publish(self.msg)


#MAIN LOOP ------
def main(args=None):
	rclpy.init(args=args)
	
	LiftRequestsSubNode = LiftRequestsSub()
	
	try:
		while rclpy.ok():

			rclpy.spin(LiftRequestsSubNode)
	except KeyboardInterrupt:
		pass

	LiftRequestsSubNode.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
	main()
