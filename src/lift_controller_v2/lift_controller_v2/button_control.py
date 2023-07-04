#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from lift_interfaces.msg import LiftRequests
from lift_interfaces.msg import LiftStates
from lift_interfaces.msg import ButtonPress

#SUBSCRIPTION NODE FOR BUTTON PRESSES ------
class ButtonPressSub(Node):
	def __init__(self):
		super().__init__("button_press_sub_node")
		self.get_logger().info("Button Press subscribing to ButtonPress ... ")
		
		self.create_subscription(ButtonPress, "/button_to_press", self.button_press_callback, 10)
		
		self.create_subscription(LiftRequests, "/lift_requests", self.lift_requests_callback, 10)
	
		#subscribe to qr_code to know what is the current floor
		#for now assume:
		self.lift_current_floor_qr = 0
	
	
	def lift_requests_callback(self, msg: LiftRequests):
		if msg.lift_id == 1 and msg.robot_id == 1:
			self.robot_status = int(msg.robot_status)
		else:
			self.get_logger().info("Lift ID or Robot ID invalid")
			
	def button_press_callback(self, msg: ButtonPress):
		button_to_press = int(msg.button_press)
		door_open_close = int(msg.door_open_close)
		
		#delete the following 1 line:
		self.lift_current_floor_qr = button_to_press
		
		#Debugging:
		#self.get_logger().info("Press button for floor " + str(button_to_press))
		
		if button_to_press == 1:
			self.get_logger().info("Press button for floor 1")
			#<insert code to press & release button 1>
			
		elif button_to_press == 2:
			self.get_logger().info("Press button for floor 2")
			#<insert code to press & release button 2>
		
		elif button_to_press == 3:
			self.get_logger().info("Press button for floor 3")
			#<insert code to press & release button 3>
		
		elif button_to_press == 4:
			self.get_logger().info("Press button for floor 4")
			#<insert code to press & release button 4>
			
			
		if door_open_close == 0:
			self.get_logger().info("Release door open button")
			#<insert code to release door open actuator>
			
		elif door_open_close == 1:
			self.get_logger().info("<Nothing happens>")
			#no action here
		
		#following loop will be in the qr_subscribe to compare
		#lift is on the floor that the button has been pressed
		if button_to_press == self.lift_current_floor_qr and button_to_press > 0:
			self.get_logger().info("Lift has reached floor " + str(self.lift_current_floor_qr) + ", pressing door open button")
			#<insert code to hold door open actuator>
			
			if self.robot_status == 0:
			#if robot is outside, call for it to come in
				self.robot_action = 0
				
			elif self.robot_status == 2:
			#if robot is inside, call for it to go out
				self.robot_action = 1
			
			self.lift_pub_ = self.create_publisher(LiftStates, "/lift_states", 10)

			self.msg = LiftStates()
			self.msg.lift_id = 1
			self.msg.robot_id = 1
			#self.msg.lift_current_floor = self.lift_current_floor_qr
			self.msg.lift_current_floor = button_to_press
			self.msg.robot_action = self.robot_action
			
			self.lift_pub_.publish(self.msg)
		
			
#MAIN LOOP ------
def main(args=None):
	rclpy.init(args=args)

	ButtonPressSubNode = ButtonPressSub()
	
	try:
		while rclpy.ok():
			rclpy.spin(ButtonPressSubNode)
	except KeyboardInterrupt:
		pass

	ButtonPressSubNode.destroy_node()

	rclpy.shutdown()

if __name__ == '__main__':
	main()
