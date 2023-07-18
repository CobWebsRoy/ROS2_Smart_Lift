#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from lift_interfaces.msg import LiftRequests
from lift_interfaces.msg import LiftStates
from lift_interfaces.msg import ButtonPress
from lift_interfaces.msg import LiftFloor

#FOR SERVOS ON I2C PCA9685
from adafruit_servokit import ServoKit
from time import sleep
kit=ServoKit(channels=16)
servo=16
#-------------------------




#FOR RELAY AND SOLENOID
#import RPi.GPIO as GPIO
#import time

#GPIO.setmode(GPIO.BCM)
#RELAY_PIN_FLOOR_2 = 18
#GPIO.setup(RELAY_PIN_FLOOR_2, GPIO.OUT)

#GPIO.output(RELAY_PIN_FLOOR_2, GPIO.HIGH)
#time.sleep(1)
#GPIO.output(RELAY_PIN_FLOOR_2, GPIO.LOW)
#----------------------

#SUBSCRIPTION NODE FOR BUTTON PRESSES ------
class ButtonPressSub(Node):
	def __init__(self):
		super().__init__("button_press_sub_node")
		self.get_logger().info("Button Press subscribing to ButtonPress ... ")
		kit.servo[0].angle=90
		kit.servo[1].angle=90
		kit.servo[2].angle=90
		
		#Subscribes to robot to know if robot is inside lift or outside lift 
		self.create_subscription(LiftRequests, "/lift_requests", self.lift_requests_callback, 10)
		
		self.lift_current_floor_qr = 0
		self.do_once = 0
	
		#Subscribes to lift_control to know which button to press now
		self.create_subscription(ButtonPress, "/button_to_press", self.button_press_callback, 10)
		
		#Subscribes to QR Decoder to know what floor the lift is currently on	
		self.create_subscription(LiftFloor, "/lift_current_floor", self.lift_current_floor_callback, 10)
	
	
	def lift_requests_callback(self, msg: LiftRequests):
		if msg.lift_id == 1 and msg.robot_id == 1:
			self.robot_status = int(msg.robot_status)
		else:
			self.get_logger().info("Lift ID or Robot ID invalid")
			
	def lift_current_floor_callback(self, msg: LiftFloor):
		self.lift_current_floor_qr = int(msg.lift_current_floor)
		#self.get_logger().info("Lift currently on floor " + str(self.lift_current_floor_qr))
		
		#following loop will be in the qr_subscribe to compare
		#lift is on the floor that the button has been pressed
		if self.button_to_press == self.lift_current_floor_qr and self.do_once == 1:
			self.get_logger().info("Lift has reached floor " + str(self.lift_current_floor_qr) + ", pressing door open button")
			#<insert code to hold door open actuator>
			kit.servo[0].angle = 150
			
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
			self.msg.lift_current_floor = self.button_to_press
			self.msg.robot_action = self.robot_action
			
			self.lift_pub_.publish(self.msg)
			
			self.do_once = 0
			
			
	def button_press_callback(self, msg: ButtonPress):
		self.button_to_press = int(msg.button_press)
		self.door_open_close = int(msg.door_open_close)
		
		#Debugging:
		#self.get_logger().info("Press button for floor " + str(button_to_press))
		
		if self.button_to_press == 1:
			self.get_logger().info("Press button for floor 1")
			self.do_once = 1
			kit.servo[1].angle=150
			sleep(2)
			kit.servo[1].angle=90
			#<insert code to press & release button 1>
			
		elif self.button_to_press == 2:
			self.get_logger().info("Press button for floor 2")
			self.do_once = 1

			#<insert code to press & release button 2>
		
		elif self.button_to_press == 3:
			self.get_logger().info("Press button for floor 3")
			self.do_once = 1
			kit.servo[2].angle=150
			sleep(2)
			kit.servo[2].angle=90
			#<insert code to press & release button 3>
		
		elif self.button_to_press == 4:
			self.get_logger().info("Press button for floor 4")
			self.do_once = 1
			#<insert code to press & release button 4>
			
			
		if self.door_open_close == 0:
			self.get_logger().info("Release door open button")
			#<insert code to release door open actuator>
			kit.servo[0].angle=90
			
		elif self.door_open_close == 1:
			self.get_logger().info("<Nothing happens>")
			#no action here
		
		
			
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
	
	GPIO.cleanup()

if __name__ == '__main__':
	main()
