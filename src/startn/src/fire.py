#!/usr/bin/env python
import rospy
import numpy as np
from ast import literal_eval
import importlib
from geometry_msgs.msg import Pose, PoseStamped, Point, PointStamped
from follow.srv import SDKControlAuthority, DroneTaskControl
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time
import os


class follow(object):
	def __init__(self):
		self.target = ""
		self.position = ""
		self.probabilityThreshlold = 0.60
		self.kill = 0
		self.ctrlFlag = 0
		self.fireFlag = 0
		self.alignX = 0.5
		self.alignY = 0.5
		self.velocityMultiplier = 6 #set max velocity to 3 m/s (0.5 x velocityMultiplier)
		self._hunter = rospy.Subscriber('/drone_hunter_topic', String, self.get_target_location)
		self.rc = rospy.Subscriber('/dji_sdk/rc', Joy, self.manual_override)
		self.setpoint = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate', Joy, queue_size = 0)
		self.goToTarget()
	
	def get_target_location(self, msg):
		if msg.data != "-999":
			self.target = literal_eval(msg.data)
			if type(self.target) is not dict: 
				temp = self.target[0]
				for element in self.target:
					if element["prob"] > temp["prob"]:
						temp = element
				self.target = temp
		if msg.data == "-999":
			self.target = literal_eval("{'objType': 'x','x': 0,'y': 0, 'w':0 , 'h':0 ,'prob':0, 'distance':0}")
			
	def manual_override(self, msg):
		if msg.axes[4] != 10000.0:
			self._coordinate = ""
			self.kill = 1
			rospy.logfatal("Manual Override")
			rospy.signal_shutdown("Manual Override")
	#Coordinate Transformation
	def cDeltaTransform(self,element):
		positionX = ((self.alignX - (element["x"]+(element["w"]/2))) * -1) * self.velocityMultiplier
		positionY = (self.alignX - (element["y"]+(element["h"]/2))) * self.velocityMultiplier
		#print("x : {}, y : {}".format(positionX, positionY))
		return positionX, positionY
	
	def goToTarget(self):
		p = 1
		pAlt = 0.3
		rate = rospy.Rate(15)
		while not rospy.is_shutdown():
			#rospy.loginfo(self.target)
			if(type(self.target) is dict):
				#print(self.target)
				if(self.target["prob"] > self.probabilityThreshlold and self.kill == 0):
					#SDK Takes control
					if(self.ctrlFlag == 0):
						rospy.wait_for_service('/dji_sdk/sdk_control_authority')
						control = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', SDKControlAuthority)
						print(control(SDKControlAuthority._request_class.REQUEST_CONTROL))
						self.ctrlFlag = 1
					#Delta Calculation and Coordinate Transformation
					x, y = self.cDeltaTransform(self.target)
					# p-controller mask
					xmask = round(x*p, 5)
					ymask = round(y*p, 5)
					zmask = 0 # localposition fusion required
					movement_offset = Joy()
					movement_offset.axes = [xmask, ymask, zmask]
					rospy.loginfo(movement_offset.axes)
					self.setpoint.publish(movement_offset)
					rate.sleep()

if __name__ == "__main__":
	rospy.init_node('reach_target', log_level = rospy.INFO)
	follow_object = follow()
	rospy.spin()
