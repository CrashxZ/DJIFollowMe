#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
from ast import literal_eval
import importlib
from startn.msg import AprilTagDetectionArray, AprilTagDetection
from startn.srv import SDKControlAuthority, DroneTaskControl
from geometry_msgs.msg import PoseWithCovariance,PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time
import os
from decimal import Decimal
import socket


class reachTarget(object):

	def __init__(self):
		self.aligntarget_x = 0.0
		self.aligntarget_y = 0.0
		self.aligntarget_z = 0.0
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.sock.bind(("127.0.0.1", 4041))
		self.localDJI = ''
		# Subscriber to get Target Position
		self.rc = rospy.Subscriber('/dji_sdk/rc', Joy, self.manual_override)
		self.ctrlFlag = 0
		# Publisher to send delta values to the drone for flying
		self.setpoint = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate', Joy, queue_size=0)
		self.djiLocalPosition = rospy.Subscriber('/dji_sdk/local_position', PointStamped, self.getLocalPosition)
		#self.addAltitude()
		self.goToTarget()

	def get_target_location(self):
		data, addr = self.sock.recvfrom(1024)
		if data != '':
			data = json.loads(data)
			if data["x"] != 0 and data["y"] != 0:
				if self.ctrlFlag == 0:
					rospy.wait_for_service('/dji_sdk/sdk_control_authority')
					control = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', SDKControlAuthority)
					print(control(SDKControlAuthority._request_class.REQUEST_CONTROL))
					self.ctrlFlag = 1
				positionX = (0.5 - data["x"]) * -1
				positionY = (0.5 - data["y"])
				#print("x : {}, y : {}".format(positionX,positionY))
				return positionX,positionY
			else:
				return '',''
	def getLocalPosition(self, msg):
		#print(msg)
		if msg.point != '':
			self.localDJI = msg.point
			
	def manual_override(self, msg):
		if msg.axes[4] != 1:
			rospy.logfatal('Manual Override')
			rospy.signal_shutdown('Manual Override')
			os.system("rosnode kill /reach_target")

	def goToTarget(self):
		p = 1
		pAlt = 0.3
		rate = rospy.Rate(15)
		land_code = 0
		while not rospy.is_shutdown():
			if land_code == 0:
				# relative distance calculation
				# XYZ movement
				movement_offset = Joy()
				x,y = self.get_target_location()
				while x != '' and y != '' and self.localDJI != '':
					print(x , y )
					xmask = round(x*p,5)
					ymask = round(y*p,5)
					#zmask = round((self.aligntarget_z-self.localDJI.z)*pAlt*-1,5)
					zmask = round(self.localDJI.z*pAlt*-1,5)
					movement_offset.axes = [xmask, ymask, zmask]
					print(movement_offset.axes)
					self.setpoint.publish(movement_offset)
					if abs(x) < 0.1 and abs(y) < 0.1 and round(abs(zmask), 2) < 0.1:
						rospy.loginfo('Landed!')
						command = rospy.ServiceProxy('/dji_sdk/drone_task_control', DroneTaskControl)
						#print(command(DroneTaskControl._request_class.TASK_LAND))
						#land_code = 1
						#break # to stop following #place code here 
					x,y = self.get_target_location()
					rate.sleep()
						
if __name__ == '__main__':
	rospy.init_node('reach_target', log_level=rospy.INFO)
	reachTarget_object = reachTarget()
	rospy.spin()
