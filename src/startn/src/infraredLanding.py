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
		#center points for the image after coordinate transformation (used for alignment)
		self.aligntarget_x = 0.0
		self.aligntarget_y = 0.0
		self.aligntarget_z = 0.0
		#socket to recieve data from the irRecog.py program
		self.sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		self.sock.bind(("127.0.0.1", 4041))
		self.localDJI = ''
		# Subscriber to get RC Status
		self.rc = rospy.Subscriber('/dji_sdk/rc', Joy, self.manual_override)
		#if controlflag = 0 , the program requests autonomous control from the Flight controller. (set 1 for testing purposes)
		self.ctrlFlag = 0
		# Publisher to send delta values to the drone for flying
		self.setpoint = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate', Joy, queue_size=0)
		self.djiLocalPosition = rospy.Subscriber('/dji_sdk/local_position', PointStamped, self.getLocalPosition)
		#self.addAltitude()
		self.goToTarget()
		
	#function to get the location data from the recognition program and transform cordinates.
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
				#Transform coordinates from camera frame to ENU Frame.
				positionX = (0.5 - data["x"]) * -1
				positionY = (0.5 - data["y"])
				#print("x : {}, y : {}".format(positionX,positionY))
				return positionX,positionY
			else:
				return '',''
	#get the local position from DJI's Controller
	def getLocalPosition(self, msg):
		#print(msg)
		if msg.point != '':
			self.localDJI = msg.point
	# monitor RC Status and override autonomous control for emergencies		
	def manual_override(self, msg):
		if msg.axes[4] != 1:
			rospy.logfatal('Manual Override')
			rospy.signal_shutdown('Manual Override')
			os.system("rosnode kill /reach_target")
	#function to check alignment status and correct the position by sending delta values from the current position to the desired position above the IR light.
	def goToTarget(self):
		#P control rate for x,y
		p = 1
		#P control rate for z (Altitude)
		pAlt = 0.3
		rate = rospy.Rate(15)
		# if land code is 1 then Altitude control is disabled
		land_code = 0
		while not rospy.is_shutdown():
			if land_code == 0:
				# relative distance calculation
				# XYZ movement
				movement_offset = Joy()
				x,y = self.get_target_location()
				while x != '' and y != '' and self.localDJI != '':
					print(x , y )
					#round of the values of z and y.
					xmask = round(x*p,5)
					ymask = round(y*p,5)
					#zmask = round((self.aligntarget_z-self.localDJI.z)*pAlt*-1,5)
					zmask = round(self.localDJI.z*pAlt*-1,5)
					#send the values to the FC
					movement_offset.axes = [xmask, ymask, zmask]
					print(movement_offset.axes)
					self.setpoint.publish(movement_offset)
					#check alignment and position the the IR light with respect to the current position and then land if the error is tolerable.
					if abs(x) < 0.1 and abs(y) < 0.1 and round(abs(zmask), 2) < 0.1:
						rospy.loginfo('Landed!')
						command = rospy.ServiceProxy('/dji_sdk/drone_task_control', DroneTaskControl)
						#uncomment if value of z tolerance is more than 0.1 (to initiate DJI's Landing sequence)
						#print(command(DroneTaskControl._request_class.TASK_LAND))
						#land_code = 1
						#break
					x,y = self.get_target_location()
					rate.sleep()
						
if __name__ == '__main__':
	rospy.init_node('reach_target', log_level=rospy.INFO)
	reachTarget_object = reachTarget()
	rospy.spin()
