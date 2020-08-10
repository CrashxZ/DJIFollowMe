#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
from ast import literal_eval
import importlib
from startn.msg import AprilTagDetectionArray, AprilTagDetection
from startn.srv import SDKControlAuthority, DroneTaskControl
from geometry_msgs.msg import PoseWithCovariance, PointStamped
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time
import os
from decimal import Decimal
import cv2

class reachTarget(object):

    def __init__(self):
        self.target = ''
        self.position = ''
        self.aligntarget_x = 0.5
        self.aligntarget_y = 0.5
        self.aligntarget_z = 1.5

		# Subscriber to get Target Position

        self._hunter = rospy.Subscriber('/dji_sdk/local_position', PointStamped, self.getLocalPosition)
        self.rc = rospy.Subscriber('/dji_sdk/rc', Joy, self.manual_override)
        self.ctrlFlag = 0

        # Publisher to send delta values to the drone for flying

        self.setpoint = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate', Joy, queue_size=0)
        
		self.target = self.getTargetLocation()
		self.goToTarget()

    # Subscriber callback for Target Position Information
    
    def getTargetLocation(self):
		frameWidth = 640
        frameHeight = 480
        cap = cv2.VideoCapture(1)
        cap.set(3, frameWidth)
        cap.set(4, frameHeight)
        cap.set(10,150)
        myColorValues = [[255,246,255]] #[197,170,196],
        myPoints =  []  ## [x , y , colorId ]
        while True:
			success, img = cap.read()
			imgResult = img.copy()
			newPoints = self.findColor(img,myColorValues)
			if len(newPoints)!=0:
				for newP in newPoints:
					myPoints.append(newP)
			if len(myPoints)!=0:
				self.drawOnCanvas(myPoints,myColorValues)
		cv2.imshow("Result", imgResult)
		if cv2.waitKey(1) and 0xFF == ord('q'):
			break
		targetPosition = {
		x: myPoints[-1][0]/640,
		y: myPoints[-1][1]/480
		}
		return targetPosition

    def getLocalPosition(self, msg):
        # print(msg)
        if msg.point != '':
			self.position = msg.point

    # rospy.loginfo(self.target)

    # Subscriber callback to listen to rc channels

    def manual_override(self, msg):
        if msg.axes[4] != 1:
            self._coordinate = ''
            rospy.logfatal('Manual Override')
            rospy.signal_shutdown('Manual Override')



    def goToTarget(self):
		#rospy.wait_for_service('/dji_sdk/sdk_control_authority')
		if self.ctrlFlag == 0:
			control = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', SDKControlAuthority)
			print(control(SDKControlAuthority._request_class.REQUEST_CONTROL))
			self.ctrlFlag = 1
		
        p = 0.6
        pAlt = 0.3
        rate = rospy.Rate(5)
        land_code = 0
        while not rospy.is_shutdown():
            if self.target != '' and land_code == 0:
                # relative distance calculation
                # XYZ movement
                movement_offset = Joy()
                while self.target != '':
					xmask = round(self.aligntarget_x-self.target.x*p,5)
					ymask = round(self.aligntarget_y-self.target.y)*p*-1,5)
					zmask = round((self.aligntarget_z-self.position.z)*pAlt,5)
					movement_offset.axes = [xmask, ymask, zmask]
					print(movement_offset.axes)
					self.setpoint.publish(movement_offset)
					if self.target != '' and round(abs(ymask), 2) < 0.2 and round(abs(xmask), 2) < 0.2 and round(abs(zmask), 2) < 0.2:
						rospy.loginfo('Landing!')
						command = rospy.ServiceProxy('/dji_sdk/drone_task_control', DroneTaskControl)
						print(command(DroneTaskControl._request_class.TASK_LAND))
						land_code = 1
						break # to stop following #place code here 
					rate.sleep()

	def findColor(img,myColorValues):
		imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		#cv2.imshow("hsv",img)
		count = 0
		newPoints=[]
		lower = np.array([0,0,63])
		upper = np.array([179,255,255])
		mask = cv2.inRange(img, lower, upper)
		x,y=getContours(mask)
		cv2.circle(imgResult,(x,y),15,myColorValues[count],cv2.FILLED)
		if x!=0 and y!=0:
			newPoints.append([x,y,count])
		count +=1
		#cv2.imshow("mas",mask)
		return newPoints
	 
	 
	 
	def getContours(img):
		contours,hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
		x,y,w,h = 0,0,0,0
		print(len(contours))
		for cnt in contours:
			area = cv2.contourArea(cnt)
			if area>1:
				cv2.drawContours(imgResult, cnt, -1, (255, 0, 0), 3)
				peri = cv2.arcLength(cnt,True)
				approx = cv2.approxPolyDP(cnt,0.02*peri,True)
				x, y, w, h = cv2.boundingRect(approx)
		return x+w//2,y
	 
	def drawOnCanvas(myPoints,myColorValues):
		for point in myPoints:
			cv2.circle(imgResult, (point[0], point[1]), 10, myColorValues[point[2]], cv2.FILLED)
			print("point:")
			print(point[0], point[1])
						
if __name__ == '__main__':
    rospy.init_node('reach_target', log_level=rospy.INFO)
    go_home_object = reachTarget()
    rospy.spin()
