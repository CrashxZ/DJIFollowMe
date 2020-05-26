#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import json
from ast import literal_eval
import importlib
from startn.msg import AprilTagDetectionArray, AprilTagDetection
from startn.srv import SDKControlAuthority
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import String
from sensor_msgs.msg import Joy
import time
import os
from decimal import Decimal


class reachTarget(object):

    def __init__(self):
        self.target = ''
        self.position = ''
        self.aligntarget_x = 0.0
        self.aligntarget_y = 0.0
        self.aligntarget_z = 4.0
        # Subscriber to get Target Position

        self._hunter = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.get_target_location)
        self.rc = rospy.Subscriber('/dji_sdk/rc', Joy, self.manual_override)

        # Publisher to send delta values to the drone for flying

        self.setpoint = rospy.Publisher('/dji_sdk/flight_control_setpoint_ENUvelocity_yawrate', Joy, queue_size=0)

        rospy.wait_for_service('/dji_sdk/sdk_control_authority')
        control = rospy.ServiceProxy('/dji_sdk/sdk_control_authority', SDKControlAuthority)
        print(control(SDKControlAuthority._request_class.REQUEST_CONTROL))
        self.addAltitude()
        self.goToTarget()

    # Subscriber callback for Target Position Information

    def get_target_location(self, msg):
        # print(msg)
        if msg.detections != '':
            if len(msg.detections) > 0:
                self.target = msg.detections[0].pose.pose.pose.position
            else:
                self.target = ''

    # rospy.loginfo(self.target)

    # Subscriber callback to listen to rc channels

    def manual_override(self, msg):
        if msg.axes[4] != 1:
            self._coordinate = ''
            rospy.logfatal('Manual Override')
            rospy.signal_shutdown('Manual Override')

    # Function to Publish data to the publisher
    def addAltitude(self):
        rate = rospy.Rate(1)
        movement_height = Joy()
        movement_height.axes = [0, 0, 1]
        for i in range(1, 5):
            self.setpoint.publish(movement_height)
            rospy.loginfo(movement_height.axes)
            rate.sleep()

    def goToTarget(self):
        p = 0.8
        rate = rospy.Rate(10)
        kill_code = 0
        while not rospy.is_shutdown():
            if self.target != '':
                # relative distance calculation
                # Right and Left movement
                movement_offset = Joy()
                while self.target != '' and self.target.x != self.aligntarget_x:
                    movement_offset.axes = [self.target.x*p, 0, 0]
                    self.setpoint.publish(movement_offset)
                    rospy.loginfo(movement_offset.axes)
                    rate.sleep()


                # front and back movement

                movement_offset = Joy()
                while self.target != '' and self.target.y != self.aligntarget_y:
                    movement_offset.axes = [0, self.target.y*p, 0]
                    self.setpoint.publish(movement_offset)
                    rospy.loginfo(movement_offset.axes)
                    rate.sleep()

                #Up and Down
                movement_offset = Joy()
                while self.target != '' and self.target.z != self.aligntarget_z:
                    movement_offset.axes = [0, 0, (self.aligntarget_z-self.target.z)*p]
                    self.setpoint.publish(movement_offset)
                    rospy.loginfo(movement_offset.axes)
                    rate.sleep()


                if self.target != '' and round(abs(self.target.y), 2) == self.aligntarget_y and round(abs(self.target.x), 2) == self.aligntarget_x:
                    rospy.loginfo('centered!')
                    continue


if __name__ == '__main__':
    rospy.init_node('reach_target', log_level=rospy.INFO)
    go_home_object = reachTarget()
    rospy.spin()
