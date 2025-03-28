#!/usr/bin/env python

import rospy
import math

from geometry_msgs.msg import Twist
from my_rb1_ros.srv import Rotate, RotateResponse
from nav_msgs.msg import Odometry

class MoveRB1():

    def __init__(self):
        self.rb1_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._service = rospy.Service('/rotate_robot', Rotate , self.callback)
        rospy.loginfo(f"Service Ready: /rotate_robot")
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(20) # 20hz
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.z_angular_pos_degrees = float(0)
        self.z_angular_cache_degrees = float(0)
        rospy.on_shutdown(self.shutdownhook)
    
    def callback(self, request):
        rospy.loginfo(f"Service Requested: {request.degrees} degrees")
        # rospy.loginfo(f"Robot rotation requested for {request.degrees}")
        
        self.z_angular_cache_degrees = self.z_angular_pos_degrees
        target = self.z_angular_cache_degrees + request.degrees

        rospy.loginfo(f"Start: {self.z_angular_cache_degrees}, target: {target}")

        if request.degrees < 1:
            self.rotate_rb1(angular_speed=-1.047197) #pi/3 rad/s
            while self.z_angular_pos_degrees > target:
                self.rate.sleep()
            self.stop_rb1()
        else:
            self.rotate_rb1(angular_speed=1.047197) #pi/3 rad/s
            while self.z_angular_pos_degrees < target:
                self.rate.sleep()
            self.stop_rb1()

        # rospy.loginfo(f"End position is: {self.z_angular_pos_degrees}")
        rospy.loginfo(f"Service Completed: {self.z_angular_pos_degrees} result vs {target} target")

        response = RotateResponse()
        response.result = (f"requested {request.degrees} rotation achieved")
        return response

    def odom_callback(self, msg):
        # current_position = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation
        self.z_angular_pos_degrees = self.convert_quaternon_to_degrees(current_orientation.z, current_orientation.w)     

    def publish_once_in_cmd_vel(self):
        while not self.ctrl_c:
            connections = self.rb1_vel_publisher.get_num_connections()
            if connections > 0:
                self.rb1_vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        self.ctrl_c = True

    def rotate_rb1(self, angular_speed=0.2):
        self.cmd.angular.z = angular_speed
        rospy.loginfo("Rotating RB1!")
        self.publish_once_in_cmd_vel()
    
    def stop_rb1(self):
        self.cmd.angular.z = 0.0
        rospy.loginfo("Stopping RB1!")
        self.publish_once_in_cmd_vel()

    def convert_quaternon_to_degrees(self, z, w):
        yaw = 2 * math.atan2(z, w)  # Convert quaternion to yaw in radians
        return math.degrees(yaw)  # Convert to degrees

rospy.init_node('service_client') 
RB1 = MoveRB1()
rospy.spin() # maintain the service open.