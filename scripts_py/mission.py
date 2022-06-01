#!/usr/bin/env python


from mavcontroller import MavController

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion, Point
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import MountControl
from sensor_msgs.msg import NavSatFix

import math
from math import atan2
import numpy as np
import time
import utm

pi_2 = math.pi / 2.0


class Missions:

#MISSIONS:
	def __init__(self):
		self.c = MavController()

	def circle(self,center,radius,target):
		print("CIRCLE MOTION: ")
		self.c.goto_xyz_rpy(center.x+radius,center.y,2,0,0,0)
		rospy.sleep(5)
		rate = rospy.Rate(5)
		for i in range(180):
			theta = i * 2.0 * math.pi / 180.0
			angle_to_goal=self.orientation_control(target)
			x = center.x+radius * math.cos(theta)
			y = center.y+radius * math.sin(theta)
			z = 2;
			self.c.goto_xyz_rpy(x, y, z, 0.0, 0.0, angle_to_goal)
			self.set_gimbal(target)
			rate.sleep()
		print("CIRCLE DONE!")

	def snake_path(self,center,width,height,n,pantilt):
		start=Point()
		start.x=center.x+width/2
		start.z=center.z-height/2
		start.y=center.y
		self.c.goto_xyz_rpy(start.x,start.y,start.z,0,0,0)
		h=math.floor(height/n)
		extreme=Point()
		for i in range (n+1):
			if(i%2==0):
				extreme.x=self.c.pose.position.x-width
			else:
				extreme.x=self.c.pose.position.x+width
			extreme.z=self.c.pose.position.z
			extreme.y=self.c.pose.position.y
			self.check_position_orientation_new(extreme,pantilt,0.2)
			self.set_gimbal(pantilt)
			print(self.c.pose.position)
			extreme.x=self.c.pose.position.x
			extreme.z=self.c.pose.position.z+h
			extreme.y=self.c.pose.position.y
			self.check_position_orientation_new(extreme,pantilt,0.2)
			self.set_gimbal(pantilt)
			print(self.c.pose.position)
		print("Snake path complete!")



#UTILITIES:
	def check_distance(self,point_1,point_2,radius):
		dist=math.sqrt((point_1.x-point_2.x)**2+(point_1.y-point_2.y)**2+(point_1.z-point_2.z)**2)
		if(dist<=radius):
			return True
	
	def orientation_control(self,target):
		if(self.c.pose.position.y-target.y==0 and self.c.pose.position.x-target.x==0):
			angle_to_goal=0
		else:
			angle_to_goal = atan2(self.c.pose.position.y-target.y,self.c.pose.position.x-target.x)+2*1.57
		return angle_to_goal

	def check_position_orientation_new(self,point,target,radius):
		self.check_distance(self.c.pose.position,point,radius)
		self.c.goto_xyz_rpy(point.x,point.y,point.z,0,0,0)
		rate = rospy.Rate(5)
		while(self.check_distance(self.c.pose.position,point,radius)!=True):
			angle_to_goal=self.orientation_control(target)
			self.c.goto_xyz_rpy(point.x,point.y,point.z,0,0,angle_to_goal)
			self.set_gimbal(target)
			rate.sleep()
		print("Set gimbal")
		print("Point reached!")
		rospy.sleep(5) 
		return(True)


	def set_gimbal(self,target):
		dist=Point()
		dist.x=self.c.pose.position.x-target.x
		dist.y=self.c.pose.position.y-target.y
		dist.z=self.c.pose.position.z-target.z
		mod_dist=math.sqrt(dist.x**2+dist.y**2+dist.z**2)
		pitch=math.asin(dist.z/mod_dist)*(180/math.pi)
		yaw=atan2(dist.y,dist.x)*(180/math.pi)+180
		self.c.gimbal_control(yaw,pitch)


	def input_gps(self):
		gps=NavSatFix()
		print("Insert your target GPS coordinates: ")
		gps.latitude=input("Latitude: ")
		gps.longitude=input("Longitude: ")
		gps.altitude=input("Altitude: ")
		return gps

	def distance_gps(self,gps_target):
		while(self.c.flag!=True):
			print(self.c.flag)
			time.sleep(2)
		easting,northing,zone_number,zone_letter=utm.from_latlon(self.c.gps.latitude,self.c.gps.longitude)
		easting_t,northing_t,zone_number_t,zone_letter_t=utm.from_latlon(gps_target.latitude,gps_target.longitude)
		print(easting_t,northing_t)
		print(easting,northing)
		distance=Point()
		distance.x=easting_t-easting
		distance.y=northing_t-northing
		distance.z=self.c.gps.altitude-gps_target.altitude
		print(distance)
		return distance
