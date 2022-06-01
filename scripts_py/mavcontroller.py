#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

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

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
	rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)
	rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.glob_callback)

	rospy.Subscriber("/pantilt",NavSatFix,self.pantilt_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)
        self.gmbl_cmd_pub = rospy.Publisher("/mavros/mount_control/command", MountControl, queue_size=1)


        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)


        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()
	self.gps = NavSatFix()

	self.pantilt=NavSatFix()

	self.flag=False

    def pantilt_callback(self,msg):
	self.pantilt=msg

    def glob_callback(self, msg):
	self.gps=msg
	if(self.flag==False):
		self.flag=True
	#self.longitude=data.longitude
	#self.altitude=data.altitude

    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.pose)


    def goto(self, pose):
        """
        Set the given pose as a the next setpoint by sending
        a SET_POSITION_TARGET_LOCAL_NED message. The copter must
        be in GUIDED mode for this to work.
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.timestamp
        pose_stamped.pose = pose

        self.cmd_pos_pub.publish(pose_stamped)

    def goto_xyz_rpy(self, x, y, z, ro, pi, ya):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya)

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.goto(pose)
        #print(quat)


    def set_vel(self, vx, vy, vz, avx=0, avy=0, avz=0):
        """
        Send comand velocities. Must be in GUIDED mode. Assumes angular
        velocities are zero by default.
        """
        cmd_vel = Twist()

        cmd_vel.linear.x = vx
        cmd_vel.linear.y = vy
        cmd_vel.linear.z = vz

        cmd_vel.angular.x = avx
        cmd_vel.angular.y = avy
        cmd_vel.angular.z = avz

        self.cmd_vel_pub.publish(cmd_vel)

    def gimbal_control(self, pan,tilt):
        gmbl_cmd = MountControl()
        gmbl_cmd.mode = 2
        gmbl_cmd.yaw =-(pan * 100 - 9000)#mavlink cmd defined in centi-degrees
	gmbl_cmd.pitch = tilt * 100 #mavlink cmd defined in centi-degrees
        self.gmbl_cmd_pub.publish(gmbl_cmd)   
	

    def arm(self):
        """
        Arm the throttle
        """
        return self.arm_service(True)

    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=1.0):
        """
        Arm the throttle, takeoff to a few feet, and set to guided mode
        """
        # Set to stabilize mode for arming(
        #mode_resp = self.mode_service(custom_mode="0")
        mode_resp = self.mode_service(custom_mode="4")
        self.arm()

        # Set to guided mode
        #mode_resp = self.mode_service(custom_mode="4")

        # Takeoff
        takeoff_resp = self.takeoff_service(altitude=height)

        #return takeoff_resp
        return mode_resp

    def land(self):
        """
        Set in LAND mode, which should cause the UAV to descend directly,
        land, and disarm.
        """
        resp = self.mode_service(custom_mode="9")
        self.disarm()

    def home(self):
	self.takeoff(2.0)
	self.goto_xyz_rpy(0.0,0.0,2.0,0.0,0.0,0.0)
	rospy.sleep(5)


