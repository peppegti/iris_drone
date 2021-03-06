#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
import math

pi_2 = math.pi / 2.0

class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback)

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()

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

        quat = tf.transformations.quaternion_from_euler(ro, pi, ya + pi_2)

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
        # Set to stabilize mode for arming
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

#NEW PART
def read_position(c):
	x=c.pose.position.x 
	y=c.pose.position.y
	z=c.pose.position.z
	return x,y,z

def check_distance(x,y,z,x_target,y_target,z_target,radius):
	dist=math.sqrt((x-x_target)**2+(y-y_target)**2+(z-z_target)**2)
	if(dist<=radius):
		return True


def check_position(x_target,y_target,z_target,radius,c):
	x,y,z=read_position(c)
	check_distance(x,y,z,x_target,y_target,z_target,radius)
	c.goto_xyz_rpy(x_target,y_target,z_target,0,0,0)
	while(check_distance(x,y,z,x_target,y_target,z_target,radius)!=True):
		x,y,z=read_position(c)
		check_distance(x,y,z,x_target,y_target,z_target,radius)
	print("Point reached!")
	rospy.sleep(5)
	return(True)	    


def input_positions():
	n_points=input("Number of points to be reached: ")
	coord_matrix=[]

	for i in range(n_points):
		n=[]
		for j in range(3):
			x=input("Point {}: \n Coordinate {}:".format(i,j))
			n.append(x)
		coord_matrix.append(n)
	return coord_matrix,n_points

def demo():
    c=MavController()
    rospy.sleep(1)

    alt=2.0

    c.takeoff(alt)
    rospy.sleep(5)
    c.goto_xyz_rpy(0.0,0.0,alt,0,0,0)
    print("HOME")
    rospy.sleep(5)
    
    coord_matrix,n_points=input_positions()
    radius=0.2
    for i in range(n_points):
	check_position(coord_matrix[i][0],coord_matrix[i][1],coord_matrix[i][2],radius,c)
       #rospy.sleep(5)
	
#    c.goto_xyz_rpy(coord_matrix[0][0],coord_matrix[0][1],coord_matrix[0][2],0,0,0)
#    rospy.sleep(10)


    print("Landing")
    c.land()

if __name__=="__main__":
    demo()




	


