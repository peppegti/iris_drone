#!/usr/bin/env python

##
#
# Control a MAV via mavros
#
##

import rospy
import tf
import sys
import argparse


import select
import tty
import termios
import time
from rosys_py_modules import hamilton 

from geometry_msgs.msg import Pose, PoseStamped, Twist, Quaternion
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import RCIn
from mavros_msgs.msg import MountControl

from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandTOL
from mavros_msgs.srv import ParamGet
from mavros_msgs.srv import ParamSet
from application_msgs.msg import imu_sensor 


from  sensor_msgs.msg import Imu

import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def getParam():
    global cable_lenght 
    global cable_specific_weight
    global cable_UGV_offset_x
    global cable_UGV_offset_y 
    global cable_UGV_offset_z
    global cable_UAV_offset_x
    global cable_UAV_offset_y
    global cable_UAV_offset_z
    global wp_nav_speed

    cable_lenght           = rospy.get_param('~cable_lenght', 4.0 )
    cable_specific_weight  = rospy.get_param('~cable_specific_weight', 10)
    cable_UGV_offset_x     = rospy.get_param('~cable_UGV_offset_x', 0.0)
    cable_UGV_offset_y     = rospy.get_param('~cable_UGV_offset_y', 0.0)
    cable_UGV_offset_z     = rospy.get_param('~cable_UGV_offset_z', 0.0)
    cable_UAV_offset_x     = rospy.get_param('~cable_UAV_offset_x', 0.0)
    cable_UAV_offset_y     = rospy.get_param('~cable_UAV_offset_y', 0.0)
    cable_UAV_offset_z     = rospy.get_param('~cable_UAV_offset_z', 0.0)

    wp_nav_speed     = rospy.get_param('~wp_nav_speed', 20)

    print('Cable lenght %f' %cable_lenght)
    print('cable_specific_weight %f' %cable_specific_weight)
    print('cable_UGV_offset_x %f' %cable_UGV_offset_x)
    print('cable_UGV_offset_y %f' %cable_UGV_offset_y)
    print('cable_UGV_offset_z %f' %cable_UGV_offset_z)
    print('cable_UAV_offset_x %f' %cable_UAV_offset_x)
    print('cable_UAV_offset_y %f' %cable_UAV_offset_y)
    print('cable_UAV_offset_z %f' %cable_UAV_offset_z)

'''
def q_conjugate(q):
    x, y, z , w = q
    return (-x, -y, -z, w)
    
def q_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return  x, y, z, w

def qv_mult(q1, v1):
    q2 = v1 + tuple([0.0])
    return q_mult(q_mult(q1, q2), q_conjugate(q1))[:4]
'''

threshold_radius=0.25
def goto_xyz_rpy_wait (controller, tgt_x, tgt_y, tgt_z, tgt_ro, tgt_pi, tgt_ya, threshold=threshold_radius):
        timeout = time.time() + 20   # sec*volte minutes from now
        controller.goto_xyz_rpy(tgt_x, tgt_y, tgt_z, tgt_ro, tgt_pi, tgt_ya)
        while(math.sqrt((controller.pose.position.x - tgt_x)**2 +(controller.pose.position.y - tgt_y)**2 )>threshold):
            pass
            if time.time()>timeout:
                print("TimeOUT")
                break

def compute_catenary_locus(delta_yaw):
    global cable_lenght
    global cable_specific_weight
    global cable_UGV_offset_x
    global cable_UGV_offset_y 
    global cable_UGV_offset_z
    global cable_UAV_offset_x
    global cable_UAV_offset_y
    global cable_UAV_offset_z
    
    wp_matrix=[]
    wp_matrix_rot=[]


    #in radians
    elevation_array = np.linspace(60*math.pi/180 , 80*math.pi/180, 10)
    azimut_array = np.linspace(-90*math.pi/180 , 90*math.pi/180, 10)
    
    #euler = tf.transformations.euler_from_quaternion(quat_yaw_only)
    #delta_yaw = euler[2]

    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.quiver(0, 0, 0, 1, 0, 0, length = 1, color = 'r')
    ax.quiver(0, 0, 0, 0, 1, 0, length = 1, color = 'g')
    ax.quiver(0, 0, 0, 0, 0, 1, length = 1, color = 'b')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')

    ax.axis('equal')    
	'''

    i = 0
    for elevation_value in elevation_array:
        
        #a computation
        T_uav = cable_lenght*cable_specific_weight/math.sin(elevation_value)
        T_0 =  T_uav*math.cos(elevation_value)
        a = T_0/cable_specific_weight
        #print('a = %f' %a )


        if a != 0.0 :
            L_horiz = a*math.log(math.tan(elevation_value) + math.sqrt(math.tan(elevation_value)**2+1) )
            L_vert = a*math.cosh(L_horiz/a) - a
        
        pos_estimation_x =  L_horiz * math.cos(azimut_array[i])  + cable_UGV_offset_x
        pos_estimation_y =  L_horiz * math.sin(azimut_array[i])  + cable_UGV_offset_y
        pos_estimation_z =  L_vert                      + cable_UGV_offset_z
        #print('pos_estimation_x = %f' %pos_estimation_x)
        #print('pos_estimation_y = %f' %pos_estimation_y)
        #print('pos_estimation_z = %f' %pos_estimation_z)
        #print('- - - - - - - -')


        #ax.scatter(pos_estimation_x, pos_estimation_y, pos_estimation_z, zdir='z', s=5, c='r', depthshade=True)

       
        
        #v_rot = qv_mult(quat_yaw_only, v)
        v = (pos_estimation_x , pos_estimation_y, pos_estimation_z)
        quat_yaw_only = tf.transformations.quaternion_from_euler(0.0, 0.0, delta_yaw)
        v_rot = hamilton.qv_mult(quat_yaw_only, v)

        #ax.scatter(v_rot[0] , v_rot[1], v_rot[2], zdir='z', s=5, c='b', depthshade=True)

        #wp_matrix.append([pos_estimation_x , pos_estimation_y, pos_estimation_z,  azimut_array[i] + delta_yaw + math.pi/2]) 
        wp_matrix_rot.append([v_rot[0] , v_rot[1], v_rot[2],   azimut_array[i] + delta_yaw + math.pi/2]) #TODO renedere math.pi/2 (orientamento finale drone) un parametro
        i = i+1

    #plt.show()

    '''
    print('wpmat non rot')
    print(wp_matrix[0][1])
    print('wpmat rot')
    print(wp_matrix_rot[0][1])
    '''
    
    #return wp_matrix # , wp_matrix_rot;
    return wp_matrix_rot
    
    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(pos_estimation_x, pos_estimation_y, pos_estimation_z, zdir='z', s=5, c='r', depthshade=True)
    ax.quiver(0, 0, 0, 1, 0, 0, 
    length = 1, color = 'r')
    ax.quiver(0, 0, 0, 0, 1, 0, 
    length = 1, color = 'g')
    ax.quiver(0, 0, 0, 0, 0, 1, 
    length = 1, color = 'b')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    

    ax.axis('equal')
    plt.show()
    '''


#dist = float(sys.argv[1]) if len(sys.argv) > 1 else 1 # replace 0 with whatever default you want# value for side square 
#nav_speed = int (sys.argv[2]) if len(sys.argv) > 1 else 50 #


class MavController:
    """
    A simple object to help interface with mavros
    """
    def __init__(self):

        rospy.init_node("mav_control_node")
        getParam()
        rospy.Subscriber('/base_imu_sensor', imu_sensor ,  self.imu_base_callback)
        
        self.heading = 0.0

        #self.wp_matrix = []

        self.cmd_pos_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=1)

        self.gmbl_cmd_pub = rospy.Publisher("/mavros/mount_control/command", MountControl, queue_size=1)
        #self.cmd_vel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel_unstamped", Twist, queue_size=1)
        #self.rc_override = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=1)

        # mode 0 = STABILIZE
        # mode 4 = GUIDED
        # mode 9 = LAND
        self.mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoff_service = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)


        # SERVICE CLIENTS
        #self.nav_speed = rospy.ServiceProxy("/mavros/param/get/WPNAV_SPEED",ParamGet)
        #print(rospy.ServiceProxyget_param("WPNAV_SPEED"))
        #self.get_param = rospy.get_param('WPNAV_SPEED',ParamGet)
        self.get_param = rospy.ServiceProxy('/mavros/param/get', ParamGet)
        self.set_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)


        self.rc = RCIn()
        self.pose = Pose()
        self.timestamp = rospy.Time()


    def imu_base_callback(self, msg):
        self.heading =  msg.heading 
         

    '''
    def rc_callback(self, data):
        """
        Keep track of the current manual RC values
        """
        self.rc = data
    '''

    def pose_callback(self, data):
        """
        Handle local position information
        """
        self.timestamp = data.header.stamp
        self.pose = data.pose

    def gimbal_control(self, roll, tilt):
        gmbl_cmd = MountControl()
        gmbl_cmd.mode = 2
        gmbl_cmd.roll = roll * 100 #mavlink cmd defined in centi-degrees
        gmbl_cmd.pitch = tilt * 100 #mavlink cmd defined in centi-degrees
        self.gmbl_cmd_pub.publish(gmbl_cmd)   

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
        
    '''
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
    '''
   

    def arm(self):
        '''
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            self.arm_service(bool) == False
        except rospy.ServiceException, e:
            print "Service arm call failed: %s"%e
        '''
        return(self.arm_service(True))




    def disarm(self):
        """
        Disarm the throttle
        """
        return self.arm_service(False)

    def takeoff(self, height=2.0):
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

    def gps_cb(self): 

        global lat
        global lon
        global alt
        lat = self.latitude
        lon = self.longitude
        alt = self.altitude


def isData():
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def catenary_demo():
    controller = MavController()
    rospy.sleep(1)
     
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        while 1:
            delta_heading_enu = controller.heading
            print("Current delta yaw between cingo and map (ENU frame): %f " %(delta_heading_enu*180/math.pi))
            print("Good value? press ENTER to confirm")
            if isData():
                char = sys.stdin.read(1)
                if char == '\n':         # x1b is ESC
                    break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)    

    #euler = tf.transformations.euler_from_quaternion([controller.q_delta.x, controller.q_delta.y, controller.q_delta.z, controller.q_delta.w])
    print('Confirmed delta_yaw: %f' %(delta_heading_enu*180/math.pi))
    
    #quat_yaw_only = tf.transformations.quaternion_from_euler(0, 0, euler)
    
    
    result = compute_catenary_locus(delta_heading_enu)
    '''
    print('result non rot')
    print(result[0][0][1])
    print('result rot')
    print(result[1][0][1])
	'''

    #print(wp_matrix[0])
    #print(wp_matrix[0][0])

    '''
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    x_array = [row[0] for row in wp_matrix]
    y_array = [row[1] for row in wp_matrix]
    z_array = [row[2] for row in wp_matrix]
    ax.scatter(x_array, y_array, z_array ,  zdir='z', s=5, c='r', depthshade=True)
    ax.quiver(0, 0, 0, 1, 0, 0, 
    length = 1, color = 'r')
    ax.quiver(0, 0, 0, 0, 1, 0, 
    length = 1, color = 'g')
    ax.quiver(0, 0, 0, 0, 0, 1, 
    length = 1, color = 'b')

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    

    ax.axis('equal')
    plt.show()
    '''


    
    #buff.write(_get_struct_qd().pack(_x.value.integer, x.value.real))

    '''
    response = controller.get_param(param_id="WPNAV_SPEED")
    print('Current WPNAV_SPEED: %s' %response.value)
    
    
    global wp_nav_speed
    setSpeedReq = ParamSet._request_class()
    setSpeedReq.param_id = 'WPNAV_SPEED'
    setSpeedReq.value.integer=0
    setSpeedReq.value.real=wp_nav_speed
    response=controller.set_param(setSpeedReq)
    #print(response)
    '''
    print("Setting gimbal")
    controller.gimbal_control(45,90) # roll, tilt in degrees
    

    
    print("Takeoff " + str(2))
    controller.takeoff()
    rospy.sleep(5)
    

    old_settings = termios.tcgetattr(sys.stdin)
    for row in result:
        flag=False
        try:
            tty.setcbreak(sys.stdin.fileno())

            while 1:
                #controller.goto_xyz_rpy(row[0],row[1],row[2], 0, 0, row[3])
                #goto_xyz_rpy_wait(controller, row[0],row[1],row[2], 0, 0, row[3])
                
                if flag == False:
                    print('Going to: %s' %row)
                    controller.goto_xyz_rpy(row[0],row[1],row[2], 0, 0, row[3])
                    rospy.sleep(5)
                    print("press ENTER to move forward")
                    flag = True
                if isData():
                    char = sys.stdin.read(1)
                    if char == '\n':         # x1b is ESC
                        break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)  

    #controller.goto_xyz_rpy(0,dist,alt,0,0,0)
    #rospy.sleep(10)

    
    #controller.goto_xyz_rpy(0.0,0.0,alt,0,0,0)
    #rospy.sleep(10)
    
    print("RTL")
    controller.goto_xyz_rpy(0.0,0.0,2.0,0,0,delta_heading_enu)
    rospy.sleep(15)

    print("Landing")
    controller.land()
    
    print("EEEECCCI!!")
    

if __name__=="__main__":
    catenary_demo()