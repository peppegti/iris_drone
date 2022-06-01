#!/usr/bin/env python

from tkinter import *
import tkSimpleDialog as simpledialog
import tkMessageBox as messagebox
#import threading

import rospy
import tf
import time
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


from mavcontroller import MavController
from mission import Missions


class GUI:
	def __init__(self, m,parent):
		self.missioni = m

		self.button=[]
		self.parent=parent

		self.TakeOffButton=Button(parent, text="TAKEOFF")
		self.TakeOffButton['background']="black"
		self.TakeOffButton['foreground']="yellow"
		self.TakeOffButton['command']=self.TakeOffButton_Click
		self.TakeOffButton.pack({"side":"top","padx":0,"pady":10})

		self.LandButton=Button(parent, text="LAND")
		self.LandButton['background']="black"
		self.LandButton['foreground']="yellow"
		self.LandButton['command']=self.LandButton_Click
		self.LandButton.pack({"side":"top","padx":0,"pady":40})


		self.HomeButton=Button(parent, text="GO HOME")
		self.HomeButton['background']="white"
		self.HomeButton['foreground']="red"
		self.HomeButton['command']=self.HomeButton_Click
		self.HomeButton.pack({"side":"bottom","padx":10,"pady":20})

		self.CircleButton=Button(parent, text="CIRCLE")
		self.CircleButton['background']="white"
		self.CircleButton['foreground']="red"
		self.CircleButton['command']=self.CircleButton_Click
		self.CircleButton.pack({"side":"bottom","padx":30,"pady":20})

		self.GoToButton=Button(parent, text="EXPLORE")
		self.GoToButton['background']="white"
		self.GoToButton['foreground']="red"
		self.GoToButton['command']=self.GoToButton_Click
		self.GoToButton.pack({"side":"bottom","padx":50,"pady":20})

		self.SnakeButton=Button(parent, text="SNAKE")
		self.SnakeButton['background']="white"
		self.SnakeButton['foreground']="red"
		self.SnakeButton['command']=self.SnakeButton_Click
#		self.image_snake = PhotoImage(file="snake.png")
#		self.SnakeButton.configure(image=self.image_snake)
		self.SnakeButton.pack({"side":"bottom","padx":70,"pady":20})

		self.Go_and_LandButton=Button(parent, text="EXPLORE AND LAND")
		self.Go_and_LandButton['background']="white"
		self.Go_and_LandButton['foreground']="red"
		self.Go_and_LandButton['command']=self.Go_and_land
		self.Go_and_LandButton.pack({"side":"bottom","padx":110,"pady":20})

		self.flag=0 #MODE: 1 CARTESIAN, 2 GPS, 0 NO SELECTION YET
		self.var1 = IntVar()
		self.var2 = IntVar()
		self.c1 = Checkbutton(parent, text='Cartesian MODE',variable=self.var1, onvalue=1, offvalue=0, command=self.print_selection)
		self.c1.pack({"side":"right","padx":100,"pady":20})
		self.c2 = Checkbutton(parent, text='GPS MODE',variable=self.var2, onvalue=1, offvalue=0, command=self.print_selection)
		self.c2.pack({"side":"left","padx":100,"pady":0})

		self.s_var = StringVar()
		self.label = Label(parent, textvariable=self.s_var, relief=RAISED )
		ptu=[self.missioni.c.pantilt.latitude,self.missioni.c.pantilt.longitude,self.missioni.c.pantilt.altitude]
		self.s_var.set(ptu)
		#self.s_var.set(self.missioni.c.pantilt)
		self.label.pack()

		self.s_var2 = StringVar()
		self.label2 = Label(parent, textvariable=self.s_var2, relief=RAISED )
		self.s_var2.set(self.missioni.distance_gps(self.missioni.c.pantilt))
		self.label2.pack()


	def print_selection(self):
    		if (self.var1.get() == 1) and (self.var2.get() == 0): #CARTESIAN MODE
      			self.flag=1
    		elif (self.var1.get() == 0) & (self.var2.get() == 1): #GPS MODE
			self.flag=2
		elif (self.var1.get() == 1) & (self.var2.get() == 1):
			messagebox.showerror("ERROR","TWO CONTEMPORARY SELECTIONS")

	def HomeButton_Click(self):
		self.missioni.c.home()
		messagebox.showinfo("MISSION CHECK", "I'm home!")

	def TakeOffButton_Click(self):
		alt=1.0
		self.missioni.c.takeoff(alt)

	def LandButton_Click(self):
		self.missioni.c.land()

	def CircleButton_Click(self):
		print(self.missioni.c.pantilt)
		radius=simpledialog.askinteger("input radius","insert your radius")
		target=self.missioni.distance_gps(self.missioni.c.pantilt)
		answer=messagebox.askquestion("CENTER SELECTION","Do you want your PTU as center point?")
		if answer=="yes":
			center=target
		else:
			center=self.Input_Target()	
		self.missioni.circle(center,radius,target)
		messagebox.showinfo("MISSION CHECK", "Circle done!")

	def Input_Target(self):
		if self.flag==1:
			target=Point()
			target.x=simpledialog.askinteger("Insert your coordinates: ","x: ")
			target.y=simpledialog.askinteger("Insert your coordinates: ","y: ")
			target.z=simpledialog.askinteger("Insert your coordinates: ","z: ")
			messagebox.showinfo("POINTS", "Submitted!")
			return target
		elif self.flag==2:
			target=self.Input_Gps()
			return target
		else:
			messagebox.showinfo("ERROR", "Check your MODE selection!")		


	def Input_Points(self):
		n_points=simpledialog.askinteger("Plan your path","Number of points to be reached:")
		coord_matrix=[]

		for i in range(n_points):
			print("Point: {}".format(i+1))
			new_point=self.Input_Target()
			coord_matrix.append(new_point)
		return coord_matrix,n_points

	def GoToButton_Click(self):
		messagebox.showinfo("MISSION CHECK", self.missioni.c.pantilt)
		coord_matrix,n_points=self.Input_Points()
		target=self.missioni.distance_gps(self.missioni.c.pantilt)
		print(target)
		for i in range (n_points):
				self.missioni.check_position_orientation_new(coord_matrix[i],target,0.2)
				messagebox.showinfo("MISSION CHECK", "Point reached!")
		self.missioni.c.home()

	def Go_and_land(self):
		messagebox.showinfo("MISSION CHECK", self.missioni.c.pantilt)
		coord_matrix,n_points=self.Input_Points()
		target=self.missioni.distance_gps(self.missioni.c.pantilt)
		time_limit=simpledialog.askinteger("TIME SETTING","Set your measurement time: ")
		print(target)
		for i in range (n_points):
				timeout=time_limit+time.time()
				self.missioni.check_position_orientation_new(coord_matrix[i],target,0.2)
				messagebox.showinfo("MISSION CHECK", "Point reached!")
				#self.missioni.c.land()
				coord_matrix[i].z=0;
				self.missioni.check_position_orientation_new(coord_matrix[i],target,0.2)
				self.missioni.c.land()
				#while(self.missioni.c.pose.position.z!=0):
				#	print(self.missioni.c.pose.position.z)
				#	pass
				rospy.sleep(10)
				#if(self.missioni.c.pose.position.z==0):
				#	print(self.missioni.c.pose.position.z)
				#	messagebox.showinfo("MISSION CHECK", "Landed!")
				while (time.time()<time_limit):
					pass
				if (time.time()>time_limit):
					self.missioni.c.takeoff(1)
					rospy.sleep(5)
		self.missioni.c.home()


	def SnakeButton_Click(self):
		messagebox.showinfo("Plan your snake path", "Insert your center coordinates: ")
		center=self.Input_Target()
		width=simpledialog.askinteger("Plan your snake path","Width:")
		height=simpledialog.askinteger("Plan your snake path","Height:")
		n=simpledialog.askinteger("Plan your snake path","Number of windings:")
		#messagebox.showinfo("Plan your snake path", "Insert your pan-tilt coordinates: ")
		pantilt=self.missioni.distance_gps(self.missioni.c.pantilt)
		self.missioni.snake_path(center,width,height,n,pantilt)
		messagebox.showinfo("MISSION CHECK", "Snake path completed!")


	def Input_Gps(self):
		gps=NavSatFix()
		gps.latitude=simpledialog.askfloat("GPS Coordinates","Latitude: ")
		gps.longitude=simpledialog.askfloat("GPS Coordinates","Longitude: ")
		gps.altitude=simpledialog.askfloat("GPS Coordinates","Altitude: ")
		return gps

def main():
	m=Missions()
	root=Tk()
	myapp=GUI(m,root)
	root=mainloop()

if __name__=='__main__':
	main()
