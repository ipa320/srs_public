#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_symbolic_grounding')
import rospy
from srs_symbolic_grounding.msg import Command

def callback(msg):
	rospy.loginfo("command received!")
	grasp = msg.grasp
	rb_x = msg.rb_x
	rb_y = msg.rb_y
	rb_theta = msg.rb_theta
	obj_x = msg.obj_x
	obj_y = msg.obj_y
	obj_theta = msg.obj_theta
	
	

	#symol grounding
	class FuzzyInference:
		def __init__(self, MF1_X, MF1_Y, MF1_Theta, MF2_X, MF2_Y, MF2_Theta, MF3_X, MF3_Y, RB_X, RB_Y, RB_Theta, Obj_X, Obj_Y, Obj_Theta, Grasp):

			#Create lists to hold MFs.
			self.MF1_X = mf1_x
			self.MF1_Y = mf1_y
			self.MF1_Theta = mf1_theta

			self.MF2_X = mf2_x
			self.MF2_Y = mf2_y
			self.MF2_Theta = mf2_theta

			self.MF3_X = mf3_x
			self.MF3_Y = mf3_y


			#get grasp type
			self.Grasp = grasp

			#Get the location and orientation of the robot base and the object.
	      	 	self.RB_X = rb_x
	      	  	self.RB_Y = rb_y
	      	 	self.RB_Theta = rb_theta
	     	  	self.Obj_X = obj_x
	     	  	self.Obj_Y = obj_y
	      	  	self.Obj_Theta = obj_theta




		#Calculate the membership of object location.
		def Reachability(self):    
			if self.Grasp == 1:
				self.BGP_X = self.RB_X - 0.8
				self.BGP_Y = self.RB_Y - 0.15
				self.Delta_X = self.Obj_X - self.BGP_X
				self.Delta_Y = self.Obj_Y - self.BGP_Y
				self.Delta_Theta = self.Obj_Theta - self.RB_Theta
				self.BBP_X = self.Obj_X + 0.8
				self.BBP_Y = self.Obj_Y + 0.15
				self.BBP_Theta = self.Obj_Theta
				if self.Delta_X > 0.1 or self.Delta_X < -0.15:
					self.Reachability = 0
				elif self.Delta_Y > 0.15 or self.Delta_Y < -0.1:
					self.Reachability = 0
				elif self.Delta_Theta > 0.175 or self.Delta_Theta < -0.175:
					self.Reachability = 0
				else:
					self.Index_X = round(self.Delta_X / 0.025) + 6
		        		self.Index_Y = round(self.Delta_Y / 0.025) + 6
		        		self.Index_Theta = round(self.Delta_Theta / 0.0167) + 10
		        		self.Index_X = int(self.Index_X)
		        		self.Index_Y = int(self.Index_Y)
		        		self.Index_Theta = int(self.Index_Theta)
		        		self.Member_X = self.MF1_X[self.Index_X]
		        		self.Member_Y = self.MF1_Y[self.Index_Y]
		        		self.Member_Theta = self.MF1_Theta[self.Index_Theta]
					#Apply the fuzzy rule.
					self.Reachability = min(self.Member_X, self.Member_Y, self.Member_Theta)
			#front grasp
			elif self.Grasp == 2:
				self.BGP_X = self.RB_X - 0.85
				self.BGP_Y = self.RB_Y - 0.1
				self.Delta_X = self.Obj_X - self.BGP_X
				self.Delta_Y = self.Obj_Y - self.BGP_Y
				self.Delta_Theta = self.Obj_Theta - self.RB_Theta
				self.BBP_X = self.Obj_X + 0.85
				self.BBP_Y = self.Obj_Y + 0.1
				self.BBP_Theta = self.Obj_Theta
				if self.Delta_X > 0.1 or self.Delta_X < -0.15:
					self.Reachability = 0
				elif self.Delta_Y > 0.1 or self.Delta_Y < -0.1:
					self.Reachability = 0
				elif self.Delta_Theta > 0.175 or self.Delta_Theta < -0.175:
					self.Reachability = 0
				else:
		        		self.Index_X = round(self.Delta_X / 0.025) + 6
		        		self.Index_Y = round(self.Delta_Y / 0.025) + 6
		        		self.Index_Theta = round(self.Delta_Theta / 0.0167) + 10
		        		self.Index_X = int(self.Index_X)
		        		self.Index_Y = int(self.Index_Y)
		        		self.Index_Theta = int(self.Index_Theta)
		        		self.Member_X = self.MF2_X[self.Index_X]
		        		self.Member_Y = self.MF2_Y[self.Index_Y]
		        		self.Member_Theta = self.MF2_Theta[self.Index_Theta]
					#Apply the fuzzy rule.
					self.Reachability = min(self.Member_X, self.Member_Y, self.Member_Theta)

			#top grasp
			elif self.Grasp == 3:
				self.BGP_X = self.RB_X - 0.8
				self.BGP_Y = self.RB_Y - 0.1
				self.Delta_X = self.Obj_X - self.BGP_X
				self.Delta_Y = self.Obj_Y - self.BGP_Y
				self.Delta_Theta = self.Obj_Theta - self.RB_Theta
				self.BBP_X = self.Obj_X + 0.8
				self.BBP_Y = self.Obj_Y + 0.1
				self.BBP_Theta = "-"
				if self.Delta_X > 0.1 or self.Delta_X < -0.15:
					self.Reachability = 0
				elif self.Delta_Y > 0.1 or self.Delta_Y < -0.1:
					self.Reachability = 0
				else:
		        		self.Index_X = round(self.Delta_X / 0.025) + 6
					self.Index_Y = round(self.Delta_Y / 0.025) + 6
					self.Index_X = int(self.Index_X)
					self.Index_Y = int(self.Index_Y)
					self.Member_X = self.MF3_X[self.Index_X]
					self.Member_Y = self.MF3_Y[self.Index_Y]
					#Apply the fuzzy rule.
					self.Reachability = min(self.Member_X, self.Member_Y)
		    
			return self.Reachability, self.BBP_X, self.BBP_Y, self.BBP_Theta

	model = FuzzyInference(mf1_x, mf1_y, mf1_theta, mf2_x, mf2_y, mf2_theta, mf3_x, mf3_y, rb_x, rb_y, rb_theta, obj_x, obj_y, obj_theta, grasp)
	reach, bbp_x, bbp_y, bbp_theta = model.Reachability()
	rospy.loginfo("reachability is %s", reach)
	rospy.loginfo("best grasp pose is %s %s %s", bbp_x, bbp_y, bbp_theta)
	

def listener():
	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("command", Command, callback)
	rospy.spin()




mf1_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
mf1_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.875, 0.75, 0.625, 0.5, 0.375, 0.25, 0.125, 0]
mf1_theta = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

mf2_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
mf2_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]
mf2_theta = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, 0.1, 0]

mf3_x = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.75, 0.5, 0.25, 0]
mf3_y = [0, 0.16, 0.33, 0.49, 0.67, 0.84, 1, 0.84, 0.67, 0.49, 0.33, 0]




if __name__ == '__main__':
	listener()
