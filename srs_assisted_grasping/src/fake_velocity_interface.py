#!/usr/bin/env python
###############################################################################
# \file
#
# $Id:$
#
# Copyright (C) Brno University of Technology
#
# This file is part of software developed by dcgm-robotics@FIT group.
# 
# Author: Zdenek Materna (imaterna@fit.vutbr.cz)
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: dd/mm/2012
#
# This file is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This file is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
# 
# You should have received a copy of the GNU Lesser General Public License
# along with this file.  If not, see <http://www.gnu.org/licenses/>.
#

import roslib; roslib.load_manifest('srs_assisted_grasping')
import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from brics_actuator.msg import JointVelocities
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
import copy
from threading import Lock


class fake_velocity_interface():
    
    def __init__(self):
    
        rospy.loginfo("Init of fake SDH velocity interface")
        
        rospy.Subscriber('velocity_in',JointVelocities,self.command_callback)
        rospy.Subscriber('state_in',JointTrajectoryControllerState,self.state_callback)
        
        self.pub = rospy.Publisher('position_out',JointTrajectory)
        
        self.last_time = rospy.Time(0)
        self.last_vel_command = None
        
        self.joint_names = ['sdh_knuckle_joint','sdh_thumb_2_joint','sdh_thumb_3_joint','sdh_finger_12_joint','sdh_finger_13_joint','sdh_finger_22_joint','sdh_finger_23_joint']
        
        self.joint_req_vel = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.lock = Lock()
        
        self.dt = 0.02
        
        
        self.act_position = None#JointTrajectoryPoint()
        self.req_pos = None
        
        self.state_received = False
        self.vel_received = False
        
                
    def spin(self):
        
        r = rospy.Rate(1.0/self.dt)
        
        while not rospy.is_shutdown():
            
            if self.req_pos is None:
                
                continue
            
            self.lock.acquire()
            req_vel = copy.deepcopy(self.joint_req_vel)
            last_vel_command  = copy.deepcopy(self.last_vel_command)
            act_position = copy.deepcopy(self.act_position)
            #req_pos = copy.deepcopy(self.req_pos)
            
            
            now = rospy.Time.now()
            
            if last_vel_command is None:
                
                self.lock.release()
                continue
                
            
            if (now - last_vel_command) > rospy.Duration(0.5):
                
                print "final positions"
                print act_position.positions
                self.last_vel_command = None
                
                self.lock.release()
                continue
            
            
            jt = JointTrajectory()
            jt.joint_names = self.joint_names
        
            jtp = JointTrajectoryPoint()
            
       
        
            jtp.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            jtp.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            jtp.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
            #tl = list(self.req_pos.positions)
        
            for idx in range(0,len(req_vel)):
                
                self.req_pos[idx] = self.req_pos[idx] - (req_vel[idx]*self.dt*1.0)
        
                jtp.positions[idx] = self.req_pos[idx]
                jtp.velocities[idx] = req_vel[idx]
                #jtp.accelerations[idx] = 150.0
            
            
                jt.header.stamp = now
                jt.points.append(jtp)
                
            self.lock.release()
        
            self.pub.publish(jt)
        
        
            r.sleep()
        
        
        
    def state_callback(self,data):
        
        if not self.state_received:
            
            rospy.loginfo('State info received')
            self.state_received = True
    
        self.lock.acquire()
        self.act_position = data.actual
        self.lock.release()
         

        
    def command_callback(self,data):
        
        if not self.vel_received:
            
            rospy.loginfo('Velocity command received')
            self.vel_received = True
        
        self.lock.acquire()
        #self.joint_req_vel = copy.deepcopy(data.velocities)
        for idx in range(0,len(data.velocities)):
            
            self.joint_req_vel[idx] = data.velocities[idx].value
        
        if self.last_vel_command is None:
            
            self.req_pos = list(copy.deepcopy(self.act_position.positions))
            
        self.last_vel_command = rospy.Time.now()
               
        
        self.lock.release()
        
        

if __name__ == '__main__':
  
  rospy.init_node('fake_velocity_interface')
  
  c = fake_velocity_interface()
  
  rospy.loginfo('Spinning...')
  c.spin()
  
  
