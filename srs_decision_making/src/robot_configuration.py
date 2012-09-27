#!/usr/bin/python
#################################################################
##\file
#
# \note
# Copyright (c) 2011 \n
# Cardiff University \n\n
#
#################################################################
#
# \note
# Project name: Multi-Role Shadow Robotic System for Independent Living
# \note
# ROS stack name: srs
# \note
# ROS package name: srs_decision_making
#
# \author
# Author: Renxi Qiu, email: renxi.qiu@gmail.com
#
# \date Date of creation: Oct 2011
#
# \brief
# Task coordination and interfacing for SRS decision making
#
#################################################################
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer. \n
#
# - Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution. \n
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License LGPL as
# published by the Free Software Foundation, either version 3 of the
# License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU Lesser General Public License LGPL for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License LGPL along with this program.
# If not, see <http://www.gnu.org/licenses/>.
#
#################################################################

"""
This file contains settings for robot configuration 

"""

class Ddict(dict):
    def __init__(self, default=None):
        self.default = default

    def __getitem__(self, key):
        if not self.has_key(key):
            self[key] = self.default()
        return dict.__getitem__(self, key)
    
    
component_list = ['torso','tray','arm','sdh','head']
    
    
robot_config_pre = Ddict(dict)

robot_config_post = Ddict(dict)

#DM = doesn't matter
#NC = no change required
#CALCULATION = need to be calculated on-line, 
#in this program CALCULATION is same as NC, as the calculation is carried out already during action, the is no change required for pre post check. 
# 
robot_config_need_no_action = ['DM','NC','CALCULATION']

#Navigation with no object 
#pre-config
robot_config_pre['navigation_no_object']['torso']='home'
robot_config_pre['navigation_no_object']['tray']='down'
robot_config_pre['navigation_no_object']['arm']='look_at_table-to-folded'
robot_config_pre['navigation_no_object']['sdh']='home'
robot_config_pre['navigation_no_object']['head']='front'
#post-config
robot_config_post['navigation_no_object']['torso']='DM' 
robot_config_post['navigation_no_object']['tray']='NC'
robot_config_post['navigation_no_object']['arm']='NC'
robot_config_post['navigation_no_object']['sdh']='DM'
robot_config_post['navigation_no_object']['head']='front'


#Navigation with object on tray and no object in sdh
#pre-config
robot_config_pre['navigation_object_on_tray']['torso']='home'
robot_config_pre['navigation_object_on_tray']['tray']='NC'
robot_config_pre['navigation_object_on_tray']['arm']='folded'
robot_config_pre['navigation_object_on_tray']['sdh']='DM'
robot_config_pre['navigation_object_on_tray']['head']='front'
#post-config
robot_config_post['navigation_object_on_tray']['torso']="DM"
robot_config_post['navigation_object_on_tray']['tray']='NC'
robot_config_post['navigation_object_on_tray']['arm']='NC'
robot_config_post['navigation_object_on_tray']['sdh']='DM'
robot_config_post['navigation_object_on_tray']['head']='front'


#Navigation with no object on tray but with object in sdh
#pre-config
robot_config_pre['navigation_object_in_sdh']['torso']='home'
robot_config_pre['navigation_object_in_sdh']['tray']='down'
robot_config_pre['navigation_object_in_sdh']['arm']='hold'
robot_config_pre['navigation_object_in_sdh']['sdh']='NC'  #should be closed, but the exact location is calculated early during grasp
robot_config_pre['navigation_object_in_sdh']['head']='front'
#post config
robot_config_post['navigation_object_in_sdh']['torso']="DM"
robot_config_post['navigation_object_in_sdh']['tray']='NC'
robot_config_post['navigation_object_in_sdh']['arm']='NC'
robot_config_post['navigation_object_in_sdh']['sdh']='NC'  #should be closed, but the exact location is calculated early
robot_config_post['navigation_object_in_sdh']['head']='front'
#Instead of hold we could also use folded, this depends on the object we are handling. In any cases the arm has to be closed 
#to the torso so that nothing is out of the rectangular footprint of the robot
#we should be able to check what object need to be hold or hold

#Navigation with  object on tray and object in sdh
#pre-config
robot_config_pre['navigation_object_on_tray_and_sdh']['torso']='home'
robot_config_pre['navigation_object_on_tray_and_sdh']['tray']='NC'
robot_config_pre['navigation_object_on_tray_and_sdh']['arm']='NC'
robot_config_pre['navigation_object_on_tray_and_sdh']['sdh']='NC'  #should be closed, but the exact location is calculated early during grasp
robot_config_pre['navigation_object_on_tray_and_sdh']['head']='front'
#post config
robot_config_post['navigation_object_on_tray_and_sdh']['torso']="DM"
robot_config_post['navigation_object_on_tray_and_sdh']['tray']='NC'
robot_config_post['navigation_object_on_tray_and_sdh']['arm']='NC'
robot_config_post['navigation_object_on_tray_and_sdh']['sdh']='NC'  #should be closed, but the exact location is calculated early
robot_config_post['navigation_object_on_tray_and_sdh']['head']='front'


#detection
#pre-config
robot_config_pre['detection']['torso']='DM'
robot_config_pre['detection']['tray']='down'
robot_config_pre['detection']['arm']='folded'
robot_config_pre['detection']['sdh']='cylclosed'
robot_config_pre['detection']['head']='DM'
#post-config
robot_config_post['detection']['torso']='home'
robot_config_post['detection']['tray']='NC'
robot_config_post['detection']['arm']='NC'
robot_config_post['detection']['sdh']='cylclosed'
robot_config_post['detection']['head']='front'   

#environment update
#pre-config
robot_config_pre['enviroment_update']['torso']='DM'
robot_config_pre['enviroment_update']['tray']='down'
robot_config_pre['enviroment_update']['arm']='folded'
robot_config_pre['enviroment_update']['sdh']='DM'
robot_config_pre['enviroment_update']['head']='DM'
#post-config
robot_config_post['enviroment_update']['torso']='home'
robot_config_post['enviroment_update']['tray']='NC'
robot_config_post['enviroment_update']['arm']='NC'
robot_config_post['enviroment_update']['sdh']='DM'
robot_config_post['enviroment_update']['head']='front'


#grasp (including detection for grasp + grasp after detection)
#pre-config
robot_config_pre['grasp']['torso']='DM'
robot_config_pre['grasp']['tray']='down'
robot_config_pre['grasp']['arm']='folded-to-look_at_table'
robot_config_pre['grasp']['sdh']='DM'
robot_config_pre['grasp']['head']='back'
#post-config
robot_config_post['grasp']['torso']='DM'
robot_config_post['grasp']['tray']='NC'
robot_config_post['grasp']['arm']='CALCULATION' # can be used for both put object on tray and  hold object
robot_config_post['grasp']['sdh']='CALCULATION'  #should be closed, but the exact location should have been calculated
robot_config_post['grasp']['head']='DM'

#Put object on tray
#pre-config
robot_config_pre['put_on_tray']['torso']='home'
robot_config_pre['put_on_tray']['tray']='up'
robot_config_pre['put_on_tray']['arm']='NC'
robot_config_pre['put_on_tray']['sdh']='NC'
robot_config_pre['put_on_tray']['head']='NC'
#post-config
robot_config_post['put_on_tray']['torso']="home"
robot_config_post['put_on_tray']['tray']='NC'
robot_config_post['put_on_tray']['arm']='folded' 
robot_config_post['put_on_tray']['sdh']='home'  
robot_config_post['put_on_tray']['head']='front'

#place object on table
#pre-config
#To be added
#post-config
#To be added
