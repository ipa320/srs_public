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
# Author: Tomas Lokaj
# Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
# Date: 19/14/2012
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

import roslib; roslib.load_manifest('srs_env_model');
from srs_env_model.msg import Context, ContextChanged
from srs_env_model.srv import GetAllPrimitivesNames, ChangeColor
from std_msgs.msg import ColorRGBA
import rospy
import srs_env_model.srv


#-------------------------------------------------------------------------------
# Definitions
#-------------------------------------------------------------------------------
SetContext_SRV = 'srs_env_model/context/set_context'
GetContext_SRV = 'srs_env_model/context/but_context/get_context'

GetAllPrimitivesNames_SRV = 'but_gui/get_all_primitives_names'
ChangeColor_SRV = 'but_gui/change_color'

ContextChanged_TOPIC = 'srs_env_model/context/context_changed'
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------

#-------------------------------------------------------------------------------
# Class ContextServer
#-------------------------------------------------------------------------------
class ContextServer(object):
    '''
    This class represents Context Server which handles context changes and 
    addapts scene according to actual state of particular tags, e.g. connection,
    action or global status. 
    
    @author: Tomas Lokaj
    @see: srs_env_model.msg.Context
    '''
    
    # actual context
    _context = Context()
    
    # context changed publisher
    _contextChangedPublisher = rospy.Publisher(ContextChanged_TOPIC,
                                               ContextChanged)
    
    # original colors of primitives
    _original_colors = {}

    def __init__(self):
        '''
        Constructor
        '''
        self._context.status_tag = Context.OK
        self._context.action_tag = Context.DEFAULT
        self._context.connection_tag = Context.UNKNOWN
        
    def set_context(self, req):
        """
        Sets new context.
        @param req: service request
        @type req: srs_env_model.srv.SetContext 
        """
        self._context_changed(req.context)
        self._context = req.context
        return []
    
    def get_context(self, req):
        """
        Gets new context.
        @param req: service request
        @type req: srs_env_model.srv.GetContext
        @return: actual context
        @rtype: srs_evn_model.msg.Context 
        """
        return self._context
    
    def _context_changed(self, context):
        """
        Handles changes of the context.
        @param context: new context 
        @type context: srs_evn_model.msg.Context
        """
        rospy.loginfo("Context changed")
        self._contextChangedPublisher.publish(context)        
        
        if self._context.status_tag != context.status_tag:
            self._status_tag_changed(context.status_tag)
        if self._context.action_tag != context.action_tag:
            self._action_tag_changed(context.action_tag)
        if self._context.connection_tag != context.connection_tag:
            self._connection_tag_changed(context.connection_tag)
            
    def _status_tag_changed(self, tag):
        """
        Status tag has changed.
        @param tag: new tage value
        @type tag: int  
        """
        rospy.loginfo("Status tag changed")
        
        if self._context.status_tag == Context.OK:
            if tag == Context.EMERGENCY: self._set_emergency_status()
        
        elif self._context.status_tag == Context.EMERGENCY:
            if tag == Context.OK: self._set_ok_status()
                
    def _action_tag_changed(self, tag):
        """
        Action tag has changed.
        @param tag: new tage value
        @type tag: int  
        """
        rospy.loginfo("Action tag changed")
    
    def _connection_tag_changed(self, tag):
        """
        Connection tag has changed.
        @param tag: new tage value
        @type tag: int  
        """
        rospy.loginfo("Connection tag changed")
        
    def _set_emergency_status(self):
        """
        Status changed from OK to EMERGENCY.
        Color all primitives to red and store their orignial color.
        """
        rospy.logwarn("Status tag changed to EMERGENCY!")
        
        primitives_names = []
        color_red = ColorRGBA(1, 0, 0, 1)
        
        rospy.wait_for_service(GetAllPrimitivesNames_SRV)
        get_all_primitives_names_service = rospy.ServiceProxy(GetAllPrimitivesNames_SRV, GetAllPrimitivesNames)
        try:
            primitives_names = get_all_primitives_names_service().primitives_names
        except:
            rospy.logerr("Cannot get primitive's names!")
            return
        
        rospy.wait_for_service(ChangeColor_SRV)
        change_color_service = rospy.ServiceProxy(ChangeColor_SRV, ChangeColor)
        for name in primitives_names:
            try:
                self._original_colors[name] = change_color_service(name, color_red).old_color
            except:
                rospy.logerr("Cannot change color of primitive %s", name)
        
    def _set_ok_status(self):
        """
        Status changed from EMERGENY to OK.
        Color all primitives to their orignial color.
        """
        rospy.logwarn("Status tag changed from EMERGENCY to OK!")
        
        rospy.wait_for_service(ChangeColor_SRV)
        change_color_service = rospy.ServiceProxy(ChangeColor_SRV, ChangeColor)
        for name in self._original_colors.keys():
            try:
                change_color_service(name, self._original_colors[name])
            except:
                rospy.logerr("Cannot change color of primitive %s", name)
#-------------------------------------------------------------------------------
# End of class ContextServer
#-------------------------------------------------------------------------------
        
        
#-------------------------------------------------------------------------------
# Main function
#-------------------------------------------------------------------------------
if __name__ == '__main__':
    
    rospy.init_node('but_context_server')
    
    contextServer = ContextServer()
    
    sc = rospy.Service(SetContext_SRV, srs_env_model.srv.SetContext,
                       contextServer.set_context)
    gt = rospy.Service(GetContext_SRV, srs_env_model.srv.GetContext,
                       contextServer.get_context)
    
    rospy.loginfo("BUT CONTEXT SERVER IS RUNNING")
    
    rospy.spin()
#-------------------------------------------------------------------------------
#-------------------------------------------------------------------------------
