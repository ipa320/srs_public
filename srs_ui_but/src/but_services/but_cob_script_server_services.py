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
# Author: Zdenek Materna
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

import roslib;  roslib.load_manifest('srs_ui_but'); roslib.load_manifest('cob_script_server')
import rospy
from srs_ui_but.srv import BUTCOBScriptServerCommand
from simple_script_server import script

class BUTCOBScriptServer(script):
    
    def __init__(self):
        script.__init__(self)
        self._component = ""

    def Initialize(self):
        """
        simple_script_server method
        """
        self.sss.init(self._component)
    
    def command(self, req):
        self._component = req.component_name
        
        if req.command_name == "move":
            self.sss.move(self._component, req.parametr_name)


if __name__ == "__main__":
    #rospy.init_node('but_sdh_controller2')
    but_cob_script_server = BUTCOBScriptServer()

    so = rospy.Service('but_cob_script_server/open_gripper', BUTCOBScriptServerCommand, but_cob_script_server.command)
    sc = rospy.Service('but_cob_script_server/close_gripper', BUTCOBScriptServerCommand, but_cob_script_server.command)
    sh = rospy.Service('but_cob_script_server/home_gripper', BUTCOBScriptServerCommand, but_cob_script_server.command)
    
    print "BUT COB Script Server ready!"
    
    rospy.spin()
