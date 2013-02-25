#!/usr/bin/env python

import roslib; roslib.load_manifest('srs_training')
import rospy
from geometry_msgs.msg import Vector3, PoseStamped, Pose
from srs_interaction_primitives.srv import GetUnknownObject, SetAllowObjectInteraction, AddUnknownObject
from srs_interaction_primitives.msg import PoseType

class bb_placement():
    
    def __init__(self):
        
        but_gui_ns = '/interaction_primitives'
        self.s_add_unknown_object = but_gui_ns + '/add_unknown_object'
        self.s_get_object = but_gui_ns + '/get_unknown_object'
        self.s_allow_interaction = but_gui_ns + '/set_allow_object_interaction'
        
        rospy.wait_for_service(self.s_add_unknown_object)
        rospy.wait_for_service(self.s_get_object)
        rospy.wait_for_service(self.s_allow_interaction)     
        
        self.a = 0.4 # height (z)
        self.b = 0.4 # length (x)
        self.c = 0.2 # width (y)
        
        self.pz = 0.0
    
    def spawn(self):
        
       bb_pose = Pose()
       
       bb_pose.position.x = 0.0
       bb_pose.position.y = 0.0
       bb_pose.position.z = self.pz
       
       bb_pose.orientation.x = 0.0
       bb_pose.orientation.y = 0.0
       bb_pose.orientation.z = 0.
       bb_pose.orientation.w = 1.0
       
       bb_lwh = Vector3()
       
       bb_lwh.x = self.b
       bb_lwh.y = self.c
       bb_lwh.z = self.a
    
       add_object = rospy.ServiceProxy(self.s_add_unknown_object, AddUnknownObject)
    
       try:
            
            add_object(frame_id='/map',
                       name='unknown_object',
                       description='Object to be grasped',
                       pose_type= PoseType.POSE_BASE,
                       pose = bb_pose,
                       scale = bb_lwh,
                       disable_material=True)
          
       except Exception, e:
          
          rospy.logerr('Cannot add IM object to the scene, error: %s',str(e))
          
          return False
          
          
       allow_interaction = rospy.ServiceProxy(self.s_allow_interaction, SetAllowObjectInteraction)
       
       try:
            
            allow_interaction(name = 'unknown_object',
                              allow = True)
          
       except Exception, e:
          
          rospy.logerr('Cannot allow interaction, error: %s',str(e))
          
          return False
       
      
       return True
        
            
        
if __name__ == '__main__':
    
    rospy.init_node('bb_placement')
    
    rospy.loginfo('Starting bb_placement node')
    
    bb = bb_placement()
    
    rospy.loginfo('Initialized')
    
    bb.spawn()
    
    rospy.loginfo('Finished')
        
        
