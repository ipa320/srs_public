#!/usr/bin/env python
import roslib; roslib.load_manifest('srs_training')
import rospy
from geometry_msgs.msg import Twist
import tf


class move_box():

    def __init__(self):
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.reference_frame = "box1"
        self.target_frame = "base_link"
        rospy.Subscriber("/base_controller/command", Twist, self.callback)
    
    def callback(self,msg):
        print msg
        
        self.tf_broadcaster = tf.TransformBroadcaster()
        
        self.tf_listener.waitForTransform(self.reference_frame, self.target_frame, rospy.Time(0), rospy.Duration(3.0))
        trans, rot = self.tf_listener.lookupTransform(self.reference_frame, self.target_frame, rospy.Time(0))
        
        print trans, rot
        
        x = msg.linear.x + trans[0]
        y = msg.linear.y + trans[1]
        theta = msg.angular.z + rot[2]
        
        print x,y, theta
               
        self.tf_broadcaster.sendTransform((x, y, 0),
            tf.transformations.quaternion_from_euler(0, 0, theta),
            rospy.Time.now(),
            self.reference_frame,
            self.target_frame)
    
    # TODO:
        # get TF
        # add step based on velocity and time
        # publish TF
        

if __name__ == '__main__':
    rospy.init_node('move_box')
    mb = move_box()
    rospy.spin()
    
