#! /usr/bin/env python
import roslib; roslib.load_manifest('srs_user_tests')

import rospy
from math import fabs, sqrt
import numpy
import tf
import rosbag
import os
import glob
import sys
from geometry_msgs.msg import PoseStamped

class TfEval(object):
    
    def __init__(self, def_frame_1='/map', def_frame_2='/rviz_cam', def_bag_file_dir=''):
        
        self.frame_1 = rospy.get_param('~frame_1', def_frame_1)
        self.frame_2 = rospy.get_param('~frame_2', def_frame_2)
        self.debug = rospy.get_param('~debug', False)
        
        # /logs/$(arg id)/$(arg exp)/$(arg task)/$(arg cond)/
        
        self.id = rospy.get_param('~id')
        self.exp = rospy.get_param('~exp')
        self.task = rospy.get_param('~task')
        self.cond = rospy.get_param('~cond')
        self.path = rospy.get_param('~path')
        self.output_file = rospy.get_param('~output_file')
        
        self.start_time = rospy.Time(rospy.get_param('~start_time',0.0))
        self.end_time = rospy.Time(rospy.get_param('~end_time',0.0))
        
        self.cache_dur = rospy.Duration(0.0)
        
        self.listener = tf.TransformListener()
        
        self.tfpublisher= rospy.Publisher("tf",tf.msg.tfMessage)
        
        if self.path[-1] != '/':
            
            self.path += '/'
            
        self.bag_file_dir = self.path + self.id + '/' + self.exp + '/' + self.task + '/' + self.cond + '/'
        
        if not os.path.isdir(self.bag_file_dir):
            
            rospy.logerr('Directory (%s) does not exist.',self.bag_file_dir)
            sys.exit()
        
        fn = glob.glob(self.bag_file_dir + '*.bag')
        
        if len(fn) == 0:
            
            rospy.logerr('There is no bag file (%s)',self.bag_file_dir)
            sys.exit()
            
        if len(fn) > 1:
            
            rospy.logerr("There are more bag files in the directory! Don't know which one to analyze.")
            sys.exit()
            
        self.bag_filename = fn[0]
        
        try:
        
            self.bag = rosbag.Bag(self.bag_filename)
            
        except rosbag.ROSBagException:
            
            msg = 'Error on openning bag file (' + self.bag_filename + ')' 
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit()
            
        except rosbag.ROSBagFormatException:
            
            msg = 'Bag file is corrupted.'
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit()
            
        self.started = False
        
        self.pose = PoseStamped()
        self.pose.header.frame_id = self.frame_2
        self.pose.header.stamp = rospy.Time(0)
        self.pose.pose.position.x = 0.0
        self.pose.pose.position.y = 0.0
        self.pose.pose.position.z = 0.0
        
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = 0.0
        self.pose.pose.orientation.w = 0.0
        
        self.last_pose = None
        
        self.changes = rospy.Duration(0)
        self.timer_period = rospy.Duration(0.01)
        
        
        
        self.timer_trig = False
        
        if os.path.isfile(self.path + self.output_file):
        
            self.csv = open(self.path + self.output_file,'a')
            
        else:
            
            rospy.loginfo('Creating new CSV file.')
            self.csv = open(self.path + self.output_file,'w')
            self.csv.write('id;experiment;task;condition;start;end;changes;total;path_len;rotations\n')
        
        #rospy.Timer(self.timer_period, self.timer)
        
        rospy.loginfo('Initialized with %s and %s frames.',self.frame_1,self.frame_2)
        rospy.loginfo('File to analyze: %s',self.bag_filename)
        dur = self.end_time.to_sec() - self.start_time.to_sec()
        rospy.loginfo('Start time: %f, end time: %f (dur: %f)',round(self.start_time.to_sec(),2), round(self.end_time.to_sec(),2),round(dur,2))
        
        self.change_occ_time = rospy.Time(0)
        self.change_last_time = rospy.Time(0)
        self.intgr = 0
        self.change = False
        self.last_path_len = 0.0
        
        
    def readBag(self,start,end,start_pose=None):
        
        
        frames_avail = rospy.Time(0)

        frames_checked = False
        
        started = False
        last_pose = start_pose
        last = None

        
        
        np = None
        
        path_len = 0.0
        
        start_real = rospy.Time(0)
        #end_real = rospy.Time(0)
        end_real = start-self.cache_dur
        
        error = False
        error_at = rospy.Time(0)
        
        path_len = 0.0
        
        rotations = 0.0
        
        changes = rospy.Duration(0)
        
        if self.debug:
            
            rospy.loginfo('Request for analysis from: %s, to: %s.',str(round(start.to_sec(),2)), str(round(end.to_sec(),2)))
    
        try:
            
                for topic, msg, t in self.bag.read_messages(topics=['/tf'],start_time=start-self.cache_dur,end_time=end):
                    
                    end_real = t
                    
                    if rospy.is_shutdown():
                        
                        break
                        
                    try:
                        
                        self.tfpublisher.publish(msg)
                        
                    except TypeError:
                        
                        if self.debug:
                            
                            rospy.logwarn('Strange TF msg.')
                            
                        pass
                        
                    #rospy.sleep(0.001)
                    
                    if not frames_checked:
                        
                        if self.listener.frameExists(self.frame_1) and self.listener.frameExists(self.frame_2):
                            
                            frames_avail = t
                            frames_checked = True
                            if self.debug:
                            
                                rospy.loginfo('Both TF frames are available. Caching TF...')
                            
                        else:
                            
                            continue
                        
                    if not started: #and (t >= frames_avail + self.cache_dur):
                        last = t
                        start_real = t
                        started = True
                        
                        if self.debug:
                        
                            rospy.loginfo('Starting analysis, at %f',round(t.to_sec(),2))
                    
                    if (t - last >= self.timer_period): # we don't want to do calculations each for iteration
                        
                        last = t
                        self.pose.header.stamp = rospy.Time(0)
        
                        try:
                
                            self.listener.waitForTransform(self.frame_1,self.frame_2,self.pose.header.stamp,rospy.Duration(0.05))
                        
                            np = self.listener.transformPose(self.frame_1, self.pose)
                    
                        except Exception, e:
                            
                            if self.debug: 
                                
                                rospy.logwarn("Can't transform.")
                                
                            continue
                        
                        if last_pose is None:
                            
                            last_pose = np
                            
                            continue
                        
                        pdist = sqrt((np.pose.position.x - last_pose.pose.position.x)**2 + (np.pose.position.y - last_pose.pose.position.y)**2 + (np.pose.position.z - last_pose.pose.position.z)**2)
                        path_len += pdist
                        
                        (r, p, y) = tf.transformations.euler_from_quaternion([np.pose.orientation.x,
                                                                             np.pose.orientation.y,
                                                                             np.pose.orientation.z,
                                                                             np.pose.orientation.w],axes='sxyz') # is this order correct??
                        
                        (lr, lp, ly) = tf.transformations.euler_from_quaternion([last_pose.pose.orientation.x,
                                                                                 last_pose.pose.orientation.y,
                                                                                 last_pose.pose.orientation.z,
                                                                                 last_pose.pose.orientation.w],axes='sxyz')
                        
                        
                        dor = fabs(r - lr)
                        dop = fabs(p - lp)
                        doy = fabs(y - ly)
                        
                        
                        # if there is any change in position/orientation, lets do some stuff
                        if path_len > self.last_path_len or dor > 0.0 or dop > 0.0 or doy > 0.0:
                        
                            if self.change==False:
                                
                                self.change_occ_time = t
                                self.change = True
                                self.intgr = 0
        
                            #cnt = 0
                            self.intgr += 1
                            self.change_last_time = t
                            
                            rotations += doy
                            last_pose = np
                            self.last_path_len = path_len
                            
                        else:
                            
                            if self.change == True and (t-self.change_last_time) >= rospy.Duration(2.0): #cnt > 50: # 20
                                
                                if self.intgr > 1:
                                
                                    dt = self.change_last_time - self.change_occ_time
                                    changes += dt
                                    
                                if self.debug: #and dt != rospy.Duration(0.0):
                                
                                    print str(dt.to_sec()) + '; ' + str(self.intgr)
                                
                                self.change = False
                            
                        
        except rosbag.ROSBagFormatException:
            
            if self.debug:
                
                rospy.logerr("Bag file format corrupted around %s.",str(round(end_real.to_sec(),2)))
                
            error = True
        
        except roslib.message.DeserializationError:
            
            if self.debug:
            
                rospy.logerr('Bag file deserialization error around %s.',str(round(end_real.to_sec(),2)))
                
            error = True
            
        except:
            
            if self.debug:
            
                rospy.logerr('Some other error around %s.',str(round(end_real.to_sec(),2)))
                
            error = True
            
        if np is None and last_pose is not None:
            
            np = last_pose
            
        return (error, start_real, end_real, changes, path_len, rotations, np)
        
        
    def analyze(self):
        
        dur = rospy.Duration(0)
        errors = 0
        spath_len = 0.0
        srotations = 0.0
        last_pose = None
        
        (error, start_real, end_real, changes, path_len, rotations, last_pose) = self.readBag(self.start_time, self.end_time, last_pose)
        
        fstart_real = start_real # first start real
        dur = end_real - start_real
        schanges = changes # sum of changes
        spath_len = path_len
        srotations = rotations
        
        while (error is True and end_real < self.end_time and not rospy.is_shutdown()):
            
            end_real += self.cache_dur + rospy.Duration(0.25)
            
            (error, start_real, end_real, changes, path_len, rotations, last_pose) = self.readBag(end_real, self.end_time, last_pose)
            
            errors += 1
            
            if start_real != rospy.Time(0):
                         
                dur += end_real - start_real
                
                schanges += changes
                spath_len += path_len
                srotations += rotations
               
        rospy.loginfo('Finished (%d errors).',errors)
        
        tot = str(round(dur.to_sec(),2))
        ch = str(round(schanges.to_sec(),2))
        
        if fstart_real != rospy.Time(0):
        
            rospy.loginfo('Real start: %s, real end: %s.',str(fstart_real.to_sec()),str(end_real.to_sec()))
            rospy.loginfo('Total duration: ' + tot + 's')
            rospy.loginfo('Changes:' + ch + 's, path length: ' + str(round(spath_len,2)) + 'm, rotations: ' + str(round(srotations,2)) + 'rads.' )
            
            self.csv.write(self.id + ';' + self.exp + ';' + self.task + ';' + self.cond + ';' + str(fstart_real.to_sec()) + ';' + str(end_real.to_sec()) + ';' + ch + ';' + tot + ';' + str(round(spath_len,2)) + ';' + str(round(srotations,2)) + '\n')
            
        else:
            
            rospy.logwarn("Can't compute anything. One of the frames was probably not available.")
        
        
        self.csv.close()
        
        
if __name__ == '__main__':

        rospy.init_node('tf_test_eval')

        node = TfEval()
        
        node.analyze()
        
