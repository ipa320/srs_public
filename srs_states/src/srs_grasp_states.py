#!/usr/bin/python

import roslib
roslib.load_manifest('srs_grasping')
import rospy
import grasping_functions
from shared_state_information import *
import copy
from srs_knowledge.srv import *
from srs_symbolic_grounding.srv import *
from srs_symbolic_grounding.msg import *
from simple_script_server import *
sss = simple_script_server()

from srs_grasping.srv import *
from srs_knowledge.srv import *

class select_srs_grasp(smach.State):

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'not_possible', 'failed', 'preempted'],
            input_keys=['object', 'target_object_id'],
            output_keys=['grasp_configuration', 'poses'])
  

    def execute(self, userdata):
        
        global listener
        print "Object pose before transformation:", userdata.object.pose
        try:
            # transform object_pose into base_link
            object_pose_in = userdata.object.pose
            object_pose_in.header.stamp = listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
            object_pose_bl = listener.transformPose("/base_link", object_pose_in)

        except rospy.ROSException, e:
            print ("Transformation not possible: %s", e)
            return 'failed'
        
        get_grasps_from_position = rospy.ServiceProxy('get_feasible_grasps', GetFeasibleGrasps)
        req = GetFeasibleGraspsRequest(userdata.target_object_id, object_pose_bl.pose)
        grasp_configuration = copy.deepcopy((get_grasps_from_position(req)).grasp_configuration)
        
        
        if len(grasp_configuration) < 1: # no valid configuration found
            print "grasp not possible"
            return 'not_possible'
        else:       
            poses=list()
            for index in range(len(grasp_configuration)):
                pose = PoseStamped()
                pose.header.frame_id = "/base_link"
                pose.pose = grasp_configuration[index].pre_grasp.pose
                poses.append(pose)
            userdata.poses = poses
            userdata.grasp_configuration = grasp_configuration
        return 'succeeded'
     

class srs_grasp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','not_completed','failed', 'preempted'], 
                             input_keys=['grasp_configuration','grasp_configuration_id'], 
                             output_keys=['grasp_categorisation'])

        self.arm_state = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state)
        self.current_arm_state = [];

    def get_joint_state(self, msg):
        self.current_arm_state = list(msg.desired.positions)


    def execute(self, userdata): 

	#TODO: THIS PART SHOULD BE IN THE STATE THAT SELECTS THE grasp_configuration_id ######################
	#TOP grasp priority. If there are not TOP configurations, the last grasp is selected.		######
	grasp_configuration_id = len(userdata.grasp_configuration)-1;					######
	#userdata.grasp_configuration_id = len(userdata.grasp_configuration)-1;				######
													######
	for i in range(0, len(userdata.grasp_configuration)):						######
		print userdata.grasp_configuration[i].category						######
		if userdata.grasp_configuration[i].category == "TOP":					######
			grasp_configuration_id = i;							######
			#userdata.grasp_configuration_id = i;						######
			break;										######
	######################################################################################################

        category = userdata.grasp_configuration[grasp_configuration_id].category
   	#category = userdata.grasp_configuration[userdata.grasp_configuration_id].category
        
	if category == "TOP":
            userdata.grasp_categorisation = 'top'
            sdh_handle=sss.move("sdh", "spheropen")
        else:
            userdata.grasp_categorisation = 'side'
            sdh_handle=sss.move("sdh", "cylopen")
        sdh_handle.wait()
        
        #Get the current arm joint states.
        while self.arm_state.get_num_connections() == 0:
    		time.sleep(0.3)
    		continue

    	#pregrasp
        pre_grasp_stamped = PoseStamped();
        pre_grasp_stamped.header.frame_id = "/base_link";
        pre_grasp_stamped.pose = userdata.grasp_configuration[grasp_configuration_id].pre_grasp.pose;
    	#pre_grasp_stamped.pose = userdata.grasp_configuration[userdata.grasp_configuration_id].pre_grasp.pose;
    	
	#grasp
        grasp_stamped = PoseStamped();
        grasp_stamped.header.frame_id = "/base_link";
        grasp_stamped.pose = userdata.grasp_configuration[grasp_configuration_id].grasp.pose;
	#grasp_stamped.pose = userdata.grasp_configuration[userdata.grasp_configuration_id].grasp.pose;    

    	#postgrasp
        post_grasp_stamped = copy.deepcopy(grasp_stamped);
        post_grasp_stamped.pose.position.z += 0.15;


        sol = False
        for w in range(0,10):
            (pre_grasp_conf, error_code) = grasping_functions.callIKSolver(self.current_arm_state, pre_grasp_stamped)		
            if(error_code.val == error_code.SUCCESS):
                for k in range(0,10):
                    (grasp_conf, error_code) = grasping_functions.callIKSolver(pre_grasp_conf, grasp_stamped)
                    if(error_code.val == error_code.SUCCESS):	
                        (post_grasp_conf, error_code) = grasping_functions.callIKSolver(pre_grasp_conf, post_grasp_stamped)	
                        if(error_code.val == error_code.SUCCESS):
                            sol = True
                            break
                if sol:
                    break

        if not sol:
	    sss.say(["I can't catch the object!"], False)
            return 'not_completed';
        else:
	    sss.say(["I am grasping the object now!"], False)

            #Move arm to pregrasp->grasp position.
            arm_handle = sss.move("arm", [pre_grasp_conf, grasp_conf])
	    rospy.sleep(4)
	    arm_handle.wait()
	    

            #Close SDH based on the grasp configuration to grasp.
            ssh_handle = sss.move("sdh", [list(userdata.grasp_configuration[grasp_configuration_id].sdh_joint_values)])
	    #ssh_handle = sss.move("sdh", [list(userdata.grasp_configuration[userdata.grasp_configuration_id].sdh_joint_values)])
	    rospy.sleep(2);
	    ssh_handle.wait()
            

            #Confirm the grasp based on force feedback
            successful_grasp = grasping_functions.sdh_tactil_sensor_result();

            if not successful_grasp:
                #Regrasp (close MORE the fingers)
                regrasp = list(userdata.grasp_configuration[grasp_configuration_id].sdh_joint_values)
 		#regrasp = list(userdata.grasp_configuration[userdata.grasp_configuration_id].sdh_joint_values)
                print "Current config, trying regrasp", regrasp
                regrasp[1] += 0.07
                regrasp[3] += 0.07
                regrasp[5] += 0.07
                print "to", regrasp

                sdh_handle = sss.move("sdh", [regrasp])
		rospy.sleep(2)
		sdh_handle.wait()
		

                successful_grasp = grasping_functions.sdh_tactil_sensor_result();
                if not successful_grasp:
		    sss.say(["I can't catch the object.!"], False)
                    return 'not_completed'
		
            arm_handle = sss.move("arm",[post_grasp_conf,"look_at_table","hold"])
	    rospy.sleep(4)
	    arm_handle.wait()
	    
	    sss.say(["I have grasped the object with success!"], False)
            return 'succeeded'
        


# estimate the best grasp position
class grasp_base_pose_estimation(smach.State):
    
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['retry', 'not_retry', 'failed', 'preempted'],
            input_keys=['object','target_workspace_name'],
            output_keys=['base_pose'])
        
        self.counter = 0 


    def execute(self, userdata):
        
        self.counter = self.counter + 1
        if self.counter > 1:
            self.counter = 0
            #only need to retry once
            return 'not_retry'
        
        global listener

        try:
            #transform object_pose into base_link
            object_pose_in = userdata.object.pose
            object_pose_in.header.stamp = listener.getLatestCommonTime("/map",object_pose_in.header.frame_id)
            object_pose_map = listener.transformPose("/map", object_pose_in)
        except rospy.ROSException, e:
            print "Transformation not possible: %s"%e
            return 'failed'

        all_workspaces_on_map = ''
        index_of_the_target_workspace = ''

        try:
            rospy.wait_for_service('get_workspace_on_map')
            getWorkspace = rospy.ServiceProxy('get_workspace_on_map', GetWorkspaceOnMap)
            all_workspaces_on_map = getWorkspace(os.environ['ROBOT_ENV'], True)
            
            #get the index of the target workspace e.g. table0    
            index_of_the_target_workspace = all_workspaces_on_map.objects.index(userdata.target_workspace_name)
            #get the pose of the workspace from knowledge service
            target_object_pose = all_workspaces_on_map.objectsInfo[index_of_the_target_workspace].pose
            #get the houseHoldID of the workspace 
            object_id = all_workspaces_on_map.houseHoldId[index_of_the_target_workspace]
            
            #rospy.loginfo ("target name: %s", userdata.target_workspace_name)      
            #rospy.loginfo ("target pose: %s", target_object_pose)
            #rospy.loginfo ("target id: %s", object_id)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
            return 'failed'

        parent_obj_geometry = SRSFurnitureGeometry()
        parent_obj_geometry.pose = target_object_pose
        parent_obj_geometry.l = all_workspaces_on_map.objectsInfo[index_of_the_target_workspace].l
        parent_obj_geometry.w = all_workspaces_on_map.objectsInfo[index_of_the_target_workspace].w
        parent_obj_geometry.h = all_workspaces_on_map.objectsInfo[index_of_the_target_workspace].h
        
        rospy.wait_for_service('symbol_grounding_grasp_base_pose_experimental')
        symbol_grounding_grasp_base_pose_experimental = rospy.ServiceProxy('symbol_grounding_grasp_base_pose_experimental', SymbolGroundingGraspBasePoseExperimental)
        try:
            result = symbol_grounding_grasp_base_pose_experimental(object_pose_map.pose, parent_obj_geometry, all_workspaces_on_map.objectsInfo)
            userdata.base_pose = [result.grasp_base_pose.x, result.grasp_base_pose.y, result.grasp_base_pose.theta]
            return 'retry'
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e
            return 'failed'

