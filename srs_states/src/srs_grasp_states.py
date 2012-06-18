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
        
        #default grasp categorisation
        #self.grasp_configuration = ""
        #self.object_id=9

    def execute(self, userdata):
        
        global listener
        print "Object pose before transformation:", userdata.object.pose
        try:
            # transform object_pose into base_link
            object_pose_in = userdata.object.pose
            print object_pose_in
            object_pose_in.header.stamp = listener.getLatestCommonTime("/base_link",object_pose_in.header.frame_id)
            object_pose_bl = listener.transformPose("/base_link", object_pose_in)

	    #################### HACK TO TEST 	    #################### 
	    #object_pose_bl.pose.position.x = 0
	    #object_pose_bl.pose.position.y = 0
	    #object_pose_bl.pose.position.x = 0
	    #object_pose_bl.pose.orientation.x = 0
	    #object_pose_bl.pose.orientation.y = 0
	    #object_pose_bl.pose.orientation.z = 0
	    #object_pose_bl.pose.orientation.w = 1
	    #################### HACK TO TEST 	    #################### 

        except rospy.ROSException, e:
            print ("Transformation not possible: %s", e)
            return 'failed'
        
        print "Object pose in base_link:",object_pose_bl
        get_grasps_from_position = rospy.ServiceProxy('get_grasps_from_position', GetGraspsFromPosition)
        req = GetGraspsFromPositionRequest(userdata.target_object_id, object_pose_bl.pose)
        grasp_configuration = copy.deepcopy((get_grasps_from_position(req)).grasp_configuration)
        
        
        #print("grasp configuration is", grasp_configuration[1])
        #print ('grasp configs %s', grasp_configuration)
        
        if len(grasp_configuration) < 1: # no valid configuration found
            print "grasp not possible"
            return 'not_possible'
        else:       
            poses=list()
            for index in range(len(grasp_configuration)):
                pose = PoseStamped()
                pose.header.frame_id = "/base_link"
                pose.pose = grasp_configuration[index].pre_grasp
                poses.append(pose)
            userdata.poses = poses
            userdata.grasp_configuration = grasp_configuration
        return 'succeeded'
        
        """
        #############################
        # I prefer to get 6 possible configuration from each direction without checking the IK and then check the possibility in the arm navigation state
        #############################
        rospy.loginfo('Waiting for /get_pregrasp service')
        get_possible_grasps = rospy.ServiceProxy('/get_pregrasps',GetPreGrasp)
        
        max_num_of_grasp_configurations_per_side = 1  # get 1 grasp configuration per side by default
        try:
            max_num_of_grasp_configurations_per_side = rospy.get_param("srs/common/max_num_of_grasp_configurations_per_side")
        except Exception, e:
            rospy.loginfo("Parameter Server not ready, use default grasp configuration per side for grasp")
        
        req = get_possible_grasps(userdata.target_object_id, object_pose_bl.pose, max_num_of_grasp_configurations_per_side)
        
        # Depend on the service update from ROB
        #userdata.grasp_configuration = (get_grasps_from_position(req)).grasp_configuration
        
        return 'succeeded'
        """


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

	#TODO: THIS PART SHOULD BE IN THE PREVIOUS STATE #####################################################
	grasp_configuration_id = len(userdata.grasp_configuration)-1;	#TODO: userdata.grasp_configuration_id

	for i in range(0, len(userdata.grasp_configuration)):
		print userdata.grasp_configuration[i].category
		if userdata.grasp_configuration[i].category == "TOP":
			grasp_configuration_id = i;			#TODO: userdata.grasp_configuration_id
			break;
	#######################################################################################################

        category = userdata.grasp_configuration[grasp_configuration_id].category	#TODO: userdata.grasp_configuration_id
   
        if category == "TOP":
            userdata.grasp_categorisation = 'top'
            sdh_handle=sss.move("sdh", "spheropen")
        else:
            userdata.grasp_categorisation = 'side'
            sdh_handle=sss.move("sdh", "cylopen")
        sdh_handle.wait()
        rospy.sleep(2)
        
        #Get the current arm joint states.
        while self.arm_state.get_num_connections() == 0:
    		time.sleep(0.3)
    		continue


    	#Move to grasp position with SDH open ---------------------------------------------
    	#pregrasp
        pre_grasp_stamped = PoseStamped();
        pre_grasp_stamped.header.frame_id = "/base_link";
        pre_grasp_stamped.pose = userdata.grasp_configuration[grasp_configuration_id].pre_grasp;	#TODO: userdata.grasp_configuration_id
    
    	#grasp
        grasp_stamped = PoseStamped();
        grasp_stamped.header.frame_id = "/base_link";
        grasp_stamped.pose = userdata.grasp_configuration[grasp_configuration_id].grasp;		#TODO: userdata.grasp_configuration_id
    
    	#postgrasp
        post_grasp_stamped = copy.deepcopy(grasp_stamped);
        post_grasp_stamped.pose.position.z += 0.1;


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
	    sss.say(["I can not catch the object!"], False)
            return 'not_completed';
        else:
	    sss.say(["I am grasping the object now!"], False)

            #Close SDH based on the grasp configuration to grasp.
            arm_handle = sss.move("arm", [pre_grasp_conf, grasp_conf])
	    arm_handle.wait()
	    rospy.sleep(4)

            #Close SDH based on the grasp configuration to grasp.
            ssh_handle = sss.move("sdh", [list(userdata.grasp_configuration[grasp_configuration_id].sdh_joint_values)])	#TODO: userdata.grasp_configuration_id
	    ssh_handle.wait()
            rospy.sleep(2);

            #Confirm the grasp based on force feedback
            successful_grasp = grasping_functions.sdh_tactil_sensor_result();#False

            if not successful_grasp:
                #Regrasp (close MORE the fingers)
                regrasp = list(userdata.grasp_configuration[grasp_configuration_id].sdh_joint_values)			#TODO: userdata.grasp_configuration_id
                print "Current config, trying regrasp", regrasp
                regrasp[1] += 0.07
                regrasp[3] += 0.07
                regrasp[5] += 0.07
                print "to", regrasp

                sdh_handle = sss.move("sdh", [regrasp])
		sdh_handle.wait()
		rospy.sleep(2)

                successful_grasp = grasping_functions.sdh_tactil_sensor_result();#True
                if not successful_grasp:
		    sss.say(["I can not catch the object.!"], False)
                    return 'not_completed'
		
            arm_handle = sss.move("arm",[post_grasp_conf,"look_at_table","hold"])
	    arm_handle.wait()
	    rospy.sleep(2)

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
            userdata.base_pose = symbol_grounding_grasp_base_pose_experimental(object_pose_map.pose, parent_obj_geometry, all_workspaces_on_map.objectsInfo)
            return 'retry'
        except rospy.ServiceException, e:
            print "Service call failed: %s" %e
            return 'failed'

