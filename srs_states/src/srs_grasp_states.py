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
        rospy.loginfo("Waiting /arm_kinematics/get_ik service...")
        rospy.wait_for_service('/arm_kinematics/get_ik')
        rospy.loginfo("/arm_kinematics/get_ik has been found!")
        self.iks = rospy.ServiceProxy('/arm_kinematics/get_ik', GetPositionIK)
        self.sub = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state)
        self.current_joint_configuration = [];

    def get_joint_state(self, msg):
        #global current_joint_configuration
        self.current_joint_configuration = list(msg.desired.positions)


    def execute(self, userdata):
        rospy.loginfo('Executing state GRASP')

    	#Open SDH at the pre-grasp position -----------------------------------------------
    	#sss.move("sdh", "cylopen")
        
        
        pre_p = userdata.grasp_configuration[userdata.grasp_configuration_id].pre_grasp.position
        g_p = userdata.grasp_configuration[userdata.grasp_configuration_id].grasp.position
        
        category = grasping_functions.get_grasp_category(pre_p , g_p);
        
        if category == "TOP" or category == "DOWN":
            userdata.grasp_categorisation = 'top'
            arm_handle=sss.move("sdh", "spheropen")
        else:
            userdata.grasp_categorisation = 'side'
            arm_handle=sss.move("sdh", "cylopen")
        arm_handle.wait()
        rospy.sleep(2)
        
        #Get the current arm joint states.
        while self.sub.get_num_connections() == 0:
    		time.sleep(0.3)
    		continue


    	#Move to grasp position with SDH open ---------------------------------------------
    	#pregrasp
        pre_grasp_stamped = PoseStamped();
        pre_grasp_stamped.header.frame_id = "/base_link";
        pre_grasp_stamped.pose = userdata.grasp_configuration[userdata.grasp_configuration_id].pre_grasp;
    
    	#grasp
        grasp_stamped = PoseStamped();
        grasp_stamped.header.frame_id = "/base_link";
        grasp_stamped.pose = userdata.grasp_configuration[userdata.grasp_configuration_id].grasp;
    
    	#postgrasp
        post_grasp_stamped = copy.deepcopy(grasp_stamped);
        post_grasp_stamped.pose.position.z += 0.1;

    	#offset
        """
        offset_x = 0#(userdata.grasp_configuration.grasp.position.x - userdata.grasp_configuration.pre_grasp.position.x)/3
        offset_y = 0#(userdata.grasp_configuration.grasp.position.y - userdata.grasp_configuration.pre_grasp.position.y)/3
        offset_z = 0#(userdata.grasp_configuration.grasp.position.z - userdata.grasp_configuration.pre_grasp.position.z)/3
    
    	pre_grasp_stamped.pose.position.x += offset_x
    	pre_grasp_stamped.pose.position.y += offset_y
    	pre_grasp_stamped.pose.position.z -= offset_z
    	grasp_stamped.pose.position.x += offset_x
    	grasp_stamped.pose.position.y += offset_y
    	grasp_stamped.pose.position.z -= offset_z
    	"""

	#global current_joint_configuration

        sol = False
        for w in range(0,10):
            (pre_grasp_conf, error_code) = grasping_functions.callIKSolver(self.current_joint_configuration, pre_grasp_stamped)		
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
            return 'failed';
        else:
            sss.move("torso","home")
            arm_handle = sss.move("arm", [pre_grasp_conf, grasp_conf], False)
            arm_handle.wait();

            #Close SDH based on the grasp configuration to grasp.
            arm_handle = sss.move("sdh", [list(userdata.grasp_configuration[userdata.grasp_configuration_id].sdh_joint_values)], False)
            arm_handle.wait();
            rospy.sleep(2);

            #TODO: Confirm the grasp based on force feedback
            successful_grasp = grasping_functions.sdh_tactil_sensor_result();

            if not successful_grasp:
                #TODO: Regrasp (close MORE the fingers)
                regrasp = list(userdata.grasp_configuration[userdata.grasp_configuration_id].sdh_joint_values)
                regrasp[2] -= 0.1
                regrasp[4] -= 0.1
                regrasp[6] -= 0.1
                arm_handle = sss.move("sdh", [regrasp], False)
                arm_handle.wait();
                successful_grasp = True#grasping_functions.sdh_tactil_sensor_result();
                if not successful_grasp:
                    return 'failed'
            sss.move("arm",[post_grasp_conf,"hold"])
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

