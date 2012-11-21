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
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from srs_msgs.msg import SRSSpatialInfo 

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
        req = GetFeasibleGraspsRequest(userdata.target_object_id, object_pose_bl.pose, [0.0, 0.0]) #[X,Z]
        grasp_configuration = copy.deepcopy((get_grasps_from_position(req)).grasp_configuration)

        if len(grasp_configuration) < 1: # no valid configurations found
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

	self.current_arm_state = [];
        self.arm_state = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, self.get_joint_state)

    def get_joint_state(self, msg):
        self.current_arm_state = list(msg.desired.positions)

    def get_fake_id(self, grasp_configuration):
	#TOP grasps priority.
	for i in range(0, len(grasp_configuration)):
		if grasp_configuration[i].category == "TOP":
			return i;
	return (len(grasp_configuration)-1);

    def execute(self, userdata): 

	grasp_configuration_id = userdata.grasp_configuration_id
	#grasp_configuration_id = self.get_fake_id(userdata.grasp_configuration);
	
        category = userdata.grasp_configuration[grasp_configuration_id].category
	if category == "TOP":
            userdata.grasp_categorisation = 'top'
            sdh_handle=sss.move("sdh", "spheropen")
        elif category == "SIDE" or category == "-SIDE":
            userdata.grasp_categorisation = 'side'
            sdh_handle=sss.move("sdh", "cylopen")
	else:
            userdata.grasp_categorisation = 'front'
            sdh_handle=sss.move("sdh", "cylopen")
        sdh_handle.wait()
        

        #Get the current arm joint states.
	rospy.sleep(3)
        while self.arm_state.get_num_connections() == 0:
    		time.sleep(0.3)
    		continue

	#pregrasps
	pre_grasp_stamped = copy.deepcopy(userdata.grasp_configuration[grasp_configuration_id].pre_grasp);

	#grasp
        grasp_stamped = copy.deepcopy(userdata.grasp_configuration[grasp_configuration_id].grasp);

	#postgrasp
        post_grasp_stamped = copy.deepcopy(grasp_stamped);
        post_grasp_stamped.pose.position.x += 0.15;
        post_grasp_stamped.pose.position.z += 0.2;

	grasp_trajectory = [];
	postgrasp_trajectory = [];

	class BadGrasp(Exception): pass
	try:
		try:
			ipa_arm_navigation = rospy.get_param("srs/ipa_arm_navigation")
		except Exception, e:
			rospy.INFO('can not read parameter of srs/ipa_arm_navigation, use the default value planned arm navigation disabled')

		if ipa_arm_navigation.lower() == 'true':
			#mode = "planned"
			grasp_trajectory.append(self.current_arm_state)
		else:
			#pre-grasp
			(pgc1, error_code) = grasping_functions.graspingutils.callIKSolver(self.current_arm_state, pre_grasp_stamped)
			if(error_code.val != error_code.SUCCESS):
				sss.say(["I can not move the arm to the pregrasp position!"])
				raise BadGrasp();
			grasp_trajectory.append(pgc1)

			#second pre-grasp
			aux_pre = pre_grasp_stamped.pose.position.x;
			aux = 0.0;
			for i in range(0,5):
				aux += 0.02;
				pre_grasp_stamped.pose.position.x = aux_pre + aux;
				(pgc2, error_code) = grasping_functions.graspingutils.callIKSolver(pgc1, pre_grasp_stamped)
				if(error_code.val == error_code.SUCCESS):
					grasp_trajectory.append(pgc2);
					break;	

		#grasp
		(gc, error_code) = grasping_functions.graspingutils.callIKSolver(grasp_trajectory[len(grasp_trajectory)-1], grasp_stamped)
		if(error_code.val != error_code.SUCCESS):
			sss.say(["I can not move the arm to the grasp position!"])
			raise BadGrasp();		
		grasp_trajectory.append(gc);

		#Move arm to pregrasp->grasp position.
		arm_handle = sss.move("arm", grasp_trajectory, False)# , mode=mode);
		rospy.sleep(4)
		arm_handle.wait(6)


                # wait while movement
                r = rospy.Rate(10)
                preempted = False
                arm_state = -1
                while True:
			preempted = self.preempt_requested()
                        arm_state = arm_handle.get_state()
                        if preempted or ( arm_state == 3) or (arm_state == 4):
                        	break # stop waiting  
                        r.sleep()
		

		#Close SDH based on the grasp configuration to grasp.
		sdh_handle = sss.move("sdh", [list(userdata.grasp_configuration[grasp_configuration_id].sdh_joint_values)], False)
		sss.say(["I am grasping the object now!"])
		rospy.sleep(3);
		sdh_handle.wait(4)
		

		#Confirm the grasp based on force feedback
		if not grasping_functions.graspingutils.sdh_tactil_sensor_result():
			#Regrasp (close MORE the fingers)
			regrasp = list(userdata.grasp_configuration[grasp_configuration_id].sdh_joint_values)
			print "Current config, trying regrasp:\n", regrasp
			regrasp[1] += 0.07
			regrasp[3] += 0.07
			regrasp[5] += 0.07
			print "to:\n", regrasp

			sdh_handle = sss.move("sdh", [regrasp])
			rospy.sleep(1)
			sdh_handle.wait(2)

			if not grasping_functions.graspingutils.sdh_tactil_sensor_result():
				sss.say(["I can not fix the object correctly!"])
				raise BadGrasp();

		#post-grasp
		aux_x = post_grasp_stamped.pose.position.x;
		aux = 0.0;

		for i in range(0,5):
			post_grasp_stamped.pose.position.x = aux_x + aux;
			(post_grasp_conf, error_code) = grasping_functions.graspingutils.callIKSolver(self.current_arm_state, post_grasp_stamped)
			aux += 0.01;
			if(error_code.val == error_code.SUCCESS):
				postgrasp_trajectory.append(post_grasp_conf);
				break;

		if len(postgrasp_trajectory) == 0:
			sss.say(["I can not move the object to the postgrasp position!"])
		else:
			#second post-grasp
			aux_x = post_grasp_stamped.pose.position.x;
			aux = 0.0;
			for i in range(0,5):
				post_grasp_stamped.pose.position.x = aux_x + aux;
				(post_grasp_conf2, error_code) = grasping_functions.graspingutils.callIKSolver(post_grasp_conf, post_grasp_stamped)
				aux += 0.02;
				if(error_code.val == error_code.SUCCESS):
					postgrasp_trajectory.append(post_grasp_conf2);
					break;

		arm_handle = sss.move("arm",postgrasp_trajectory, False)
		sss.say(["I have grasped the object with success!"])
		rospy.sleep(2)
		arm_handle.wait(3)

		return 'succeeded'


	except BadGrasp:
		sss.say(["I can not catch the object!"], False)
		return 'not_completed';

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

        parent_obj_geometry = SRSSpatialInfo()
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

