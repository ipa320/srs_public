/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
 * 
 * This file is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This file is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with this file.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <srs_assisted_arm_navigation/assisted_arm_navigation/arm_manip_node.h>

using namespace arm_navigation_msgs;
using namespace planning_scene_utils;
using namespace srs_assisted_arm_navigation;
using namespace srs_assisted_arm_navigation_msgs;

/**
 *  \todo use controllerDoneCallback / armHasStoppedMoving to check if the arm finished trajectory
 *
 */


CArmManipulationEditor::CArmManipulationEditor(planning_scene_utils::PlanningSceneParameters& params, std::vector<string> clist) : PlanningSceneEditor(params)
{

    action_server_ptr_ = NULL;
    inited = false;
    params_ = params;

    // ros::param::param<std::string>("~world_frame",world_frame_,WORLD_FRAME);

    ros::param::param<std::string>("~world_frame",collision_objects_frame_id_,WORLD_FRAME);

    // TODO remove collision_objects_frame_id and keep just world_frame
    world_frame_ =  collision_objects_frame_id_;

    ros::param::param<std::string>("~aco_link",aco_link_,"arm_7_link");


    ROS_INFO("Using %s frame as world frame",collision_objects_frame_id_.c_str());

    //collision_objects_frame_id_ = "/map"; // TODO read from parameter

    // TODO make it configurable through param.
    aco_ = true;

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/spacenav/joy",10,&CArmManipulationEditor::joyCallback,this);

    spacenav.offset_received_ = false;
    spacenav.rot_offset_received_ = false;
    spacenav.lock_orientation_ = false;
    spacenav.lock_position_ = false;

    spacenav.buttons_.push_back(0);
    spacenav.buttons_.push_back(0);

    ros::param::param<double>("~spacenav/max_val",spacenav.max_val_,500.0);
    ros::param::param<double>("~spacenav/step",spacenav.step_,0.1);
    ros::param::param<double>("~spacenav/rot_step",spacenav.rot_step_,0.05);
    ros::param::param<bool>("~spacenav/use_rviz_cam",spacenav.use_rviz_cam_,true);
    ros::param::param<std::string>("~spacenav/rviz_cam_link",spacenav.rviz_cam_link_,"/rviz_cam");

    ros::param::param<bool>("~joint_controls",joint_controls_,false);

    spacenav_timer_ = nh_.createTimer(ros::Duration(0.05),&CArmManipulationEditor::timerCallback,this);

    if (spacenav.use_rviz_cam_) tf_timer_ = nh_.createTimer(ros::Duration(0.01),&CArmManipulationEditor::tfTimerCallback,this);

    offset_sub_ = nh_.subscribe("/spacenav/offset",1,&CArmManipulationEditor::spacenavOffsetCallback,this);
    rot_offset_sub_ = nh_.subscribe("/spacenav/rot_offset",1,&CArmManipulationEditor::spacenavRotOffsetCallback,this);

    arm_nav_state_pub_ = nh_.advertise<AssistedArmNavigationState>(TOP_STATE,5);

    links_ = clist;

    gripper_poses_ = new boost::circular_buffer<geometry_msgs::Pose>(20);

    gripper_poses_thread_ = boost::thread(&CArmManipulationEditor::GripperPoses,this);

    step_used_ = false;

    planning_scene_id = "";
    mpr_id = 0;

}


bool CArmManipulationEditor::transf(std::string target_frame,geometry_msgs::PoseStamped& pose) {

	// transform pose of camera into world
	try {

			if (tfl_->waitForTransform(target_frame, pose.header.frame_id, pose.header.stamp, ros::Duration(2.0))) {

			  tfl_->transformPose(target_frame,pose,pose);

			} else {

			  ROS_ERROR("Could not get TF transf. from %s into %s.",pose.header.frame_id.c_str(),target_frame.c_str());
			  return false;

			}

	} catch(tf::TransformException& ex){
	   std::cerr << "Transform error: " << ex.what() << std::endl;

	   return false;
	}

	return true;

}

void CArmManipulationEditor::processSpaceNav() {


	ROS_INFO_ONCE("Processing spacenav data");

	//boost::mutex::scoped_lock(spacenav.mutex_);
	spacenav.mutex_.lock();
	bool ret = false;

	// there is nothing to do
	if (spacenav.lock_orientation_ && spacenav.lock_position_) ret = true;

	if (!spacenav.offset_received_) ret = true;
	else spacenav.offset_received_ = false;

	if (!spacenav.rot_offset_received_) ret = true;
	else spacenav.rot_offset_received_ = false;

	// continue only if the planning was started
	if (!inited) ret = true;

	spacenav.mutex_.unlock();

	if (ret) return;


	unsigned int state = action_server_ptr_->get_state();

	if (state != ManualArmManipActionServer::S_NEW && state !=ManualArmManipActionServer::S_NONE) {

		ROS_DEBUG("State is %u",state);

		return;

	}

	//boost::mutex::scoped_lock(im_server_mutex_);

	im_server_mutex_.lock();

	visualization_msgs::InteractiveMarker marker;
	//geometry_msgs::Pose new_pose;

	if (!(interactive_marker_server_->get("MPR 0_end_control",marker))) {

		ROS_ERROR_ONCE("Can't get gripper IM pose.");

		im_server_mutex_.unlock();
		return;

	}

	im_server_mutex_.unlock();

	ros::Time now = /*ros::Time(0);*/ ros::Time::now();


	geometry_msgs::PoseStamped npose;

	if (spacenav.use_rviz_cam_) {

		//std::cout << marker.header.frame_id << std::endl;

		npose.pose = marker.pose;
		npose.header.stamp = ros::Time(0);
		npose.header.frame_id = world_frame_;

		if (!transf(spacenav.rviz_cam_link_ + "_add",npose)) return;

	} else {

		npose.pose = marker.pose;
		npose.header.stamp = now;
		npose.header.frame_id = world_frame_;

	}

	if (!spacenav.lock_position_) {

		npose.pose.position.x += (spacenav.offset.x/spacenav.max_val_)*spacenav.step_;
		npose.pose.position.y += (spacenav.offset.y/spacenav.max_val_)*spacenav.step_;
		npose.pose.position.z += (spacenav.offset.z/spacenav.max_val_)*spacenav.step_;

	}

	geometry_msgs::Vector3 rpy = GetAsEuler(npose.pose.orientation);

	//ROS_DEBUG("Gripper current RPY: %f, %f, %f (DEG)",rpy.x,rpy.y,rpy.z);

	if (!spacenav.lock_orientation_) {

		rpy.x += (spacenav.rot_offset.x/spacenav.max_val_)*spacenav.rot_step_;
		rpy.y += (spacenav.rot_offset.y/spacenav.max_val_)*spacenav.rot_step_;
		rpy.z += (spacenav.rot_offset.z/spacenav.max_val_)*spacenav.rot_step_;

	}

	//ROS_DEBUG("Gripper new RPY: %f, %f, %f (DEG)",rpy.x,rpy.y,rpy.z);


	npose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rpy.x,rpy.y,rpy.z);

	if (spacenav.use_rviz_cam_) {

		if (!transf(world_frame_,npose)) return;

	}

	im_server_mutex_.lock();

	if ((interactive_marker_server_->setPose("MPR 0_end_control",npose.pose,npose.header))) {

		  interactive_marker_server_->applyChanges();

		  findIK(npose.pose);

	  }

	im_server_mutex_.unlock();



}

// timer for publishing TF for additional camera frame
void CArmManipulationEditor::tfTimerCallback(const ros::TimerEvent& ev) {

	ROS_INFO_ONCE("Publishing TF for additional RVIZ camera.");

	//boost::mutex::scoped_lock(im_server_mutex_);
	im_server_mutex_.lock();

	visualization_msgs::InteractiveMarker marker;
	//geometry_msgs::Pose new_pose;

	if (!(interactive_marker_server_->get("MPR 0_end_control",marker))) {

		ROS_ERROR_ONCE("Can't get gripper IM pose.");
		im_server_mutex_.unlock();

		return;

	}

	im_server_mutex_.unlock();

	ros::Time now = /*ros::Time(0);*/ ros::Time::now();

	// publish TF for additional camera frame
	geometry_msgs::PoseStamped cam_pose;

	cam_pose.pose.position.x = 0.0;
	cam_pose.pose.position.y = 0.0;
	cam_pose.pose.position.z = 0.0;

	cam_pose.pose.orientation.x = 0.0;
	cam_pose.pose.orientation.y = 0.0;
	cam_pose.pose.orientation.z = 0.0;
	cam_pose.pose.orientation.w = 1.0;

	cam_pose.header.stamp = now;
	cam_pose.header.frame_id = spacenav.rviz_cam_link_;

	if (!transf(world_frame_,cam_pose)) return;

	// set Z of camera to be same as Z of IM
	cam_pose.pose.position = marker.pose.position;

	// "reset" pitch
	geometry_msgs::Vector3 rpy = GetAsEuler(cam_pose.pose.orientation);

	rpy.y = 0;

	cam_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rpy.x,rpy.y,rpy.z);

	tf::Transform tr;
	tr.setOrigin(tf::Vector3(cam_pose.pose.position.x,cam_pose.pose.position.y,cam_pose.pose.position.z));
	tr.setRotation(tf::Quaternion(cam_pose.pose.orientation.x,cam_pose.pose.orientation.y,cam_pose.pose.orientation.z,cam_pose.pose.orientation.w));
	br_.sendTransform(tf::StampedTransform(tr, now, world_frame_, spacenav.rviz_cam_link_ + "_add"));



}

// general timer
void CArmManipulationEditor::timerCallback(const ros::TimerEvent& ev) {

	ROS_INFO_ONCE("Assisted arm nav timer callback triggered.");

	// publish state of spacenav buttons (and internal state of arm nav. -> TBD)
	AssistedArmNavigationState msg;
	msg.orientation_locked = spacenav.lock_orientation_;
	msg.position_locked = spacenav.lock_position_;

	arm_nav_state_pub_.publish(msg);

	// update position of IM according to spacenav data
	processSpaceNav();

}

void CArmManipulationEditor::spacenavOffsetCallback(const geometry_msgs::Vector3ConstPtr& offset) {


	ROS_INFO_ONCE("Spacenav offset received!");

	spacenav.mutex_.lock();

	spacenav.offset_received_ = true;

	spacenav.offset = *offset;

	if (spacenav.offset.x > spacenav.max_val_) spacenav.offset.x = spacenav.max_val_;
	if (spacenav.offset.x < -spacenav.max_val_) spacenav.offset.x = -spacenav.max_val_;

	if (spacenav.offset.y > spacenav.max_val_) spacenav.offset.y = spacenav.max_val_;
	if (spacenav.offset.y < -spacenav.max_val_) spacenav.offset.y = -spacenav.max_val_;

	if (spacenav.offset.z > spacenav.max_val_) spacenav.offset.z = spacenav.max_val_;
	if (spacenav.offset.z < -spacenav.max_val_) spacenav.offset.z = -spacenav.max_val_;

	spacenav.mutex_.unlock();

}

void CArmManipulationEditor::spacenavRotOffsetCallback(const geometry_msgs::Vector3ConstPtr& rot_offset) {


	ROS_INFO_ONCE("Spacenav rot_offset received!");

	spacenav.mutex_.lock();

	spacenav.rot_offset_received_ = true;

	spacenav.rot_offset = *rot_offset;

	if (spacenav.rot_offset.x > spacenav.max_val_) spacenav.rot_offset.x = spacenav.max_val_;
	if (spacenav.rot_offset.x < -spacenav.max_val_) spacenav.rot_offset.x = -spacenav.max_val_;

	if (spacenav.rot_offset.y > spacenav.max_val_) spacenav.rot_offset.y = spacenav.max_val_;
	if (spacenav.rot_offset.y < -spacenav.max_val_) spacenav.rot_offset.y = -spacenav.max_val_;

	if (spacenav.rot_offset.z > spacenav.max_val_) spacenav.rot_offset.z = spacenav.max_val_;
	if (spacenav.rot_offset.z < -spacenav.max_val_) spacenav.rot_offset.z = -spacenav.max_val_;

	spacenav.mutex_.unlock();

}

void CArmManipulationEditor::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	if (joy->buttons.size()==2) {

		ROS_INFO_ONCE("Spacenav data received!");

		boost::mutex::scoped_lock(spacenav.mutex_);

		// left button pressed
		if (spacenav.buttons_[0] == 0 && joy->buttons[0] == 1) {

			if (spacenav.lock_position_) {

				ROS_INFO("Spacenav - unlocking position.");
				spacenav.lock_position_ = false;

			} else {

				ROS_INFO("Spacenav - locking position.");
				spacenav.lock_position_ = true;

			}


		}

		// right button pressed
		if (spacenav.buttons_[1] == 0 && joy->buttons[1] == 1) {

			if (spacenav.lock_orientation_) {

				ROS_INFO("Spacenav - unlocking orientation.");
				spacenav.lock_orientation_ = false;

			} else {

				ROS_INFO("Spacenav - locking orientation.");
				spacenav.lock_orientation_ = true;

			}


		}


		spacenav.buttons_ = joy->buttons;

	}


};

CArmManipulationEditor::~CArmManipulationEditor() {

  gripper_poses_thread_.join();

  if (action_server_ptr_!=NULL) delete action_server_ptr_;

  if (tfl_!=NULL) delete tfl_;

  if (gripper_poses_!= NULL) delete gripper_poses_;

}

bool CArmManipulationEditor::checkPose(geometry_msgs::PoseStamped &p, std::string frame) {


 if (p.header.frame_id==frame) {

	 return true;

  } else {

	ros::Time now = ros::Time::now();

	try {

		ROS_INFO("Pose frame_id (%s) differs from expected (%s), transforming.",p.header.frame_id.c_str(),frame.c_str());

			if (tfl_->waitForTransform(frame, p.header.frame_id, now, ros::Duration(2.0))) {

			  tfl_->transformPose(frame,p,p);

			} else {

			  p.header.stamp = ros::Time(0);
			  tfl_->transformPose(frame,p,p);
			  ROS_WARN("Using latest transform available, may be wrong.");

			}

			return true;

	   }

		// In case of absence of transformation path
		catch(tf::TransformException& ex){
		   std::cerr << "Transform error: " << ex.what() << std::endl;
		   return false;
		}



  }

 return false;

}


void CArmManipulationEditor::onPlanningSceneLoaded() {};
void CArmManipulationEditor::updateState() {};
void CArmManipulationEditor::planCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode) {

  if(errorCode.val != ArmNavigationErrorCodes::SUCCESS)
    {

    ROS_ERROR("Planning failed with error: %s (%d)",armNavigationErrorCodeToString(errorCode).c_str(), errorCode.val);


    }

};
void CArmManipulationEditor::filterCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode) {

  if(errorCode.val != ArmNavigationErrorCodes::SUCCESS)
    {

    ROS_ERROR("Filtering failed with error: %s (%d)",armNavigationErrorCodeToString(errorCode).c_str(),errorCode.val);


    }

};
void CArmManipulationEditor::attachObjectCallback(const std::string& name) {};
void CArmManipulationEditor::selectedTrajectoryCurrentPointChanged( unsigned int new_current_point ) {};


void CArmManipulationEditor::remove_coll_objects() {

  coll_obj_det.clear();

}

void CArmManipulationEditor::reset() {

  std::vector<std::string>::iterator tmp;

  /// delete all attached collision objects
  for(tmp = coll_obj_attached_id.begin(); tmp != coll_obj_attached_id.end(); tmp++) {

    if (*tmp != "") deleteCollisionObject(*tmp);

    }

  coll_obj_attached_id.clear();

  std::vector<t_det_obj>::iterator tmpo;

  /// delete all collision objects corresponding to detected objects
  for(tmpo = coll_obj_det.begin(); tmpo != coll_obj_det.end(); tmpo++) {

    if ((*tmpo).id != "") deleteCollisionObject((*tmpo).id);

    }


  GripperPosesClean();


  if (mpr_id!=9999) {

	  std::vector<unsigned int> erased_trajectories;
	  deleteMotionPlanRequest(mpr_id,erased_trajectories);

	  ROS_INFO("Motion plan request was removed from warehouse.");

	  mpr_id = 9999; // special value

  }


  inited = false;

  if (!refresh()) ROS_WARN("Error on refreshing planning scene during reset");

}

bool CArmManipulationEditor::refresh() {

  PlanningSceneData& planningSceneData = planning_scene_map_[planning_scene_id];

  ROS_INFO("Sending planning scene id=%u",planningSceneData.getId());

  return sendPlanningScene(planningSceneData);

}


void CArmManipulationEditor::spin_callback(const ros::TimerEvent&)
{

  if (inited==true) sendMarkers();


}

void CArmManipulationEditor::findIK(geometry_msgs::Pose new_pose)
{

  tf::Transform pose = toBulletTransform(new_pose);

  motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].getGoalState()->
          updateKinematicStateWithLinkAt(motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].getEndEffectorLink(),
                                         pose);

  updateState();

  PositionType type = GoalPosition;

  if(!solveIKForEndEffectorPose(motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)], type, true))
    {
      if(motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].hasGoodIKSolution(type))
      {
        motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].refreshColors();
      }
      motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].setHasGoodIKSolution(false, type);
    }
    else
    {
      if(!motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].hasGoodIKSolution(type))
      {
        motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].refreshColors();
      }
      motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)].setHasGoodIKSolution(true, type);
    }


}


// Return the rotation in Euler angles
geometry_msgs::Vector3 CArmManipulationEditor::GetAsEuler(geometry_msgs::Quaternion quat)
{
  geometry_msgs::Vector3 vec;

   double squ;
   double sqx;
   double sqy;
   double sqz;

   squ = quat.w * quat.w;
   sqx = quat.x * quat.x;
   sqy = quat.y * quat.y;
   sqz = quat.z * quat.z;

   // Roll
   vec.x = atan2(2 * (quat.y*quat.z + quat.w*quat.x), squ - sqx - sqy + sqz);

   // Pitch
   vec.y = asin(-2 * (quat.x*quat.z - quat.w * quat.y));

   // Yaw
   vec.z = atan2(2 * (quat.x*quat.y + quat.w*quat.z), squ + sqx - sqy - sqz);

   return vec;
}

void CArmManipulationEditor::GripperPosesClean() {

  boost::mutex::scoped_lock(im_server_mutex_);

  gripper_poses_->clear();

}

void CArmManipulationEditor::GripperPoses()
{

  ros::Rate r(1);

  visualization_msgs::InteractiveMarker marker;
  geometry_msgs::Pose last_pose;
  double distance;
  double max_change_in_angle;

  while(ros::ok()) {

    if (inited==true && disable_gripper_poses_==false) {

      boost::mutex::scoped_lock(im_server_mutex_);

      if (interactive_marker_server_->get("MPR 0_end_control",marker)) {

        if (gripper_poses_->empty()) {

          ROS_INFO("Storing first pose of end effector IM");
          gripper_poses_->push_back(marker.pose);

         } else {

             // check if current position differs from last stored one
             // if yes, store it

             last_pose = gripper_poses_->back();

             distance = sqrt(pow((last_pose.position.x - marker.pose.position.x),2) +
                        pow((last_pose.position.y - marker.pose.position.y),2) +
                        pow((last_pose.position.z - marker.pose.position.z),2));


             geometry_msgs::Vector3 last_angles = GetAsEuler(last_pose.orientation);
             geometry_msgs::Vector3 current_angles = GetAsEuler(marker.pose.orientation);

             std_msgs::Int32MultiArray array;
             array.data.clear();
             array.data.push_back(current_angles.x/M_PI*360);
             array.data.push_back(current_angles.y/M_PI*360);
             array.data.push_back(current_angles.z/M_PI*360);

             gripper_rpy_publisher_.publish(array);

             /*geometry_msgs::Vector3 last_angles;
             geometry_msgs::Vector3 current_angles;

             tf::Quaternion q(last_pose.orientation.x,last_pose.orientation.y,last_pose.orientation.z,last_pose.orientation.z);
             btMatrix3x3(q).getEulerRPY((btScalar)last_angles.x,(btScalar)last_angles.y,(btScalar)last_angles.z);*/


             double tmp;

             // TODO normalize angles!!!

             max_change_in_angle = fabs(current_angles.x - last_angles.x)/M_PI*360;

             tmp = fabs(current_angles.y - last_angles.y)/M_PI*360;
             if (tmp>max_change_in_angle) max_change_in_angle = tmp;

             tmp = fabs(current_angles.z - last_angles.z)/M_PI*360;
             if (tmp>max_change_in_angle) max_change_in_angle = tmp;


             if (distance>0.1 || max_change_in_angle>1) {

               //if (step_used_==false) {

               ROS_DEBUG("Distance is %f",distance);
               ROS_DEBUG("Roll: %f, pitch: %f, yaw: %f, max. change: %f ",
                                     current_angles.x/M_PI*360,
                                     current_angles.y/M_PI*360,
                                     current_angles.z/M_PI*360,
                                     max_change_in_angle);

               ROS_INFO("Storing another pose of end effector IM (size of list is: %d)",(int)gripper_poses_->size());
               gripper_poses_->push_back(marker.pose);

               //} else step_used_ = false;

             }

         }


      } else ROS_ERROR("Can't get pose of gripper IM");

      };

    r.sleep();

  } // while

  ROS_INFO("End of gripper poses thread...");

}



int main(int argc, char** argv)
{

      CArmManipulationEditor * ps_editor = NULL;
      planning_scene_utils::PlanningSceneParameters params;


      ROS_INFO("Starting");
      ros::init(argc, argv, "but_simple_manual_arm_navigation");

      /// Load parameters from server
      ros::param::param<std::string>("set_planning_scene_diff_name", params.set_planning_scene_diff_name_, SET_PLANNING_SCENE_DIFF_NAME);
      ros::param::param<std::string>("left_ik_name", params.left_ik_name_, LEFT_IK_NAME);
      ros::param::param<std::string>("left_interpolate_service_name", params.left_interpolate_service_name_, LEFT_INTERPOLATE_SERVICE_NAME);
      ros::param::param<std::string>("non_coll_left_ik_name", params.non_coll_left_ik_name_, NON_COLL_LEFT_IK_NAME);
      ros::param::param<std::string>("non_coll_right_ik_name", params.non_coll_right_ik_name_, NON_COLL_RIGHT_IK_NAME);
      ros::param::param<std::string>("planner_1_service_name", params.planner_1_service_name_, PLANNER_1_SERVICE_NAME);
      ros::param::param<std::string>("planner_2_service_name", params.planner_2_service_name_, PLANNER_2_SERVICE_NAME);
      ros::param::param<std::string>("proximity_space_planner_name", params.proximity_space_planner_name_, PROXIMITY_SPACE_PLANNER_NAME);
      ros::param::param<std::string>("proximity_space_service_name",  params.proximity_space_service_name_, PROXIMITY_SPACE_SERVICE_NAME);
      ros::param::param<std::string>("proximity_space_validity_name",  params.proximity_space_validity_name_,  PROXIMITY_SPACE_VALIDITY_NAME);
      ros::param::param<std::string>("right_ik_name", params.right_ik_name_, RIGHT_IK_NAME);
      ros::param::param<std::string>("right_interpolate_service_name", params.right_interpolate_service_name_, RIGHT_INTERPOLATE_SERVICE_NAME);
      ros::param::param<std::string>("trajectory_filter_1_service_name", params.trajectory_filter_1_service_name_, TRAJECTORY_FILTER_1_SERVICE_NAME);
      ros::param::param<std::string>("trajectory_filter_2_service_name", params.trajectory_filter_2_service_name_, TRAJECTORY_FILTER_2_SERVICE_NAME);
      ros::param::param<std::string>("vis_topic_name", params.vis_topic_name_ , VIS_TOPIC_NAME);
      ros::param::param<std::string>("right_ik_link", params.right_ik_link_ , RIGHT_IK_LINK);
      ros::param::param<std::string>("left_ik_link", params.left_ik_link_ , LEFT_IK_LINK);
      ros::param::param<std::string>("right_arm_group", params.right_arm_group_ , RIGHT_ARM_GROUP);
      ros::param::param<std::string>("left_arm_group", params.left_arm_group_ , LEFT_ARM_GROUP);
      ros::param::param<std::string>("right_redundancy", params.right_redundancy_ , RIGHT_ARM_REDUNDANCY);
      ros::param::param<std::string>("left_redundancy", params.left_redundancy_ , LEFT_ARM_REDUNDANCY);
      ros::param::param<std::string>("execute_left_trajectory", params.execute_left_trajectory_ , EXECUTE_LEFT_TRAJECTORY);
      ros::param::param<std::string>("execute_right_trajectory", params.execute_right_trajectory_ , EXECUTE_RIGHT_TRAJECTORY);
      ros::param::param<std::string>("list_controllers_service", params.list_controllers_service_, LIST_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("load_controllers_service", params.load_controllers_service_, LOAD_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("unload_controllers_service", params.unload_controllers_service_, UNLOAD_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("switch_controllers_service", params.switch_controllers_service_, SWITCH_CONTROLLERS_SERVICE);
      ros::param::param<std::string>("gazebo_robot_model", params.gazebo_model_name_, GAZEBO_ROBOT_MODEL);
      ros::param::param<std::string>("robot_description_param", params.robot_description_param_, ROBOT_DESCRIPTION_PARAM);
      ros::param::param<bool>("use_robot_data", params.use_robot_data_, true);
      params.sync_robot_state_with_gazebo_ = false;


      std::vector<std::string> links;

      /// @todo move to configuration file
      /*links.push_back("arm_7_link");
      links.push_back("sdh_palm_link");
      links.push_back("sdh_grasp_link");
      links.push_back("sdh_tip_link");

      links.push_back("sdh_finger_21_link");
      links.push_back("sdh_finger_22_link");
      links.push_back("sdh_finger_23_link");

      links.push_back("sdh_finger_11_link");
      links.push_back("sdh_finger_12_link");
      links.push_back("sdh_finger_13_link");

      links.push_back("sdh_thumb_1_link");
      links.push_back("sdh_thumb_2_link");
      links.push_back("sdh_thumb_3_link");*/


      ros::NodeHandle n;
      ros::NodeHandle nh("~");



      XmlRpc::XmlRpcValue v;


      if (nh.getParam("arm_links", v)) {

          for(int i =0; i < v.size(); i++)
          {
            links.push_back(v[i]);
            std::cerr << "link names: " << links[i] << std::endl;
          }

      } else {

    	  ROS_ERROR("Could not get param: arm_links");
    	  return 0;

      }

      ROS_INFO("Creating planning scene editor");
      ps_editor = new CArmManipulationEditor(params,links);

      ROS_INFO("Advertising services");

      ros::ServiceServer service_new = n.advertiseService(SRV_NEW, &CArmManipulationEditor::ArmNavNew,ps_editor);
      ros::ServiceServer service_plan = n.advertiseService(SRV_PLAN, &CArmManipulationEditor::ArmNavPlan,ps_editor);
      ros::ServiceServer service_play = n.advertiseService(SRV_PLAY, &CArmManipulationEditor::ArmNavPlay,ps_editor);
      ros::ServiceServer service_execute = n.advertiseService(SRV_EXECUTE, &CArmManipulationEditor::ArmNavExecute,ps_editor);
      ros::ServiceServer service_reset = n.advertiseService(SRV_RESET, &CArmManipulationEditor::ArmNavReset,ps_editor);
      ros::ServiceServer service_refresh = n.advertiseService(SRV_REFRESH, &CArmManipulationEditor::ArmNavRefresh,ps_editor);

      ros::ServiceServer service_success = n.advertiseService(SRV_SUCCESS, &CArmManipulationEditor::ArmNavSuccess,ps_editor);
      ros::ServiceServer service_failed = n.advertiseService(SRV_FAILED, &CArmManipulationEditor::ArmNavFailed,ps_editor);
      ros::ServiceServer service_repeat = n.advertiseService(SRV_REPEAT, &CArmManipulationEditor::ArmNavRepeat,ps_editor);

      ros::ServiceServer service_collobj = n.advertiseService(SRV_COLLOBJ, &CArmManipulationEditor::ArmNavCollObj,ps_editor);
      ros::ServiceServer service_movepalmlink = n.advertiseService(SRV_MOVE_PALM_LINK, &CArmManipulationEditor::ArmNavMovePalmLink,ps_editor);
      ros::ServiceServer service_movepalmlinkrel = n.advertiseService(SRV_MOVE_PALM_LINK_REL, &CArmManipulationEditor::ArmNavMovePalmLinkRel,ps_editor);

      ros::ServiceServer service_switch_aco = n.advertiseService(SRV_SWITCH, &CArmManipulationEditor::ArmNavSwitchACO,ps_editor);

      ros::ServiceServer service_step = n.advertiseService(SRV_STEP, &CArmManipulationEditor::ArmNavStep,ps_editor);
      ros::ServiceServer service_stop = n.advertiseService(SRV_STOP, &CArmManipulationEditor::ArmNavStop,ps_editor);

      ros::Timer timer1 = n.createTimer(ros::Duration(0.01), &CArmManipulationEditor::spin_callback,ps_editor);

      ManualArmManipActionServer act_server(ACT_ARM_MANIP);


      ps_editor->gripper_rpy_publisher_ = n.advertise<std_msgs::Int32MultiArray>(TOP_GRIPPER_RPY, 10);

      double tmp;

      ros::param::param<double>("~start_timeout",tmp,START_TIMEOUT);
      if (tmp>0) act_server.start_timeout_ = ros::Duration(tmp);
      else act_server.start_timeout_ = ros::Duration(START_TIMEOUT);

      ros::param::param<double>("~solve_timeout",tmp,SOLVE_TIMEOUT);
      if (tmp>0) act_server.solve_timeout_ = ros::Duration(tmp);
      else act_server.solve_timeout_ = ros::Duration(SOLVE_TIMEOUT);

      ros::param::param<double>("~inflate_bb",tmp,INFLATE_BB);
      if (tmp<1.0 || tmp>2.0) {

        tmp = 1.0;
        ROS_WARN("Param. inflate_bb should be in range <1.0;2.0>");

      }

      ps_editor->inflate_bb_ = tmp;

      ps_editor->action_server_ptr_ = &act_server;

      ps_editor->tfl_ = new tf::TransformListener();


      ROS_INFO("Spinning");

      ros::spin();


      ros::shutdown();

      delete ps_editor;

}
