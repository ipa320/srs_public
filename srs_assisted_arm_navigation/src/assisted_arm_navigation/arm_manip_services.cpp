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

using namespace planning_scene_utils;
using namespace srs_assisted_arm_navigation;
using namespace srs_assisted_arm_navigation_msgs;


bool CArmManipulationEditor::ArmNavNew(ArmNavNew::Request &req, ArmNavNew::Response &res) {


	if (planning_scene_id == "") {

		planning_scene_id = createNewPlanningScene();

		setCurrentPlanningScene(planning_scene_id,true,true);

		ROS_INFO("Created a new planning scene: %s", planning_scene_id.c_str());

	}

    createMotionPlanRequest(*getRobotState(), // start state
                           *getRobotState(), // end state
                           params_.left_arm_group_, // group_name
                           params_.left_ik_link_, // end effector name
                           planning_scene_utils::getPlanningSceneIdFromName(planning_scene_id), // planning_scene_name
                           true, // start from robot state
                           mpr_id); // motion plan id out


   ROS_INFO("Created a new MPR: %d", mpr_id);

   MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];

   ROS_INFO("Performing planning for %s end effector link.",data.getEndEffectorLink().c_str());

   data.setStartVisible(false);
   data.setJointControlsVisible(joint_controls_,this);

   #define TR 0.1

   if (aco_) {

     ROS_INFO("Adding attached collision object to planning scene");
     coll_obj_attached_id.push_back(add_coll_obj_attached(0,0,0.25,0.2,0.15));

   }

   /// @bug why it's not possible to add more objects?? -> Size mismatch between poses size 2 and shapes size 1
   /*coll_obj_id.push_back(add_coll_obj(TR,TR,0.15,0.05,0.25));
   coll_obj_id.push_back(add_coll_obj(TR,-TR,0.15,0.05,0.25));
   coll_obj_id.push_back(add_coll_obj(-TR,TR,0.2));
   coll_obj_id.push_back(add_coll_obj(-TR,-TR,0.2));*/

   /// add all detected objects
   std::vector<t_det_obj>::iterator tmp;

    for(tmp = coll_obj_det.begin(); tmp != coll_obj_det.end(); tmp++) {

      (*tmp).id = add_coll_obj_bb((*tmp).name,(*tmp).pose,(*tmp).bb_lwh, (*tmp).allow_collision);

      }

   res.completed = true;
   res.error = "";

   /// set current state to NEW (it's used in action callback to provide correct feedback)
   if (action_server_ptr_!=NULL)
     action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_NEW);


   GripperPosesClean();

   inited = true;
   disable_gripper_poses_ = false;

  return true;
}

bool CArmManipulationEditor::ArmNavPlan(ArmNavPlan::Request &req, ArmNavPlan::Response &res) {

  planned_ = false;

  ROS_DEBUG("Planning trajectory...");

  MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];

  if (planToRequest(data,traj_id)) {

    ROS_INFO("Trajectory successfully planned");

    TrajectoryData& trajectory = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(traj_id)];

    unsigned int filterID;


    if (filterTrajectory(data,trajectory,filterID)) {

      ROS_INFO("Successfully filtered, playing started");

      filt_traj_id = filterID;

      TrajectoryData& f_trajectory = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(filt_traj_id)];
      playTrajectory(data,f_trajectory);

      res.completed = true;

      boost::mutex::scoped_lock(im_server_mutex_);

      planned_ = true;
      disable_gripper_poses_ = true;

      //data.setGoalEditable(false);
      //data.hideGoal();

      if (!interactive_marker_server_->erase("MPR 0_end_control")) {

        ROS_WARN("Cannot remove IM.");

      } else interactive_marker_server_->applyChanges();


    } else {


      res.completed = false;
      res.error = "Error on filtering";


    }


  } else {


    res.completed = false;
    res.error = "Error on planning";

  }

  if (action_server_ptr_!=NULL)
     action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_PLAN);

  return true;
}

bool CArmManipulationEditor::ArmNavPlay(ArmNavPlay::Request &req, ArmNavPlay::Response &res) {

  if (!planned_) {

    ROS_ERROR("Nothing to play!");
    res.completed = false;
    return false;

  }

  ROS_INFO("Playing trajectory...");

  MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];
  TrajectoryData& f_trajectory = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(filt_traj_id)];

  playTrajectory(data,f_trajectory);

  res.completed = true;

  return true;
}

bool CArmManipulationEditor::ArmNavExecute(ArmNavExecute::Request &req, ArmNavExecute::Response &res) {

  if (!planned_) {

    ROS_ERROR("Nothing to execute!");
    res.completed = false;
    return false;

  }

  ROS_INFO("Executing trajectory...");

  executeTrajectory(getMotionPlanRequestNameFromId(mpr_id),getTrajectoryNameFromId(filt_traj_id));

  reset();

  res.completed = true;

  /// @todo Wait for stop of arm movement (optionally, should be configurable by param).

  if (action_server_ptr_!=NULL)
    action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_EXECUTE);

  return true;
}

bool CArmManipulationEditor::ArmNavReset(ArmNavReset::Request &req, ArmNavReset::Response &res) {

  reset();

  res.completed = true;

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_RESET);

  return true;
}

bool CArmManipulationEditor::ArmNavSuccess(ArmNavSuccess::Request &req, ArmNavSuccess::Response &res) {

  reset();

  remove_coll_objects();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_SUCCESS);

  return true;
}

bool CArmManipulationEditor::ArmNavFailed(ArmNavFailed::Request &req, ArmNavFailed::Response &res) {

  reset();

  remove_coll_objects();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_FAILED);

  return true;
}

bool CArmManipulationEditor::ArmNavRepeat(ArmNavRepeat::Request &req, ArmNavRepeat::Response &res) {

  reset();

  remove_coll_objects();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_REPEAT);

  return true;
}

bool CArmManipulationEditor::ArmNavRefresh(ArmNavRefresh::Request &req, ArmNavRefresh::Response &res) {

  if (refresh()) {

    res.completed = true;
    ROS_INFO("Refreshing planning scene");
    return true;

  } else {

    res.completed = false;
    ROS_ERROR("Error on refreshing planning scene");
    return false;

  }

}



/**
 * @todo Store object coords. in map frame. Then on each refresh add it transformed into base_link
 * @param req
 * @param res
 * @return
 */
bool CArmManipulationEditor::ArmNavCollObj(ArmNavCollObj::Request &req, ArmNavCollObj::Response &res) {

  ROS_INFO("Trying to add collision object name: %s",req.object_name.c_str());

  geometry_msgs::PoseStamped opose = req.pose;

  if (checkPose(opose,"/map")) {

	  ROS_INFO("Ok, object pose is in /map coord. system. Lets store it.");


  } else {

	ROS_INFO("Object is not in map frame, lets transform it first.");

	
	if (!transf("/map",opose)) {

		ROS_ERROR("Error on transforming collision object.");
		return false;

	};

}

  t_det_obj obj;

  obj.name = req.object_name;
  obj.bb_lwh = req.bb_lwh;
  obj.pose = opose;

  obj.allow_collision = req.allow_collision;

  coll_obj_det.push_back(obj);

  return true;

}

bool CArmManipulationEditor::ArmNavMovePalmLink(ArmNavMovePalmLink::Request &req, ArmNavMovePalmLink::Response &res) {


  if (inited) {

	geometry_msgs::PoseStamped ps(req.sdh_palm_link_pose);

	//if (ps.header.frame_id != collision_objects_frame_id_) ROS_INFO("Setting position of end eff. to x: %f, y: %f, z: %f (in %s frame)",ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,collision_objects_frame_id_.c_str());

	if (!checkPose(ps,collision_objects_frame_id_)) return false;

	ROS_INFO("Setting position of end eff. to x: %f, y: %f, z: %f (in %s frame)",ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,ps.header.frame_id.c_str());

    boost::mutex::scoped_lock(im_server_mutex_);

    std_msgs::Header h;

    h.frame_id = "map";
    h.stamp = ros::Time::now();

    if (interactive_marker_server_->setPose("MPR 0_end_control",ps.pose,h)) {

      interactive_marker_server_->applyChanges();

      findIK(ps.pose);

      res.completed = true;
      return true;

    }

    res.completed = false;
    return true;

  } else {

    std::string str = SRV_NEW;

    ROS_ERROR("Cannot move arm to specified position. Call %s service first.",str.c_str());
    res.completed = false;
    return false;

  }

}

bool CArmManipulationEditor::ArmNavMovePalmLinkRel(ArmNavMovePalmLinkRel::Request &req, ArmNavMovePalmLinkRel::Response &res) {

  //ROS_INFO("Lets try to move arm IMs little bit... :) Relatively...");

  if (!inited) {

    std::string str = SRV_NEW;

    ROS_ERROR("Cannot move arm to specified position. Call %s service first.",str.c_str());
    res.completed = false;
    return false;

  }

  boost::mutex::scoped_lock(im_server_mutex_);

  visualization_msgs::InteractiveMarker marker;
  geometry_msgs::Pose new_pose;

  if (!(interactive_marker_server_->get("MPR 0_end_control",marker))) {

    ROS_ERROR("Can´t get gripper IM pose.");
    res.completed = false;
    return true;
  }

  new_pose = marker.pose;

  new_pose.position.x += req.relative_shift.x;
  new_pose.position.y += req.relative_shift.y;
  new_pose.position.z += req.relative_shift.z;


  if (!(interactive_marker_server_->setPose("MPR 0_end_control",new_pose))) {

    res.completed = false;
    return true;

  }

  interactive_marker_server_->applyChanges();

  findIK(new_pose);

  res.completed = true;
  return true;

}

bool CArmManipulationEditor::ArmNavStep(ArmNavStep::Request &req, ArmNavStep::Response &res) {

  if (!inited) {

    res.completed = false;
    res.msg = "Start planning first!";
    return true;

  }

    if (req.undo) {

      boost::mutex::scoped_lock(im_server_mutex_);

      if (gripper_poses_->size()<=1) {

        res.completed = false;
        res.msg = "There is no stored position.";
        res.b_steps_left = 0;
        res.f_steps_left = 0;
        return true;

      }

      gripper_poses_->pop_back(); // discard current position
      geometry_msgs::Pose p = gripper_poses_->back();

      res.b_steps_left = gripper_poses_->size()-1;

      ROS_INFO("Undoing change in gripper IM (%d steps left)",res.b_steps_left);

      if (!(interactive_marker_server_->setPose("MPR 0_end_control",p))) {

         ROS_ERROR("Error on changing IM pose");
         res.msg = "Error on changing IM pose";
         res.completed = false;
         return true;

     }

     interactive_marker_server_->applyChanges();

     findIK(p);

     res.msg = "Undoing last change";
     res.completed = true;
     return true;

    } // undo

    if (req.redo) {

      // TODO: implement
      res.completed = false;
      res.msg = "Not implemented yet";

    }

  return false;

}

bool CArmManipulationEditor::ArmNavSwitchACO(ArmNavSwitchAttCO::Request &req, ArmNavSwitchAttCO::Response &res) {

  // TODO allow the change only if inited=false

  aco_ = req.state;

  if (aco_ ) ROS_INFO("ACO set to true");
  else ROS_INFO("ACO set to false");

  res.completed = true;
  return true;

}

bool CArmManipulationEditor::ArmNavStop(ArmNavStop::Request &req, ArmNavStop::Response &res) {

  ROS_INFO("Canceling all goals.");
  this->arm_controller_map_[params_.left_arm_group_]->cancelAllGoals();

  return true;

}
