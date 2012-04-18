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

#include "but_arm_manipulation_node.h"


using namespace planning_scene_utils;



bool CArmManipulationEditor::ArmNavNew(srs_assisted_arm_navigation::ArmNavNew::Request &req, srs_assisted_arm_navigation::ArmNavNew::Response &res) {


    planning_scene_id = createNewPlanningScene();

    setCurrentPlanningScene(planning_scene_id,true,true);

    ROS_DEBUG("Created a new planning scene: %s", planning_scene_id.c_str());

    createMotionPlanRequest(*getRobotState(), // start state
                           *getRobotState(), // end state
                           params_.left_arm_group_, // group_name
                           params_.left_ik_link_, // end effector name
                           planning_scene_utils::getPlanningSceneIdFromName(planning_scene_id), // planning_scene_name
                           true, // start from robot state
                           mpr_id); // motion plan id out


   ROS_DEBUG("Created a new MPR: %d", mpr_id);

   inited = true;

   MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];

   data.setStartVisible(false);

   #define TR 0.1

   coll_obj_attached_id.push_back(add_coll_obj_attached(0,0,0.25,0.2,0.15));

   /// @bug why it's not possible to add more objects?? -> Size mismatch between poses size 2 and shapes size 1
   /*coll_obj_id.push_back(add_coll_obj(TR,TR,0.15,0.05,0.25));
   coll_obj_id.push_back(add_coll_obj(TR,-TR,0.15,0.05,0.25));
   coll_obj_id.push_back(add_coll_obj(-TR,TR,0.2));
   coll_obj_id.push_back(add_coll_obj(-TR,-TR,0.2));*/

   /// add all detected objects
   std::vector<t_det_obj>::iterator tmp;

    for(tmp = coll_obj_det.begin(); tmp != coll_obj_det.end(); tmp++) {

      (*tmp).id = add_coll_obj_bb((*tmp).name,(*tmp).pose,(*tmp).bb_lwh);

      }

   res.completed = true;
   res.error = "";

   /// set current state to NEW (it's used in action callback to provide correct feedback)
   if (action_server_ptr_!=NULL)
     action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_NEW);



  return true;
}

bool CArmManipulationEditor::ArmNavPlan(srs_assisted_arm_navigation::ArmNavPlan::Request &req, srs_assisted_arm_navigation::ArmNavPlan::Response &res) {

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

bool CArmManipulationEditor::ArmNavPlay(srs_assisted_arm_navigation::ArmNavPlay::Request &req, srs_assisted_arm_navigation::ArmNavPlay::Response &res) {

  ROS_INFO("Playing trajectory...");

  MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];
  TrajectoryData& f_trajectory = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(filt_traj_id)];

  playTrajectory(data,f_trajectory);

  res.completed = true;

  return true;
}

bool CArmManipulationEditor::ArmNavExecute(srs_assisted_arm_navigation::ArmNavExecute::Request &req, srs_assisted_arm_navigation::ArmNavExecute::Response &res) {

  ROS_INFO("Executing trajectory...");

  executeTrajectory(getMotionPlanRequestNameFromId(mpr_id),getTrajectoryNameFromId(filt_traj_id));

  reset();

  res.completed = true;

  /// @todo Wait for stop of arm movement (optionally, should be configurable by param).

  if (action_server_ptr_!=NULL)
    action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_EXECUTE);

  return true;
}

bool CArmManipulationEditor::ArmNavReset(srs_assisted_arm_navigation::ArmNavReset::Request &req, srs_assisted_arm_navigation::ArmNavReset::Response &res) {

  reset();

  res.completed = true;

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_RESET);

  return true;
}

bool CArmManipulationEditor::ArmNavSuccess(srs_assisted_arm_navigation::ArmNavSuccess::Request &req, srs_assisted_arm_navigation::ArmNavSuccess::Response &res) {

  reset();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_SUCCESS);

  return true;
}

bool CArmManipulationEditor::ArmNavFailed(srs_assisted_arm_navigation::ArmNavFailed::Request &req, srs_assisted_arm_navigation::ArmNavFailed::Response &res) {

  reset();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_FAILED);

  return true;
}

bool CArmManipulationEditor::ArmNavRefresh(srs_assisted_arm_navigation::ArmNavRefresh::Request &req, srs_assisted_arm_navigation::ArmNavRefresh::Response &res) {

  PlanningSceneData& planningSceneData = planning_scene_map_[planning_scene_id];

  if (sendPlanningScene(planningSceneData)) {

    res.completed = true;
    ROS_INFO("Refreshing planning scene");
    return true;

  } else {

    res.completed = false;
    ROS_ERROR("Error on refreshing planning scene");
    return false;

  }

}

bool CArmManipulationEditor::ArmNavCollObj(srs_assisted_arm_navigation::ArmNavCollObj::Request &req, srs_assisted_arm_navigation::ArmNavCollObj::Response &res) {

  ROS_INFO("Trying to add collision object name: %s",req.object_name.c_str());

  if (req.pose.header.frame_id=="/base_link") {

    ROS_INFO("Ok, object pose is in /base_link coord. system. Lets store it.");

    t_det_obj obj;

    obj.name = req.object_name;
    obj.bb_lwh = req.bb_lwh;
    obj.pose = req.pose;

    coll_obj_det.push_back(obj);

  } else {

    /// @todo Implement transformation of collision object if it is not in base_link coord. system.
    ROS_WARN("We have to do transform - NOT implemented yet.");

  }

  return true;

}
