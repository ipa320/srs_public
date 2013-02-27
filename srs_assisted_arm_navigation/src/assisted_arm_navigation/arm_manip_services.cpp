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

   if (data.getEndEffectorLink() != end_eff_link_) {

	   ROS_ERROR("End effector configuration mismatch. Setting end_eff_link to be %s",data.getEndEffectorLink().c_str());

	   end_eff_link_ = data.getEndEffectorLink();

   }

   planning_models::KinematicState *robot_state = getRobotState();
   planning_models::KinematicState::JointStateGroup* jsg = robot_state->getJointStateGroup(data.getGroupName());
   if ( !(robot_state->areJointsWithinBounds(jsg->getJointNames())) ) {

	   // TODO print which one !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	   //ROS_FATAL("Some joint is out of limits! Can't start planning.");

	   arm_nav_state_.out_of_limits = true;

	   std::vector<std::string> joints = jsg->getJointNames();

	   for (unsigned int i=0; i < joints.size(); i++) {

		   if ( !robot_state->isJointWithinBounds(joints[i]) ) {

			   ROS_FATAL("Joint %s is out of limits! Can't start planning.",joints[i].c_str());

		   }

	   }

	   reset();

	   res.error = "Some joint is out of limits.";
	   res.completed = false;
	   return true;

   } else arm_nav_state_.out_of_limits = false;


   ROS_INFO("Performing planning for %s end effector link.",end_eff_link_.c_str());

   data.setStartVisible(false);
   data.setEndVisible(true);
   data.setJointControlsVisible(joint_controls_,this);

   std_msgs::ColorRGBA goal_color;

   goal_color.r = 0.0;
   goal_color.g = 1.0;
   goal_color.b = 0.0;
   goal_color.a = 0.6;

   data.setGoalColor(goal_color);

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

      (*tmp).id = add_coll_obj_bb(*tmp);

      }

   res.completed = true;
   res.error = "";

   /// set current state to NEW (it's used in action callback to provide correct feedback)
   if (action_server_ptr_!=NULL)
     action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_NEW);


   GripperPosesClean();

   if (ros::service::exists("sn_teleop_srv_dis",true)) {

	   std_srvs::Empty srv;
	   ros::service::call("sn_teleop_srv_dis",srv);

   }

   inited = true;
   disable_gripper_poses_ = false;

  return true;
}

bool CArmManipulationEditor::ArmNavPlan(ArmNavPlan::Request &req, ArmNavPlan::Response &res) {

  planned_ = false;

  MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];

  visualization_msgs::MarkerArray arr = data.getCollisionMarkers();

  if (arr.markers.size() != 0) {

	  ROS_ERROR("Goal position is in collision. Can't plan.");
	  res.completed = false;
	  res.error = "Goal position is in collision.";
	  return true;

  }


  if (!data.hasGoodIKSolution(planning_scene_utils::GoalPosition)) {

	  ROS_ERROR("There is no IK solution for goal position. Can't plan.");
	  res.completed = false;
	  res.error = "Goal position is not reachable.";
	  return true;

  }


  ROS_DEBUG("Planning trajectory...");

  //MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];

  if (planToRequest(data,traj_id)) {

	TrajectoryData& trajectory = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(traj_id)];

	if (trajectory.getTrajectorySize() < 3) {

		ROS_ERROR("Strange trajectory with only %d points.",(int)trajectory.getTrajectorySize());
		res.completed = false;
	    res.error = "Planning failed.";
	    return true;

	}

    ROS_INFO("Trajectory successfully planned");

    unsigned int filterID;


    if (filterTrajectory(data,trajectory,filterID)) {

      ROS_INFO("Successfully filtered, playing started");

      filt_traj_id = filterID;

      TrajectoryData& f_trajectory = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(filt_traj_id)];
      playTrajectory(data,f_trajectory);

      res.completed = true;

      //boost::mutex::scoped_lock(im_server_mutex_);
      lockScene();

      planned_ = true;
      disable_gripper_poses_ = true;

      //data.setGoalEditable(false);
      //data.hideGoal();

      if (!interactive_marker_server_->erase("MPR 0_end_control")) {

        ROS_WARN("Cannot remove gripper IM.");

      } else interactive_marker_server_->applyChanges();

      unlockScene();

    } else {


      res.completed = false;
      res.error = "Error on filtering";
      return true;


    }


  } else {


    res.completed = false;
    res.error = "Error on planning";
    return false;

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
    reset();
    return false;

  }

  ROS_INFO("Executing trajectory...");

  executeTrajectory(getMotionPlanRequestNameFromId(mpr_id),getTrajectoryNameFromId(filt_traj_id));
  
  ROS_INFO("Trajectory was sent...");

  reset();
  
  ROS_INFO("Reset of stuff after executing trajectory.");

  ros::Rate r(10);

  // TODO put some timeout there !!!!
  while (monitor_status_ == Executing) r.sleep();

  ROS_INFO("Trajectory should be executed.");

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

bool CArmManipulationEditor::ArmNavRemoveCollObjects(ArmNavRemoveCollObjects::Request &req, ArmNavRemoveCollObjects::Response &res) {

	std::vector<CollisionObject> objects;

	for (unsigned int i=0; i < coll_obj_det.size(); i++) {

		CollisionObject tmp;

		tmp.allow_collision = coll_obj_det[i].allow_collision;
		tmp.allow_pregrasps = coll_obj_det[i].allow_pregrasps;
		tmp.attached = coll_obj_det[i].attached;
		tmp.bb_lwh = coll_obj_det[i].bb_lwh;
		tmp.id = coll_obj_det[i].id;
		tmp.name = coll_obj_det[i].name;
		tmp.pose = coll_obj_det[i].pose;

		objects.push_back(tmp);

	}

	res.objects = objects;

	remove_coll_objects();
	return true;

}

bool CArmManipulationEditor::ArmNavSuccess(ArmNavSuccess::Request &req, ArmNavSuccess::Response &res) {

  reset();

  //remove_coll_objects();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_SUCCESS);

  return true;
}

bool CArmManipulationEditor::ArmNavFailed(ArmNavFailed::Request &req, ArmNavFailed::Response &res) {

  reset();

  //remove_coll_objects();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_set_state(ManualArmManipActionServer::S_FAILED);

  return true;
}

bool CArmManipulationEditor::ArmNavRepeat(ArmNavRepeat::Request &req, ArmNavRepeat::Response &res) {

  reset();

  //remove_coll_objects();

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

bool CArmManipulationEditor::ArmNavSetAttached(ArmNavSetAttached::Request &req, ArmNavSetAttached::Response &res) {


	unsigned int idx = 0;
	bool f = false;

	for (unsigned int i=0; i < coll_obj_det.size(); i++) {

		if (coll_obj_det[i].name == req.object_name) {

			idx = i;
			f = true;
			break;
		}


	}

	if (!f) {

		ROS_ERROR("Collision object %s does not exist.",req.object_name.c_str());
		res.completed = false;
		return true;

	} else {

		if (req.attached) ROS_INFO("Setting %s object to be attached.",req.object_name.c_str());
		else ROS_INFO("Setting %s object to NOT be attached.",req.object_name.c_str());

		//MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];
		std::string target = end_eff_link_;

		if (req.attached) {

			// attaching -> we are going to transform object to end effector link

			coll_obj_det[idx].pose.header.stamp = ros::Time::now();

			if (!transf(target,coll_obj_det[idx].pose)) {

				ROS_ERROR("Failed to attach collision object (TF error).");
				res.completed = false;
				return true;

			}
			
			// once attached, we will remove pregrasps
			coll_obj_det[idx].allow_pregrasps = false;

		} else {

			coll_obj_det[idx].pose.header.stamp = ros::Time::now();

			// dis-attaching -> transform it back to world frame id
			if (coll_obj_det[idx].pose.header.frame_id != world_frame_) {

				if (!transf(world_frame_,coll_obj_det[idx].pose)) {

					ROS_ERROR("Failed to dis-attach collision object (TF error).");
					res.completed = false;
					return true;

				}


			}

		}

		coll_obj_det[idx].attached = req.attached;

		res.completed = true;
		return true;

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
  obj.attached = false;

  obj.allow_collision = req.allow_collision;
  obj.allow_pregrasps = req.allow_pregrasps;

  coll_obj_det.push_back(obj);

  return true;

}

bool CArmManipulationEditor::ArmNavMovePalmLink(ArmNavMovePalmLink::Request &req, ArmNavMovePalmLink::Response &res) {


  if (inited) {

	geometry_msgs::PoseStamped ps(req.sdh_palm_link_pose);

	//if (ps.header.frame_id != collision_objects_frame_id_) ROS_INFO("Setting position of end eff. to x: %f, y: %f, z: %f (in %s frame)",ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,collision_objects_frame_id_.c_str());

	if (!checkPose(ps,collision_objects_frame_id_)) return false;

	ROS_INFO("Setting position of end eff. to x: %f, y: %f, z: %f (in %s frame)",ps.pose.position.x,ps.pose.position.y,ps.pose.position.z,ps.header.frame_id.c_str());

    //boost::mutex::scoped_lock(im_server_mutex_);


    /*std_msgs::Header h;

    h.frame_id = "map";
    h.stamp = ros::Time::now();*/

    lockScene();

    if (interactive_marker_server_->setPose("MPR 0_end_control",ps.pose/*,h*/)) {

      interactive_marker_server_->applyChanges();

      unlockScene();

      if (!findIK(ps.pose)) {

    	  ros::Duration(0.25).sleep();

    	  // try it for second time... just for sure...
    	  findIK(ps.pose);

      }

      res.completed = true;
      return true;

    } else unlockScene();


    res.completed = false;
    return true;

  } else {

    std::string str = SRV_NEW;

    ROS_ERROR("Cannot move arm to specified position. Call %s service first.",str.c_str());
    res.completed = false;
    reset();
    return false;

  }

}

bool CArmManipulationEditor::ArmNavMovePalmLinkRel(ArmNavMovePalmLinkRel::Request &req, ArmNavMovePalmLinkRel::Response &res) {

  //ROS_INFO("Lets try to move arm IMs little bit... :) Relatively...");

  if (!inited) {

    std::string str = SRV_NEW;

    ROS_ERROR("Cannot move arm to specified position. Call %s service first.",str.c_str());
    res.completed = false;
    reset();
    return false;

  }

  //boost::mutex::scoped_lock(im_server_mutex_);

  lockScene();

  visualization_msgs::InteractiveMarker marker;
  geometry_msgs::Pose new_pose;

  if (!(interactive_marker_server_->get("MPR 0_end_control",marker))) {

	unlockScene();

    ROS_ERROR("Can´t get gripper IM pose.");
    res.completed = false;
    reset();
    return false;
  }

  new_pose = marker.pose;

  //new_pose.position = req.relative_movement.position;
  new_pose.position.x += req.relative_movement.position.x;
  new_pose.position.y += req.relative_movement.position.y;
  new_pose.position.z += req.relative_movement.position.z;

  tf::Quaternion o;
  tf::quaternionMsgToTF(req.relative_movement.orientation,o);

  tf::Quaternion g;
  tf::quaternionMsgToTF(new_pose.orientation,g);

  tf::quaternionTFToMsg(o*g,new_pose.orientation);



  if (!(interactive_marker_server_->setPose("MPR 0_end_control",new_pose))) {

	ROS_ERROR("Can't set new pose for MPR 0_end_control");

	unlockScene();

    res.completed = false;
    reset();
    return false;

  }

  interactive_marker_server_->applyChanges();

  unlockScene();

  if (!findIK(new_pose)) {

	  ros::Duration(0.25).sleep();

	  // try it for second time... just for sure
	  findIK(new_pose);

  }

  res.completed = true;
  return true;

}

bool CArmManipulationEditor::ArmNavStep(ArmNavStep::Request &req, ArmNavStep::Response &res) {

  if (!inited) {

    res.completed = false;
    res.msg = "Start planning first!";
    reset();
    return true;

  }

    if (req.undo) {

      //boost::mutex::scoped_lock(im_server_mutex_);
      lockScene();

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

    	 unlockScene();

         ROS_ERROR("Error on changing IM pose");
         res.msg = "Error on changing IM pose";
         res.completed = false;
         reset();
         return false;

     }

     interactive_marker_server_->applyChanges();

     unlockScene();

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

  if (inited == true) {

	  ROS_ERROR("ACO can bee switched on/off only before start of planning.");
	  return false;

  }

  aco_ = req.state;

  if (aco_ ) ROS_INFO("ACO set to true");
  else ROS_INFO("ACO set to false");

  res.completed = true;
  return true;

}

bool CArmManipulationEditor::ArmNavStop(ArmNavStop::Request &req, ArmNavStop::Response &res) {

  ROS_INFO("Canceling all goals.");
  this->arm_controller_map_[params_.left_arm_group_]->cancelAllGoals();

  reset();

  return true;

}
