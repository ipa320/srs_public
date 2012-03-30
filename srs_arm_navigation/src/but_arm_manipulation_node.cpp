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


using namespace collision_space;
using namespace kinematics_msgs;
using namespace arm_navigation_msgs;
using namespace head_monitor_msgs;
using namespace move_arm_warehouse;
using namespace planning_environment;
using namespace planning_models;
using namespace std;
using namespace trajectory_msgs;
using namespace planning_scene_utils;
using namespace ros::param;

// TODO: pridani znameho (detekovaneho) objektu -> asi budu muset z GenericState volat service, který to tady provede, mesh ulozit do docasneho souboru...
//  createMeshObject / createCollisionObject



// TODO: controllerDoneCallback / armHasStoppedMoving -> zkusit použít tohle - sledování stavu provádění trajektorie



planning_scene_utils::PlanningSceneParameters params;

CArmManipulationEditor::CArmManipulationEditor(planning_scene_utils::PlanningSceneParameters& params) : PlanningSceneEditor(params)
{

    action_server_ptr_ = NULL;
    inited = false;

}

CArmManipulationEditor::~CArmManipulationEditor() {

  if (action_server_ptr_!=NULL) delete action_server_ptr_;

  if (tfl_!=NULL) delete tfl_;

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

bool CArmManipulationEditor::ArmNavNew(srs_ui_but::ArmNavNew::Request &req, srs_ui_but::ArmNavNew::Response &res) {


    planning_scene_id = createNewPlanningScene();

    setCurrentPlanningScene(planning_scene_id,true,true);

    ROS_DEBUG("Created a new planning scene: %s", planning_scene_id.c_str());

   createMotionPlanRequest(*getRobotState(), // start state
                           *getRobotState(), // end state
                           params.left_arm_group_, // group_name
                           params.left_ik_link_, // end effector name
                           getPlanningSceneIdFromName(planning_scene_id), // planning_scene_name
                           true, // start from robot state
                           mpr_id); // motion plan id out


   ROS_DEBUG("Created a new MPR: %d", mpr_id);

   inited = true;

   MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];

   data.setStartVisible(false);

   res.completed = true;
   res.error = "";

   if (action_server_ptr_!=NULL)
     action_server_ptr_->srv_new();

   ros::Time now;
   now = ros::Time::now();

   std::string target = "/base_link";

   // TODO: createCollisionObject + attachCollisionObject -> vytvorit utvar pripojeny k ruce (gripper)
   geometry_msgs::PoseStamped pose;
   pose.header.frame_id = "/arm_7_link";
   //pose.header.stamp = ros::Time(ros::WallTime::now().toSec());
   pose.header.stamp = now;
   //pose.header.stamp = ros::Time(0);
   pose.pose.position.x = 0;
   pose.pose.position.y = 0;
   pose.pose.position.z = 0.2;
   pose.pose.orientation.x = 0;
   pose.pose.orientation.y = 0;
   pose.pose.orientation.z = 0;
   pose.pose.orientation.w = 1;

   /*if (ros::Time::isSimTime()) {

     ROS_INFO("Sim time detected. We will sleep for some time (to solve tf issue).");

     ros::Duration sl;
     sl.sec = 2;
     sl.sleep();

   }*/

   bool transf = false;

   //ros::spinOnce(); // just experiment...

   ROS_INFO("Waiting for transformation between %s and %s",pose.header.frame_id.c_str(),target.c_str());
   try {

         if (tfl_->waitForTransform(target, pose.header.frame_id, now, ros::Duration(2.0))) {

           tfl_->transformPose(target,pose,pose);

         } else {

           pose.header.stamp = ros::Time(0);
           tfl_->transformPose(target,pose,pose);
           ROS_WARN("Using latest transform available, may be wrong.");

         }

         transf = true;

    }

       // In case of absence of transformation path
       catch(tf::TransformException& ex){
           std::cerr << "Transform error: " << ex.what() << std::endl;
           transf = false;
       }


   if (transf) {

     ROS_INFO("Transformation succeeded");

     std::string name = "gripper";

     PlanningSceneEditor::GeneratedShape shape;

     shape = PlanningSceneEditor::Cylinder;

     std_msgs::ColorRGBA color;

     color.r = 1.0;
     color.g = 0;
     color.b = 0;
     color.a = 0.25;

     coll_obj_id = "";
     coll_obj_id = createCollisionObject(name,
                           pose.pose,
                           shape,
                           0.2,
                           0.2,
                           0.3,
                           color);

     if (coll_obj_id!="") {

       std::vector<std::string> links;

       // TODO move to configuration file
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
       links.push_back("sdh_thumb_3_link");

       attachCollisionObject(coll_obj_id,
                             "arm_7_link",
                             links);

       //changeToAttached(name);


     } else {

       ROS_ERROR("Gripper collision object was not created successfully");

     }

   } // if transf

  return true;
}

bool CArmManipulationEditor::ArmNavPlan(srs_ui_but::ArmNavPlan::Request &req, srs_ui_but::ArmNavPlan::Response &res) {

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
     action_server_ptr_->srv_plan();

  return true;
}

bool CArmManipulationEditor::ArmNavPlay(srs_ui_but::ArmNavPlay::Request &req, srs_ui_but::ArmNavPlay::Response &res) {

  ROS_INFO("Playing trajectory...");

  MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];
  TrajectoryData& f_trajectory = trajectory_map_[getMotionPlanRequestNameFromId(mpr_id)][getTrajectoryNameFromId(filt_traj_id)];

  playTrajectory(data,f_trajectory);

  res.completed = true;

  return true;
}

bool CArmManipulationEditor::ArmNavExecute(srs_ui_but::ArmNavExecute::Request &req, srs_ui_but::ArmNavExecute::Response &res) {

  ROS_INFO("Executing trajectory...");

  executeTrajectory(getMotionPlanRequestNameFromId(mpr_id),getTrajectoryNameFromId(filt_traj_id));

  reset();

  deleteCollisionObject(coll_obj_id);

  res.completed = true;

  if (action_server_ptr_!=NULL)
    action_server_ptr_->srv_execute();

  return true;
}

bool CArmManipulationEditor::ArmNavReset(srs_ui_but::ArmNavReset::Request &req, srs_ui_but::ArmNavReset::Response &res) {


  reset();


  //std::string name = "gripper";
  deleteCollisionObject(coll_obj_id);

  res.completed = true;

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_reset();

  return true;
}

bool CArmManipulationEditor::ArmNavSuccess(srs_ui_but::ArmNavSuccess::Request &req, srs_ui_but::ArmNavSuccess::Response &res) {

  reset();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_success();

  return true;
}

bool CArmManipulationEditor::ArmNavFailed(srs_ui_but::ArmNavFailed::Request &req, srs_ui_but::ArmNavFailed::Response &res) {

  reset();

  if (action_server_ptr_!=NULL)
      action_server_ptr_->srv_failed();

  return true;
}


void CArmManipulationEditor::reset() {


  lockScene();
  std::vector<unsigned int> erased_trajectories;

  deleteMotionPlanRequest(mpr_id,erased_trajectories);

  unlockScene();


}



void CArmManipulationEditor::spin_callback(const ros::TimerEvent&)
{

  if (inited==true) sendMarkers();


}


void ManualArmManipActionServer::executeCB(const srs_arm_navigation::ManualArmManipGoalConstPtr &goal) {

  ros::Rate r(1);
  ros::Rate r5(5);
  srs_ui_but::ArmNavStart srv_start;

  result_.result.collision = false;
  result_.result.success = false;
  result_.result.timeout = false;
  result_.result.time_elapsed = ros::Duration(0);

  ROS_INFO("Executing ManualArmManipAction");

  start_time_ = ros::Time::now();
  timeout_ = ros::Time::now();
  state_ = S_NONE;

  while(!ros::service::exists(SRV_START,true)) {

    ROS_INFO("Waiting for arm_nav_start service");
    r5.sleep();

  };

  // move arm to pre-grasp position or away
  if (((goal->pregrasp == true) ^ (goal->away == true)) == 1) {

    if (goal->pregrasp) {

    srv_start.request.pregrasp = true;
    srv_start.request.object_name = goal->object_name;

    } else {

      srv_start.request.away = true;

    }

    ros::service::call(SRV_START,srv_start);

    inited_ = true;

    ROS_INFO("Actionserver: starting looping");

    // 1 Hz loop
    while(ros::ok()) {

      state_ = new_state_;

      // test for sleeping user
      if (state_==S_NONE && ((ros::Time::now()-start_time_) > start_timeout_)) {

        ROS_ERROR("%s: Canceled. Start timeout reached.", action_name_.c_str());

        result_.result.timeout = true;
        result_.result.time_elapsed = ros::Time::now() - start_time_;
        server_.setAborted(result_.result,"User is probably sleeping.");
        break;

      }

      // another test for sleeping user
      if (state_!=S_NONE && ((ros::Time::now()-timeout_) > solve_timeout_)) {

         ROS_ERROR("%s: Canceled. Solve timeout reached.", action_name_.c_str());

         result_.result.timeout = true;
         result_.result.time_elapsed = ros::Time::now() - start_time_;
         server_.setAborted(result_.result,"User is probably sleeping.");
         break;

      }

      // cancel action...
      if (server_.isPreemptRequested() || !ros::ok()) {

        ROS_INFO("%s: Preempted", action_name_.c_str());

        // TODO: clean things...

        result_.result.time_elapsed = ros::Time::now() - start_time_;
        server_.setPreempted(result_.result);

        break;

      }


      if ((state_ != S_SUCCESS) && (state_ != S_FAILED)) {

        // send feedback
        setFeedbackFalse();

        if (state_ == S_NEW) feedback_.feedback.starting = true;
        if (state_ == S_PLAN) feedback_.feedback.planning = true;
        if (state_ == S_EXECUTE) feedback_.feedback.executing = true;
        if (state_ == S_RESET) feedback_.feedback.reset = true;

        server_.publishFeedback(feedback_.feedback);

      } else {

        // send result
        result_.result.time_elapsed = ros::Time::now() - start_time_;

        if (state_ == S_SUCCESS) {

          result_.result.success = true;
          server_.setSucceeded(result_.result);

        }

        if (state_ == S_FAILED) {

          result_.result.failed = true;
          server_.setAborted(result_.result);

        }


        break;

      }

      r.sleep();

    } // loop

    inited_ = false;
    new_state_ = S_NONE;

  // undefined
  } else {

    ROS_INFO("Can't move both to pregrasp pos. and away!");
     server_.setAborted(result_.result,"Can't move both to pregrasp pos. and away!");

  }

}

void ManualArmManipActionServer::srv_new() {

  if (inited_) {
    new_state_ = S_NEW;
    timeout_ = ros::Time::now();
  }

}

void ManualArmManipActionServer::srv_plan() {

  if (inited_) {
    new_state_ = S_PLAN;
    timeout_ = ros::Time::now();
  }

}

void ManualArmManipActionServer::srv_execute() {

  if (inited_) {
    new_state_ = S_EXECUTE;
    timeout_ = ros::Time::now();
  }

}

void ManualArmManipActionServer::srv_reset() {

  if (inited_) {
    new_state_ = S_RESET;
    timeout_ = ros::Time::now();
  }

}

void ManualArmManipActionServer::srv_success() {

  if (inited_) {
    new_state_ = S_SUCCESS;
    timeout_ = ros::Time::now();
  }

}

void ManualArmManipActionServer::srv_failed() {

  if (inited_) {
    new_state_ = S_FAILED;
    timeout_ = ros::Time::now();
  }

}

void ManualArmManipActionServer::setFeedbackFalse() {

  feedback_.feedback.starting = false;
  feedback_.feedback.executing = false;
  feedback_.feedback.planning = false;
  feedback_.feedback.reset = false;

}


int main(int argc, char** argv)
{

      CArmManipulationEditor * ps_editor = NULL;


      ROS_INFO("Starting");
      ros::init(argc, argv, "but_simple_manual_arm_navigation");

      param<string>("set_planning_scene_diff_name", params.set_planning_scene_diff_name_, SET_PLANNING_SCENE_DIFF_NAME);
      param<string>("left_ik_name", params.left_ik_name_, LEFT_IK_NAME);
      param<string>("left_interpolate_service_name", params.left_interpolate_service_name_, LEFT_INTERPOLATE_SERVICE_NAME);
      param<string>("non_coll_left_ik_name", params.non_coll_left_ik_name_, NON_COLL_LEFT_IK_NAME);
      param<string>("non_coll_right_ik_name", params.non_coll_right_ik_name_, NON_COLL_RIGHT_IK_NAME);
      param<string>("planner_1_service_name", params.planner_1_service_name_, PLANNER_1_SERVICE_NAME);
      param<string>("planner_2_service_name", params.planner_2_service_name_, PLANNER_2_SERVICE_NAME);
      param<string>("proximity_space_planner_name", params.proximity_space_planner_name_, PROXIMITY_SPACE_PLANNER_NAME);
      param<string>("proximity_space_service_name",  params.proximity_space_service_name_, PROXIMITY_SPACE_SERVICE_NAME);
      param<string>("proximity_space_validity_name",  params.proximity_space_validity_name_,  PROXIMITY_SPACE_VALIDITY_NAME);
      param<string>("right_ik_name", params.right_ik_name_, RIGHT_IK_NAME);
      param<string>("right_interpolate_service_name", params.right_interpolate_service_name_, RIGHT_INTERPOLATE_SERVICE_NAME);
      param<string>("trajectory_filter_1_service_name", params.trajectory_filter_1_service_name_, TRAJECTORY_FILTER_1_SERVICE_NAME);
      param<string>("trajectory_filter_2_service_name", params.trajectory_filter_2_service_name_, TRAJECTORY_FILTER_2_SERVICE_NAME);
      param<string>("vis_topic_name", params.vis_topic_name_ , VIS_TOPIC_NAME);
      param<string>("right_ik_link", params.right_ik_link_ , RIGHT_IK_LINK);
      param<string>("left_ik_link", params.left_ik_link_ , LEFT_IK_LINK);
      param<string>("right_arm_group", params.right_arm_group_ , RIGHT_ARM_GROUP);
      param<string>("left_arm_group", params.left_arm_group_ , LEFT_ARM_GROUP);
      param<string>("right_redundancy", params.right_redundancy_ , RIGHT_ARM_REDUNDANCY);
      param<string>("left_redundancy", params.left_redundancy_ , LEFT_ARM_REDUNDANCY);
      param<string>("execute_left_trajectory", params.execute_left_trajectory_ , EXECUTE_LEFT_TRAJECTORY);
      param<string>("execute_right_trajectory", params.execute_right_trajectory_ , EXECUTE_RIGHT_TRAJECTORY);
      param<string>("list_controllers_service", params.list_controllers_service_, LIST_CONTROLLERS_SERVICE);
      param<string>("load_controllers_service", params.load_controllers_service_, LOAD_CONTROLLERS_SERVICE);
      param<string>("unload_controllers_service", params.unload_controllers_service_, UNLOAD_CONTROLLERS_SERVICE);
      param<string>("switch_controllers_service", params.switch_controllers_service_, SWITCH_CONTROLLERS_SERVICE);
      param<string>("gazebo_robot_model", params.gazebo_model_name_, GAZEBO_ROBOT_MODEL);
      param<string>("robot_description_param", params.robot_description_param_, ROBOT_DESCRIPTION_PARAM);
      param<bool>("use_robot_data", params.use_robot_data_, true);
      params.sync_robot_state_with_gazebo_ = false;



      ROS_INFO("Creating planning scene editor");
      ps_editor = new CArmManipulationEditor(params);

      ROS_INFO("Advertising services");
      ros::NodeHandle n;
      ros::ServiceServer service_new = n.advertiseService(SRV_NEW, &CArmManipulationEditor::ArmNavNew,ps_editor);
      ros::ServiceServer service_plan = n.advertiseService(SRV_PLAN, &CArmManipulationEditor::ArmNavPlan,ps_editor);
      ros::ServiceServer service_play = n.advertiseService(SRV_PLAY, &CArmManipulationEditor::ArmNavPlay,ps_editor);
      ros::ServiceServer service_execute = n.advertiseService(SRV_EXECUTE, &CArmManipulationEditor::ArmNavExecute,ps_editor);
      ros::ServiceServer service_reset = n.advertiseService(SRV_RESET, &CArmManipulationEditor::ArmNavReset,ps_editor);

      ros::ServiceServer service_success = n.advertiseService(SRV_SUCCESS, &CArmManipulationEditor::ArmNavSuccess,ps_editor);
      ros::ServiceServer service_failed = n.advertiseService(SRV_FAILED, &CArmManipulationEditor::ArmNavFailed,ps_editor);

      ros::Timer timer1 = n.createTimer(ros::Duration(0.01), &CArmManipulationEditor::spin_callback,ps_editor);

      ManualArmManipActionServer act_server(ACT_ARM_MANIP);

      ps_editor->action_server_ptr_ = &act_server;

      ps_editor->tfl_ = new tf::TransformListener();

      ROS_INFO("Spinning");

      ros::spin();


      ros::shutdown();

      delete ps_editor;

}
