/******************************************************************************
 * \file but_arm_manipulation_node.h
 * \brief Definition of ManualArmManipActionServer and CArmManipulationEditor classes.
 * \author Zdenek Materna (imaterna@fit.vutbr.cz)
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

/**
 * This is node which acts as bridge between functionality of warehouse viewer and RVIZ plugin.
 * It provides actionlib interface (server) for communication with i.e. decision making and some services for
 * communication with RVIZ arm manipulation plugin.
 *
 */

#pragma once
#ifndef BUT_ARMNAVIGATION_NODE_H
#define BUT_ARMNAVIGATION_NODE_H


#include <ros/ros.h>
#include "srs_assisted_arm_navigation/move_arm_utils/move_arm_utils.h"
#include <assert.h>
#include <unistd.h>
#include <time.h>
#include <boost/circular_buffer.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/Vector3.h>
#include "math.h"

#include "srs_assisted_arm_navigation_msgs/ArmNavNew.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavPlan.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavPlay.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavReset.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavRefresh.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavExecute.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavStart.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavCollObj.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavSetAttached.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavMovePalmLink.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavMovePalmLinkRel.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavSwitchAttCO.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavStep.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavStop.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavSuccess.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavFailed.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavRepeat.h"
#include "srs_assisted_arm_navigation_msgs/AssistedArmNavigationState.h"

#include "srs_assisted_arm_navigation_msgs/ManualArmManipAction.h"

#include <actionlib/server/simple_action_server.h>

#include "srs_assisted_arm_navigation/services_list.h"
#include "srs_assisted_arm_navigation/topics_list.h"

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "sensor_msgs/Joy.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
//#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/Vector3.h"
#include <tf/transform_broadcaster.h>

namespace srs_assisted_arm_navigation {

/**
 * Definition of services
 *
 * arm_nav_start: when called a messagebox will pop-up in RVIZ plugin
 * arm_nav_new: this will create planning scene
 * arm_nav_plan: plan and filter trajectory
 * arm_nav_play: play previously planned trajectory
 * arm_nav_execute: execute planned trajectory (it doesn't wait for finish of execution)
 * arm_nav_reset: delete planning scene, start from beginning
 * arm_nav_success: user marked action as successful
 * arm_nav_failed: user marked action as failed
 * arm_nav_refresh: send current planning scene
 *
 */


/**
 * Definition of default values for parameters needed by PlanningSceneEditor. Values are customized for use with COB.
 */
static const std::string VIS_TOPIC_NAME = "planning_scene_visualizer_markers";
static const std::string EXECUTE_RIGHT_TRAJECTORY = "none";
static const std::string EXECUTE_LEFT_TRAJECTORY = "/arm_controller/follow_joint_trajectory";
static const std::string LEFT_IK_NAME = "/cob3_arm_kinematics/get_constraint_aware_ik";
static const std::string RIGHT_IK_NAME = "none";
static const std::string NON_COLL_LEFT_IK_NAME = "/cob3_arm_kinematics/get_ik";
static const std::string NON_COLL_RIGHT_IK_NAME = "none";
static const std::string RIGHT_ARM_GROUP = "none";
static const std::string LEFT_ARM_GROUP = "arm";
static const std::string RIGHT_ARM_REDUNDANCY = "none";
static const std::string LEFT_ARM_REDUNDANCY = "none";
static const std::string LEFT_IK_LINK = "arm_7_link";
static const std::string RIGHT_IK_LINK = "none";
static const std::string PLANNER_1_SERVICE_NAME = "/ompl_planning/plan_kinematic_path";
static const std::string PLANNER_2_SERVICE_NAME = "none";
static const std::string LEFT_INTERPOLATE_SERVICE_NAME = "none";
static const std::string RIGHT_INTERPOLATE_SERVICE_NAME = "none";
static const std::string TRAJECTORY_FILTER_1_SERVICE_NAME = "/trajectory_filter_server/filter_trajectory_with_constraints";
static const std::string TRAJECTORY_FILTER_2_SERVICE_NAME = "none";
static const std::string PROXIMITY_SPACE_SERVICE_NAME = "none";
static const std::string PROXIMITY_SPACE_VALIDITY_NAME = "none";
static const std::string PROXIMITY_SPACE_PLANNER_NAME = "none";
static const std::string LIST_CONTROLLERS_SERVICE = "/pr2_controller_manager/list_controllers";
static const std::string LOAD_CONTROLLERS_SERVICE = "/pr2_controller_manager/load_controller";
static const std::string UNLOAD_CONTROLLERS_SERVICE = "/pr2_controller_manager/unload_controller";
static const std::string SWITCH_CONTROLLERS_SERVICE = "/pr2_controller_manager/switch_controller";
static const std::string GAZEBO_ROBOT_MODEL = "robot";
static const std::string ROBOT_DESCRIPTION_PARAM = "robot_description";
static const ros::Duration PLANNING_DURATION = ros::Duration(5.0);
static const std::string SET_PLANNING_SCENE_DIFF_NAME = "environment_server/set_planning_scene_diff";

static const std::string WORLD_FRAME = "/map";



static const double START_TIMEOUT = 60.0; /**< This parameter defines time in which a user must click on NEW button (in RVIZ plugin) after start of action */
static const double SOLVE_TIMEOUT = (5*60.0); /**< This parameter defines maximum amount of time in which the user has to solve arm navigation task */
/*! This parameter can be used to make bounding box of detected object (which can be inserted to planning scene)
 * slightly bigger. It can be useful if planning scene suffers from noise in collision map. */
static const double INFLATE_BB = 1.0;

/*!
 * ManualArmManipActionServer
 *
 * \brief Class is used to wrap actionlib server and related variables.
 *
 * Class implements actionlib server,
 * which can be called from some client (there is also example for a client in this package: arm_manip_state_machine.py).
 * It's also maintaining state of manually assisted arm manipulation.
 *
 * \author Zdenek Materna
 *
 */
class ManualArmManipActionServer{

public:

  /*!
   * Constructor of class.
   *
   * \param name Name of the action.
   *
   */
  ManualArmManipActionServer(std::string name):
    server_(nh_, name, boost::bind(&ManualArmManipActionServer::executeCB, this, _1), false), action_name_(name) {

    inited_ = false;
    state_ = S_NONE;

    start_timeout_ = ros::Duration(START_TIMEOUT);
    solve_timeout_ = ros::Duration(SOLVE_TIMEOUT);

    server_.start(); /**< start of actionlib server */

  }

  ~ManualArmManipActionServer(void) {};


  /// List of possible states of arm manipulation task
  enum {S_NONE,S_NEW,S_PLAN,S_EXECUTE,S_RESET,S_SUCCESS,S_FAILED, S_REPEAT};

  /*!
   * Method for changing state of execution (state, new_state variables).
   *
   */

  unsigned int get_state() {

	  return state_;

  }

  void srv_set_state(unsigned int n);

  ros::Duration start_timeout_; /**< Action should start before start_time_ + start_timeout_ */
  ros::Duration solve_timeout_; /**< This is timeout between user actions */


  protected:

    unsigned int state_;             /**< Variable for holding current state of execution */
    unsigned int new_state_;         /**< Variable for holding new state of execution */

    ros::NodeHandle nh_;

    ///Actionlib server
    actionlib::SimpleActionServer<srs_assisted_arm_navigation_msgs::ManualArmManipAction> server_;
    std::string action_name_;
    srs_assisted_arm_navigation_msgs::ManualArmManipActionFeedback feedback_;
    srs_assisted_arm_navigation_msgs::ManualArmManipActionResult result_;

    ros::Time start_time_; /**< time at which the task was started */
    ros::Time timeout_; /**< updated on each user action */

    bool inited_; /**< Flag indicating that arm_nav_start was called */

    planning_scene_utils::PlanningSceneParameters params_;



    /**
     * Feedback of
     *
     */
    void setFeedbackFalse();

    /*!
     * This method maintains state of execution, sends feedback and result of the action. It's used as callback of actionlib server.
     *
     * \param goal Goal of action (pregrasp position / away)
     *
     */
    void executeCB(const srs_assisted_arm_navigation_msgs::ManualArmManipGoalConstPtr &goal);


  private:


};

/*!
 * CArmManipulationEditor
 *
 * \brief Class inherited from planning_scene_utils::PlanningSceneEditor
 *
 * These class provides access to functionality of PlanningSceneEditor.
 *
 */
class CArmManipulationEditor : public planning_scene_utils::PlanningSceneEditor
{

public:

  /**
   * Class constructor.
   *
   * @param params Parameters for planning scene editor.
   * @param clist List of links which may be in collision with artificial collision object connected to arm gripper.
   */
   CArmManipulationEditor(planning_scene_utils::PlanningSceneParameters& params,  std::vector<string> clist);
  ~CArmManipulationEditor();

  /**
   * Various callbacks - currently not used.
   */
  virtual void onPlanningSceneLoaded();
  virtual void updateState();

  virtual void planCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode);
  virtual void filterCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode);

  virtual void attachObjectCallback(const std::string& name);
  virtual void selectedTrajectoryCurrentPointChanged( unsigned int new_current_point );

  /**
   * Method which creates artificial collision object connected to arm (gripper).
   * @todo Make it configurable by parameters in launch file (size and position of object)
   */
  void createAttachedObj();

  /*!
   * Service callbacks. Each callback performes some action and sets new state of execution.
   *
   */
  bool ArmNavNew(srs_assisted_arm_navigation_msgs::ArmNavNew::Request &req, srs_assisted_arm_navigation_msgs::ArmNavNew::Response &res);
  bool ArmNavPlan(srs_assisted_arm_navigation_msgs::ArmNavPlan::Request &req, srs_assisted_arm_navigation_msgs::ArmNavPlan::Response &res);
  bool ArmNavPlay(srs_assisted_arm_navigation_msgs::ArmNavPlay::Request &req, srs_assisted_arm_navigation_msgs::ArmNavPlay::Response &res);
  bool ArmNavExecute(srs_assisted_arm_navigation_msgs::ArmNavExecute::Request &req, srs_assisted_arm_navigation_msgs::ArmNavExecute::Response &res);
  bool ArmNavReset(srs_assisted_arm_navigation_msgs::ArmNavReset::Request &req, srs_assisted_arm_navigation_msgs::ArmNavReset::Response &res);
  bool ArmNavRefresh(srs_assisted_arm_navigation_msgs::ArmNavRefresh::Request &req, srs_assisted_arm_navigation_msgs::ArmNavRefresh::Response &res);

  bool ArmNavSuccess(srs_assisted_arm_navigation_msgs::ArmNavSuccess::Request &req, srs_assisted_arm_navigation_msgs::ArmNavSuccess::Response &res);
  bool ArmNavFailed(srs_assisted_arm_navigation_msgs::ArmNavFailed::Request &req, srs_assisted_arm_navigation_msgs::ArmNavFailed::Response &res);
  bool ArmNavRepeat(srs_assisted_arm_navigation_msgs::ArmNavRepeat::Request &req, srs_assisted_arm_navigation_msgs::ArmNavRepeat::Response &res);

  bool ArmNavCollObj(srs_assisted_arm_navigation_msgs::ArmNavCollObj::Request &req, srs_assisted_arm_navigation_msgs::ArmNavCollObj::Response &res);
  bool ArmNavSetAttached(srs_assisted_arm_navigation_msgs::ArmNavSetAttached::Request &req, srs_assisted_arm_navigation_msgs::ArmNavSetAttached::Response &res);
  bool ArmNavMovePalmLink(srs_assisted_arm_navigation_msgs::ArmNavMovePalmLink::Request &req, srs_assisted_arm_navigation_msgs::ArmNavMovePalmLink::Response &res);
  bool ArmNavMovePalmLinkRel(srs_assisted_arm_navigation_msgs::ArmNavMovePalmLinkRel::Request &req, srs_assisted_arm_navigation_msgs::ArmNavMovePalmLinkRel::Response &res);
  bool ArmNavSwitchACO(srs_assisted_arm_navigation_msgs::ArmNavSwitchAttCO::Request &req, srs_assisted_arm_navigation_msgs::ArmNavSwitchAttCO::Response &res);
  bool ArmNavStep(srs_assisted_arm_navigation_msgs::ArmNavStep::Request &req, srs_assisted_arm_navigation_msgs::ArmNavStep::Response &res);
  
  bool ArmNavStop(srs_assisted_arm_navigation_msgs::ArmNavStop::Request &req, srs_assisted_arm_navigation_msgs::ArmNavStop::Response &res);


  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

  typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::Vector3, geometry_msgs::Vector3> MySyncPolicy;

  ros::Subscriber offset_sub_;
  ros::Subscriber rot_offset_sub_;
  ros::Subscriber joy_sub_;

  void spacenavOffsetCallback(const geometry_msgs::Vector3ConstPtr& offset);
  void spacenavRotOffsetCallback(const geometry_msgs::Vector3ConstPtr& rot_offset);



  //ros::Subscriber joy_sub_;

  /** This callback is used to send interactive markers. It uses TimerEvent.
   *  @todo Make the period configurable.
   */
  void spin_callback(const ros::TimerEvent&);

  /**
   * This method removes all artificial objects and all trajectory data from warehouse.
   * @todo Test if it does its job correctly.
   */
  void reset();

  /**
   * Remove detected objects from list.
   */
  void remove_coll_objects();

  bool refresh();

  /// Pointer to action server. It's used to get access to srv_set_state method of ManualArmManipActionServer class
  ManualArmManipActionServer * action_server_ptr_;
  tf::TransformListener *tfl_;

  /** Variable to store inflate_bb parameter.
   * \see INFLATE_BB
   */
  double inflate_bb_;

  bool aco_;

  ros::Publisher gripper_rpy_publisher_;

  std::string world_frame_;

  bool transf(std::string target_frame, geometry_msgs::PoseStamped& pose);

protected:

  tf::TransformBroadcaster br_;

  void timerCallback(const ros::TimerEvent& ev);

  void tfTimerCallback(const ros::TimerEvent& ev);

  ros::Timer spacenav_timer_;

  ros::Timer tf_timer_;

  bool use_spacenav_;

  struct {

	  boost::mutex mutex_;

	  bool offset_received_;
	  bool rot_offset_received_;

	  bool lock_position_;
	  bool lock_orientation_;

	  geometry_msgs::Vector3 offset;
	  geometry_msgs::Vector3 rot_offset;

	  double max_val_;
	  double min_val_th_;
	  double step_;
	  double rot_step_;
	  bool use_rviz_cam_;
	  std::string rviz_cam_link_;

	  vector<int32_t> buttons_;


  } spacenav;

  ros::Publisher arm_nav_state_pub_;

  void normAngle(double& a);

  void processSpaceNav();


  std::string planning_scene_id; /**< ID of current planing scene */
  unsigned int mpr_id; /**< ID of current motion plan request */
  unsigned int traj_id; /**< ID of planned trajectory */
  unsigned int filt_traj_id; /**< ID of filtered trajectory */
  std::vector<std::string> coll_obj_attached_id; /**< List of IDs of attached collision objects (artificial objects attached to arm gripper) */
  //std::vector<std::string> coll_obj_bb_id;
  bool inited; /**< Indicates if the arm_nav_new was already called  */

  bool planned_;

  std::vector<std::string> links_; /**< List of links which can be in collision of collision object */

  boost::circular_buffer<geometry_msgs::Pose> * gripper_poses_;

  boost::thread gripper_poses_thread_;

  bool disable_gripper_poses_;

  void GripperPoses();
  void GripperPosesClean();

  geometry_msgs::Vector3 GetAsEuler(geometry_msgs::Quaternion quat);

  bool step_used_;

  //boost::mutex im_server_mutex_;

  /**
   * Method which can be used to add attached collision object to planning scene. Added object is cylinder attached to /arm_7_link.
   * Position coordinates are in /arm_7_link frame (method transforms them to base_link frame).
   * @param x position
   * @param y position
   * @param z position
   * @param scx scale in x and y (cylinder is always symmetric)
   * @param scz length of cylinder
   * @return ID of added collision object.
   * @todo Add color as param.
   * @todo Make frame configurable.
   * @bug Fails if it's called multiple times. For one object it works fine.
   */
  std::string add_coll_obj_attached(double x, double y, double z, double scx, double scz);
  std::string add_coll_obj_bb(std::string name, geometry_msgs::PoseStamped pose, geometry_msgs::Point bb_lwh, bool coll, bool attached);

  std::string collision_objects_frame_id_;

  //void armHasStoppedMoving();

  /**
   * Struct to store data of detected object - in /map coord. system. It's recalculated to /base_link on each creation of planning scene.
   */
  typedef struct {

    std::string name;
    std::string id;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Point bb_lwh;
    bool allow_collision;
    bool attached;

  } t_det_obj;

  /**
   * List of detected objects which should be in added in collision map.
   */
  std::vector<t_det_obj> coll_obj_det;

  void findIK(geometry_msgs::Pose new_pose);

  bool checkPose(geometry_msgs::PoseStamped &p, std::string frame);

  bool joint_controls_;
  std::string aco_link_;

  std::string end_eff_link_;
  //bool aco_state_;

  struct {

	  arm_navigation_msgs::ArmNavigationErrorCodes plan_error_code;
	  arm_navigation_msgs::ArmNavigationErrorCodes filter_error_code;

	  std::string plan_desc;
	  std::string filter_desc;

	  bool out_of_limits;

  } arm_nav_state_;




private:

};

}

#endif
