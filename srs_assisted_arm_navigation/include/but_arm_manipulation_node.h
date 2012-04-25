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

#ifndef BUT_ARMNAVIGATION_NODE_H
#define BUT_ARMNAVIGATION_NODE_H


#include <ros/ros.h>
#include "move_arm_warehouse/move_arm_utils.h"
#include <assert.h>
#include <unistd.h>
#include <time.h>

#include "srs_assisted_arm_navigation/ArmNavNew.h"
#include "srs_assisted_arm_navigation/ArmNavPlan.h"
#include "srs_assisted_arm_navigation/ArmNavPlay.h"
#include "srs_assisted_arm_navigation/ArmNavReset.h"
#include "srs_assisted_arm_navigation/ArmNavRefresh.h"
#include "srs_assisted_arm_navigation/ArmNavExecute.h"
#include "srs_assisted_arm_navigation/ArmNavStart.h"
#include "srs_assisted_arm_navigation/ArmNavCollObj.h"
#include "srs_assisted_arm_navigation/ArmNavMovePalmLink.h"
#include "srs_assisted_arm_navigation/ArmNavSwitchAttCO.h"

#include "srs_assisted_arm_navigation/ArmNavSuccess.h"
#include "srs_assisted_arm_navigation/ArmNavFailed.h"
#include "srs_assisted_arm_navigation/ArmNavRepeat.h"

#include "srs_assisted_arm_navigation/ManualArmManipAction.h"
#include <actionlib/server/simple_action_server.h>

using namespace srs_assisted_arm_navigation;

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

#define BUT_PREFIX std::string("/but_arm_manip")

#define BUT_SERVICE(topic) BUT_PREFIX + std::string(topic)
#define BUT_TOPIC(topic) BUT_PREFIX + std::string(topic)
#define BUT_ACTION(topic) BUT_PREFIX + std::string(topic)

#define SRV_START BUT_SERVICE("/arm_nav_start")
#define SRV_NEW BUT_SERVICE("/arm_nav_new")
#define SRV_PLAN BUT_SERVICE("/arm_nav_plan")
#define SRV_PLAY BUT_SERVICE("/arm_nav_play")
#define SRV_EXECUTE BUT_SERVICE("/arm_nav_execute")
#define SRV_RESET BUT_SERVICE("/arm_nav_reset")
#define SRV_SUCCESS BUT_SERVICE("/arm_nav_success")
#define SRV_FAILED BUT_SERVICE("/arm_nav_failed")
#define SRV_REFRESH BUT_SERVICE("/arm_nav_refresh")
#define SRV_COLLOBJ BUT_SERVICE("/arm_nav_coll_obj")
#define SRV_MOVE_PALM_LINK BUT_SERVICE("/arm_nav_move_palm_link")
#define SRV_SWITCH BUT_SERVICE("/arm_nav_switch_aco")
#define SRV_REPEAT BUT_SERVICE("/arm_nav_repeat")

#define ACT_ARM_MANIP BUT_ACTION("/manual_arm_manip_action")


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
  void srv_set_state(unsigned int n);

  ros::Duration start_timeout_; /**< Action should start before start_time_ + start_timeout_ */
  ros::Duration solve_timeout_; /**< This is timeout between user actions */

  protected:

    unsigned int state_;             /**< Variable for holding current state of execution */
    unsigned int new_state_;         /**< Variable for holding new state of execution */

    ros::NodeHandle nh_;

    ///Actionlib server
    actionlib::SimpleActionServer<ManualArmManipAction> server_;
    std::string action_name_;
    ManualArmManipActionFeedback feedback_;
    ManualArmManipActionResult result_;

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
    void executeCB(const ManualArmManipGoalConstPtr &goal);


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
  bool ArmNavNew(ArmNavNew::Request &req, ArmNavNew::Response &res);
  bool ArmNavPlan(ArmNavPlan::Request &req, ArmNavPlan::Response &res);
  bool ArmNavPlay(ArmNavPlay::Request &req, ArmNavPlay::Response &res);
  bool ArmNavExecute(ArmNavExecute::Request &req, ArmNavExecute::Response &res);
  bool ArmNavReset(ArmNavReset::Request &req, ArmNavReset::Response &res);
  bool ArmNavRefresh(ArmNavRefresh::Request &req, ArmNavRefresh::Response &res);

  bool ArmNavSuccess(ArmNavSuccess::Request &req, ArmNavSuccess::Response &res);
  bool ArmNavFailed(ArmNavFailed::Request &req, ArmNavFailed::Response &res);
  bool ArmNavRepeat(ArmNavRepeat::Request &req, ArmNavRepeat::Response &res);

  bool ArmNavCollObj(ArmNavCollObj::Request &req, ArmNavCollObj::Response &res);
  bool ArmNavMovePalmLink(ArmNavMovePalmLink::Request &req, ArmNavMovePalmLink::Response &res);
  bool ArmNavSwitchACO(ArmNavSwitchAttCO::Request &req, ArmNavSwitchAttCO::Response &res);

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

protected:


  std::string planning_scene_id; /**< ID of current planing scene */
  unsigned int mpr_id; /**< ID of current motion plan request */
  unsigned int traj_id; /**< ID of planned trajectory */
  unsigned int filt_traj_id; /**< ID of filtered trajectory */
  std::vector<std::string> coll_obj_attached_id; /**< List of IDs of attached collision objects (artificial objects attached to arm gripper) */
  //std::vector<std::string> coll_obj_bb_id;
  bool inited; /**< Indicates if the arm_nav_new was already called  */

  std::vector<std::string> links_; /**< List of links which can be in collision of collision object */

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
  std::string add_coll_obj_bb(std::string name, geometry_msgs::PoseStamped pose, geometry_msgs::Point bb_lwh);

  void armHasStoppedMoving();

  /**
   * Struct to store data of detected object - in /map coord. system. It's recalculated to /base_link on each creation of planning scene.
   */
  typedef struct {

    std::string name;
    std::string id;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Point bb_lwh;

  } t_det_obj;

  /**
   * List of detected objects which should be in added in collision map.
   */
  std::vector<t_det_obj> coll_obj_det;


private:

};

#endif
