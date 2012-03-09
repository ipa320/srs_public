#ifndef BUT_ARMNAVIGATION_NODE_H
#define BUT_ARMNAVIGATION_NODE_H


#include <ros/ros.h>
#include "move_arm_warehouse/move_arm_utils.h"
#include <assert.h>
#include <unistd.h>
//#include <boost/thread.hpp>
#include <time.h>

#include "srs_ui_but/ArmNavNew.h"
#include "srs_ui_but/ArmNavPlan.h"
#include "srs_ui_but/ArmNavPlay.h"
#include "srs_ui_but/ArmNavReset.h"
#include "srs_ui_but/ArmNavExecute.h"
#include "srs_ui_but/ArmNavStart.h"
//#include "srs_ui_but/ArmNavFinish.h"
#include "srs_ui_but/ArmNavSuccess.h"
#include "srs_ui_but/ArmNavFailed.h"

#include "srs_arm_navigation/ManualArmManipAction.h"
#include <actionlib/server/simple_action_server.h>

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


class ManualArmManipActionServer{

  protected:

    typedef enum {S_NONE,S_NEW,S_PLAN,S_EXECUTE,S_RESET,S_SUCCESS,S_FAILED} t_state;

    t_state state_;
    t_state new_state_;

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<srs_arm_navigation::ManualArmManipAction> server_;
    std::string action_name_;
    srs_arm_navigation::ManualArmManipActionFeedback feedback_;
    srs_arm_navigation::ManualArmManipActionResult result_;
    ros::Time start_time_;
    ros::Time timeout_; // update on each user action

    ros::Duration start_timeout_;
    ros::Duration solve_timeout_;

    bool inited_;

    void setFeedbackFalse();
    void executeCB(const srs_arm_navigation::ManualArmManipGoalConstPtr &goal);

  public:

    ManualArmManipActionServer(std::string name):
      server_(nh_, name, boost::bind(&ManualArmManipActionServer::executeCB, this, _1), false), action_name_(name) {

      inited_ = false;
      state_ = S_NONE;
      start_timeout_ = ros::Duration(60); // 1 minute TODO: load it from param server
      solve_timeout_ = ros::Duration(10*60); // 10 minutes

      server_.start();

    }

    ~ManualArmManipActionServer(void) {};

    void srv_new();
    void srv_plan();
    void srv_execute();
    //void srv_finish();
    void srv_success();
    void srv_failed();
    void srv_reset();

  private:


};

class CArmManipulationEditor : public planning_scene_utils::PlanningSceneEditor
{

public:

   CArmManipulationEditor(planning_scene_utils::PlanningSceneParameters& params);
  ~CArmManipulationEditor();

  virtual void onPlanningSceneLoaded();
  virtual void updateState();
  virtual void planCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode);
  virtual void filterCallback(arm_navigation_msgs::ArmNavigationErrorCodes& errorCode);
  virtual void attachObjectCallback(const std::string& name);
  virtual void selectedTrajectoryCurrentPointChanged( unsigned int new_current_point );


  bool ArmNavNew(srs_ui_but::ArmNavNew::Request &req, srs_ui_but::ArmNavNew::Response &res);
  bool ArmNavPlan(srs_ui_but::ArmNavPlan::Request &req, srs_ui_but::ArmNavPlan::Response &res);
  bool ArmNavPlay(srs_ui_but::ArmNavPlay::Request &req, srs_ui_but::ArmNavPlay::Response &res);
  bool ArmNavExecute(srs_ui_but::ArmNavExecute::Request &req, srs_ui_but::ArmNavExecute::Response &res);
  bool ArmNavReset(srs_ui_but::ArmNavReset::Request &req, srs_ui_but::ArmNavReset::Response &res);
  //bool ArmNavFinish(srs_ui_but::ArmNavFinish::Request &req, srs_ui_but::ArmNavFinish::Response &res);
  bool ArmNavSuccess(srs_ui_but::ArmNavSuccess::Request &req, srs_ui_but::ArmNavSuccess::Response &res);
  bool ArmNavFailed(srs_ui_but::ArmNavFailed::Request &req, srs_ui_but::ArmNavFailed::Response &res);
  void spin_callback(const ros::TimerEvent&);

  void reset();

  ManualArmManipActionServer * action_server_ptr_;

protected:

  std::string planning_scene_id;
  unsigned int mpr_id;
  unsigned int traj_id;
  unsigned int filt_traj_id;
  bool inited;

private:

};

#endif
