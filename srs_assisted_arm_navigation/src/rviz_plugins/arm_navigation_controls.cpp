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
 * Date: 5/4/2012
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
 * @todo Add timeout to ArmNavNew message. Show current timeout to user.
 * @todo Add button for move the gripper to selected pre-grasp position.
 * @todo Add messagebox which will be shown on finish of trajectory execution
 * @todo Add ID of reached trajectory to the ArmNavFinish
 */

#include <srs_assisted_arm_navigation/rviz_plugins/arm_navigation_controls.h>

#include <rviz/window_manager_interface.h>

using namespace std;
using namespace srs_assisted_arm_navigation;

const int ID_BUTTON_NEW(101);
const int ID_BUTTON_PLAN(102);
const int ID_BUTTON_PLAY(103);
const int ID_BUTTON_EXECUTE(104);
const int ID_BUTTON_RESET(105);
const int ID_BUTTON_SUCCESS(106);
const int ID_BUTTON_FAILED(107);

const int ID_BUTTON_GRIPPER_O(109);
const int ID_BUTTON_GRIPPER_C(110);
const int ID_BUTTON_LOOK(111);
const int ID_BUTTON_REFRESH(112);
const int ID_BUTTON_SWITCH(113);
const int ID_BUTTON_REPEAT(114);

const int ID_BUTTON_UNDO(115);

const int ID_BUTTON_MLEFT(117);
const int ID_BUTTON_MRIGHT(118);
const int ID_BUTTON_MUP(119);
const int ID_BUTTON_MDOWN(120);
const int ID_BUTTON_MFORW(121);
const int ID_BUTTON_MBACK(122);


const int ID_BUTTON_STOP_TRAJ(123);
const int ID_BUTTON_LOCK_CMAP(124);


/**
 Constructor
 */
CArmManipulationControls::CArmManipulationControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi )
{


    parent_ = parent;
    
    ros::param::param<bool>("~wait_for_start", wait_for_start_ , WAIT_FOR_START);

    buttons_["stop"] = new wxButton(this, ID_BUTTON_STOP_TRAJ, wxT("Stop"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["reset"] = new wxButton(this, ID_BUTTON_RESET, wxT("Reset"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    buttons_["gripper_o"] = new wxButton(this, ID_BUTTON_GRIPPER_O, wxT("Open gripper"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["gripper_c"] = new wxButton(this, ID_BUTTON_GRIPPER_C, wxT("Close gripper"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["look_around"] = new wxButton(this, ID_BUTTON_LOOK, wxT("Look around"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["refresh"] = new wxButton(this, ID_BUTTON_REFRESH, wxT("Refresh"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    buttons_["switch"] = new wxButton(this, ID_BUTTON_SWITCH, wxT("Disable ACO"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    buttons_["success"] = new wxButton(this, ID_BUTTON_SUCCESS, wxT("Success"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["failed"] = new wxButton(this, ID_BUTTON_FAILED, wxT("Failed"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["repeat"] = new wxButton(this, ID_BUTTON_REPEAT, wxT("Repeat"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    //---------------------------------------------------------------
    // end effector controls
    buttons_["step_back"] = new wxButton(this, ID_BUTTON_UNDO, wxT("Undo"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    buttons_["move_left"] = new wxButton(this, ID_BUTTON_MLEFT, wxT("L"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_right"] = new wxButton(this, ID_BUTTON_MRIGHT, wxT("R"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_up"] = new wxButton(this, ID_BUTTON_MUP, wxT("U"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_down"] = new wxButton(this, ID_BUTTON_MDOWN, wxT("D"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_forw"] = new wxButton(this, ID_BUTTON_MFORW, wxT("F"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_back"] = new wxButton(this, ID_BUTTON_MBACK, wxT("B"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    buttons_["new"] = new wxButton(this, ID_BUTTON_NEW, wxT("New"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["plan"] = new wxButton(this, ID_BUTTON_PLAN, wxT("Plan"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["play"] = new wxButton(this, ID_BUTTON_PLAY, wxT("Play"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["execute"] = new wxButton(this, ID_BUTTON_EXECUTE, wxT("Execute"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);


    m_lock_cmap_ = new wxToggleButton(this, ID_BUTTON_LOCK_CMAP, wxT("Lock coll. map"), wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    Connect(ID_BUTTON_LOCK_CMAP, wxEVT_COMMAND_TOGGLEBUTTON_CLICKED,
          wxCommandEventHandler(CArmManipulationControls::OnLockCmap));

    //m_lock_cmap_->Show(false);
    m_lock_cmap_->SetValue(false);
    cmap_locked_ = false;

    wxSizer *vsizer_endef = new wxStaticBoxSizer(wxVERTICAL,this,wxT("End effector controls"));
    wxSizer *hsizer_endef_top = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_endef_mid = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_endef_mid2 = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *vsizer_endef_bot = new wxBoxSizer(wxVERTICAL);

    hsizer_endef_top->Add(buttons_["step_back"], ID_BUTTON_UNDO);

    hsizer_endef_mid->Add(buttons_["move_left"], ID_BUTTON_MLEFT);
    hsizer_endef_mid->Add(buttons_["move_right"], ID_BUTTON_MRIGHT);
    hsizer_endef_mid->Add(buttons_["move_up"], ID_BUTTON_MUP);
    hsizer_endef_mid->Add(buttons_["move_down"], ID_BUTTON_MDOWN);
    hsizer_endef_mid->Add(buttons_["move_forw"], ID_BUTTON_MFORW);
    hsizer_endef_mid->Add(buttons_["move_back"], ID_BUTTON_MBACK);

    vsizer_endef->Add(hsizer_endef_top,1,wxEXPAND);
    vsizer_endef->Add(hsizer_endef_mid,1,wxEXPAND);
    vsizer_endef->Add(hsizer_endef_mid2,1,wxEXPAND);
    vsizer_endef->Add(vsizer_endef_bot,1,wxEXPAND);

    //---------------------------------------------------------------

    m_text_status = new wxStaticText(this, -1, wxT("status: waiting"));
    m_text_object = new wxStaticText(this, -1, wxT("object: none"));
    m_text_action_ = new wxStaticText(this, -1, wxT("action: none"));
    m_text_timeout = new wxStaticText(this, -1, wxT("timeout: none"));

    if (wait_for_start_) setButton("new",false);
    else setButton("new",true);
    buttons_["plan"]->Enable(false);

    buttons_["play"]->Enable(false);
    buttons_["execute"]->Enable(false);
    buttons_["reset"]->Enable(false);
    buttons_["stop"]->Enable(false);

    buttons_["success"]->Enable(false);
    buttons_["repeat"]->Enable(false);
    buttons_["failed"]->Enable(false);

    buttons_["step_back"]->Enable(false);
    buttons_["move_left"]->Enable(false);
    buttons_["move_right"]->Enable(false);
    buttons_["move_up"]->Enable(false);
    buttons_["move_down"]->Enable(false);
    buttons_["move_forw"]->Enable(false);
    buttons_["move_back"]->Enable(false);

    buttons_["switch"]->Enable(true);

    buttons_["gripper_o"]->Enable(true); // TODO povolit tlacitka jen pokud bude k dispozici actionserver?
    buttons_["gripper_c"]->Enable(true);
    buttons_["look_around"]->Enable(true);
    buttons_["refresh"]->Enable(true);

    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL); // top sizer

    wxSizer *vsizer_top = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Trajectory planning"));
    wxSizer *hsizer_traj_top = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_mid = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_bot = new wxBoxSizer(wxHORIZONTAL);

    wxSizer *vsizer_mes = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Messages"));

    wxSizer *vsizer_add = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Additional controls"));

    wxSizer *hsizer_add_top = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_add_mid = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_add_bot = new wxBoxSizer(wxHORIZONTAL);

    /* Trajectory planning related buttons, on top*/
    hsizer_traj_top->Add(buttons_["new"], ID_BUTTON_NEW);
    hsizer_traj_top->Add(buttons_["plan"], ID_BUTTON_PLAN);
    hsizer_traj_top->Add(buttons_["execute"], ID_BUTTON_EXECUTE);
    hsizer_traj_top->Add(buttons_["stop"], ID_BUTTON_STOP_TRAJ);

    hsizer_traj_mid->Add(buttons_["play"], ID_BUTTON_PLAY);

    hsizer_traj_mid->Add(buttons_["reset"], ID_BUTTON_RESET);

    hsizer_traj_bot->Add(buttons_["success"], ID_BUTTON_SUCCESS);
    hsizer_traj_bot->Add(buttons_["repeat"], ID_BUTTON_REPEAT);
    hsizer_traj_bot->Add(buttons_["failed"], ID_BUTTON_FAILED);

    vsizer_top->Add(hsizer_traj_top,1,wxEXPAND);
    vsizer_top->Add(hsizer_traj_mid,1,wxEXPAND);
    vsizer_top->Add(hsizer_traj_bot,1,wxEXPAND);

    /* Status messages*/
    vsizer_mes->Add(m_text_status);
    vsizer_mes->Add(m_text_action_);
    vsizer_mes->Add(m_text_object);
    vsizer_mes->Add(m_text_timeout);

    hsizer_add_top->Add(buttons_["gripper_o"]);
    hsizer_add_top->Add(buttons_["gripper_c"]);
    hsizer_add_mid->Add(buttons_["look_around"]);
    hsizer_add_mid->Add(buttons_["refresh"]);
    hsizer_add_mid->Add(buttons_["switch"]);

    hsizer_add_bot->Add(m_lock_cmap_);

    vsizer_add->Add(hsizer_add_top,1,wxEXPAND);
    vsizer_add->Add(hsizer_add_mid,1,wxEXPAND);
    vsizer_add->Add(hsizer_add_bot,1,wxEXPAND);

    // tady pridat check box???

    vsizer->Add(vsizer_top,0,wxEXPAND);
    vsizer->Add(vsizer_add,0,wxEXPAND);
    vsizer->Add(vsizer_mes,0,wxEXPAND);
    vsizer->Add(vsizer_endef,0,wxEXPAND);

    allow_repeat_ = false;

    // TODO make it configurable ? - read same parameter from here and from but_arm_manip_node...
    aco_ = true;

    if (aco_) buttons_["switch"]->SetLabel(wxT("ACO enabled"));
    else buttons_["switch"]->SetLabel(wxT("ACO disabled"));


    vsizer->SetSizeHints(this);
    this->SetSizerAndFit(vsizer);

    ros::NodeHandle nh;

    /**
     * Service provided by plugin. It can be used to inform user that his/her action is required.
     */
    service_start_ = nh.advertiseService(SRV_START,&CArmManipulationControls::nav_start,this);

    cob_script = new cob_client("/script_server",true);

    cob_script_inited = false;



}
///////////////////////////////////////////////////////////////////////////////

void CArmManipulationControls::setButton(string but, bool state) {

  if (buttons_[but]!=NULL)
    buttons_[but]->Enable(state);

}



CArmManipulationControls::~CArmManipulationControls() {

  if (cob_script!=NULL) delete cob_script;

  ButtonsMap::iterator it;

  for (it = buttons_.begin(); it != buttons_.end(); ++it)
    delete it->second;

  buttons_.clear();

  delete m_text_status;
  delete m_text_object;
  delete m_text_timeout;

  delete m_lock_cmap_;

}

void CArmManipulationControls::NewThread() {

  ROS_INFO("Request for new trajectory");

  srs_assisted_arm_navigation::ArmNavNew srv;
  std::string status = "";
  bool success = false;

   if (ros::service::call(SRV_NEW,srv) ) {

     if (srv.response.completed) {

         success = true;

         status = "status: Perform requested action.";

         /*if (goal_pregrasp) status = "status: Move arm to desired position.";
         if (goal_away) status = "status: Move arm to safe position.";*/

     } else {

       status = "status: " + srv.response.error;

     }

   } else {

     ROS_ERROR("Failed when calling arm_nav_new service");
     status = "status: Communication error.";

   }

   wxMutexGuiEnter();

   m_text_status->SetLabel(wxString::FromAscii(status.c_str()));

   if (success) {

     buttons_["plan"]->Enable(true);
     buttons_["reset"]->Enable(true);

     if (allow_repeat_) buttons_["repeat"]->Enable(true);

     buttons_["failed"]->Enable(true);

     buttons_["step_back"]->Enable(true);
     buttons_["move_left"]->Enable(true);
     buttons_["move_right"]->Enable(true);
     buttons_["move_up"]->Enable(true);
     buttons_["move_down"]->Enable(true);
     buttons_["move_forw"]->Enable(true);
     buttons_["move_back"]->Enable(true);


   } else {

     setButton("new",true);
     if (allow_repeat_) buttons_["repeat"]->Enable(true);
     buttons_["failed"]->Enable(true);

   }

   wxMutexGuiLeave();

}


void CArmManipulationControls::OnNew(wxCommandEvent& event)
{

  if (!ros::service::exists(SRV_NEW,true)) {

    m_text_status->SetLabel(wxString::FromAscii("status: communication error"));
    ROS_ERROR("Service %s is not ready.",((std::string)SRV_NEW).c_str());
    return;

  }

  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

  /// wait for some time
  if (t_new.timed_join(td)) {

    buttons_["plan"]->Enable(false);
    buttons_["reset"]->Enable(false);
    setButton("new",false);
    buttons_["play"]->Enable(false);
    buttons_["success"]->Enable(false);
    buttons_["switch"]->Enable(false);
    buttons_["repeat"]->Enable(false);
    setButton("stop",false);

    m_text_status->SetLabel(wxString::FromAscii("status: Please wait..."));

    t_new = boost::thread(&CArmManipulationControls::NewThread,this);

  } else ROS_INFO("We have to wait until new thread finishes.");


}


void CArmManipulationControls::PlanThread() {

   ROS_INFO("Starting planning and filtering of new trajectory");

   std::string status = "";
   bool success = false;

   srs_assisted_arm_navigation::ArmNavPlan srv;

   if (ros::service::call(SRV_PLAN,srv) ) {

     success = true;

     if (srv.response.completed) status = "status: Trajectory is ready";
     else status = "status: " + srv.response.error;

   } else {

     success = false;
     ROS_ERROR("failed when calling service");
     status = "status: Communication error";

   }

   wxMutexGuiEnter();

   m_text_status->SetLabel(wxString::FromAscii(status.c_str()));

   if (success) {


     setButton("new",false);
     buttons_["plan"]->Enable(false);
     buttons_["play"]->Enable(true);
     buttons_["execute"]->Enable(true);
     buttons_["reset"]->Enable(true);
     buttons_["success"]->Enable(false);
     buttons_["repeat"]->Enable(true);
     buttons_["failed"]->Enable(true);

   } else {

     setButton("new",false);
     buttons_["plan"]->Enable(true);
     buttons_["play"]->Enable(true);
     buttons_["execute"]->Enable(true);
     buttons_["reset"]->Enable(true);
     buttons_["success"]->Enable(false);
     buttons_["repeat"]->Enable(true);
     buttons_["failed"]->Enable(true);


   }

   wxMutexGuiLeave();

}

void CArmManipulationControls::OnPlan(wxCommandEvent& event)
{

  if (!ros::service::exists(SRV_PLAN,true)) {

    m_text_status->SetLabel(wxString::FromAscii("status: communication error"));
    ROS_ERROR("Service %s is not ready.",((std::string)SRV_NEW).c_str());
    return;

  }

  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

  /// wait for some time
  if (t_plan.timed_join(td)) {

    m_text_status->SetLabel(wxString::FromAscii("status: Planning. Please wait..."));
    buttons_["plan"]->Enable(false);
    buttons_["reset"]->Enable(false);

    t_plan = boost::thread(&CArmManipulationControls::PlanThread,this);

  } else ROS_INFO("We have to wait until PLAN thread finishes.");

}

void CArmManipulationControls::OnPlay(wxCommandEvent& event)
{

   ROS_INFO("Starting planning and filtering of new trajectory");

   srs_assisted_arm_navigation::ArmNavPlay srv;

   if ( ros::service::exists(SRV_PLAY,true) && ros::service::call(SRV_PLAY,srv) ) {

     if (srv.response.completed) {

       m_text_status->SetLabel(wxString::FromAscii("status: Playing trajectory..."));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_play service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }

   setButton("new",false);
   buttons_["plan"]->Enable(false);
   buttons_["play"]->Enable(true);
   buttons_["execute"]->Enable(true);
   buttons_["reset"]->Enable(true);
   buttons_["success"]->Enable(false);
   buttons_["repeat"]->Enable(true);
   buttons_["failed"]->Enable(true);

}

void CArmManipulationControls::OnSwitch(wxCommandEvent& event)
{

  if (aco_) ROS_INFO("Lets switch attached collision object OFF");
  else ROS_INFO("Lets switch attached collision object ON");

   srs_assisted_arm_navigation::ArmNavSwitchAttCO srv;

   srv.request.state = !aco_;

   if ( ros::service::exists(SRV_SWITCH,true) && ros::service::call(SRV_SWITCH,srv) ) {

     if (srv.response.completed) {

       //m_text_status->SetLabel(wxString::FromAscii("status: Playing trajectory..."));

       aco_ = !aco_;

       if (aco_) buttons_["switch"]->SetLabel(wxT("ACO enabled"));
       else buttons_["switch"]->SetLabel(wxT("ACO disabled"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii("Can't switch state of ACO"));

     }

   } else {

     std::string tmp = SRV_SWITCH;

     ROS_ERROR("failed when calling %s service",tmp.c_str());
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }


}


void CArmManipulationControls::OnExecute(wxCommandEvent& event) {

  if (!ros::service::exists(SRV_EXECUTE,true)) {

    m_text_status->SetLabel(wxString::FromAscii("status: communication error"));
    ROS_ERROR("Service %s is not ready.",((std::string)SRV_NEW).c_str());
    return;

  }

  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

  /// wait for some time
  if (t_execute.timed_join(td)) {

    setButton("stop",true);

    t_execute = boost::thread(&CArmManipulationControls::ExecuteThread,this);

  } else ROS_INFO("We have to wait until EXECUTE thread finishes.");

}

void CArmManipulationControls::ExecuteThread()
{
   ROS_INFO("Execution of planned trajectory has been started");

   srs_assisted_arm_navigation::ArmNavExecute srv;

   std::string status = "";
   bool success = false;

   if (ros::service::call(SRV_EXECUTE,srv) ) {

     if (srv.response.completed) {

       success = true;
       status = "status: Executing trajectory...";

     } else {

       success = false;
       status = srv.response.error;

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_execute service");
     success = false;
     status = "status: Communication error";

   }

   wxMutexGuiEnter();

   m_text_status->SetLabel(wxString::FromAscii(status.c_str()));

   setButton("step_back",false);
   setButton("move_left",false);
   setButton("move_right",false);
   setButton("move_up",false);
   setButton("move_down",false);
   setButton("move_forw",false);
   setButton("move_back",false);

   if (success) {

     buttons_["plan"]->Enable(false);
     buttons_["execute"]->Enable(false);
     buttons_["reset"]->Enable(false);
     buttons_["play"]->Enable(false);
     setButton("new",true);
     buttons_["success"]->Enable(true);
     buttons_["failed"]->Enable(true);
     buttons_["repeat"]->Enable(true);
     buttons_["switch"]->Enable(true);

   } else {

     buttons_["plan"]->Enable(false);
     buttons_["execute"]->Enable(false);
     buttons_["reset"]->Enable(true);
     buttons_["play"]->Enable(false);
     setButton("new",false);
     buttons_["success"]->Enable(false);
     buttons_["repeat"]->Enable(true);
     buttons_["failed"]->Enable(true);
     buttons_["switch"]->Enable(true);

   }

   wxMutexGuiLeave();

}

void CArmManipulationControls::OnReset(wxCommandEvent& event)
{
   ROS_INFO("Reset planning stuff to initial state");

   srs_assisted_arm_navigation::ArmNavReset srv;

   if ( ros::service::exists(SRV_RESET,true) && ros::service::call(SRV_RESET,srv) ) {

     if (srv.response.completed) {

       m_text_status->SetLabel(wxString::FromAscii("status: Ok, try it again"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_reset service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }

   buttons_["plan"]->Enable(false);
   buttons_["execute"]->Enable(false);
   buttons_["reset"]->Enable(false);
   buttons_["play"]->Enable(false);
   setButton("new",true);
   buttons_["success"]->Enable(false);
   buttons_["repeat"]->Enable(true);
   buttons_["failed"]->Enable(true);
   buttons_["switch"]->Enable(true);
   setButton("stop",false);

}

void CArmManipulationControls::OnSuccess(wxCommandEvent& event)
{
   ROS_INFO("Finishing manual arm manipulation task");

   srs_assisted_arm_navigation::ArmNavSuccess srv;

   if ( ros::service::exists(SRV_SUCCESS,true) && ros::service::call(SRV_SUCCESS,srv) ) {

       m_text_status->SetLabel(wxString::FromAscii("status: Succeeded :-)"));

   } else {

     ROS_ERROR("failed when calling arm_nav_success service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }

   buttons_["plan"]->Enable(false);
   buttons_["execute"]->Enable(false);
   buttons_["reset"]->Enable(false);
   buttons_["play"]->Enable(false);
   if (wait_for_start_)  setButton("new",false);
   else  setButton("new",true);

   setButton("stop",false);

   m_text_object->SetLabel(wxString::FromAscii("object: none"));
   m_text_action_->SetLabel(wxString::FromAscii("action: none"));

   buttons_["success"]->Enable(false);
   buttons_["failed"]->Enable(false);
   buttons_["repeat"]->Enable(false);
   buttons_["switch"]->Enable(true);


}

void CArmManipulationControls::OnFailed(wxCommandEvent& event)
{
   ROS_ERROR("Manual arm manipulation task failed");

   srs_assisted_arm_navigation::ArmNavFailed srv;

   if ( ros::service::exists(SRV_FAILED,true) && ros::service::call(SRV_FAILED,srv) ) {

       buttons_["failed"]->Enable(false);

       m_text_status->SetLabel(wxString::FromAscii("status: Failed :-("));
       m_text_object->SetLabel(wxString::FromAscii("object: none"));
       m_text_action_->SetLabel(wxString::FromAscii("action: none"));

   } else {

     ROS_ERROR("failed when calling arm_nav_failed service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

     buttons_["failed"]->Enable(true);

   }

   buttons_["plan"]->Enable(false);
   buttons_["execute"]->Enable(false);
   buttons_["reset"]->Enable(false);
   buttons_["play"]->Enable(false);
   if (wait_for_start_) setButton("new",false);
   else setButton("new",true);

   buttons_["success"]->Enable(false);
   buttons_["repeat"]->Enable(false);
   buttons_["repeat"]->Enable(false);
   buttons_["switch"]->Enable(true);

   setButton("stop",false);

}

void CArmManipulationControls::OnRepeat(wxCommandEvent& event)
{
   ROS_ERROR("Request for repeat of manual arm navigation task");

   srs_assisted_arm_navigation::ArmNavRepeat srv;

   if ( ros::service::exists(SRV_REPEAT,true) && ros::service::call(SRV_REPEAT,srv) ) {

       buttons_["repeat"]->Enable(false);

       m_text_status->SetLabel(wxString::FromAscii("status: Repeating action..."));
       m_text_object->SetLabel(wxString::FromAscii("object: none"));
       m_text_action_->SetLabel(wxString::FromAscii("action: none"));

   } else {

     ROS_ERROR("failed when calling arm_nav_repeat service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

     buttons_["repeat"]->Enable(true);

   }

   buttons_["plan"]->Enable(false);
   buttons_["execute"]->Enable(false);
   buttons_["reset"]->Enable(false);
   buttons_["play"]->Enable(false);
   if (wait_for_start_) setButton("new",false);
   else setButton("new",true);

   buttons_["failed"]->Enable(false);
   buttons_["success"]->Enable(false);
   buttons_["repeat"]->Enable(false);
   buttons_["switch"]->Enable(true);

}

bool CArmManipulationControls::InitCobScript() {

  if (cob_script==NULL) {

    return false;

      }

  if (!cob_script_inited) {

    if (!cob_script->waitForServer(ros::Duration(3.0))) {

        ROS_ERROR("No response from cob_script action server");
        return false;

      } else {

        cob_script_server::ScriptAction goal;

        goal.action_goal.goal.component_name = "sdh";
        goal.action_goal.goal.function_name = "init";
        goal.action_goal.goal.mode = "";
        goal.action_goal.goal.parameter_name = "";

        cob_script->sendGoal(goal.action_goal.goal);
        cob_script->waitForResult((ros::Duration(5.0)));

        if (cob_script->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) return false;

        goal.action_goal.goal.component_name = "torso";

        cob_script->sendGoal(goal.action_goal.goal);
        cob_script->waitForResult((ros::Duration(5.0)));

        if (cob_script->getState() != actionlib::SimpleClientGoalState::SUCCEEDED) return false;

        cob_script_inited = true;

      }

  }

  if (cob_script_inited) return true;
  else return false;

}

/// @todo Add GUI lock mutex!!!!!!!
void CArmManipulationControls::GripperThread(unsigned char action) {

  if (!InitCobScript()) {

      /// @todo manage this situation somehow
        return;

    }

    std::string status = "";


    cob_script_server::ScriptAction goal;

    goal.action_goal.goal.component_name = "sdh";
    goal.action_goal.goal.function_name = "move";
    goal.action_goal.goal.mode = "";

    if (action==G_OPEN) goal.action_goal.goal.parameter_name = "cylopen";
    else goal.action_goal.goal.parameter_name = "cylclosed";

    cob_script->sendGoal(goal.action_goal.goal);

    cob_script->waitForResult((ros::Duration(5.0)));

    if (cob_script->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {


      if (action==G_OPEN) {

          //m_text_status->SetLabel(wxString::FromAscii("status: Gripper opened."));
          status = "status: Gripper opened.";
          ROS_INFO("Gripper should be opened...");
      }
      else {

        //m_text_status->SetLabel(wxString::FromAscii("status: Gripper closed."));
        status = "status: Gripper closed.";
        ROS_INFO("Gripper should be closed...");

      }

    } else {

      //m_text_status->SetLabel(wxString::FromAscii("status: Error during gripper action."));
      status = "status: Error during gripper action.";
      ROS_ERROR("Error on opening/closing gripper.");

    }

    wxMutexGuiEnter();

    m_text_status->SetLabel(wxString::FromAscii(status.c_str()));
    buttons_["gripper_o"]->Enable(true);
    buttons_["gripper_c"]->Enable(true);
    buttons_["look_around"]->Enable(true);
    buttons_["refresh"]->Enable(true);

    wxMutexGuiLeave();



}

void CArmManipulationControls::OnGripperO(wxCommandEvent& event) {

  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

  // wait for some time
  if (t_gripper.timed_join(td)) {

    unsigned char action = G_OPEN;

    buttons_["gripper_o"]->Enable(false);
    buttons_["gripper_c"]->Enable(false);
    buttons_["look_around"]->Enable(false);
    buttons_["refresh"]->Enable(false);

    m_text_status->SetLabel(wxString::FromAscii("status: Opening gripper."));

    t_gripper = boost::thread(&CArmManipulationControls::GripperThread,this,action);

  } else ROS_INFO("We have to wait until gripper thread finishes.");

}

/// @todo Add guimutex!!!!!!!!!!
void CArmManipulationControls::LookThread() {

  if (!InitCobScript()) {

      /// @todo manage it
      return;

  }

  std::string status = "";

  cob_script_server::ScriptAction goal;

  goal.action_goal.goal.component_name = "torso";
  goal.action_goal.goal.function_name = "move";
  goal.action_goal.goal.mode = "";
  goal.action_goal.goal.parameter_name = "front_left";

  /// @todo Are the timeouts sufficient?
  cob_script->sendGoal(goal.action_goal.goal);
  cob_script->waitForResult((ros::Duration(5.0)));

  goal.action_goal.goal.parameter_name = "front";
  cob_script->sendGoal(goal.action_goal.goal);
  cob_script->waitForResult((ros::Duration(5.0)));

  goal.action_goal.goal.parameter_name = "front_right";
  cob_script->sendGoal(goal.action_goal.goal);
  cob_script->waitForResult((ros::Duration(5.0)));

  goal.action_goal.goal.parameter_name = "back_right";
  cob_script->sendGoal(goal.action_goal.goal);
  cob_script->waitForResult((ros::Duration(5.0)));

  goal.action_goal.goal.parameter_name = "back";
  cob_script->sendGoal(goal.action_goal.goal);
  cob_script->waitForResult((ros::Duration(5.0)));

  goal.action_goal.goal.parameter_name = "back_left";
  cob_script->sendGoal(goal.action_goal.goal);
  cob_script->waitForResult((ros::Duration(5.0)));

  goal.action_goal.goal.parameter_name = "home";
  cob_script->sendGoal(goal.action_goal.goal);
  cob_script->waitForResult((ros::Duration(5.0)));

  if (cob_script->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

    //m_text_status->SetLabel(wxString::FromAscii("status: Looking around completed."));
    status = "status: Looking around completed.";
    ROS_INFO("Looking around completed - collision map should be improved.");

  } else {

      //m_text_status->SetLabel(wxString::FromAscii("status: Looking around failed."));
      status = "status: Looking around failed.";
      ROS_INFO("Looking around failed.");

  }


  wxMutexGuiEnter();

  m_text_status->SetLabel(wxString::FromAscii(status.c_str()));

  buttons_["look_around"]->Enable(true);
  buttons_["gripper_o"]->Enable(true);
  buttons_["gripper_c"]->Enable(true);
  buttons_["refresh"]->Enable(true);

  wxMutexGuiLeave();

}

void CArmManipulationControls::OnLook(wxCommandEvent& event) {

  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

  // wait for some time
  if (t_look.timed_join(td)) {

    buttons_["gripper_o"]->Enable(false);
    buttons_["gripper_c"]->Enable(false);
    buttons_["look_around"]->Enable(false);
    buttons_["refresh"]->Enable(false);

    m_text_status->SetLabel(wxString::FromAscii("status: Looking around."));
    ROS_INFO("Looking around to improve collision map.");

    t_look = boost::thread(&CArmManipulationControls::LookThread,this);

  } else ROS_INFO("We have to wait until LOOKAROUND thread finishes.");

}

/// @todo Add another thread - sometimes it takes a lot of time.
void CArmManipulationControls::OnRefresh(wxCommandEvent& event) {

  ROS_INFO("Refreshing planning scene");

   srs_assisted_arm_navigation::ArmNavRefresh srv;

   if ( ros::service::exists(SRV_REFRESH,true) && ros::service::call(SRV_REFRESH,srv) ) {

     if (srv.response.completed) {

       m_text_status->SetLabel(wxString::FromAscii("status: Refreshed."));

     } else {

       m_text_status->SetLabel(wxString::FromAscii("status: Error on refreshing."));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_refresh service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }

}

void CArmManipulationControls::OnGripperC(wxCommandEvent& event) {

  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

  // wait for some time
  if (t_gripper.timed_join(td)) {

    unsigned char action = G_CLOSE;

    buttons_["gripper_o"]->Enable(false);
    buttons_["gripper_c"]->Enable(false);
    buttons_["look_around"]->Enable(false);
    buttons_["refresh"]->Enable(false);

    m_text_status->SetLabel(wxString::FromAscii("status: Closing gripper."));

    t_gripper = boost::thread(&CArmManipulationControls::GripperThread,this,action);

  } else ROS_INFO("We have to wait until gripper thread finishes.");

}

bool CArmManipulationControls::nav_start(srs_assisted_arm_navigation::ArmNavStart::Request &req, srs_assisted_arm_navigation::ArmNavStart::Response &res) {

  char str[120];

  action_ = req.action;
  object_name_ = req.object_name;
  allow_repeat_ = req.allow_repeat;


  if (req.object_name!="") {

    snprintf(str,120,"%s (%s)",req.action.c_str(),req.object_name.c_str());

    std::string tmp;
    tmp = std::string("object_name: ") + object_name_;
    m_text_object->SetLabel(wxString::FromAscii(tmp.c_str()));

  } else {

	  snprintf(str,120,"%s",req.action.c_str());

	  std::string tmp;
	  tmp = std::string("object_name: none");
	  m_text_object->SetLabel(wxString::FromAscii(tmp.c_str()));

  }

  std::string tmp;
  tmp = std::string("action: ") + req.action;
  m_text_action_->SetLabel(wxString::FromAscii(tmp.c_str()));


  wxMessageBox(wxString::FromAscii(str), wxString::FromAscii("Manual arm navigation"), wxOK, parent_,-1,-1);

  setButton("new",true);


  res.completed = true;

  return true;

};


void CArmManipulationControls::OnStepBack(wxCommandEvent& event) {

  ROS_INFO("Undoing last change in IM pose");

   srs_assisted_arm_navigation::ArmNavStep srv;

   srv.request.undo = true;

   if ( ros::service::exists(SRV_STEP,true) && ros::service::call(SRV_STEP,srv) ) {

     if (srv.response.completed) {

       m_text_status->SetLabel(wxString::FromAscii("status: Undo was successful."));

     } else {

       m_text_status->SetLabel(wxString::FromAscii("status: Error on undoing."));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_step service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }

}



void CArmManipulationControls::OnMoveRel(wxCommandEvent& event) {

  int id = event.GetId();

  float step = 0.01; // 10 mm

  geometry_msgs::Point move;

  move.x = 0.0;
  move.y = 0.0;
  move.z = 0.0;

  srs_assisted_arm_navigation::ArmNavMovePalmLinkRel srv;


  switch (id) {

    case ID_BUTTON_MLEFT:    move.y -= step; break;
    case ID_BUTTON_MRIGHT:   move.y += step; break;

    case ID_BUTTON_MDOWN:    move.z -= step; break;
    case ID_BUTTON_MUP:      move.z += step; break;

    case ID_BUTTON_MBACK:    move.x -= step; break;
    case ID_BUTTON_MFORW:    move.x += step; break;

  }

  srv.request.relative_shift = move;

  if ( ros::service::exists(SRV_MOVE_PALM_LINK_REL,true) && ros::service::call(SRV_MOVE_PALM_LINK_REL,srv) ) {

      if (srv.response.completed) {

        m_text_status->SetLabel(wxString::FromAscii("status: Gripper shifted."));

      } else {

        m_text_status->SetLabel(wxString::FromAscii("status: Error on shifting IM."));

      }

    } else {

      ROS_ERROR("failed when calling %s service",SRV_MOVE_PALM_LINK_REL.c_str());
      m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

    }


}


void CArmManipulationControls::OnStopTraj(wxCommandEvent& event) {

  ROS_INFO("Stopping execution of trajectory");

   srs_assisted_arm_navigation::ArmNavStop srv;

   if ( ros::service::exists(SRV_STOP,true) && ros::service::call(SRV_STOP,srv) ) {

       m_text_status->SetLabel(wxString::FromAscii("status: Execution canceled."));

   } else {

     ROS_ERROR("failed when calling arm_nav_stop service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }


}

void CArmManipulationControls::OnLockCmap(wxCommandEvent& event) {

	srs_env_model::LockCollisionMap srv;

	if (cmap_locked_) {

		ROS_INFO("Unlocking collision map");
		srv.request.lock = 0;

	} else {

		ROS_INFO("Locking collision map");
		srv.request.lock = 1;

	}

	if ( ros::service::exists(srs_env_model::LockCMap_SRV,true) && ros::service::call(srs_env_model::LockCMap_SRV,srv) ) {

	       if (cmap_locked_) {

	    	   m_text_status->SetLabel(wxString::FromAscii("status: Coll. map unlocked."));
	    	   cmap_locked_ = false;
	    	   m_lock_cmap_->SetValue(false);


	       } else {

	    	   m_text_status->SetLabel(wxString::FromAscii("status: Coll. map locked."));
	    	   cmap_locked_ = true;
	    	   m_lock_cmap_->SetValue(true);

	       }

	   } else {

	     ROS_ERROR("failed when calling coll. map lock service");
	     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

	     m_lock_cmap_->SetValue(cmap_locked_);

	   }



}


///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////


BEGIN_EVENT_TABLE(CArmManipulationControls, wxPanel)
    EVT_BUTTON(ID_BUTTON_NEW,  CArmManipulationControls::OnNew)
    EVT_BUTTON(ID_BUTTON_PLAN,  CArmManipulationControls::OnPlan)
    EVT_BUTTON(ID_BUTTON_PLAY,  CArmManipulationControls::OnPlay)
    EVT_BUTTON(ID_BUTTON_EXECUTE,  CArmManipulationControls::OnExecute)
    EVT_BUTTON(ID_BUTTON_RESET,  CArmManipulationControls::OnReset)
    EVT_BUTTON(ID_BUTTON_SUCCESS,  CArmManipulationControls::OnSuccess)
    EVT_BUTTON(ID_BUTTON_FAILED,  CArmManipulationControls::OnFailed)
    EVT_BUTTON(ID_BUTTON_GRIPPER_O,  CArmManipulationControls::OnGripperO)
    EVT_BUTTON(ID_BUTTON_GRIPPER_C,  CArmManipulationControls::OnGripperC)
    EVT_BUTTON(ID_BUTTON_LOOK,  CArmManipulationControls::OnLook)
    EVT_BUTTON(ID_BUTTON_REFRESH,  CArmManipulationControls::OnRefresh)
    EVT_BUTTON(ID_BUTTON_SWITCH,  CArmManipulationControls::OnSwitch)
    EVT_BUTTON(ID_BUTTON_REPEAT,  CArmManipulationControls::OnRepeat)

    EVT_BUTTON(ID_BUTTON_UNDO,  CArmManipulationControls::OnStepBack)

    EVT_BUTTON(ID_BUTTON_MLEFT,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MRIGHT,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MUP,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MDOWN,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MFORW,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MBACK,  CArmManipulationControls::OnMoveRel)

    EVT_BUTTON(ID_BUTTON_STOP_TRAJ,  CArmManipulationControls::OnStopTraj)
    //EVT_BUTTON(ID_BUTTON_LOCK_CMAP,  CArmManipulationControls::OnLockCmap)

END_EVENT_TABLE()

