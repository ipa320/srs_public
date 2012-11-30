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

#include "srs_assisted_arm_navigation_ui/arm_navigation_controls.h"

#include <rviz/window_manager_interface.h>

using namespace std;
using namespace srs_assisted_arm_navigation_ui;
using namespace srs_assisted_arm_navigation;
using namespace srs_assisted_arm_navigation_msgs;

const int ID_BUTTON_NEW(101);
const int ID_BUTTON_PLAN(102);

const int ID_BUTTON_EXECUTE(104);
const int ID_BUTTON_RESET(105);
const int ID_BUTTON_SUCCESS(106);
const int ID_BUTTON_FAILED(107);


//const int ID_BUTTON_SWITCH(113);
const int ID_BUTTON_REPEAT(114);

const int ID_BUTTON_UNDO(115);

const int ID_BUTTON_LIFT(118);
const int ID_BUTTON_MOVE_TO_HOLD(119);


/*const int ID_BUTTON_MLEFT(117);
const int ID_BUTTON_MRIGHT(118);
const int ID_BUTTON_MUP(119);
const int ID_BUTTON_MDOWN(120);
const int ID_BUTTON_MFORW(121);
const int ID_BUTTON_MBACK(122);*/

const int ID_CHECKBOX_POS(125);
const int ID_CHECKBOX_OR(126);

const int ID_CHECKBOX_ACO(120);


const int ID_BUTTON_STOP_TRAJ(123);

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
    //b_switch_ = new wxToggleButton(this, ID_BUTTON_SWITCH, wxT("Avoid finger collisions"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["success"] = new wxButton(this, ID_BUTTON_SUCCESS, wxT("Task completed"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["failed"] = new wxButton(this, ID_BUTTON_FAILED, wxT("Cannot complete task"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["repeat"] = new wxButton(this, ID_BUTTON_REPEAT, wxT("Repeat detection"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    //---------------------------------------------------------------
    // end effector controls
    buttons_["undo"] = new wxButton(this, ID_BUTTON_UNDO, wxT("Undo marker movement"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
   /* buttons_["move_left"] = new wxButton(this, ID_BUTTON_MLEFT, wxT("L"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_right"] = new wxButton(this, ID_BUTTON_MRIGHT, wxT("R"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_up"] = new wxButton(this, ID_BUTTON_MUP, wxT("U"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_down"] = new wxButton(this, ID_BUTTON_MDOWN, wxT("D"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_forw"] = new wxButton(this, ID_BUTTON_MFORW, wxT("F"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["move_back"] = new wxButton(this, ID_BUTTON_MBACK, wxT("B"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);*/

    buttons_["new"] = new wxButton(this, ID_BUTTON_NEW, wxT("Start arm planning"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["plan"] = new wxButton(this, ID_BUTTON_PLAN, wxT("Simulate movement"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["execute"] = new wxButton(this, ID_BUTTON_EXECUTE, wxT("Execute"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    buttons_["lift"] = new wxButton(this, ID_BUTTON_LIFT, wxT("Lift the object"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    setButton("lift",false);

    buttons_["hold"] = new wxButton(this, ID_BUTTON_MOVE_TO_HOLD, wxT("Move to hold pos."),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    setButton("hold",false);

    buttons_["stop"]->SetForegroundColour (wxColour (255, 255, 255));
    buttons_["stop"]->SetBackgroundColour (wxColour (255, 108, 108));

    m_pos_lock_ = new wxCheckBox(this,ID_CHECKBOX_POS,wxT("position"),wxDefaultPosition,wxDefaultSize);
    m_or_lock_ = new wxCheckBox(this,ID_CHECKBOX_OR,wxT("orientation"),wxDefaultPosition,wxDefaultSize);

    m_aco_ = new wxCheckBox(this,ID_CHECKBOX_ACO,wxT("Avoid fingers collisions"),wxDefaultPosition,wxDefaultSize);
    // TODO read initial state of m_aco_ from (state) topic !!!

    m_aco_->Enable(false);
    m_aco_->Set3StateValue(wxCHK_UNDETERMINED);

    m_pos_lock_->Enable(false);
    m_or_lock_->Enable(false);

    m_pos_lock_->Set3StateValue(wxCHK_UNDETERMINED);
    m_or_lock_->Set3StateValue(wxCHK_UNDETERMINED);


    /*wxSizer *vsizer_endef = new wxStaticBoxSizer(wxVERTICAL,this,wxT("End effector controls"));
    wxSizer *hsizer_endef_top = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_endef_mid = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_endef_mid2 = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *vsizer_endef_bot = new wxBoxSizer(wxVERTICAL);*/

    //hsizer_endef_top->Add(buttons_["undo"], ID_BUTTON_UNDO);

    /*hsizer_endef_mid->Add(buttons_["move_left"], ID_BUTTON_MLEFT);
    hsizer_endef_mid->Add(buttons_["move_right"], ID_BUTTON_MRIGHT);
    hsizer_endef_mid->Add(buttons_["move_up"], ID_BUTTON_MUP);
    hsizer_endef_mid->Add(buttons_["move_down"], ID_BUTTON_MDOWN);
    hsizer_endef_mid->Add(buttons_["move_forw"], ID_BUTTON_MFORW);
    hsizer_endef_mid->Add(buttons_["move_back"], ID_BUTTON_MBACK);*/

    /*vsizer_endef->Add(hsizer_endef_top,1,wxEXPAND);
    vsizer_endef->Add(hsizer_endef_mid,1,wxEXPAND);
    vsizer_endef->Add(hsizer_endef_mid2,1,wxEXPAND);
    vsizer_endef->Add(vsizer_endef_bot,1,wxEXPAND);*/

    //---------------------------------------------------------------

    m_text_status = new wxStaticText(this, -1, wxT("status: not initialized yet."));

    //m_text_object = new wxStaticText(this, -1, wxT("object: none"));
    m_text_action_ = new wxStaticText(this, -1, wxT("action: none"));
    //m_text_timeout = new wxStaticText(this, -1, wxT("timeout: none"));

    disableControls();

    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL); // top sizer

    wxSizer *vsizer_top = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Trajectory planning"));

    wxSizer *vsizer_mes = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Task"));

    // -----------------
    wxSizer *vsizer_sn = new wxStaticBoxSizer(wxVERTICAL,this,wxT("SpaceNavigator locks"));
    wxSizer *hsizer_sn_in = new wxBoxSizer(wxHORIZONTAL);

    hsizer_sn_in->Add(m_pos_lock_,ID_CHECKBOX_POS);
    hsizer_sn_in->Add(m_or_lock_,ID_CHECKBOX_OR);

    vsizer_sn->Add(hsizer_sn_in,1,wxEXPAND);

    // -----------------

    wxSizer *vsizer_add = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Additional controls"));



    vsizer_top->Add(buttons_["new"]);
    vsizer_top->Add(buttons_["plan"]);
    vsizer_top->Add(buttons_["execute"]);
    vsizer_top->Add(buttons_["stop"]);
    vsizer_top->Add(m_text_status);





    vsizer_mes->Add(buttons_["success"]);
    vsizer_mes->Add(buttons_["repeat"]);
    vsizer_mes->Add(buttons_["failed"]);
    vsizer_mes->Add(m_text_action_);


    //vsizer_add->Add(b_switch_);
    vsizer_add->Add(m_aco_);
    vsizer_add->Add(buttons_["undo"]);
    vsizer_add->Add(buttons_["lift"]);
    vsizer_add->Add(buttons_["hold"]);


    vsizer->Add(vsizer_top,0,wxEXPAND|wxHORIZONTAL);
    vsizer->Add(vsizer_mes,0,wxEXPAND|wxHORIZONTAL);
    vsizer->Add(vsizer_add,0,wxEXPAND|wxHORIZONTAL);
    vsizer->Add(vsizer_sn,0,wxEXPAND|wxHORIZONTAL);

    allow_repeat_ = false;

    // TODO make it configurable ? - read same parameter from here and from but_arm_manip_node...
    aco_ = false;

    //b_switch_->SetValue(aco_);


    spacenav_pos_lock_ = false;
    spacenav_or_lock_ = false;

    /*if (aco_) buttons_["switch"]->SetLabel(wxT("ACO enabled"));
    else buttons_["switch"]->SetLabel(wxT("ACO disabled"));*/


    vsizer->SetSizeHints(this);
    this->SetSizerAndFit(vsizer);

    ros::NodeHandle nh;

    arm_nav_state_sub_ = nh.subscribe("/but_arm_manip/state",10, &CArmManipulationControls::stateCallback,this);

    initialized_ = false;
    state_received_ = false;
    stop_gui_thread_ = false;
    planning_started_ = false;
    trajectory_planned_ = false;
    arm_nav_called_ = false;

    t_gui_update = boost::thread(&CArmManipulationControls::GuiUpdateThread,this);

    /**
     * Service provided by plugin. It can be used to inform user that his/her action is required.
     */
    service_start_ = nh.advertiseService(SRV_START,&CArmManipulationControls::nav_start,this);

    //cob_script = new cob_client("/script_server",true);

    //cob_script_inited = false;



}
///////////////////////////////////////////////////////////////////////////////


void CArmManipulationControls::setControlsToDefaultState() {

	planning_started_ = false;

	buttons_["new"]->SetLabel(wxT("Start arm planning"));

	if (wait_for_start_) setButton("new",false);
	else setButton("new",true);

	setButton("plan",false);
	setButton("execute",false);
	//setButton("switch",true);
	//b_switch_->Enable(true);
	m_aco_->Enable(true);
	setButton("stop",false);
	setButton("undo",false);
	setButton("lift",false);
	setButton("hold",false);

	if (!arm_nav_called_) {

		setButton("success",false);
		setButton("failed",false);
		setButton("repeat",false);

	}

}

void CArmManipulationControls::disableControls() {

	setButton("new",false);

	setButton("plan",false);
	setButton("execute",false);
	//setButton("switch",false);
	m_aco_->Enable(false);

	setButton("success",false);
	setButton("failed",false);
	setButton("repeat",false);

	setButton("undo",false);

	setButton("stop",false);
	setButton("lift",false);
	setButton("hold",false);

}


void CArmManipulationControls::stateCallback(const AssistedArmNavigationState::ConstPtr& msg) {

	ROS_INFO_ONCE("State received.");

	if (!state_received_) {

		state_received_ = true;

		aco_ = msg->aco_state;

	}

	if ( (spacenav_pos_lock_ != msg->position_locked) || (spacenav_or_lock_ != msg->orientation_locked) ) {

		spacenav_pos_lock_ = msg->position_locked;
		spacenav_or_lock_ = msg->orientation_locked;

	}

}

void CArmManipulationControls::GuiUpdateThread() {

	ROS_INFO("Assisted arm nav. GUI thread started.");

	ros::Rate r(5);

	while(!stop_gui_thread_ && ros::ok()) {

		if ( (!initialized_) && state_received_) {

			wxMutexGuiEnter();

			if (aco_) m_aco_->Set3StateValue(wxCHK_CHECKED);
			else m_aco_->Set3StateValue(wxCHK_UNCHECKED);

			setControlsToDefaultState();

			/*if (wait_for_start_) m_text_status = new wxStaticText(this, -1, wxT("status: waiting for task"));
			else m_text_status = new wxStaticText(this, -1, wxT("status: let's start"));*/
			if (wait_for_start_) m_text_status->SetLabel(wxT("status: waiting for task"));
			else m_text_status->SetLabel(wxT("status: let's start"));

			wxMutexGuiLeave();

			ROS_INFO("Assisted arm nav. plugin initialized.");

			initialized_ = true;

		}

		if (planning_started_) {

			// update spacenav
			wxMutexGuiEnter();

			if (spacenav_pos_lock_) m_pos_lock_->Set3StateValue(wxCHK_CHECKED);
			else m_pos_lock_->Set3StateValue(wxCHK_UNCHECKED);

			if (spacenav_or_lock_) m_or_lock_->Set3StateValue(wxCHK_CHECKED);
			else m_or_lock_->Set3StateValue(wxCHK_UNCHECKED);

			wxMutexGuiLeave();

		}

		r.sleep();

	}

	ROS_INFO("Stopping assisted arm nav. GUI thread.");
	stop_gui_thread_ = false;

}


void CArmManipulationControls::setButton(string but, bool state) {

  if (buttons_[but]!=NULL)
    buttons_[but]->Enable(state);

}



CArmManipulationControls::~CArmManipulationControls() {

  stop_gui_thread_ = true;

  ROS_INFO("Waiting for threads to finish.");

  t_gui_update.join();

  //if (cob_script!=NULL) delete cob_script;

  ButtonsMap::iterator it;

  for (it = buttons_.begin(); it != buttons_.end(); ++it)
    delete it->second;

  buttons_.clear();

  delete m_aco_;
  delete m_text_status;
  //delete m_text_object;
  //delete m_text_timeout;


  ROS_INFO("Exiting...");

}

void CArmManipulationControls::NewThread() {


   // first, let's try to refresh planning scene
  if (!refresh()) {

	  setControlsToDefaultState();

	  return;

  }

  ROS_INFO("Request for new trajectory");

  ArmNavNew srv;
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

	 disableControls();

	 buttons_["new"]->SetLabel(wxT("Cancel planning"));
	 planning_started_ = true;

	 setButton("new",true);
	 setButton("plan",true);
     setButton("undo",true);
     setButton("lift",true);
     setButton("hold",true);


     if (arm_nav_called_) {

    	 if (allow_repeat_) buttons_["repeat"]->Enable(true);
    	 buttons_["failed"]->Enable(true);

     }


   } else {

	   // handle error on communication
	   setControlsToDefaultState();

   }

   wxMutexGuiLeave();

}


bool CArmManipulationControls::refresh() {

	ROS_INFO("Refreshing planning scene");

   ArmNavRefresh srv;

   if ( ros::service::exists(SRV_REFRESH,true) && ros::service::call(SRV_REFRESH,srv) ) {

	 if (srv.response.completed) {

	   wxMutexGuiEnter();
	   m_text_status->SetLabel(wxString::FromAscii("status: Refreshed."));
	   wxMutexGuiLeave();

	 } else {

	   wxMutexGuiEnter();
	   m_text_status->SetLabel(wxString::FromAscii("status: Error on refreshing."));
	   wxMutexGuiLeave();
	   return false;

	 }

   } else {

	 ROS_ERROR("failed when calling arm_nav_refresh service");
	 wxMutexGuiEnter();
	 m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));
	 wxMutexGuiLeave();
	 return false;

   }

  return true;

}

void CArmManipulationControls::OnHold(wxCommandEvent& event) {

	ROS_INFO("We will try to lift gripper a bit.");

	disableControls();

	geometry_msgs::PoseStamped npose;

	npose.header.stamp = ros::Time::now();
	npose.header.frame_id = "/base_link";

	// TODO read it from some parameter
  npose.pose.position.x = -0.223;
  npose.pose.position.y = 0.046;
  npose.pose.position.z = 0.920;

  npose.pose.orientation.x = 0.020;
  npose.pose.orientation.y = 0.707;
  npose.pose.orientation.z = -0.706;
  npose.pose.orientation.w = 0.033;

  ArmNavMovePalmLink srv;

  //srv.request.relative_shift = move;
  srv.request.sdh_palm_link_pose = npose;

  if ( ros::service::exists(SRV_MOVE_PALM_LINK,true) && ros::service::call(SRV_MOVE_PALM_LINK,srv) ) {

	  if (!srv.response.completed) {

		m_text_status->SetLabel(wxString::FromAscii("status: Error on moving to home pos."));
		setControlsToDefaultState();
		return;

	  }

	} else {

	  ROS_ERROR("failed when calling %s service",SRV_MOVE_PALM_LINK.c_str());
	  m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

	  setControlsToDefaultState();
	  return;

	}

  wxCommandEvent ev;

  // plan trajectory
  OnPlan(ev);


}

void CArmManipulationControls::OnLift(wxCommandEvent& event) {

	ROS_INFO("We will try to lift gripper a bit.");

	disableControls();

	geometry_msgs::Point move;

	move.x = 0.0;
	move.y = 0.0;
	move.z = 0.15;

  ArmNavMovePalmLinkRel srv;

  srv.request.relative_shift = move;

  if ( ros::service::exists(SRV_MOVE_PALM_LINK_REL,true) && ros::service::call(SRV_MOVE_PALM_LINK_REL,srv) ) {

	  if (!srv.response.completed) {

		m_text_status->SetLabel(wxString::FromAscii("status: Error on lift."));
		setControlsToDefaultState();
		return;

	  }

	} else {

	  ROS_ERROR("failed when calling %s service",SRV_MOVE_PALM_LINK_REL.c_str());
	  m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

	  setControlsToDefaultState();
	  return;

	}

  wxCommandEvent ev;

  // plan trajectory
  OnPlan(ev);

}

void CArmManipulationControls::OnNew(wxCommandEvent& event) {

	trajectory_planned_ = false;

	if (!planning_started_) {

		// planning was not started, so we want to start it

		  if (!ros::service::exists(SRV_NEW,true)) {

			m_text_status->SetLabel(wxString::FromAscii("status: communication error"));
			ROS_ERROR("Service %s is not ready.",((std::string)SRV_NEW).c_str());
			return;

		  }

		  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

		  /// wait for some time
		  if (t_new.timed_join(td)) {

			disableControls();

			m_text_status->SetLabel(wxString::FromAscii("status: Please wait..."));

			t_new = boost::thread(&CArmManipulationControls::NewThread,this);

		  } else ROS_INFO("We have to wait until new thread finishes.");

	} else {

	   //  planning has been already started, so we want to cancel it
	   ROS_INFO("Reset planning stuff to initial state");

	   ArmNavReset srv;

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

	   setControlsToDefaultState();

	   buttons_["new"]->SetLabel(wxT("Start arm planning"));
	   planning_started_ = false;

	}

}


void CArmManipulationControls::PlanThread() {

   ROS_INFO("Starting planning and filtering of new trajectory");

   std::string status = "";
   bool success = false;

   ArmNavPlan srv;

   if (ros::service::call(SRV_PLAN,srv) ) {

     success = true;

     if (srv.response.completed) {

    	 status = "status: Trajectory is ready";
    	 trajectory_planned_ = true;

     }
     else status = "status: " + srv.response.error;

   } else {

     success = false;
     ROS_ERROR("failed when calling service");
     status = "status: Communication error";

   }

   wxMutexGuiEnter();

   m_text_status->SetLabel(wxString::FromAscii(status.c_str()));

   if (success) {

	 disableControls();

     setButton("new",true);
     setButton("execute",true);
     setButton("plan",true);

     if (arm_nav_called_) {

		 if (allow_repeat_) buttons_["repeat"]->Enable(true);
		 buttons_["failed"]->Enable(true);

     }

   } else {

	 disableControls();
	 setButton("new",true);
	 setButton("plan",true);

	 if (arm_nav_called_) {

		 if (allow_repeat_) buttons_["repeat"]->Enable(true);
		 buttons_["failed"]->Enable(true);

	 }

   }

   wxMutexGuiLeave();

}

void CArmManipulationControls::OnPlan(wxCommandEvent& event)
{

  if (!trajectory_planned_) {

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

		t_plan = boost::thread(&CArmManipulationControls::PlanThread,this);

	  } else ROS_INFO("We have to wait until PLAN thread finishes.");

  } else {

	 ArmNavPlay srv;

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

  }

}



void CArmManipulationControls::OnSwitch(wxCommandEvent& event)
{

  if (aco_) ROS_INFO("Lets switch attached collision object OFF");
  else ROS_INFO("Lets switch attached collision object ON");

   ArmNavSwitchAttCO srv;

   srv.request.state = !aco_;

   if ( ros::service::exists(SRV_SWITCH,true) && ros::service::call(SRV_SWITCH,srv) ) {

     if (srv.response.completed) {

       //m_text_status->SetLabel(wxString::FromAscii("status: Playing trajectory..."));

       aco_ = !aco_;

       /*if (aco_) buttons_["switch"]->SetLabel(wxT("ACO enabled"));
       else buttons_["switch"]->SetLabel(wxT("ACO disabled"));*/

     } else {

       m_text_status->SetLabel(wxString::FromAscii("Can't switch state of ACO"));
       //b_switch_->SetValue(aco_);
       if (aco_) m_aco_->Set3StateValue(wxCHK_CHECKED);
       else m_aco_->Set3StateValue(wxCHK_UNCHECKED);

     }

   } else {

     std::string tmp = SRV_SWITCH;

     ROS_ERROR("failed when calling %s service",tmp.c_str());
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));
     //b_switch_->SetValue(aco_);
     if (aco_) m_aco_->Set3StateValue(wxCHK_CHECKED);
     else m_aco_->Set3StateValue(wxCHK_UNCHECKED);

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

    disableControls();
    setButton("stop",true);
    m_text_status->SetLabel(wxT("status: Executing trajectory..."));

    t_execute = boost::thread(&CArmManipulationControls::ExecuteThread,this);

  } else ROS_INFO("We have to wait until EXECUTE thread finishes.");

}

void CArmManipulationControls::ExecuteThread()
{
   ROS_INFO("Execution of planned trajectory has been started");

   ArmNavExecute srv;

   std::string status = "";
   bool success = false;

   if (ros::service::call(SRV_EXECUTE,srv) ) {

     if (srv.response.completed) {

       success = true;
       status = "status: Trajectory was executed.";

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

   //if (success) {

	 setControlsToDefaultState();
	 //setButton("stop",true);

	 if (arm_nav_called_) {

		 if (allow_repeat_) setButton("repeat",true);
		 setButton("failed",true);
		 setButton("success",true);

	 }


   //} else {



   //}

   buttons_["new"]->SetLabel(wxT("Start arm planning"));
   m_pos_lock_->Set3StateValue(wxCHK_UNDETERMINED);
   m_or_lock_->Set3StateValue(wxCHK_UNDETERMINED);

   wxMutexGuiLeave();

}


void CArmManipulationControls::OnSuccess(wxCommandEvent& event)
{
   ROS_INFO("Finishing manual arm manipulation task");

   ArmNavSuccess srv;

   if ( ros::service::exists(SRV_SUCCESS,true) && ros::service::call(SRV_SUCCESS,srv) ) {

       m_text_status->SetLabel(wxString::FromAscii("status: Succeeded :-)"));

   } else {

     ROS_ERROR("failed when calling arm_nav_success service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }


   //m_text_object->SetLabel(wxString::FromAscii("object: none"));
   m_text_action_->SetLabel(wxString::FromAscii("action: none"));

   arm_nav_called_ = false;
   setControlsToDefaultState();

}

void CArmManipulationControls::OnFailed(wxCommandEvent& event)
{
   ROS_ERROR("Manual arm manipulation task failed");

   ArmNavFailed srv;

   if ( ros::service::exists(SRV_FAILED,true) && ros::service::call(SRV_FAILED,srv) ) {

       m_text_status->SetLabel(wxString::FromAscii("status: Failed :-("));
       //m_text_object->SetLabel(wxString::FromAscii("object: none"));
       m_text_action_->SetLabel(wxString::FromAscii("action: none"));

   } else {

     ROS_ERROR("failed when calling arm_nav_failed service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }

   arm_nav_called_ = false;
   setControlsToDefaultState();

}

void CArmManipulationControls::OnRepeat(wxCommandEvent& event)
{
   ROS_ERROR("Request for repeat of manual arm navigation task");

   ArmNavRepeat srv;

   if ( ros::service::exists(SRV_REPEAT,true) && ros::service::call(SRV_REPEAT,srv) ) {

       m_text_status->SetLabel(wxString::FromAscii("status: Repeating action..."));
       //m_text_object->SetLabel(wxString::FromAscii("object: none"));
       m_text_action_->SetLabel(wxString::FromAscii("action: none"));

   } else {

     ROS_ERROR("failed when calling arm_nav_repeat service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }

   arm_nav_called_ = false;
   setControlsToDefaultState();

}


bool CArmManipulationControls::nav_start(ArmNavStart::Request &req, ArmNavStart::Response &res) {

  char str[120];

  action_ = req.action;
  object_name_ = req.object_name;
  allow_repeat_ = req.allow_repeat;

  if (allow_repeat_) ROS_INFO("Received request for action. Repeated detection is allowed.");
  else ROS_INFO("Received request for action. Repeated detection is NOT allowed.");


  if (req.object_name!="") {

    snprintf(str,120,"%s (%s)",req.action.c_str(),req.object_name.c_str());

    std::string tmp;
    tmp = std::string("object_name: ") + object_name_;
    //m_text_object->SetLabel(wxString::FromAscii(tmp.c_str()));

  } else {

	  snprintf(str,120,"%s",req.action.c_str());

	  std::string tmp;
	  tmp = std::string("object_name: none");
	  //m_text_object->SetLabel(wxString::FromAscii(tmp.c_str()));

  }

  std::string tmp;
  tmp = std::string("action: ") + req.action;
  m_text_action_->SetLabel(wxString::FromAscii(tmp.c_str()));


  wxMessageBox(wxString::FromAscii(str), wxString::FromAscii("Manual arm navigation"), wxOK, parent_,-1,-1);

  setButton("new",true);
  arm_nav_called_ = true;


  res.completed = true;

  return true;

};


void CArmManipulationControls::OnStepBack(wxCommandEvent& event) {

  ROS_INFO("Undoing last change in IM pose");

  setButton("undo",false);

   ArmNavStep srv;

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

   setButton("undo",true);

}



/*void CArmManipulationControls::OnMoveRel(wxCommandEvent& event) {

  int id = event.GetId();

  float step = 0.01; // 10 mm

  geometry_msgs::Point move;

  move.x = 0.0;
  move.y = 0.0;
  move.z = 0.0;

  ArmNavMovePalmLinkRel srv;


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


}*/


void CArmManipulationControls::OnStopTraj(wxCommandEvent& event) {

  ROS_INFO("Stopping execution of trajectory");

   ArmNavStop srv;

   if ( ros::service::exists(SRV_STOP,true) && ros::service::call(SRV_STOP,srv) ) {

       m_text_status->SetLabel(wxString::FromAscii("status: Execution canceled."));

   } else {

     ROS_ERROR("failed when calling arm_nav_stop service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }

   m_pos_lock_->Set3StateValue(wxCHK_UNDETERMINED);
   m_or_lock_->Set3StateValue(wxCHK_UNDETERMINED);

   //planning_started_ = false;

   setControlsToDefaultState();

   if (arm_nav_called_) {

   		 if (allow_repeat_) buttons_["repeat"]->Enable(true);
   		 buttons_["failed"]->Enable(true);

   	 }

}




///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////


BEGIN_EVENT_TABLE(CArmManipulationControls, wxPanel)
    EVT_BUTTON(ID_BUTTON_NEW,  CArmManipulationControls::OnNew)
    EVT_BUTTON(ID_BUTTON_PLAN,  CArmManipulationControls::OnPlan)
    EVT_BUTTON(ID_BUTTON_EXECUTE,  CArmManipulationControls::OnExecute)
    EVT_BUTTON(ID_BUTTON_SUCCESS,  CArmManipulationControls::OnSuccess)
    EVT_BUTTON(ID_BUTTON_FAILED,  CArmManipulationControls::OnFailed)
    //EVT_TOGGLEBUTTON(ID_BUTTON_SWITCH,  CArmManipulationControls::OnSwitch)
    EVT_CHECKBOX(ID_CHECKBOX_ACO,CArmManipulationControls::OnSwitch)
    EVT_BUTTON(ID_BUTTON_REPEAT,  CArmManipulationControls::OnRepeat)
    EVT_BUTTON(ID_BUTTON_UNDO,  CArmManipulationControls::OnStepBack)
    /*EVT_BUTTON(ID_BUTTON_MLEFT,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MRIGHT,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MUP,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MDOWN,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MFORW,  CArmManipulationControls::OnMoveRel)
    EVT_BUTTON(ID_BUTTON_MBACK,  CArmManipulationControls::OnMoveRel)*/
    EVT_BUTTON(ID_BUTTON_STOP_TRAJ,  CArmManipulationControls::OnStopTraj)
    EVT_BUTTON(ID_BUTTON_LIFT,  CArmManipulationControls::OnLift)
    EVT_BUTTON(ID_BUTTON_MOVE_TO_HOLD,  CArmManipulationControls::OnHold)
END_EVENT_TABLE()

