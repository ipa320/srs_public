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

//const int ID_BUTTON_LIFT(118);
//const int ID_BUTTON_MOVE_TO_HOLD(119);


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

const int ID_CHOICE(175);

/**
 Constructor
 */
CArmManipulationControls::CArmManipulationControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi )
{


    parent_ = parent;
    
    traj_executed_ = 0;

    ros::param::param<bool>("~wait_for_start", wait_for_start_ , WAIT_FOR_START);

    ros::NodeHandle nh("~");

    XmlRpc::XmlRpcValue pres;

        if (nh.getParam("arm_nav_presets",pres)) {

        	ROS_ASSERT(pres.getType() == XmlRpc::XmlRpcValue::TypeArray);

        	ROS_INFO("%d presets for assisted arm navigation plugin.",pres.size());

        	for (int i=0; i < pres.size(); i++) {

        		Preset pr;

        		XmlRpc::XmlRpcValue xpr = pres[i];

    			if (xpr.getType() != XmlRpc::XmlRpcValue::TypeStruct) {

    			  ROS_ERROR("Wrong syntax in YAML config.");
    			  continue;

    			}

    			// read the name
    			if (!xpr.hasMember("name")) {

    			  ROS_ERROR("Preset doesn't have 'name' property defined.");
    			  continue;

    			} else {

    			  XmlRpc::XmlRpcValue name = xpr["name"];

    			  std::string tmp = static_cast<std::string>(name);

    			  pr.name = tmp;

    			}

    			// read the positions
    			if (!xpr.hasMember("position")) {

    			  ROS_ERROR("Preset doesn't have 'position' property defined.");
    			  continue;

    			} else {

    			  XmlRpc::XmlRpcValue pos = xpr["position"];

    			  if (pos.getType() == XmlRpc::XmlRpcValue::TypeArray) {

    				  if (pos.size() != 3) {

    					  ROS_ERROR("Strange number of positions (%d)",pos.size());
    					  continue;

    				  }


					  XmlRpc::XmlRpcValue p = pos[0];
					  double tmp = static_cast<double>(p);
					  pr.pose.position.x = tmp;

					  p = pos[1];
					  tmp = static_cast<double>(p);
					  pr.pose.position.y = tmp;

					  p = pos[2];
					  tmp = static_cast<double>(p);
					  pr.pose.position.z = tmp;

    			  } else {

    				  ROS_ERROR("Property 'forces' is defined in bad way.");
    				  continue;

    			  }

    			} // else forces


    			// read the orientation
    			if (!xpr.hasMember("orientation")) {

    			  ROS_ERROR("Preset doesn't have 'orientation' property defined.");
    			  continue;

    			} else {

    			  XmlRpc::XmlRpcValue ori = xpr["orientation"];

    			  if (ori.getType() == XmlRpc::XmlRpcValue::TypeArray) {

    				  // RPY
    				  if (ori.size() == 3) {

    					  geometry_msgs::Vector3 rpy;

						  XmlRpc::XmlRpcValue p = ori[0];
						  double tmp = static_cast<double>(p);

						  rpy.x = tmp;

						  p = ori[1];
						  tmp = static_cast<double>(p);
						  rpy.y = tmp;

						  p = ori[2];
						  tmp = static_cast<double>(p);
						  rpy.z = tmp;

    					  pr.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(rpy.x,rpy.y,rpy.z);


    				  } else if (ori.size() == 4) {



						  XmlRpc::XmlRpcValue p = ori[0];
						  double tmp = static_cast<double>(p);
						  pr.pose.orientation.x = tmp;

						  p = ori[1];
						  tmp = static_cast<double>(p);
						  pr.pose.orientation.y = tmp;

						  p = ori[2];
						  tmp = static_cast<double>(p);
						  pr.pose.orientation.z = tmp;

						  p = ori[3];
						  tmp = static_cast<double>(p);
						  pr.pose.orientation.w = tmp;

    				  } else {

    					  ROS_ERROR("Prop 'orientation' should have 3 or 4 values (%d).",ori.size());
    					  continue;

    				  }





    			  } else {

    				  ROS_ERROR("Property 'target_pos' is defined in bad way.");
    				  continue;

    			  }

    			} // else target_pos

    			// read the name
				if (!xpr.hasMember("relative")) {

				  ROS_ERROR("Preset doesn't have 'relative' property defined.");
				  continue;

				} else {

				  XmlRpc::XmlRpcValue rel = xpr["relative"];

				  bool tmp = static_cast<bool>(rel);

				  pr.relative = tmp;

				}


    			presets_.push_back(pr);

        	} // for around all presets

        } else {

			ROS_ERROR("Can't get presets for predefined IM positions.");

		}

	wxArrayString prt;

	prt.Add(wxString::FromAscii("none"));

	for(unsigned int i=0; i < presets_.size(); i++) {

		std::string tmp;

		tmp = presets_[i].name;

		if (presets_[i].relative) tmp += " (R)";
		else tmp += " (A)";

		prt.Add(wxString::FromAscii(tmp.c_str()));

	}

	presets_choice_ = new wxChoice(this, ID_CHOICE,wxDefaultPosition,wxDefaultSize,prt);

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

    //buttons_["lift"] = new wxButton(this, ID_BUTTON_LIFT, wxT("Lift the object"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    //setButton("lift",false);

    //buttons_["hold"] = new wxButton(this, ID_BUTTON_MOVE_TO_HOLD, wxT("Move to hold pos."),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    //setButton("hold",false);

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

    m_text_predef_pos_ = new wxStaticText(this, -1, wxT("Move to predefined position"));

    //m_text_object = new wxStaticText(this, -1, wxT("object: none"));
    m_text_action_ = new wxStaticText(this, -1, wxT("Current task:"));
    m_text_task_ = new wxTextCtrl(this,-1,wxT("None."),wxDefaultPosition,wxDefaultSize,wxTE_MULTILINE|wxTE_READONLY);
    m_text_task_->SetSizeHints(200,100);


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
    vsizer_mes->Add(m_text_task_);

    //vsizer_add->Add(b_switch_);
    vsizer_add->Add(m_aco_);
    vsizer_add->Add(buttons_["undo"]);
    /*vsizer_add->Add(buttons_["lift"]);
    vsizer_add->Add(buttons_["hold"]);*/
    vsizer_add->Add(m_text_predef_pos_);
    vsizer_add->Add(presets_choice_);


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

    //ros::NodeHandle nh;

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

	if (arm_nav_called_) {

		setButton("new",true); // if action was called, we should allow new try

		if (allow_repeat_) setButton("repeat",true);
		else setButton("repeat",false);

		if (traj_executed_ > 0) {

			setButton("success",true); // allow success only if at least one trajectory was executed
		}

	} else {

		setButton("success",false);
		setButton("failed",false);
		setButton("repeat",false);

	}

	setButton("plan",false);
	setButton("execute",false);
	m_aco_->Enable(true);
	setButton("stop",false);
	setButton("undo",false);
	presets_choice_->Enable(false);
	presets_choice_->Select(0);

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
	/*setButton("lift",false);
	setButton("hold",false);*/
	presets_choice_->Enable(false);

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
     /*setButton("lift",true);
     setButton("hold",true);*/
     presets_choice_->Enable(true);


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
	   m_text_status->SetLabel(wxString::FromAscii("status: Refreshed. Please wait."));
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

		disableControls();

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
       traj_executed_++;

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
   //m_text_action_->SetLabel(wxString::FromAscii("action: none"));
   setTask("None.");


   arm_nav_called_ = false;
   setControlsToDefaultState();

}

void CArmManipulationControls::setTask(std::string text) {

	m_text_task_->Clear();
	m_text_task_->AppendText(wxString::FromAscii(text.c_str()));

}

void CArmManipulationControls::OnFailed(wxCommandEvent& event)
{
   ROS_ERROR("Manual arm manipulation task failed");

   ArmNavFailed srv;

   if ( ros::service::exists(SRV_FAILED,true) && ros::service::call(SRV_FAILED,srv) ) {

       m_text_status->SetLabel(wxString::FromAscii("status: Failed :-("));
       //m_text_object->SetLabel(wxString::FromAscii("object: none"));
       //m_text_action_->SetLabel(wxString::FromAscii("action: none"));
       setTask("None.");

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
       //m_text_action_->SetLabel(wxString::FromAscii("action: none"));
       setTask("None.");

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
  traj_executed_ = 0;

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
  //m_text_action_->SetLabel(wxString::FromAscii(tmp.c_str()));
  setTask(req.action);


  wxMessageBox(wxString::FromAscii(str), wxString::FromAscii("Assisted arm navigation"), wxOK, parent_,-1,-1);

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

// to be called from GUI callbacks only
bool CArmManipulationControls::checkService(std::string srv) {

	if (!ros::service::exists(srv,true)) {

		ROS_ERROR("Service %s is not ready.",srv.c_str());
		m_text_status->SetLabel(wxString::FromAscii("status: communication error"));
		setControlsToDefaultState();
		return false;

	} else return true;

}

void CArmManipulationControls::OnChoice(wxCommandEvent& event) {

	unsigned int choice;
	if (presets_choice_->GetSelection() > 0) choice = (presets_choice_->GetSelection()) - 1;
	else return;

	if (presets_[choice].relative) {

		if (!checkService(SRV_MOVE_PALM_LINK_REL)) return;

		boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

		  /// wait for some time
		  if (t_move_rel.timed_join(td)) {

		    disableControls();
		    m_text_status->SetLabel(wxT("status: Moving to predefined position."));

		    t_move_rel = boost::thread(&CArmManipulationControls::MoveRel,this);

		  } else ROS_DEBUG("We have to wait until MOVE_REL thread finishes.");

		//MoveRel(choice);

	} else {


		if (!checkService(SRV_MOVE_PALM_LINK)) return;

		boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

		  /// wait for some time
		  if (t_move_abs.timed_join(td)) {

			disableControls();
			m_text_status->SetLabel(wxT("status: Moving to predefined position."));

			t_move_abs = boost::thread(&CArmManipulationControls::MoveAbs,this);

		  } else ROS_DEBUG("We have to wait until MOVE_ABS thread finishes.");

		//MoveAbs(choice);

	}


}

void CArmManipulationControls::MoveAbs() {


	unsigned int choice;
	wxMutexGuiEnter();
	if (presets_choice_->GetSelection() > 0) choice = (presets_choice_->GetSelection()) - 1;
	else {

		wxMutexGuiLeave();
		return;
	}
	wxMutexGuiLeave();

	geometry_msgs::PoseStamped npose;

	npose.header.stamp = ros::Time::now();
	npose.header.frame_id = "/base_link"; // TODO read it from param !!!!!!!!!!!!!!!!!!!!

	npose.pose = presets_[choice].pose;

  ArmNavMovePalmLink srv;

  srv.request.sdh_palm_link_pose = npose;

  bool ret = false;;

  if ( ros::service::call(SRV_MOVE_PALM_LINK,srv) ) {

	  if (!srv.response.completed) ret = false;
	  else ret = true;

	} else {

		ROS_ERROR("failed when called %s service",SRV_MOVE_PALM_LINK.c_str());
		ret = false;

	}


  if (!ret) {

	  wxMutexGuiEnter();
	  m_text_status->SetLabel(wxString::FromAscii("status: Error on moving IM to new position."));
	  setControlsToDefaultState();
	  wxMutexGuiLeave();
	  return;

  } else {

	  wxMutexGuiEnter();
	  m_text_status->SetLabel(wxString::FromAscii("status: Gripper is at new position."));

	  setButton("new",true); // cancel
	  setButton("plan",true);
	  setButton("undo",true);

	  presets_choice_->Enable(true);
	  presets_choice_->Select(0);

	  if (arm_nav_called_) {

		 if (allow_repeat_) buttons_["repeat"]->Enable(true);
		 buttons_["failed"]->Enable(true);

	   }

	  wxMutexGuiLeave();
	  return;


  }

}

void CArmManipulationControls::MoveRel() {

	unsigned int choice;
	wxMutexGuiEnter();
	if (presets_choice_->GetSelection() > 0) choice = (presets_choice_->GetSelection()) - 1;
	else {

		wxMutexGuiLeave();
		return;
	}
	wxMutexGuiLeave();

  geometry_msgs::Pose move;
  move = presets_[choice].pose;

  ArmNavMovePalmLinkRel srv;

  srv.request.relative_movement = move;

  bool ret = false;
  bool err = false;

  if ( ros::service::call(SRV_MOVE_PALM_LINK_REL,srv) ) {

	  if (srv.response.completed) ret = true;
	  else ret = false;

  } else {

	  ROS_ERROR("failed when called %s service",SRV_MOVE_PALM_LINK_REL.c_str());
	  err = true;

	}


  if (!ret || err) {

	  wxMutexGuiEnter();

	  if (err) {

		  m_text_status->SetLabel(wxString::FromAscii("status: Comunnication error."));
		  setControlsToDefaultState();

	  }

	  if (!ret) {

		  m_text_status->SetLabel(wxString::FromAscii("status: Error on moving IM to new position."));

		  setButton("new",true); // cancel
		  setButton("plan",true);
		  setButton("undo",true);

		  presets_choice_->Enable(true);
		  presets_choice_->Select(0);

		  if (arm_nav_called_) {

			 if (allow_repeat_) buttons_["repeat"]->Enable(true);
			 buttons_["failed"]->Enable(true);

		   }

	  }

	  wxMutexGuiLeave();
	  return;

  } else {

	  wxMutexGuiEnter();
	  m_text_status->SetLabel(wxString::FromAscii("status: Gripper is at new position."));

	  setButton("new",true); // cancel
	  setButton("plan",true);
	  setButton("undo",true);

	  presets_choice_->Enable(true);
	  presets_choice_->Select(0);

	  if (arm_nav_called_) {

		 if (allow_repeat_) buttons_["repeat"]->Enable(true);
		 buttons_["failed"]->Enable(true);

	   }

	  wxMutexGuiLeave();
	  return;

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
    /*EVT_BUTTON(ID_BUTTON_LIFT,  CArmManipulationControls::OnLift)
    EVT_BUTTON(ID_BUTTON_MOVE_TO_HOLD,  CArmManipulationControls::OnHold)*/
    EVT_CHOICE(ID_CHOICE,CArmManipulationControls::OnChoice)
END_EVENT_TABLE()

