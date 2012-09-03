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

#include <srs_assisted_arm_navigation/rviz_plugins/grasping_controls.h>

#include <rviz/window_manager_interface.h>

using namespace std;
using namespace srs_assisted_arm_navigation;

const int ID_BUTTON_GRASP(101);
const int ID_BUTTON_STOP(102);
//const int ID_BUTTON_OPEN(103);
const int ID_SLIDER_FORCE(104);

const int ID_SLIDER_FORCE_1(105);
const int ID_SLIDER_FORCE_2(106);
const int ID_SLIDER_FORCE_3(107);

const int ID_RADIO_GRASP_TYPE(108);
const int ID_RADIO_PRESET(109);
const int ID_CHECK_TWOF(110);
const int ID_CHECK_OPF(111);

/**
 Constructor
 */
CButGraspingControls::CButGraspingControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi )
{

    parent_ = parent;

    ros::param::param<double>("~abs_max_force", abs_max_force_ , ABS_MAX_FORCE);
    ros::param::param<bool>("~wait_for_allow", wait_for_allow_ , WAIT_FOR_ALLOW);

    //buttons_["open"] = new wxButton(this, ID_BUTTON_OPEN, wxT("Open"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["grasp"] = new wxButton(this, ID_BUTTON_GRASP, wxT("Grasp"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    buttons_["stop"] = new wxButton(this, ID_BUTTON_STOP, wxT("Stop"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    m_text_status_ = new wxStaticText(this, -1, wxT("status: idle"));

    stringstream ss (stringstream::in | stringstream::out);

    ss << "max. force: ";
    ss << abs_max_force_/2.0;

    string tmp = ss.str();

    m_text_max_force_ = new wxStaticText(this, -1, wxString::FromAscii(tmp.c_str()));

    max_force_slider_ = new wxSlider(this,ID_SLIDER_FORCE,abs_max_force_/2.0,0,abs_max_force_,wxDefaultPosition,wxDefaultSize,wxSL_LABELS);

    finger1_force_slider_ = new wxSlider(this,ID_SLIDER_FORCE_1,0.0,0,abs_max_force_);
    finger2_force_slider_ = new wxSlider(this,ID_SLIDER_FORCE_2,0.0,0,abs_max_force_);
    finger3_force_slider_ = new wxSlider(this,ID_SLIDER_FORCE_3,0.0,0,abs_max_force_);

    wxSizer *hsizer_buttons = new wxBoxSizer(wxHORIZONTAL);

    wxArrayString grasp_types;

    grasp_types.Add(wxT("Round"));
    grasp_types.Add(wxT("Square"));
    //grasp_types.Add(wxT("Cylindric"));

    grasp_type_ = new wxRadioBox(this,ID_RADIO_GRASP_TYPE,wxT("Grasp type"),wxDefaultPosition,wxDefaultSize,grasp_types,3,wxRA_SPECIFY_COLS);

    wxArrayString force_presets;

    force_presets.Add(wxT("None"));
    force_presets.Add(wxT("Paper box"));
    force_presets.Add(wxT("Bottle"));
    force_presets.Add(wxT("Book"));

    preset_ = new wxRadioBox(this,ID_RADIO_PRESET,wxT("Presets"),wxDefaultPosition,wxDefaultSize,force_presets,4,wxRA_SPECIFY_COLS);

    two_fingers_contact_ = new wxCheckBox(this,ID_CHECK_TWOF,wxT("Two fingers contact"));

    do_not_open_fingers_ = new wxCheckBox(this,ID_CHECK_OPF,wxT("Do not open fingers"));

    hsizer_buttons->Add(buttons_["grasp"], ID_BUTTON_GRASP);
    hsizer_buttons->Add(buttons_["stop"], ID_BUTTON_STOP);

    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL); // top sizer

    vsizer->Add(hsizer_buttons);
    vsizer->Add(preset_);
    vsizer->Add(grasp_type_);
    vsizer->Add(two_fingers_contact_);
    vsizer->Add(do_not_open_fingers_);
    vsizer->Add(m_text_status_);
    vsizer->Add(m_text_max_force_);

    wxSizer *vsizer_sliders = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Force sliders"));

    vsizer_sliders->Add(max_force_slider_,1,wxEXPAND);

    vsizer_sliders->Add(finger1_force_slider_,1,wxEXPAND);
    vsizer_sliders->Add(finger2_force_slider_,1,wxEXPAND);
    vsizer_sliders->Add(finger3_force_slider_,1,wxEXPAND);

    vsizer->Add(vsizer_sliders,1,wxEXPAND);



    finger1_force_slider_->Enable(false);
    finger2_force_slider_->Enable(false);
    finger3_force_slider_->Enable(false);

    setButton("stop",false);

    grasping_allowed_ = false;

    if (wait_for_allow_) {

    	DisableControls();

    } else {

    	EnableControls();
    }

    Connect(ID_SLIDER_FORCE, wxEVT_SCROLL_THUMBRELEASE,
              wxCommandEventHandler(CButGraspingControls::OnMaxForceSlider));

    vsizer->SetSizeHints(this);
    this->SetSizerAndFit(vsizer);

    as_client_ = new grasping_action_client(ACT_GRASP,true);

    ros::NodeHandle nh;

    service_grasping_allow_ = nh.advertiseService(SRV_ALLOW,&CButGraspingControls::GraspingAllow,this);


}

void CButGraspingControls::setButton(string but, bool state) {

  if (buttons_[but]!=NULL)
    buttons_[but]->Enable(state);

}


CButGraspingControls::~CButGraspingControls() {


  ButtonsMap::iterator it;

    for (it = buttons_.begin(); it != buttons_.end(); ++it)
      delete it->second;

    buttons_.clear();

    delete as_client_;

}


void CButGraspingControls::GraspingThread() {

  ROS_INFO("Let's start grasping...");

  if (!as_client_->waitForServer(ros::Duration(3.0))) {

    ROS_ERROR("Grasping action server not running!");

    wxMutexGuiEnter();
    m_text_status_->SetLabel(wxString::FromAscii("status: Communication error."));

    EnableControls();

    wxMutexGuiLeave();

    return;

  }

  ManualGraspingGoal goal;

  grasping_finished_ = false;

  goal.max_force = (float)max_force_slider_->GetValue();
  goal.grasp_type = (uint8_t)grasp_type_->GetSelection();
  goal.accept_two_fingers_contact = two_fingers_contact_->GetValue();
  goal.do_not_open_fingers = do_not_open_fingers_->GetValue();

  as_client_->sendGoal(goal,
		  boost::bind(&CButGraspingControls::GraspingDone, this, _1, _2),
		  boost::bind(&CButGraspingControls::GraspingActive, this),
		  boost::bind(&CButGraspingControls::GraspingFeedback, this, _1));

  // TODO timeout!!!!!!!!!!!!!! treat case if action fails.....

  uint16_t i=0;

  ros::Duration dur(1);

  // wait for 60 seconds
  while( (i < 60) && (!grasping_finished_) ) {

	  dur.sleep();

  }

  if (!grasping_finished_) {

	  ROS_ERROR("Manual grasping action timeouted.");

	  wxMutexGuiEnter();
	  m_text_status_->SetLabel(wxString::FromAscii("status: Timeout."));

	  setButton("stop",false);

	  if (grasping_allowed_ || !wait_for_allow_) {

		  EnableControls();

	  }

	  wxMutexGuiLeave();

	  as_client_->cancelAllGoals();

  }

  return;


}

void CButGraspingControls::GraspingFeedback(const ManualGraspingFeedbackConstPtr& feedback) {

	static bool feedback_received = false;

	if (!feedback_received) {

		feedback_received = true;
		ROS_INFO("Grasping action feedback received");

	}

	wxMutexGuiEnter();

	finger1_force_slider_->SetValue((unsigned int)feedback->tip1_force);
	finger2_force_slider_->SetValue((unsigned int)feedback->tip2_force);
	finger3_force_slider_->SetValue((unsigned int)feedback->tip3_force);

	wxMutexGuiLeave();

}

void CButGraspingControls::GraspingDone(const actionlib::SimpleClientGoalState& state,
					  const ManualGraspingResultConstPtr& result) {

	ROS_INFO("Grasping action finished...");

	grasping_finished_ = true;

	wxMutexGuiEnter();

	  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {


	    ROS_INFO("Object should be now grasped.");
	    m_text_status_->SetLabel(wxString::FromAscii("status: Grasping completed."));

	  } else {

	    ROS_ERROR("Grasping failed.");
	    m_text_status_->SetLabel(wxString::FromAscii("status: Grasping failed."));

	  }

	  if (grasping_allowed_ || !wait_for_allow_) {

		  EnableControls();

	      }

	  setButton("stop",false);

	  finger1_force_slider_->SetValue(0);
	  finger2_force_slider_->SetValue(0);
	  finger3_force_slider_->SetValue(0);

	  wxMutexGuiLeave();

}

void CButGraspingControls::OnStop(wxCommandEvent& event) {

  as_client_->cancelAllGoals();

  if (grasping_allowed_ || !wait_for_allow_) {

	  EnableControls();

  } else {

	  DisableControls();

  }

}

void CButGraspingControls::EnableControls() {

	setButton("grasp",true);
	max_force_slider_->Enable(true);
	grasp_type_->Enable(true);
	preset_->Enable(true);
	two_fingers_contact_->Enable(true);
	do_not_open_fingers_->Enable(true);

}

void CButGraspingControls::DisableControls(bool state_of_stop_button) {

	setButton("grasp",false);
	max_force_slider_->Enable(false);
	grasp_type_->Enable(false);
	setButton("stop",state_of_stop_button);
	preset_->Enable(false);
	two_fingers_contact_->Enable(false);
	do_not_open_fingers_->Enable(false);

}

void CButGraspingControls::OnGrasp(wxCommandEvent& event) {

  boost::posix_time::time_duration td = boost::posix_time::milliseconds(100);

  /// wait for some time
  if (t_grasping.timed_join(td)) {

    m_text_status_->SetLabel(wxString::FromAscii("status: Trying to grasp. Please wait..."));

    DisableControls(true);

    t_grasping = boost::thread(&CButGraspingControls::GraspingThread,this);

  } else ROS_INFO("We have to wait until GRASPING thread finishes.");


}

void CButGraspingControls::OnMaxForceSlider(wxCommandEvent& event) {

  stringstream ss (stringstream::in | stringstream::out);

  ss << "max. force: ";
  ss << (double)max_force_slider_->GetValue();

  string tmp = ss.str();

  m_text_max_force_->SetLabel(wxString::FromAscii(tmp.c_str()));

  preset_->Select(0);

}

bool CButGraspingControls::GraspingAllow(GraspingAllow::Request &req, GraspingAllow::Response &res) {

	if (req.allow) {

		EnableControls();
		grasping_allowed_ = true;

	} else {

		DisableControls();
		grasping_allowed_ = false;

	}

	return true;

}

void CButGraspingControls::OnGraspType(wxCommandEvent& event) {

	preset_->Select(0);

}


void CButGraspingControls::OnPreset(wxCommandEvent& event) {

	int value = preset_->GetSelection();

	if (value == 0) return;

	/* force_presets.Add(wxT("Paper box"));
	    force_presets.Add(wxT("Bottle"));
	    force_presets.Add(wxT("Heavy object"));*/

	switch(value) {

		case 1: { // paper box

			max_force_slider_->SetValue((int)(abs_max_force_*0.3));
			grasp_type_->Select(1); // square

		} break;

		case 2: { // bottle

			max_force_slider_->SetValue((int)(abs_max_force_*0.5));
			grasp_type_->Select(0); // round


		} break;

		case 3: { // book

			max_force_slider_->SetValue((int)(abs_max_force_*0.75));
			grasp_type_->Select(1); // square


		} break;




	} // switch


	stringstream ss (stringstream::in | stringstream::out);

	ss << "max. force: ";
	ss << max_force_slider_->GetValue();

	string tmp = ss.str();
	m_text_max_force_->SetLabel(wxString::FromAscii(tmp.c_str()));

}




BEGIN_EVENT_TABLE(CButGraspingControls, wxPanel)
    EVT_BUTTON(ID_BUTTON_GRASP,  CButGraspingControls::OnGrasp)
    EVT_BUTTON(ID_BUTTON_STOP,  CButGraspingControls::OnStop)
    EVT_RADIOBOX(ID_RADIO_GRASP_TYPE, CButGraspingControls::OnGraspType)
    EVT_RADIOBOX(ID_RADIO_PRESET, CButGraspingControls::OnPreset)
END_EVENT_TABLE()

