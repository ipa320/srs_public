/******************************************************************************
 * \file
 * \brief RVIZ plugin which provides simplified GUI based on Warehouse Viewer.
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

#pragma once
#ifndef BUT_GRASPING_H
#define BUT_GRASPING_H

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/radiobox.h>
#include <wx/checkbox.h>
#include <ros/ros.h>
#include <string.h>


#include "cob_script_server/ScriptAction.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <map>
#include <iostream>
#include <sstream>

#include "srs_assisted_arm_navigation/ManualGraspingAction.h"
#include "srs_assisted_arm_navigation/GraspingAllow.h"
#include "srs_assisted_arm_navigation/services_list.h"
#include "srs_assisted_arm_navigation/topics_list.h"

namespace rviz
{
    class WindowManagerInterface;
}

namespace srs_assisted_arm_navigation {

static const double ABS_MAX_FORCE = 500.0;
static const bool WAIT_FOR_ALLOW = true;


class CButGraspingControls : public wxPanel
{
public:
    /// Constructor
    CButGraspingControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );
    ~CButGraspingControls();

    void GraspingFeedback(const ManualGraspingFeedbackConstPtr& feedback);
    void GraspingActive() {};
    void GraspingDone(const actionlib::SimpleClientGoalState& state,
					  const ManualGraspingResultConstPtr& result);


    bool GraspingAllow(GraspingAllow::Request &req, GraspingAllow::Response &res);


protected:

    typedef actionlib::SimpleActionClient<ManualGraspingAction> grasping_action_client;

    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    typedef std::map<std::string, wxButton *> ButtonsMap;

    ButtonsMap buttons_;

    wxWindow *parent_;

    wxStaticText *m_text_status_;
    wxStaticText *m_text_max_force_;

    wxSlider *max_force_slider_;

    wxSlider *finger1_force_slider_;
    wxSlider *finger2_force_slider_;
    wxSlider *finger3_force_slider_;

    wxRadioBox *grasp_type_;
    wxRadioBox *preset_;

    wxCheckBox *two_fingers_contact_;
    wxCheckBox *do_not_open_fingers_;

    bool grasping_finished_;

    bool grasping_allowed_;

    void setButton(std::string but, bool state);

    void OnStop(wxCommandEvent& event);
    void OnGrasp(wxCommandEvent& event);
    void OnMaxForceSlider(wxCommandEvent& event);
    void OnGraspType(wxCommandEvent& event);
    void OnPreset(wxCommandEvent& event);

    void EnableControls(); // get ready for grasping
    void DisableControls(bool state_of_stop_button=false);


    bool wait_for_allow_;

    double abs_max_force_;

    grasping_action_client *as_client_;

    void GraspingThread();

    boost::thread t_grasping;

    ros::ServiceServer service_grasping_allow_;

private:

    DECLARE_EVENT_TABLE();


};

} // namespace

#endif // BUT_GRASPING_H
