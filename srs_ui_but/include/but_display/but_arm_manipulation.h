/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2011
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

#ifndef BUT_ARMNAVIGATION_H
#define BUT_ARMNAVIGATION_H

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <ros/ros.h>
#include <string.h>
#include "ArmNavExecute.h"
#include "ArmNavNew.h"
#include "ArmNavPlan.h"
#include "ArmNavPlay.h"
#include "ArmNavReset.h"
#include "ArmNavStart.h"
//#include "ArmNavFinish.h"
#include "ArmNavSuccess.h"
#include "ArmNavFailed.h"

#include "cob_script_server/ScriptAction.h"
#include <actionlib/client/simple_action_client.h>

#define BUT_PREFIX std::string("/but_arm_manip")

#define BUT_SERVICE(topic) BUT_PREFIX + std::string(topic)
#define BUT_TOPIC(topic) BUT_PREFIX + std::string(topic)

#define SRV_START BUT_SERVICE("/arm_nav_start")
#define SRV_NEW BUT_SERVICE("/arm_nav_new")
#define SRV_PLAN BUT_SERVICE("/arm_nav_plan")
#define SRV_PLAY BUT_SERVICE("/arm_nav_play")
#define SRV_EXECUTE BUT_SERVICE("/arm_nav_execute")
#define SRV_RESET BUT_SERVICE("/arm_nav_reset")
#define SRV_SUCCESS BUT_SERVICE("/arm_nav_success")
#define SRV_FAILED BUT_SERVICE("/arm_nav_failed")

typedef actionlib::SimpleActionClient<cob_script_server::ScriptAction> cob_client;

namespace rviz
{
    class WindowManagerInterface;
}

class CArmManipulationControls : public wxPanel
{
public:
    /// Constructor
    CArmManipulationControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );
    ~CArmManipulationControls();

    virtual void OnNew(wxCommandEvent& event);
    virtual void OnPlan(wxCommandEvent& event);
    virtual void OnPlay(wxCommandEvent& event);
    virtual void OnExecute(wxCommandEvent& event);
    virtual void OnReset(wxCommandEvent& event);
    //virtual void OnFinish(wxCommandEvent& event);
    virtual void OnSuccess(wxCommandEvent& event);
    virtual void OnFailed(wxCommandEvent& event);

    virtual void OnGripperO(wxCommandEvent& event);
    virtual void OnGripperC(wxCommandEvent& event);

    void OnSetText(wxCommandEvent & event);
    bool nav_start(srs_ui_but::ArmNavStart::Request &req, srs_ui_but::ArmNavStart::Response &res);

protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    //! Button - new trajectory
    wxButton * m_button_new;
    
    //! Button - plan and filter trajectory
    wxButton * m_button_plan;
    
    wxButton * m_button_play;

    //! Button - execute trajectory plan on robot
    wxButton * m_button_execute;
    
    //! Button - reset planning
    wxButton * m_button_reset;
    
    wxButton * m_button_success;
    wxButton * m_button_failed;

    wxButton * m_button_autoadj;
    wxButton * m_button_gripper_o;
    wxButton * m_button_gripper_c;

    wxStaticText *m_text_status;
    wxStaticText *m_text_object;
    wxStaticText *m_text_timeout;
    wxStaticText *m_text_dist; // distance to closest pregrasp position

    ros::ServiceServer service_start_;
    ros::ServiceServer service_timeout_;

    wxWindow *parent_;

    bool goal_away;
    bool goal_pregrasp;

    std::string object_name;

    cob_client * cob_script;

    bool gripper_initialized;

private:
    DECLARE_EVENT_TABLE()

};

#endif // BUT_ARMNAVIGATION_H
