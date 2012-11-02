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
#ifndef BUT_ARMNAVIGATION_H
#define BUT_ARMNAVIGATION_H

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/tglbtn.h>
#include <wx/checkbox.h>

#include <ros/ros.h>
#include <string.h>

#include "srs_assisted_arm_navigation_msgs/ArmNavExecute.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavNew.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavPlan.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavPlay.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavReset.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavStart.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavSuccess.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavFailed.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavRefresh.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavSwitchAttCO.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavRepeat.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavStep.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavStop.h"
#include "srs_assisted_arm_navigation_msgs/ArmNavMovePalmLinkRel.h"
#include "srs_assisted_arm_navigation_msgs/AssistedArmNavigationState.h"

//#include "srs_env_model/LockCollisionMap.h"

#include "cob_script_server/ScriptAction.h"
#include <actionlib/client/simple_action_client.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <map>

#include "srs_assisted_arm_navigation/services_list.h"
#include "srs_env_model/services_list.h"

namespace rviz
{
    class WindowManagerInterface;
}

namespace srs_assisted_arm_navigation_ui {

/** This parameter defines if it's possible to start arm planning at any time.
* When it's true, the NEW button is always active. It can be configured in launch file.
* If it's false, the NEW button becomes active only when new action is triggered.
*/
static const bool WAIT_FOR_START = true;

/**
 * Actionlib client for communication with cob script server (used to move torso, open/close gripper).
 */
typedef actionlib::SimpleActionClient<cob_script_server::ScriptAction> cob_client;


class CArmManipulationControls : public wxPanel
{
public:
    /// Constructor
    CArmManipulationControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );
    ~CArmManipulationControls();

    /// Callbacks of buttons.
    virtual void OnNew(wxCommandEvent& event);
    virtual void OnPlan(wxCommandEvent& event);

    virtual void OnExecute(wxCommandEvent& event);

    //virtual void OnFinish(wxCommandEvent& event);
    virtual void OnSuccess(wxCommandEvent& event);
    virtual void OnFailed(wxCommandEvent& event);
    virtual void OnRepeat(wxCommandEvent& event);

    virtual void OnSwitch(wxCommandEvent& event);



    void OnStepBack(wxCommandEvent& event);

    void OnMoveRel(wxCommandEvent& event);

    void OnGrasp(wxCommandEvent& event);
    void OnStopGrasp(wxCommandEvent& event);

    void OnStopTraj(wxCommandEvent& event);

    void OnRoll(wxCommandEvent& event);
    void OnPitch(wxCommandEvent& event);
    void OnYaw(wxCommandEvent& event);

    void setControlsToDefaultState();
    void disableControls();

    bool refresh();


    void OnSetText(wxCommandEvent & event);

    /// Callback for arm_nav_start service. This service can be used to inform user that his/her activity is required.
    bool nav_start(srs_assisted_arm_navigation_msgs::ArmNavStart::Request &req, srs_assisted_arm_navigation_msgs::ArmNavStart::Response &res);

protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    typedef std::map<std::string, wxButton *> ButtonsMap;

    ButtonsMap buttons_;

    /*wxSlider * m_slider_roll;
    wxSlider * m_slider_pitch;
    wxSlider * m_slider_yaw;*/


    wxButton * m_button_switch;

    wxStaticText *m_text_status;
    wxStaticText *m_text_action_;
    //wxStaticText *m_text_object;
    //wxStaticText *m_text_timeout;
    //wxStaticText *m_text_dist; // distance to closest pregrasp position

    //wxToggleButton *m_lock_cmap_;

    ros::ServiceServer service_start_;
    ros::ServiceServer service_timeout_;

    wxWindow *parent_;

    /*bool goal_away;
    bool goal_pregrasp;*/

    bool wait_for_start_;

    bool allow_repeat_;
    //bool cmap_locked_;

    std::string object_name_;
    std::string action_;

    //cob_client * cob_script;

    //bool gripper_initialized;

    void setButton(std::string but, bool state);

    void stateCallback(const srs_assisted_arm_navigation_msgs::AssistedArmNavigationState::ConstPtr& msg);

private:

    DECLARE_EVENT_TABLE();

    enum {G_OPEN,G_CLOSE};

    //enum {S_INIT, S_NEW, S_PLAN, S_EXECUTE, S_PLAY, S_ADJUST, S_RESET, S_SUCCESS, S_FAILED, S_GOPEN, S_GCLOSE, S_LOOKAROUND, S_REFRESH};

    /** There are several thread which are started from button callbacks. Threads are used because calls to some services
     * may take a lot of time and if such service is called from button callback, the GUI of RVIZ is not responding.
     * @todo Add thread for refreshing planning scene (refresh button).
    */
    //void GripperThread(unsigned char action);
    void NewThread();
    void PlanThread();
    void ExecuteThread();
    //void LookThread();

    //boost::thread t_gripper;
    boost::thread t_new;
    boost::thread t_plan;
    boost::thread t_execute;
    //boost::thread t_look;

    //bool cob_script_inited;

    bool aco_;

    bool initialized_;

    bool state_received_;

    bool spacenav_pos_lock_;
    bool spacenav_or_lock_;

    ros::Subscriber arm_nav_state_sub_;

    wxCheckBox *m_pos_lock_;
    wxCheckBox *m_or_lock_;

    wxToggleButton *b_switch_;

    boost::thread t_gui_update;
    void GuiUpdateThread();
    bool stop_gui_thread_;

    bool planning_started_;
    bool trajectory_planned_;

    bool arm_nav_called_;

};

} // namespace

#endif // BUT_ARMNAVIGATION_H
