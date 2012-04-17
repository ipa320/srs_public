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
#include "srs_assisted_arm_navigation/ArmNavExecute.h"
#include "srs_assisted_arm_navigation/ArmNavNew.h"
#include "srs_assisted_arm_navigation/ArmNavPlan.h"
#include "srs_assisted_arm_navigation/ArmNavPlay.h"
#include "srs_assisted_arm_navigation/ArmNavReset.h"
#include "srs_assisted_arm_navigation/ArmNavStart.h"
#include "srs_assisted_arm_navigation/ArmNavSuccess.h"
#include "srs_assisted_arm_navigation/ArmNavFailed.h"
#include "srs_assisted_arm_navigation/ArmNavRefresh.h"

#include "cob_script_server/ScriptAction.h"
#include <actionlib/client/simple_action_client.h>

#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <map>

/// Prefix for services which are called from this RVIZ plugin.
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
#define SRV_REFRESH BUT_SERVICE("/arm_nav_refresh")

/** This parameter defines if it's possible to start arm planning at any time.
* When it's true, the NEW button is always active. It can be configured in launch file.
* If it's false, the NEW button becomes active only when new action is triggered.
*/
static const bool WAIT_FOR_START = true;

/**
 * Actionlib client for communication with cob script server (used to move torso, open/close gripper).
 */
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

    /// Callbacks of buttons.
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
    virtual void OnLook(wxCommandEvent& event);
    virtual void OnRefresh(wxCommandEvent& event);

    //void UpdateGui(wxCommandEvent &event);

    /// Initialization of cob script server
    bool InitCobScript();

    void OnSetText(wxCommandEvent & event);

    /// Callback for arm_nav_start service. This service can be used to inform user that his/her activity is required.
    bool nav_start(srs_assisted_arm_navigation::ArmNavStart::Request &req, srs_assisted_arm_navigation::ArmNavStart::Response &res);

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
    wxButton * m_button_look_around;
    wxButton * m_button_refresh;

    wxStaticText *m_text_status;
    wxStaticText *m_text_object;
    wxStaticText *m_text_timeout;
    wxStaticText *m_text_dist; // distance to closest pregrasp position

    ros::ServiceServer service_start_;
    ros::ServiceServer service_timeout_;

    wxWindow *parent_;

    bool goal_away;
    bool goal_pregrasp;

    bool wait_for_start_;

    std::string object_name;

    cob_client * cob_script;

    bool gripper_initialized;

private:

    DECLARE_EVENT_TABLE();

    enum {G_OPEN,G_CLOSE};

    //enum {S_INIT, S_NEW, S_PLAN, S_EXECUTE, S_PLAY, S_ADJUST, S_RESET, S_SUCCESS, S_FAILED, S_GOPEN, S_GCLOSE, S_LOOKAROUND, S_REFRESH};

    /** There are several thread which are started from button callbacks. Threads are used because calls to some services
     * may take a lot of time and if such service is called from button callback, the GUI of RVIZ is not responding.
     * @todo Add thread for refreshing planning scene (refresh button).
    */
    void GripperThread(unsigned char action);
    void NewThread();
    void PlanThread();
    void ExecuteThread();
    void LookThread();

    boost::thread t_gripper;
    boost::thread t_new;
    boost::thread t_plan;
    boost::thread t_execute;
    boost::thread t_look;

    bool cob_script_inited;

};



#endif // BUT_ARMNAVIGATION_H
