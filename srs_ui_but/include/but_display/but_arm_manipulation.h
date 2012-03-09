/**
 * $Id: but_examplepane.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
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

private:
    DECLARE_EVENT_TABLE()

};

#endif // BUT_ARMNAVIGATION_H
