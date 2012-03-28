/**
 * $Id: but_examplepane.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

// TODO pridat do ArmNavNew jaky je timeout, zobrazovat ubihajici cas v RVIZu
// TODO vraceni ramena do vychozi polohy pri kliku na reset ????
// TODO hledani nejblizsi pregrasp pos.
// TODO dat do odpovedi ID pozice
// TODO messagebox pri vyprseni timeoutu

#include "but_arm_manipulation.h"
#include <rviz/window_manager_interface.h>

using namespace std;

const int ID_BUTTON_NEW(101);
const int ID_BUTTON_PLAN(102);
const int ID_BUTTON_PLAY(103);
const int ID_BUTTON_EXECUTE(104);
const int ID_BUTTON_RESET(105);
const int ID_BUTTON_SUCCESS(106);
const int ID_BUTTON_FAILED(107);
const int ID_BUTTON_AUTOADJ(108);
const int ID_BUTTON_GRIPPER_O(109);
const int ID_BUTTON_GRIPPER_C(110);

CArmManipulationControls::~CArmManipulationControls() {



}

/**
 Constructor
 */
CArmManipulationControls::CArmManipulationControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi )
{
    // Create controls
    //m_button = new wxButton(this, ID_RESET_BUTTON, wxT("Reset map"));

    parent_ = parent;
    
    m_button_new = new wxButton(this, ID_BUTTON_NEW, wxT("New"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    m_button_plan = new wxButton(this, ID_BUTTON_PLAN, wxT("Plan"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    m_button_play = new wxButton(this, ID_BUTTON_PLAY, wxT("Play"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    m_button_execute = new wxButton(this, ID_BUTTON_EXECUTE, wxT("Execute"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    m_button_reset = new wxButton(this, ID_BUTTON_RESET, wxT("Reset"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    m_button_autoadj = new wxButton(this, ID_BUTTON_AUTOADJ, wxT("Adjust"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    m_button_gripper_o = new wxButton(this, ID_BUTTON_GRIPPER_O, wxT("Open gripper"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);
    m_button_gripper_c = new wxButton(this, ID_BUTTON_GRIPPER_C, wxT("Close gripper"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    m_button_success = new wxButton(this, ID_BUTTON_SUCCESS, wxT("Success"));
    m_button_failed = new wxButton(this, ID_BUTTON_FAILED, wxT("Failed"));

    m_text_status = new wxStaticText(this, -1, wxT("status: waiting"));
    m_text_object = new wxStaticText(this, -1, wxT("object: none"));
    m_text_timeout = new wxStaticText(this, -1, wxT("timeout: none"));
    m_text_dist = new wxStaticText(this, -1, wxT("closest pos.: none"));

    m_button_new->Enable(false);
    m_button_plan->Enable(false);
    m_button_play->Enable(false);
    m_button_execute->Enable(false);
    m_button_reset->Enable(false);

    m_button_success->Enable(false);
    m_button_failed->Enable(false);

    m_button_autoadj->Enable(false);
    m_button_gripper_o->Enable(false);
    m_button_gripper_c->Enable(false);

    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL); // top sizer


    wxSizer *vsizer_top = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Trajectory planning"));
    wxSizer *hsizer_traj_top = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_mid = new wxBoxSizer(wxHORIZONTAL);
    wxSizer *hsizer_traj_bot = new wxBoxSizer(wxHORIZONTAL);


    wxSizer *vsizer_mes = new wxStaticBoxSizer(wxVERTICAL,this,wxT("Messages"));

    wxSizer *hsizer_add = new wxStaticBoxSizer(wxHORIZONTAL,this,wxT("Additional controls"));


    /* Trajectory planning related buttons, on top*/
    hsizer_traj_top->Add(m_button_new, ID_BUTTON_NEW);
    hsizer_traj_top->Add(m_button_plan, ID_BUTTON_PLAN);
    hsizer_traj_top->Add(m_button_execute, ID_BUTTON_EXECUTE);

    hsizer_traj_mid->Add(m_button_play, ID_BUTTON_PLAY);
    hsizer_traj_mid->Add(m_button_autoadj, ID_BUTTON_AUTOADJ);
    hsizer_traj_mid->Add(m_button_reset, ID_BUTTON_RESET);

    hsizer_traj_bot->Add(m_button_success, ID_BUTTON_SUCCESS);
    hsizer_traj_bot->Add(m_button_failed, ID_BUTTON_FAILED);

    vsizer_top->Add(hsizer_traj_top,1,wxEXPAND);
    vsizer_top->Add(hsizer_traj_mid,1,wxEXPAND);
    vsizer_top->Add(hsizer_traj_bot,1,wxEXPAND);


    /* Status messages*/
    vsizer_mes->Add(m_text_status);
    vsizer_mes->Add(m_text_object);
    vsizer_mes->Add(m_text_timeout);
    vsizer_mes->Add(m_text_dist);

    hsizer_add->Add(m_button_gripper_o);
    hsizer_add->Add(m_button_gripper_c);

    vsizer->Add(vsizer_top,0,wxEXPAND);
    vsizer->Add(hsizer_add,0,wxEXPAND);
    vsizer->Add(vsizer_mes,0,wxEXPAND);

    // TODO: co s temi ID pozic???

    goal_pregrasp = false;
    goal_away = false;


    vsizer->SetSizeHints(this);
    this->SetSizerAndFit(vsizer);

    ros::NodeHandle nh;
    service_start_ = nh.advertiseService("arm_nav_start",&CArmManipulationControls::nav_start,this);

}
///////////////////////////////////////////////////////////////////////////////

void CArmManipulationControls::OnNew(wxCommandEvent& event)
{

   ROS_INFO("Request for new trajectory");

   srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_new",true) && ros::service::call("arm_nav_new",srv) ) {

     if (srv.response.completed) {

         m_button_new->Enable(false);
         m_button_plan->Enable(true);
         m_button_play->Enable(false);
         m_button_reset->Enable(true);
         m_button_success->Enable(false);
         m_button_failed->Enable(true);

         if (goal_pregrasp) {

           m_text_status->SetLabel(wxString::FromAscii("status: Move gripper to desired position"));

           /*std::string tmp;
           tmp = std::string("object_name: ") + object_name;
           m_text_object->SetLabel(wxString::FromAscii(tmp.c_str()));*/

         }
         if (goal_away) m_text_status->SetLabel(wxString::FromAscii("status: Move gripper to safe position"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_new service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }


}

void CArmManipulationControls::OnPlan(wxCommandEvent& event)
{

   ROS_INFO("Starting planning and filtering of new trajectory");

   srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_plan",true) && ros::service::call("arm_nav_plan",srv) ) {

     if (srv.response.completed) {

       m_button_new->Enable(false);
       m_button_plan->Enable(false);
       m_button_play->Enable(true);
       m_button_execute->Enable(true);
       //m_button_finish->Enable(false);
       m_button_success->Enable(false);
       m_button_failed->Enable(true);

       m_text_status->SetLabel(wxString::FromAscii("status: Trajectory is ready"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_plan service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }


}

void CArmManipulationControls::OnPlay(wxCommandEvent& event)
{

   ROS_INFO("Starting planning and filtering of new trajectory");

   srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_play",true) && ros::service::call("arm_nav_play",srv) ) {

     if (srv.response.completed) {

       m_button_new->Enable(false);
       m_button_plan->Enable(false);
       m_button_execute->Enable(true);
       //m_button_finish->Enable(false);
       m_button_success->Enable(false);
       m_button_failed->Enable(true);

       m_text_status->SetLabel(wxString::FromAscii("status: Playing trajectory..."));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_play service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

   }

}

void CArmManipulationControls::OnExecute(wxCommandEvent& event)
{
   ROS_INFO("Execution of planned trajectory has been started");

   srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_execute",true) && ros::service::call("arm_nav_execute",srv) ) {

     if (srv.response.completed) {

       m_button_plan->Enable(false);
       m_button_execute->Enable(false);
       m_button_reset->Enable(false);
       m_button_play->Enable(false);
       m_button_new->Enable(false);
       //m_button_finish->Enable(true);
       m_button_success->Enable(true);
       m_button_failed->Enable(true);

       m_text_status->SetLabel(wxString::FromAscii("status: Executing trajectory..."));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_execute service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

   }

}

void CArmManipulationControls::OnReset(wxCommandEvent& event)
{
   ROS_INFO("Reset planning stuff to initial state");

   srs_ui_but::ArmNavNew srv;

   if ( ros::service::exists("arm_nav_reset",true) && ros::service::call("arm_nav_reset",srv) ) {

     if (srv.response.completed) {

       m_button_plan->Enable(false);
       m_button_execute->Enable(false);
       m_button_reset->Enable(false);
       m_button_play->Enable(false);
       m_button_new->Enable(true);
       //m_button_finish->Enable(true);
       m_button_success->Enable(false);
       m_button_failed->Enable(true);

       // TODO: vratit rameno do puvodni polohy!!!!!

       m_text_status->SetLabel(wxString::FromAscii("status: Ok, try it again"));

     } else {

       m_text_status->SetLabel(wxString::FromAscii(srv.response.error.c_str()));

     }

   } else {

     ROS_ERROR("failed when calling arm_nav_reset service");
     m_text_status->SetLabel(wxString::FromAscii("status: Communication error"));

     m_button_plan->Enable(false);
     m_button_execute->Enable(false);
     m_button_reset->Enable(false);
     m_button_play->Enable(false);
     m_button_new->Enable(true);
     //m_button_finish->Enable(false);
     m_button_success->Enable(false);
     m_button_failed->Enable(true);

   }

}

void CArmManipulationControls::OnSuccess(wxCommandEvent& event)
{
   ROS_INFO("Finishing manual arm manipulation task");

   srs_ui_but::ArmNavSuccess srv;

   if ( ros::service::exists("arm_nav_success",true) && ros::service::call("arm_nav_success",srv) ) {

       m_button_plan->Enable(false);
       m_button_execute->Enable(false);
       m_button_reset->Enable(false);
       m_button_play->Enable(false);
       m_button_new->Enable(false);
       //m_button_finish->Enable(false);

       m_button_success->Enable(false);
       m_button_failed->Enable(false);

       m_text_status->SetLabel(wxString::FromAscii("status: Succeeded :-)"));
       m_text_object->SetLabel(wxString::FromAscii("object: none"));


   } else {

     ROS_ERROR("failed when calling arm_nav_success service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

     m_button_plan->Enable(false);
     m_button_execute->Enable(false);
     m_button_reset->Enable(false);
     //m_button_finish->Enable(false);
     m_button_success->Enable(false);
     m_button_failed->Enable(true);

     m_button_play->Enable(false);
     m_button_new->Enable(false);

   }

}

void CArmManipulationControls::OnFailed(wxCommandEvent& event)
{
   ROS_ERROR("Manual arm manipulation task failed");

   srs_ui_but::ArmNavFailed srv;

   if ( ros::service::exists("arm_nav_failed",true) && ros::service::call("arm_nav_failed",srv) ) {

       m_button_plan->Enable(false);
       m_button_execute->Enable(false);
       m_button_reset->Enable(false);
       m_button_play->Enable(false);
       m_button_new->Enable(false);
       //m_button_finish->Enable(false);

       m_button_success->Enable(false);
       m_button_failed->Enable(false);

       m_text_status->SetLabel(wxString::FromAscii("status: Failed :-("));
       m_text_object->SetLabel(wxString::FromAscii("object: none"));

   } else {

     ROS_ERROR("failed when calling arm_nav_failed service");
     m_text_status->SetLabel(wxString::FromAscii("Communication error"));

     m_button_plan->Enable(false);
     m_button_execute->Enable(false);
     m_button_reset->Enable(false);
     //m_button_finish->Enable(false);
     m_button_success->Enable(false);
     m_button_failed->Enable(true);

     m_button_play->Enable(false);
     m_button_new->Enable(false);

   }

}


bool CArmManipulationControls::nav_start(srs_ui_but::ArmNavStart::Request &req, srs_ui_but::ArmNavStart::Response &res) {


  if (req.pregrasp) {

    char str[80];

    snprintf(str,80,"Please navigate arm to pregrasp position for: %s",req.object_name.c_str());

    wxMessageBox(wxString::FromAscii(str), wxString::FromAscii("Manual arm navigation"), wxOK, parent_,-1,-1);


    goal_pregrasp = true;
    goal_away = false;
    object_name = req.object_name;

    std::string tmp;
    tmp = std::string("object_name: ") + object_name;
    m_text_object->SetLabel(wxString::FromAscii(tmp.c_str()));

  }

  if (req.away) {

      wxMessageBox(wxString::FromAscii("Please navigate arm to safe position"), wxString::FromAscii("Manual arm navigation"), wxOK, parent_,-1,-1);

      goal_pregrasp = false;
      goal_away = true;

    }

  m_button_new->Enable(true);
  res.completed = true;

  return true;

};

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
END_EVENT_TABLE()
