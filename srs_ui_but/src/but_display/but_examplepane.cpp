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

#include "but_examplepane.h"
#include <rviz/window_manager_interface.h>
#include <srs_env_model/ResetOctomap.h>
#include <ros/service.h>


/**
 Constructor
 */
CExamplePanel::CExamplePanel(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi)
: wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
, m_wmi( wmi )
{

    // Connect event handler
    Connect(wxID_EXIT, wxEVT_COMMAND_BUTTON_CLICKED,
            wxCommandEventHandler(CExamplePanel::OnQuit));

    wxButton *button = new wxButton(this, wxID_EXIT, wxT("Quit"));

    wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(sizer);

    sizer->Add(button, wxID_EXIT, wxALIGN_RIGHT);

    sizer->SetSizeHints(this);

    // Set pane position
    Centre();
}

///////////////////////////////////////////////////////////////////////////////

/**
    On quit command event handler
*/
void CExamplePanel::OnQuit(wxCommandEvent& event)
{
    std::cerr << "Close called..." << std::endl;
    if( m_wmi != 0 )
        m_wmi->closePane( this );

    Close(true);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

/**
 Constructor
 */
CExampleDialog::CExampleDialog(wxWindow *parent, const wxString& title)
: wxDialog( parent, wxID_ANY, title, wxDefaultPosition, wxSize(280, 180))

{

    // Connect event handler
    Connect(wxID_EXIT, wxEVT_COMMAND_BUTTON_CLICKED,
            wxCommandEventHandler(CExampleDialog::OnQuit));

    wxButton *button = new wxButton(this, wxID_EXIT, wxT("Quit"));

    wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(sizer);

    sizer->Add(button, wxID_EXIT, wxALIGN_RIGHT);

    sizer->SetSizeHints(this);

    // Set pane position
    Centre();
}

///////////////////////////////////////////////////////////////////////////////

/**
    On quit command event handler
*/
void CExampleDialog::OnQuit(wxCommandEvent& event)
{
    std::cerr << "Close called..." << std::endl;
    Close(true);
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

const int ID_CHECKBOX(100);
const int ID_RESET_BUTTON(101);

/**
 Constructor
 */
CExamplePanelControls::CExamplePanelControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi )
{
    // Create controls
    m_button = new wxButton(this, ID_RESET_BUTTON, wxT("Reset map"));

    m_chkb = new wxCheckBox(this, ID_CHECKBOX, wxT("Chceckbox sample"),
                            wxPoint(20, 20));
    m_chkb->SetValue( true );

    // Create layout
    wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(sizer);
    wxSizer *hsizer = new wxBoxSizer(wxHORIZONTAL);

    sizer->Add(m_button, ID_RESET_BUTTON, wxALIGN_RIGHT);
    sizer->Add(hsizer, 0, wxALIGN_LEFT);

    hsizer->Add(m_chkb);

    sizer->SetSizeHints(this);

}
///////////////////////////////////////////////////////////////////////////////

/**
    On checkbox toggle
   */
void CExamplePanelControls::OnChckToggle(wxCommandEvent& event)
{
    if( m_chkb->GetValue() )
        std::cerr << "Chcekbox toggle ON" << std::endl;
    else
        std::cerr << "Chcekbox toggle OFF" << std::endl;
}

///////////////////////////////////////////////////////////////////////////////

/**
    On quit command event handler
*/
void CExamplePanelControls::OnReset(wxCommandEvent& event)
{
    srs_env_model::ResetOctomap reset;

    if( ros::service::call("reset_octomap", reset) )
    {
        std::cerr << "Reseting octomap..." << std::endl;
    }
}

///////////////////////////////////////////////////////////////////////////////
BEGIN_EVENT_TABLE(CExamplePanelControls, wxPanel)
    EVT_BUTTON(ID_RESET_BUTTON,  CExamplePanelControls::OnReset)
    EVT_CHECKBOX(ID_CHECKBOX, CExamplePanelControls::OnChckToggle )
END_EVENT_TABLE()
