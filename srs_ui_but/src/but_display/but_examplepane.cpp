/******************************************************************************
 * \file
 *
 * $Id: but_examplepane.cpp 396 2012-03-29 12:24:03Z spanel $
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
