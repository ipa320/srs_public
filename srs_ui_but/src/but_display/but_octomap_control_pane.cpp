/******************************************************************************
 * \file
 *
 * $Id$
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

#include "but_octomap_control_pane.h"
#include <rviz/window_manager_interface.h>
#include <rviz/visualization_manager.h>
#include <srs_env_model/ResetOctomap.h>
#include <ros/service.h>

#define GIZMO_NAME "OctomapClearingBoxGizmo"


///////////////////////////////////////////////////////////////////////////////

const int ID_RESET_BUTTON(101);
const int ID_CLEARBOX_BUTTON(102);
const int ID_MAPCLEAR_BUTTON(103);


/**
 Constructor
 */
COctomapControlPane::COctomapControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi )
{
    // Create controls
    m_buttonReset = new wxButton(this, ID_RESET_BUTTON, wxT("Reset map"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonClearBoxAdd = new wxButton( this, ID_CLEARBOX_BUTTON, wxT("Add box"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonClearMap = new wxButton( this, ID_MAPCLEAR_BUTTON, wxT("Clear map"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);

    // Create layout
    wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(sizer);

    wxSizer *hsizer1 = new wxBoxSizer(wxHORIZONTAL);
    hsizer1->Add(m_buttonReset, ID_RESET_BUTTON, wxALIGN_LEFT);

    wxSizer *hsizer2 = new wxBoxSizer(wxHORIZONTAL);
    hsizer2->Add(m_buttonClearBoxAdd, ID_CLEARBOX_BUTTON, wxALIGN_LEFT);
    hsizer2->Add(m_buttonClearMap, ID_MAPCLEAR_BUTTON, wxALIGN_LEFT);
    m_buttonClearMap->Enable(false);

    sizer->Add(hsizer1, 0, wxALIGN_LEFT);
    sizer->Add(hsizer2, 0, wxALIGN_LEFT);

    sizer->SetSizeHints(this);

    // Create server
    m_server.reset(new InteractiveMarkerServer("server_name", "", false));

    // Create gizmo
    m_uoGizmo = new but_interaction_primitives::UnknownObject(m_server, "/world", GIZMO_NAME);
}


///////////////////////////////////////////////////////////////////////////////

/**
    On quit command event handler
*/
void COctomapControlPane::OnReset(wxCommandEvent& event)
{
    srs_env_model::ResetOctomap reset;

    if( ros::service::call("reset_octomap", reset) )
    {
        std::cerr << "Reseting octomap..." << std::endl;
    }
}

/**
 * On create clearing box event handler
 */
void COctomapControlPane::OnClearingBox(wxCommandEvent &event)
{
	std::cerr << "Add clearing box to the scene." << std::endl;

	addGizmo();

	m_buttonClearMap->Enable(true);
}

/**
 * On clear map event handler
 */

void COctomapControlPane::OnClearMap( wxCommandEvent &event )
{
	std::cerr << "Clear map event" << std::endl;

	// Read gizmo position and size

	// remove gizmo
	removeGizmo();

	// Disable button
	m_buttonClearMap->Enable(false);
}

/**
 * Add gizmo to the scene
 */
void COctomapControlPane::addGizmo()
{
	// Remove old gizmo
//	removeGizmo();

	// Positioning
	geometry_msgs::Pose p;
	p.position.x = p.position.y = p.position.z = 1.0;
	p.orientation.x = 0.0; p.orientation.y = p.orientation.z = p.orientation.w = 0.6;

	// Scaling
	but_interaction_primitives::Scale s;
	s.x = s.y = s.z = 10.0;

	// Set position
	m_uoGizmo->setPose(p);

	// Set size
	m_uoGizmo->setScale(s);

	// Insert it to the server
	m_uoGizmo->insert();

	// Refresh server
	m_server->applyChanges();
}

void COctomapControlPane::removeGizmo()
{
	m_server->erase( GIZMO_NAME );
}

///////////////////////////////////////////////////////////////////////////////
BEGIN_EVENT_TABLE(COctomapControlPane, wxPanel)
    EVT_BUTTON(ID_RESET_BUTTON,  COctomapControlPane::OnReset)
    EVT_BUTTON(ID_CLEARBOX_BUTTON,  COctomapControlPane::OnClearingBox)
    EVT_BUTTON(ID_MAPCLEAR_BUTTON,  COctomapControlPane::OnClearMap)
END_EVENT_TABLE()
