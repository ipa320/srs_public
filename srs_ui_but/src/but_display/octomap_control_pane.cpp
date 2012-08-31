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

#include "octomap_control_pane.h"
#include <rviz/window_manager_interface.h>
#include <rviz/visualization_manager.h>
#include <srs_env_model/ResetOctomap.h>
#include <ros/service.h>

// Services
#include <srs_interaction_primitives/RemovePrimitive.h>
#include <srs_env_model/RemoveCube.h>

#define GIZMO_NAME "OctomapClearingBoxGizmo"
#define GIZMO_FRAME_ID "/map"
#define RESET_OCTOMAP_SERVICE_NAME "/but_env_model/reset_octomap"
#define GIZMO_POSE_TOPIC "/interaction_primitives/OctomapClearingBoxGizmo/update/pose_changed"
#define GIZMO_SCALE_TOPIC "/interaction_primitives/OctomapClearingBoxGizmo/update/scale_changed"
#define CLEAR_OCTOMAP_BOX_SERVICE_NAME "/but_env_model/remove_cube"



///////////////////////////////////////////////////////////////////////////////

const int ID_RESET_BUTTON(101);
const int ID_CLEARBOX_BUTTON(102);
const int ID_MAPCLEAR_BUTTON(103);
const int ID_CANCEL_BUTTON(104);
const int ID_OBSTACLE_BUTTON(105);
const int ID_TEXT_BOX(106);


/**
 Constructor
 */
srs_ui_but::COctomapControlPane::COctomapControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi )
	, m_bGizmoAdded( false )
{
    // Create controls
    m_buttonReset = new wxButton(this, ID_RESET_BUTTON, wxT("Reset map"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonBoxAdd = new wxButton( this, ID_CLEARBOX_BUTTON, wxT("Add box"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonClearBox = new wxButton( this, ID_MAPCLEAR_BUTTON, wxT("Clear box on map"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonCancelClear = new wxButton( this, ID_CANCEL_BUTTON, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonObstacleAdd = new wxButton( this, ID_OBSTACLE_BUTTON, wxT("Add obstacle"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_textBox = new wxTextCtrl( this, ID_TEXT_BOX, wxT(""));

    // Create layout
    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(vsizer);

    wxSizer *hsizer1 = new wxBoxSizer(wxHORIZONTAL);
    hsizer1->Add(m_buttonReset, ID_RESET_BUTTON, wxALIGN_LEFT);

    wxSizer *hsizer2 = new wxBoxSizer(wxHORIZONTAL);
    hsizer2->Add(m_buttonBoxAdd, ID_CLEARBOX_BUTTON, wxALIGN_LEFT);
    hsizer2->Add(m_buttonClearBox, ID_MAPCLEAR_BUTTON, wxALIGN_LEFT);
    hsizer2->Add( m_buttonObstacleAdd, ID_OBSTACLE_BUTTON, wxALIGN_LEFT);
    hsizer2->Add(m_buttonCancelClear, ID_CANCEL_BUTTON, wxALIGN_LEFT);

    wxSizer *hsizer3 = new wxBoxSizer(wxHORIZONTAL);
    hsizer3->Add(m_textBox, ID_TEXT_BOX, wxALIGN_LEFT);

    // No gizmo - nothing to do or to cancel.
    m_buttonClearBox->Enable(false);
    m_buttonCancelClear->Enable(false);
    m_buttonObstacleAdd->Enable(false);

    vsizer->Add(hsizer1, 0, wxALIGN_LEFT);
    vsizer->Add(hsizer2, 0, wxALIGN_LEFT);
    vsizer->Add(hsizer3, 0, wxALIGN_LEFT);

    vsizer->SetSizeHints(this);

    // Connect to services
    ros::NodeHandle node_handle;
    m_srvAddGizmo = node_handle.serviceClient<srs_interaction_primitives::AddUnknownObject>("/interaction_primitives/add_unknown_object");
    m_srvRemoveGizmo = node_handle.serviceClient<srs_interaction_primitives::RemovePrimitive>("/interaction_primitives/remove_primitive");
    m_srvRemoveCubeFromOCmap = node_handle.serviceClient<srs_env_model::RemoveCube>(CLEAR_OCTOMAP_BOX_SERVICE_NAME);

    // Set gizmo name and defaults
    m_uoGizmo.request.name = GIZMO_NAME;
    m_uoGizmo.request.frame_id = GIZMO_FRAME_ID;

    // Set default values - pose and scale
    m_gizmoPose.position.x = 0.0;
    m_gizmoPose.position.y = 0.0;
    m_gizmoPose.position.z = 1.0;

    m_gizmoPose.orientation.x = 0.0;
    m_gizmoPose.orientation.y = 0.0;
    m_gizmoPose.orientation.z = 0.0;
    m_gizmoPose.orientation.w = 1.0;

    m_gizmoScale.x = 2.0;
    m_gizmoScale.y = 2.0;
    m_gizmoScale.z = 2.0;


    // Connect to the position/scale topics
    m_subGizmoPose = node_handle.subscribe<srs_interaction_primitives::PoseChanged> ( GIZMO_POSE_TOPIC, 10, &COctomapControlPane::gizmoPoseCB, this );
    m_subGizmoScale = node_handle.subscribe<srs_interaction_primitives::ScaleChanged>( GIZMO_SCALE_TOPIC, 10, &COctomapControlPane::gizmoScaleCB, this );

    // Set text label parameters
    m_textBox->SetEditable(false);

}


///////////////////////////////////////////////////////////////////////////////

/**
 * On quit command event handler
 */
void srs_ui_but::COctomapControlPane::OnReset(wxCommandEvent& event)
{
    srs_env_model::ResetOctomap reset;

    if( ros::service::call(RESET_OCTOMAP_SERVICE_NAME, reset) )
    {
        std::cerr << "Reseting octomap..." << std::endl;
    }
}

/**
 * On create clearing box event handler
 */
void srs_ui_but::COctomapControlPane::OnAddBoxGizmo(wxCommandEvent &event)
{
	std::cerr << "Adding box to the scene." << std::endl;

	addGizmo();

	m_buttonClearBox->Enable(true);
	m_buttonObstacleAdd->Enable(true);
	m_buttonCancelClear->Enable(true);
}

/**
 * On clear map event handler
 */

void srs_ui_but::COctomapControlPane::OnClearBox( wxCommandEvent &event )
{
	// Create message
	srs_env_model::RemoveCube rc;
	rc.request.frame_id = GIZMO_FRAME_ID;
	rc.request.pose = m_gizmoPose;
	rc.request.size = m_gizmoScale;

	// call
	m_srvRemoveCubeFromOCmap.call( rc );

	std::cerr << "Clear box area on map event" << std::endl;

	// Remove gizmo and disable buttons
	OnCancelBoxGizmo(event);
}

/**
 * On clear map event handler
 */

void srs_ui_but::COctomapControlPane::OnAddObstacle( wxCommandEvent &event )
{
	// Create message
	srs_env_model::RemoveCube rc;
	rc.request.frame_id = GIZMO_FRAME_ID;
	rc.request.pose = m_gizmoPose;
	rc.request.size = m_gizmoScale;

	// call
	m_srvRemoveCubeFromOCmap.call( rc );

	std::cerr << "Clear map event" << std::endl;

	// Remove gizmo and disable buttons
	OnCancelBoxGizmo(event);
}

/**
 *On cancel box gizmo map
 */
void srs_ui_but::COctomapControlPane::OnCancelBoxGizmo( wxCommandEvent &event )
{
	// remove gizmo
	removeGizmo();

	// Disable button
	m_buttonClearBox->Enable(false);
	m_buttonObstacleAdd->Enable(false);
	m_buttonCancelClear->Enable(false);
}

/////////////////////// Gizmo operations //////////////////////////////////////

/**
 * Add gizmo to the scene
 */
void srs_ui_but::COctomapControlPane::addGizmo()
{
	std::cerr << "remove old gizmo..." << std::endl;
	// Remove old gizmo
	removeGizmo();


	if( ! m_srvAddGizmo.call( m_uoGizmo ) )
	{
		std::cerr << "Service call failed: " << m_srvAddGizmo.getService() << std::endl;
	}
	m_bGizmoAdded = true;

	m_textBox->SetLabel( getGizmoStatusStr() );
}
/**
 * Remove gizmo from server
 */
void srs_ui_but::COctomapControlPane::removeGizmo()
{
	if( ! m_bGizmoAdded )
		return;

	setGizmoPoseScale();

	srs_interaction_primitives::RemovePrimitive rmo;
	rmo.request.name = GIZMO_NAME;
	m_srvRemoveGizmo.call( rmo );

	m_bGizmoAdded = false;

	m_textBox->SetLabel( wxT("") );
}

/**
 * Gizmo pose topic callback
 */
void srs_ui_but::COctomapControlPane::gizmoPoseCB( const srs_interaction_primitives::PoseChangedConstPtr &marker_update )
{
	if( marker_update->marker_name != GIZMO_NAME )
		return;

	m_gizmoPose = marker_update->new_pose;

	m_textBox->SetLabel( getGizmoStatusStr() );
}

/**
 * Gizmo pose topic callback
 */
void srs_ui_but::COctomapControlPane::gizmoScaleCB( const srs_interaction_primitives::ScaleChangedConstPtr &marker_update )
{
	if( marker_update->marker_name != GIZMO_NAME )
		return;

	m_gizmoScale.x = marker_update->new_scale.x;
	m_gizmoScale.y = marker_update->new_scale.y;
	m_gizmoScale.z = marker_update->new_scale.z;

	m_textBox->SetLabel( getGizmoStatusStr() );
}


/**
 * Set gizmo pose and scale from the local copy of the data
 */
void srs_ui_but::COctomapControlPane::setGizmoPoseScale()
{
	m_uoGizmo.request.pose.position.x = m_gizmoPose.position.x;
	m_uoGizmo.request.pose.position.y = m_gizmoPose.position.y;
	m_uoGizmo.request.pose.position.z = m_gizmoPose.position.z;

	m_uoGizmo.request.pose.orientation.x = m_gizmoPose.orientation.x;
	m_uoGizmo.request.pose.orientation.y = m_gizmoPose.orientation.y;
	m_uoGizmo.request.pose.orientation.z = m_gizmoPose.orientation.z;
	m_uoGizmo.request.pose.orientation.w = m_gizmoPose.orientation.w;

	m_uoGizmo.request.scale.x = m_gizmoScale.x;
	m_uoGizmo.request.scale.y = m_gizmoScale.y;
	m_uoGizmo.request.scale.z = m_gizmoScale.z;
}

//! Create gizmo status string
wxString srs_ui_but::COctomapControlPane::getGizmoStatusStr()
{
	wxString s;

	s <<  wxString::Format(wxT("Position: %d, %d, %d"), m_gizmoPose.position.x, m_gizmoPose.position.y, m_gizmoPose.position.z);
	std::cerr << "Writing text: " << s.c_str() << std::endl;

	return s;
}

///////////////////////////////////////////////////////////////////////////////
BEGIN_EVENT_TABLE(srs_ui_but::COctomapControlPane, wxPanel)
    EVT_BUTTON(ID_RESET_BUTTON,  srs_ui_but::COctomapControlPane::OnReset)
    EVT_BUTTON(ID_CLEARBOX_BUTTON,  srs_ui_but::COctomapControlPane::OnAddBoxGizmo)
    EVT_BUTTON(ID_MAPCLEAR_BUTTON,  srs_ui_but::COctomapControlPane::OnClearBox)
    EVT_BUTTON(ID_CANCEL_BUTTON,  srs_ui_but::COctomapControlPane::OnCancelBoxGizmo)
END_EVENT_TABLE()
