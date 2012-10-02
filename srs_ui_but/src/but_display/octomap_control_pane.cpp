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
#include <srs_env_model/services_list.h>
#include <srs_interaction_primitives/RemovePrimitive.h>
#include <srs_env_model/RemoveCube.h>
#include <srs_env_model/AddCube.h>
#include <srs_env_model/LockCollisionMap.h>
#include <srs_env_model/LockServer.h>
#include <srs_env_model/ButServerPause.h>

#define GIZMO_NAME "OctomapClearingBoxGizmo"
#define GIZMO_FRAME_ID "/map"
#define GIZMO_POSE_TOPIC "/interaction_primitives/OctomapClearingBoxGizmo/update/pose_changed"
#define GIZMO_SCALE_TOPIC "/interaction_primitives/OctomapClearingBoxGizmo/update/scale_changed"



///////////////////////////////////////////////////////////////////////////////

const int ID_RESET_OCTOMAP_BUTTON(101);
const int ID_ADD_BOX_BUTTON(102);
const int ID_OCTOMAP_CLEAR_BUTTON(103);
const int ID_CMAP_CLEAR_BUTTON(104);
const int ID_CANCEL_BUTTON(105);
const int ID_OBSTACLE_OCTOMAP_BUTTON(106);
const int ID_OBSTACLE_CMAP_BUTTON(108);
const int ID_TEXT_BOX(107);
const int ID_LOCK_OCTOMAP_CHECKBOX(109);
const int ID_LOCK_CMAP_CHECKBOX(110);

/**
 Constructor
 */
srs_ui_but::COctomapControlPane::COctomapControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxCLOSE_BOX|wxTAB_TRAVERSAL | wxVSCROLL, title)
    , m_wmi( wmi )
	, m_bGizmoAdded( false )
{
	/*
	 * +------------------------------------+
	 * | Add box | Cancel                   |
	 * +------------------------------------+
	 * |
	 * | +Octomap---------------------------+
	 * | | Add obstacle						|
	 * | | Clear Box					    |
	 * | | Reset						    |
	 * | | x Lock						    |
	 * | +----------------------------------+
	 * | +Collision map---------------------+
	 * | | Obstacle							|
	 * | | Clear box						|
	 * | | x Lock							|
	 * | +----------------------------------+
	 */
    // Create controls
//	m_panel = new wxPanel( parent, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxTAB_TRAVERSAL | wxVSCROLL, title);
    m_buttonResetOctomap = new wxButton(this, ID_RESET_OCTOMAP_BUTTON, wxT("Reset map"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonBoxAdd = new wxButton( this, ID_ADD_BOX_BUTTON, wxT("Add box"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonClearBoxOctomap = new wxButton( this, ID_OCTOMAP_CLEAR_BUTTON, wxT("Clear box"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonClearBoxCMap = new wxButton( this, ID_CMAP_CLEAR_BUTTON, wxT("Clear box"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonCancelBox = new wxButton( this, ID_CANCEL_BUTTON, wxT("Cancel"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonObstacleOctomap = new wxButton( this, ID_OBSTACLE_OCTOMAP_BUTTON, wxT("Add obstacle"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_buttonObstacleCMap = new wxButton( this, ID_OBSTACLE_CMAP_BUTTON, wxT("Add obstacle"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);
    m_cbLockOctomap = new wxCheckBox( this, ID_LOCK_OCTOMAP_CHECKBOX, wxT("Lock - pause server"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
    m_cbLockCMap = new wxCheckBox( this, ID_LOCK_CMAP_CHECKBOX, wxT("Lock"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
    m_textBox = new wxTextCtrl( this, ID_TEXT_BOX, wxT(""));

    // Create layout
    wxSizer *vsizer = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(vsizer);

    // Box control buttons
    wxSizer *hsizerBoxControls = new wxBoxSizer(wxHORIZONTAL);
    hsizerBoxControls->Add( m_buttonBoxAdd, ID_ADD_BOX_BUTTON, wxALIGN_LEFT );
    hsizerBoxControls->Add( m_buttonCancelBox, ID_CANCEL_BUTTON, wxALIGN_LEFT );

    // Octomap controls
    wxStaticBoxSizer *sizerOctomapControls = new wxStaticBoxSizer( wxVERTICAL, this, wxT("Octomap") );
    sizerOctomapControls->Add( m_buttonObstacleOctomap, ID_OBSTACLE_OCTOMAP_BUTTON, wxALIGN_LEFT );
    sizerOctomapControls->Add( m_buttonClearBoxOctomap, ID_OCTOMAP_CLEAR_BUTTON, wxALIGN_LEFT );
    sizerOctomapControls->Add( m_buttonResetOctomap, ID_RESET_OCTOMAP_BUTTON, wxALIGN_LEFT );
    sizerOctomapControls->Add( m_cbLockOctomap, ID_LOCK_OCTOMAP_CHECKBOX, wxALIGN_LEFT );

    // Collision map controls
    wxStaticBoxSizer * sizerCMapControls = new wxStaticBoxSizer( wxVERTICAL, this, wxT("Collision map") );
    sizerCMapControls->Add( m_buttonObstacleCMap, ID_OBSTACLE_CMAP_BUTTON, wxALIGN_LEFT );
    sizerCMapControls->Add( m_buttonClearBoxCMap, ID_CMAP_CLEAR_BUTTON, wxALIGN_LEFT );
    sizerCMapControls->Add( m_cbLockCMap, ID_LOCK_CMAP_CHECKBOX, wxALIGN_LEFT );

    // Text output
    wxSizer *hsizerText = new wxBoxSizer(wxHORIZONTAL);
    hsizerText->Add(m_textBox, ID_TEXT_BOX, wxALIGN_LEFT);

    // No gizmo - nothing to do or to cancel.
    m_buttonClearBoxOctomap->Enable(false);
    m_buttonCancelBox->Enable(false);
    m_buttonObstacleOctomap->Enable(false);
    m_buttonClearBoxCMap->Enable(false);
    m_buttonObstacleCMap->Enable(false);

    vsizer->Add(hsizerBoxControls, 0, wxALIGN_LEFT);
    vsizer->Add(sizerOctomapControls, 0, wxALIGN_LEFT);
    vsizer->Add(sizerCMapControls, 0, wxALIGN_LEFT);
    vsizer->Add(hsizerText, 0, wxALIGN_LEFT);

    vsizer->SetSizeHints(this);

    // Connect to services
    ros::NodeHandle node_handle;
    m_srvAddGizmo = node_handle.serviceClient<srs_interaction_primitives::AddUnknownObject>(srs_interaction_primitives::AddUnknownObject_SRV);
    m_srvRemoveGizmo = node_handle.serviceClient<srs_interaction_primitives::RemovePrimitive>(srs_interaction_primitives::RemovePrimitive_SRV);
    m_srvRemoveCubeFromOctomap = node_handle.serviceClient<srs_env_model::RemoveCube>(srs_env_model::RemoveCubeOctomap_SRV);
    m_srvRemoveCubeFromCMap = node_handle.serviceClient<srs_env_model::RemoveCube>(srs_env_model::RemoveCubeCMP_SRV);
    m_srvAddCubeToOctomap = node_handle.serviceClient<srs_env_model::AddCube>(srs_env_model::AddCubeOctomap_SRV);
    m_srvAddCubeToCMap = node_handle.serviceClient<srs_env_model::AddCube>(srs_env_model::AddCubeCMP_SRV);
    m_srvLockOctomap = node_handle.serviceClient<srs_env_model::ButServerPause>(srs_env_model::ServerPause_SRV);
    m_srvLockCMap = node_handle.serviceClient<srs_env_model::LockCollisionMap>(srs_env_model::LockCMap_SRV);

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

    if( ros::service::call(srs_env_model::ResetOctomap_SRV, reset) )
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

	m_buttonClearBoxOctomap->Enable(true);
	m_buttonObstacleOctomap->Enable(true);
	m_buttonClearBoxCMap->Enable(true);
	m_buttonObstacleCMap->Enable(true);
	m_buttonCancelBox->Enable(true);
}

/**
 * On clear map event handler
 */

void srs_ui_but::COctomapControlPane::OnClearBoxOctomap( wxCommandEvent &event )
{
	// Create message
	srs_env_model::RemoveCube rc;
	rc.request.frame_id = GIZMO_FRAME_ID;
	rc.request.pose = m_gizmoPose;
	rc.request.size = m_gizmoScale;

	// call
	m_srvRemoveCubeFromOctomap.call( rc );

//	std::cerr << "Clear box area on octomap event" << std::endl;

	// Remove gizmo and disable buttons
	OnCancelBoxGizmo(event);
}

/*
 * On clear collision map event handler
 */
void srs_ui_but::COctomapControlPane::OnClearBoxCMap( wxCommandEvent &event )
{
	// Create message
	srs_env_model::RemoveCube rc;
	rc.request.frame_id = GIZMO_FRAME_ID;
	rc.request.pose = m_gizmoPose;
	rc.request.size = m_gizmoScale;

	// call
	m_srvRemoveCubeFromCMap.call( rc );

//	std::cerr << "Clear box area on collision map event" << std::endl;

	// Remove gizmo and disable buttons
	OnCancelBoxGizmo(event);
}


/**
 * On add box obstacle to octomap event handler
 */

void srs_ui_but::COctomapControlPane::OnAddObstacleOctomap( wxCommandEvent &event )
{
	// Create message
	srs_env_model::RemoveCube rc;
	rc.request.frame_id = GIZMO_FRAME_ID;
	rc.request.pose = m_gizmoPose;
	rc.request.size = m_gizmoScale;

	// call
	m_srvAddCubeToOctomap.call( rc );

//	std::cerr << "Add box to octomap event" << std::endl;

	// Remove gizmo and disable buttons
	OnCancelBoxGizmo(event);
}

/**
 * On add box obstacle to collision map event handler
 */

void srs_ui_but::COctomapControlPane::OnAddObstacleCMap( wxCommandEvent &event )
{
	// Create message
	srs_env_model::RemoveCube rc;
	rc.request.frame_id = GIZMO_FRAME_ID;
	rc.request.pose = m_gizmoPose;
	rc.request.size = m_gizmoScale;

	// call
	m_srvAddCubeToCMap.call( rc );

//	std::cerr << "Add obstacle to collision map event" << std::endl;

	// Remove gizmo and disable buttons
	OnCancelBoxGizmo(event);
}

/**
 * On octomap lock event handler
 */
void srs_ui_but::COctomapControlPane::OnLockOctomap( wxCommandEvent & event )
{
	// Get checkbox state
	bool locked( m_cbLockOctomap->GetValue() );

	// Create message
	srs_env_model::ButServerPause message;
	message.request.pause = locked ? 1 : 0;

	// call
	m_srvLockOctomap.call( message );

//	std::cerr << "Lock octomap event. " << std::endl;

}

/**
 * On collision map lock event handler
 */
void srs_ui_but::COctomapControlPane::OnLockCMap( wxCommandEvent & event )
{
	// Get checkbox state
	bool locked( m_cbLockCMap->GetValue() );

	// Create message
	srs_env_model::LockCollisionMap message;
	message.request.lock = locked ? 1 : 0;

	// call
	m_srvLockCMap.call( message );

//	std::cerr << "Lock collision map event. " << std::endl;

}


/**
 *On cancel box gizmo map
 */
void srs_ui_but::COctomapControlPane::OnCancelBoxGizmo( wxCommandEvent &event )
{
	// remove gizmo
	removeGizmo();

	// Disable button
	m_buttonClearBoxOctomap->Enable(false);
	m_buttonObstacleOctomap->Enable(false);
	m_buttonClearBoxCMap->Enable(false);
	m_buttonObstacleCMap->Enable(false);
	m_buttonCancelBox->Enable(false);
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
    EVT_BUTTON(ID_RESET_OCTOMAP_BUTTON,  srs_ui_but::COctomapControlPane::OnReset)
    EVT_BUTTON(ID_ADD_BOX_BUTTON,  srs_ui_but::COctomapControlPane::OnAddBoxGizmo)
    EVT_BUTTON(ID_OCTOMAP_CLEAR_BUTTON,  srs_ui_but::COctomapControlPane::OnClearBoxOctomap)
    EVT_BUTTON(ID_CMAP_CLEAR_BUTTON,  srs_ui_but::COctomapControlPane::OnClearBoxCMap)
    EVT_BUTTON(ID_OBSTACLE_OCTOMAP_BUTTON,  srs_ui_but::COctomapControlPane::OnAddObstacleOctomap)
	EVT_BUTTON(ID_OBSTACLE_CMAP_BUTTON,  srs_ui_but::COctomapControlPane::OnAddObstacleCMap)
	EVT_CHECKBOX(ID_LOCK_OCTOMAP_CHECKBOX, srs_ui_but::COctomapControlPane::OnLockOctomap)
	EVT_CHECKBOX(ID_LOCK_CMAP_CHECKBOX, srs_ui_but::COctomapControlPane::OnLockCMap)
    EVT_BUTTON(ID_CANCEL_BUTTON,  srs_ui_but::COctomapControlPane::OnCancelBoxGizmo)
END_EVENT_TABLE()

