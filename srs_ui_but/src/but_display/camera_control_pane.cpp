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

#include <srs_ui_but/but_display/camera_control_pane.h>
#include <rviz/window_manager_interface.h>
#include <rviz/visualization_manager.h>

#include <OgreCamera.h>

#define MAX_RANGE 500

// Controls
const int ID_NEAR_PLANE_SPIN(201);
const int ID_FAR_PLANE_SPIN(202);
const int ID_NEAR_PLANE_TEXT(203);
const int ID_FAR_PLANE_TEXT(204);
const int ID_UPDATE_CLIPPING_LIMITS(205);
const int ID_FAR_MAX_SPIN(206);

/**
 Constructor
 */
srs_ui_but::CCameraControlPane::CCameraControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxCLOSE_BOX|wxTAB_TRAVERSAL | wxVSCROLL, title)
    , m_wmi( wmi )
	, m_camera( 0 )
{

	/*
		 * +------------------------------------+
		 * | +Clipping planes-------------------+
		 * | | Near plane						|
		 * | | Far plane						|
		 * | | Far plane max spin  				|
		 * +------------------------------------+
	*/

	// Create controls
	wxStaticText * stNearPlane = new wxStaticText(this, ID_NEAR_PLANE_TEXT, _T("Near plane"));
	m_sliderNearClipPlane = new wxSlider(this, ID_NEAR_PLANE_SPIN, 50, 0, 100, wxDefaultPosition, wxSize(150, 0), wxBU_EXACTFIT);
	wxStaticText * stFarPlane = new wxStaticText(this, ID_FAR_PLANE_TEXT, _T("Far plane"));
	m_sliderFarClipPlane = new wxSlider(this, ID_FAR_PLANE_SPIN, 50, 0, 100, wxDefaultPosition, wxSize(150, 0), wxBU_EXACTFIT);
	m_spinFarPlane = new wxSpinCtrl(this, ID_FAR_MAX_SPIN, _T("100"), wxDefaultPosition, wxSize(150, 0), wxBU_EXACTFIT);

	// Set sliders limits
	m_sliderNearClipPlane->SetRange( 0, MAX_RANGE );
	m_sliderNearClipPlane->SetValue( 0 );

	m_sliderFarClipPlane->SetRange(0, MAX_RANGE);
	m_sliderFarClipPlane->SetValue( 100 );

	m_spinFarPlane->SetRange(1, 10000);
	m_spinFarPlane->SetValue(100);

	// Create layout
	wxSizer *vsizer = new wxBoxSizer(wxVERTICAL);
	this->SetSizer(vsizer);

	// Clipping planes
	wxStaticBoxSizer *hsizerPlanesControls = new wxStaticBoxSizer(wxVERTICAL, this, wxT("Clipping planes"));
	hsizerPlanesControls->Add( stNearPlane, ID_NEAR_PLANE_TEXT, wxALIGN_LEFT );
	hsizerPlanesControls->Add( m_sliderNearClipPlane, ID_NEAR_PLANE_SPIN, wxALIGN_LEFT );
	hsizerPlanesControls->Add( stFarPlane, ID_FAR_PLANE_TEXT, wxALIGN_LEFT );
	hsizerPlanesControls->Add( m_sliderFarClipPlane, ID_FAR_PLANE_SPIN, wxALIGN_LEFT );
	hsizerPlanesControls->Add( m_spinFarPlane, ID_FAR_MAX_SPIN, wxALIGN_LEFT);

	// Finalize
	vsizer->Add(hsizerPlanesControls, 0, wxALIGN_LEFT);
	vsizer->SetSizeHints(this);
}

/**
 * Set used camera node
 */
void srs_ui_but::CCameraControlPane::setCamera( Ogre::Camera * camera )
{
	m_camera = camera;

	if( camera == 0 )
	{
		return;
	}
	m_nearDistanceMin = m_farDistanceMin = camera->getNearClipDistance();

	m_near = m_nearDistanceMin;
	m_far = camera->getFarClipDistance();

	updateClippingDistancesInfo();
}

/**
 * Near plane slide event
 */
void srs_ui_but::CCameraControlPane::OnNearPlane(wxScrollEvent& event)
{
	if( m_camera != 0 && m_nearDistanceMax > 0.0 )
	{
		m_near = m_nearDistanceMin + Ogre::Real( m_sliderNearClipPlane->GetValue()) / Ogre::Real(MAX_RANGE) * m_nearDistanceMax;
//		std::cerr << "new near: " << m_near << std::endl;
		m_camera->setNearClipDistance(m_near);
	}
}

/**
 * Far plane slide event
 */
void srs_ui_but::CCameraControlPane::OnFarPlane(wxScrollEvent& event)
{
	if( m_camera != 0 && m_farDistanceMax > 0.0 )
	{
		m_far = m_farDistanceMin + Ogre::Real( m_sliderFarClipPlane->GetValue()) / Ogre::Real(MAX_RANGE) * m_farDistanceMax;
//		std::cerr << "new far: " << m_far << std::endl;
		m_camera->setFarClipDistance(m_far);
	}
}

/**
 * Update clipping distances info
 */
void srs_ui_but::CCameraControlPane::updateClippingDistancesInfo()
{
	if( m_camera == 0 )
		return;

	m_nearDistanceMax = m_farDistanceMax = m_spinFarPlane->GetValue();
	m_nearDistanceMin = m_farDistanceMin = 0.03;

//	std::cerr << "Current: " << m_near << ", " << m_far << std::endl;
//	std::cerr << "Near limits: " << m_nearDistanceMin << ", " << m_nearDistanceMax << std::endl;
//	std::cerr << "Far limits: " << m_farDistanceMin << ", " << m_farDistanceMax << std::endl;


	// Set new limit to camera
	if( m_near > m_nearDistanceMax )
	{
		m_camera->setNearClipDistance( m_nearDistanceMax );
		m_near = m_nearDistanceMax;
		m_camera->setNearClipDistance(m_near);
	}

	if( m_far > m_farDistanceMax )
	{
		m_camera->setFarClipDistance( m_farDistanceMax );
		m_far = m_farDistanceMax;
		m_camera->setFarClipDistance(m_far);
	}

	// Update slider positions
	float n((Ogre::Real(MAX_RANGE) - 0.4 ) * m_near / m_nearDistanceMax);
	float f((Ogre::Real(MAX_RANGE) - 0.4 ) * m_far / m_farDistanceMax);

//	std::cerr << "New slider values: " << n << ", " << f << std::endl;

	m_sliderNearClipPlane->SetValue( int( n ) );
	m_sliderFarClipPlane->SetValue( int( f ) );
}

/**
 * Maximal limit spin control value changed
 */
void srs_ui_but::CCameraControlPane::OnMaxSpin(wxSpinEvent & event)
{
	updateClippingDistancesInfo();
}

///////////////////////////////////////////////////////////////////////////////

BEGIN_EVENT_TABLE(srs_ui_but::CCameraControlPane, wxPanel)
	EVT_COMMAND_SCROLL(ID_NEAR_PLANE_SPIN,  srs_ui_but::CCameraControlPane::OnNearPlane)
	EVT_COMMAND_SCROLL(ID_FAR_PLANE_SPIN,  srs_ui_but::CCameraControlPane::OnFarPlane)
    EVT_SPINCTRL(ID_FAR_MAX_SPIN, srs_ui_but::CCameraControlPane::OnMaxSpin)
END_EVENT_TABLE()
