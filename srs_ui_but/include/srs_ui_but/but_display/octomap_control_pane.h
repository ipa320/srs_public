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
#pragma once
#ifndef but_octomap_control_pane_H_included
#define but_octomap_control_pane_H_included

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/textctrl.h>

#include <boost/shared_ptr.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <srs_interaction_primitives/AddUnknownObject.h>
#include <srs_interaction_primitives/PoseChanged.h>
#include <srs_interaction_primitives/ScaleChanged.h>
#include <ros/ros.h>

namespace rviz
{
    class WindowManagerInterface;
}

namespace srs_ui_but
{


class COctomapControlPane : public wxPanel
{
public:
	//! Define interactive markers server pointer
	typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

public:
	//!Constructor
	COctomapControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );

	//! On reset octomap command event handler
	virtual void OnReset(wxCommandEvent& event);

	//! On reset octomap command event handler
	virtual void OnAddBoxGizmo(wxCommandEvent& event);

	/// On clear map event handler
	virtual void OnClearBox( wxCommandEvent &event );

	/// On add obstacle event handler
	virtual void OnAddObstacle( wxCommandEvent &event );

	/// On cancel clera map
	virtual void OnCancelBoxGizmo( wxCommandEvent &event );

protected:
	//! Add gizmo to the scene
	void addGizmo();

	//! Remove gizmo from the scene
	void removeGizmo();

	//! Gizmo pose topic callback
	void gizmoPoseCB( const srs_interaction_primitives::PoseChangedConstPtr &marker_update );

	//! Gizmo scale topic callback
	void gizmoScaleCB( const srs_interaction_primitives::ScaleChangedConstPtr &marker_update );

	//! Set gizmo pose-scale from the local copy
	void setGizmoPoseScale();

	//! Create gizmo status string
	wxString getGizmoStatusStr();

protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    //! Reset octomap button
    wxButton * m_buttonReset;

    //! Create add box button
    wxButton *m_buttonBoxAdd;

    //! Create obstacle add button
    wxButton *m_buttonObstacleAdd;

    //! Do map erase
    wxButton *m_buttonClearBox;

    //! Cancel clearing
    wxButton *m_buttonCancelClear;

    //! Static text control displaying gizmo info
    wxTextCtrl * m_textBox;

	//! Gizmo
    srs_interaction_primitives::AddUnknownObject m_uoGizmo;

	//! Used services - add gizmo
	ros::ServiceClient m_srvAddGizmo;

	//! Used services - remove gizmo
	ros::ServiceClient m_srvRemoveGizmo;

	//! Used services - remove cube from octomap
	ros::ServiceClient m_srvRemoveCubeFromOCmap;

	//! Subscribers - gizmo position and scale
    ros::Subscriber m_subGizmoPose, m_subGizmoScale;

    //! Gizmo pose
    geometry_msgs::Pose m_gizmoPose;

    //! Gizmo scale
    geometry_msgs::Point m_gizmoScale;

    //! Gizmo was added to the scene
    bool m_bGizmoAdded;

private:
    DECLARE_EVENT_TABLE()

}; // class COctomapControlPane

} // namespace srs_ui_but

// but_octomap_control_pane_H_included
#endif

