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

#ifndef but_octomap_control_pane_H_included
#define but_octomap_control_pane_H_included

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>

#include <boost/shared_ptr.hpp>
#include <interactive_markers/interactive_marker_server.h>
#include <but_interaction_primitives/UnknownObject.h>

namespace rviz
{
    class WindowManagerInterface;
}

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
	virtual void OnClearingBox(wxCommandEvent& event);

	/// On clear map event handler
	virtual void OnClearMap( wxCommandEvent &event );

protected:
	//! Add gizmo to the scene
	void addGizmo();

	//! Remove gizmo from the scene
	void removeGizmo();



protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    //! Reset octomap button
    wxButton * m_buttonReset;

    //! Create clearbox button
    wxButton *m_buttonClearBoxAdd;

    //! Do map erase
    wxButton *m_buttonClearMap;

    /// Interactive server
	InteractiveMarkerServerPtr m_server;

	//! Gizmo
	but_interaction_primitives::UnknownObject * m_uoGizmo;


private:
    DECLARE_EVENT_TABLE()

}; // class COctomapControlPane


// but_octomap_control_pane_H_included
#endif

