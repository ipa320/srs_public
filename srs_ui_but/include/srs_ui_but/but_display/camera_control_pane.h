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
#ifndef but_camera_control_pane_H_included
#define but_camera_control_pane_H_included

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/textctrl.h>
#include <wx/statbox.h>
#include <wx/checkbox.h>
#include <wx/spinctrl.h>

#include <Ogre.h>

namespace rviz
{
    class WindowManagerInterface;
}

namespace Ogre
{
	class Camera;
}

namespace srs_ui_but
{


class CCameraControlPane : public wxPanel
{


public:
	//!Constructor
	CCameraControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );

	//! Fixed frame has changed
	void fixedFrameChanged( const std::string & frameId ) { m_fixedFrameId = frameId; std::cerr << "New fixed frame: " << frameId << std::endl; }

	//! Target frame has changed
	void targetFrameChanged( const std::string & frameId ) { m_targetFrameId = frameId; std::cerr << "New target frame: " << frameId << std::endl; }

	//! Set used camera node
	void setCamera( Ogre::Camera * camera );

protected:
	//! Near plane slide event
	virtual void OnNearPlane(wxScrollEvent& event);

	//! Far plane slide event
	virtual void OnFarPlane(wxScrollEvent& event);

	//! Update distances
	void updateClippingDistancesInfo();

	//! Maximal limit spin control value changed
	void OnMaxSpin(wxSpinEvent & event);

protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    //! Set near clip plane slider
    wxSlider * m_sliderNearClipPlane;

    //! Set far clip plane slider
    wxSlider * m_sliderFarClipPlane;

    //! Spin control to set far plane distance limit
    wxSpinCtrl * m_spinFarPlane;

    //! Fixed frame id
    std::string m_fixedFrameId;

    //! Target frame id
    std::string m_targetFrameId;

    //! Camera node
    Ogre::Camera * m_camera;

    //! Near clipping distance limits
    Ogre::Real m_nearDistanceMin, m_nearDistanceMax;

    //! Far clipping distance limits
    Ogre::Real m_farDistanceMin, m_farDistanceMax;

    //! Near and far distance values
    Ogre::Real m_near, m_far;

private:
    DECLARE_EVENT_TABLE()

}; // class CCameraControlPane

} // namespace srs_ui_but

#endif // but_camera_control_pane_H_included

