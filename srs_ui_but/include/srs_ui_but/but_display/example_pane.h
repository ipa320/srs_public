/******************************************************************************
 * \file
 *
 * $Id: example_pane.h 824 2012-05-23 13:14:09Z spanel $
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
#ifndef BUT_EXAMPLEPANE_H
#define BUT_EXAMPLEPANE_H

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
// #include "checkbox.h"

namespace rviz
{
    class WindowManagerInterface;
}

namespace srs_ui_but
{


/**
  Example panel window
  */
class CExamplePanel : public wxPanel
{
public:
    //! Constructor
    CExamplePanel(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );

    //! On quit command event handler
    virtual void OnQuit(wxCommandEvent& event);
protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

}; // class CExamplePanel

/**
  Example dialog window
  */
class CExampleDialog : public wxDialog
{
public:
    //! Constructor
    CExampleDialog( wxWindow * parent, const wxString & title );

    //! On quit command event handler
    void OnQuit(wxCommandEvent& event);

}; // class CExampleDialog

/**
  Example panel window - controls
  */
class CExamplePanelControls : public wxPanel
{
public:
    /// Constructor
    CExamplePanelControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );

    /// On checkbox toggle
    void OnChckToggle(wxCommandEvent& event);

    //! On reset octomap command event handler
    virtual void OnReset(wxCommandEvent& event);

protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    //! Chcekbox
    wxCheckBox * m_chkb;

    //! Button
    wxButton * m_button;

private:
    DECLARE_EVENT_TABLE()

};

} // namespace srs_ui_but



#endif // BUT_EXAMPLEPANE_H
