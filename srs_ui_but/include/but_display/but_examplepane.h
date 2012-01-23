/**
 * $Id: but_examplepane.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

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

#endif // BUT_EXAMPLEPANE_H
