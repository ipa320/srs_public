/**
 * $Id$
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#ifndef BUT_CAMCAST_H
#define BUT_CAMCAST_H

#include <rviz/display.h>
#include <rviz/view_controller.h>
#include "rviz/properties/forwards.h"
#include "rviz/properties/property.h"
#include "rviz/properties/edit_enum_property.h"
#include "rviz/properties/property_manager.h"
#include <OGRE/OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreCamera.h>
#include <string>
#include <OGRE/OgreTexture.h>
#include "CRosRttTexture.h"
#include <wx/wx.h>

namespace rviz
{
  class WindowManagerInterface;
}

class CButCamCast : public rviz::Display
{
public:
  class CControllPane : public wxPanel
  {
  public:
    /// Checkbox state changed signal type
    typedef boost::signal< void (bool) > tSigCheckboxState;

    /// Save screenshot signal type
    typedef boost::signal< void ( std::string ) > tSigSave;

  public:
    /// Constructor
    CControllPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );

    /// On checkbox toggle
    void OnChckToggle(wxCommandEvent& event);

    //! On save screenshot button
    virtual void OnSave(wxCommandEvent& event);

    //! Get checkbox state changed signal
    tSigCheckboxState & getSigChckBox(){ return m_sigCheckBox; }

    //! Get save signal
    tSigSave & getSigSave(){ return m_sigSave; }

  protected:
    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    //! Chcekbox
    wxCheckBox * m_chkb;

    //! Button
    wxButton * m_button;

    //! Checkbox state changed signal
    tSigCheckboxState m_sigCheckBox;

    /// Save screenshot signal
    tSigSave m_sigSave;
  private:
    DECLARE_EVENT_TABLE()
  }; // class CControllPane

public:
    //!Constructor
    CButCamCast(const std::string & name,rviz::VisualizationManager * manager);

    //!Destructor
    ~CButCamCast();

    //OverridesfromDisplay
    virtual void targetFrameChanged(){}
    virtual void fixedFrameChanged(){}
    virtual void createProperties(){}

    //! Update display
    virtual void update (float wall_dt, float ros_dt);

protected:
    //overridesfromDisplay
    virtual void onEnable();
    virtual void onDisable();

    //! Create geometry (includes scene node initialization)
    bool createGeometry(const ros::NodeHandle & nh);

    //! Destroy geometry
    void destroyGeometry();

    //! Set timer period
    //void setTimerPeriod(float period);

    //! Timer publishing callback function
    void onTimerPublish(const ros::TimerEvent&);

    //! On publishing start/stop
    void onPublishStateChanged(bool state);

    /// On screenshot save signal
    void onSave( std::string filename );

protected:
    //! Scene node
    Ogre::SceneNode * m_sceneNode;

    //! Camera screencast publishing topic name
    std::string m_camCastPublisherName;

    //! Camera screencast publisher
    ros::Publisher m_camCastPublisher;

    //! Used texture
    rviz::CRosRttTexture * m_textureWithRtt;

    //! Publishing timer
    ros::Timer m_timer;

    //! Timer period
    double m_timerPeriod;

    //! Publishing enabled?
    bool m_bPublish;

    //! Display pane
    CControllPane * m_pane;

};//class CButCamCast


#endif // BUT_CAMCAST_H
