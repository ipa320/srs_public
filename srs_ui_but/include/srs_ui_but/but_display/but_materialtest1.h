/******************************************************************************
 * \file
 *
 * $Id:$
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
#ifndef BUT_MATERIALTEST1_H
#define BUT_MATERIALTEST1_H

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
#include "ros_rtt_texture.h"
#include <wx/wx.h>

namespace rviz
{
  class WindowManagerInterface;
}

namespace srs_ui_but
{


class CButMaterialTest1 : public rviz::Display
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
  CButMaterialTest1(const std::string & name,rviz::VisualizationManager * manager);

    //!Destructor
    ~CButMaterialTest1();

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

    //! Material part - get material manager, load groups etc.
    void createMaterials(Ogre::Camera * camera);

protected:
    //! Scene node
    Ogre::SceneNode * m_sceneNode;

    //! Camera screencast publishing topic name
    std::string m_camCastPublisherName;

    //! Camera screencast publisher
    ros::Publisher m_camCastPublisher;

    //! Publishing timer
    ros::Timer m_timer;

    //! Timer period
    double m_timerPeriod;

    //! Publishing enabled?
    bool m_bPublish;

    //! Display pane
    CControllPane * m_pane;

};//class CButCamCast

} // namespace srs_ui_but

#endif // BUT_MATERIALTEST1_H


