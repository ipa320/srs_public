/******************************************************************************
 * \file
 *
 * $Id: but_display.h 2050 2012-12-03 15:38:13Z spanel $
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
#ifndef BUT_DISPLAY_H
#define BUT_DISPLAY_H

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

#include "example_pane.h"
#include "camera_control_pane.h"


namespace srs_ui_but
{

class CButDisplay:public rviz::Display
{
public:
    //! Constructor
    CButDisplay(const std::string & name, rviz::VisualizationManager * manager);

    //! Destructor
    ~CButDisplay();

    // Overrides from Display
    virtual void targetFrameChanged();
    virtual void fixedFrameChanged();
    virtual void createProperties();

    //! Update display
    virtual void update(float wall_dt, float ros_dt);

protected:
    // Overrides from Display
    virtual void onEnable();
    virtual void onDisable();

    //! Create geometry (includes scene node initialization)
    bool createGeometry();

    //! Destroy geometry
    void destroyGeometry();

    //! View controller has changed signal slot
    void onViewControllerChange( rviz::ViewController * c );

protected:
    //! Scene node
    Ogre::SceneNode * m_sceneNode;

    //! Geometry manual object
    Ogre::ManualObject * m_manualObject;

    //! Simple window example
    CExamplePanel * m_child_window;

    //! Simple dialog example
    CExampleDialog * m_dialog_window;

    //! Controls window example
    CExamplePanelControls * m_controls_window;
    
    //! Rviz camera control pane
    CCameraControlPane * m_camera_window;

};//classCBasicPlugin


} // namespace srs_ui_but

#endif // BUT_DISPLAY_H
