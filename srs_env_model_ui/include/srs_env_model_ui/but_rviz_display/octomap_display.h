/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
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
#ifndef BUT_COctomapDisplay_H
#define BUT_COctomapDisplay_H

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

#include "octomap_control_pane.h"


namespace srs_env_model_ui
{

/**
 * \class COctomapDisplay
 * \brief ...
 */

class COctomapDisplay : public rviz::Display
{
public:
    //!Constructor
    COctomapDisplay(const std::string & name,rviz::VisualizationManager * manager);

    //!Destructor
    ~COctomapDisplay();

    //OverridesfromDisplay
    virtual void targetFrameChanged();
    virtual void fixedFrameChanged();
    virtual void createProperties();

    //! Update display
    virtual void update (float wall_dt, float ros_dt);

protected:
    //overridesfromDisplay
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

    //! Octomap controls window
    COctomapControlPane * m_ocmap_window;
};


} // namespace srs_env_model_ui

#endif // BUT_COctomapDisplay_H

