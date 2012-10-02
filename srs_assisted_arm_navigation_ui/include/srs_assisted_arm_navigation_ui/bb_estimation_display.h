/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 5/4/2012
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

#ifndef BUT_BB_ESTIMATION_DISPLAY_H
#define BUT_BB_ESTIMATION_DISPLAY_H

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

#include "bb_estimation_controls.h"


namespace srs_assisted_arm_navigation_ui {

class CButBBEstimationDisplay : public rviz::Display
{

public:
    //!Constructor
	CButBBEstimationDisplay(const std::string & name,rviz::VisualizationManager * manager);

    //!Destructor
    ~CButBBEstimationDisplay();

    //OverridesfromDisplay
    virtual void targetFrameChanged(){}
    virtual void fixedFrameChanged(){}
    virtual void createProperties() {};

protected:
    //overridesfromDisplay
    virtual void onEnable() {};
    virtual void onDisable() {};


protected:

    //! Controls window example
    CButBBEstimationControls * m_bb_estimation_window;

};//classCBasicPlugin


}

#endif // BUT_GRASPING_DISPLAY_H

