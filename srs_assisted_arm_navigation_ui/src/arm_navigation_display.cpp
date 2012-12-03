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

#include "srs_assisted_arm_navigation_ui/arm_navigation_display.h"

#include <OGRE/OgreSceneManager.h>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>

#include <sstream>

using namespace srs_assisted_arm_navigation_ui;


/**
 * Constructor
 */
CButArmNavDisplay::CButArmNavDisplay(const std::string & name,rviz::VisualizationManager * manager)
    : Display( name, manager )
{
    // Get node handle
    ros::NodeHandle private_nh("~");

    // Connect to the controller changed signal
    //vis_manager_->getViewControllerTypeChangedSignal().connect( boost::bind(&CButArmNavDisplay::onViewControllerChange, this, _1 ) );

    // Get scene node
    //m_sceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // Try to connect camera listener
    //connectListener();

    // Try to create, add and show example window
    rviz::WindowManagerInterface * wi( manager->getWindowManager() );

    if( wi != 0 )
    {
        // Arm manipulation controls
        m_armmanipulation_window = new CArmManipulationControls( wi->getParentWindow(), wxT("Manual arm navigation"), wi);

        if( m_armmanipulation_window != 0 )
        {
            std::cerr << "Adding to the window manager..." << std::endl;
            wi->addPane( "Assisted arm navigation", m_armmanipulation_window );
            wi->showPane( m_armmanipulation_window );
            std::cerr << "Added..." << std::endl;
        }
    }else{
        std::cerr << "No window manager, no panes :( " << std::endl;
    }
}


/*
 *  Destructor
 */
CButArmNavDisplay::~CButArmNavDisplay()
{
}

