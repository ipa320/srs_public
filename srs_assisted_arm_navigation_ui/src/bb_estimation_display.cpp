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
 * Date: 31/7/2012
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

#include <srs_assisted_arm_navigation_ui/bb_estimation_display.h>

#include <OGRE/OgreSceneManager.h>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>

#include <sstream>

using namespace srs_assisted_arm_navigation_ui;


/**
 * Constructor
 */
CButBBEstimationDisplay::CButBBEstimationDisplay(const std::string & name,rviz::VisualizationManager * manager)
    : Display( name, manager )
{
    // Get node handle
    ros::NodeHandle private_nh("~");

    // Try to create, add and show example window
    rviz::WindowManagerInterface * wi( manager->getWindowManager() );

    if( wi != 0 )
    {
        // Arm manipulation controls
    	m_bb_estimation_window = new CButBBEstimationControls( wi->getParentWindow(), wxT("BB estimator plugin"), wi);

        if( m_bb_estimation_window != 0 )
        {
            std::cerr << "Adding to the window manager..." << std::endl;
            wi->addPane( "Assisted object detection", m_bb_estimation_window );
            wi->showPane( m_bb_estimation_window );
            std::cerr << "Added..." << std::endl;
        }
    }else{
        std::cerr << "No window manager, no panes :( " << std::endl;
    }
}


/*
 *  Destructor
 */
CButBBEstimationDisplay::~CButBBEstimationDisplay()
{
}

