/******************************************************************************
 * \file
 *
 * $Id: but_display.cpp 810 2012-05-19 21:47:51Z stancl $
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

#include "but_display.h"
#include <OGRE/OgreSceneManager.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <sstream>
#include <srs_ui_but/topics_list.h>
#include <tf/transform_broadcaster.h>

/*
 *  Constructor
 */
srs_ui_but::CButDisplay::CButDisplay(const std::string & name, rviz::VisualizationManager * manager)
    : Display( name, manager )
    , m_manualObject( 0 )
    , m_child_window( 0 )
    , m_dialog_window( 0 )
    , m_controls_window( 0 )
{
    // Get node handle
    ros::NodeHandle private_nh("~");

    // Connect to the controller changed signal
    vis_manager_->getViewControllerTypeChangedSignal().connect( boost::bind(&CButDisplay::onViewControllerChange, this, _1 ) );

    // Get scene node
    m_sceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // Try to create, add and show example window

    rviz::WindowManagerInterface * wi( manager->getWindowManager() );

    if( wi != 0 )
    {
    	// Create camera control pane
    	std::cerr << "Creating camera pane" << std::endl;

    	m_camera_window = new CCameraControlPane( wi->getParentWindow(), _T("Camera control"), wi );

    	if( m_camera_window != 0 )
    	{
    		m_camera_window->fixedFrameChanged( fixed_frame_ );
    		m_camera_window->targetFrameChanged( target_frame_ );

    		wi->addPane( "Camera control", m_camera_window );
    		wi->showPane( m_camera_window );
    		ROS_DEBUG("Camera control pane added...");
    	}

    }
    else
    {
        std::cerr << "No window manager, no panes :( " << std::endl;
    }
}

/*
 *  Destructor
 */
srs_ui_but::CButDisplay::~CButDisplay()
{
    // Destroy all geometry
    destroyGeometry();
}

/**
  *
  */
void srs_ui_but::CButDisplay::targetFrameChanged()
{
}

/**
  *
  */
void srs_ui_but::CButDisplay::fixedFrameChanged()
{
}

/**
  *
  */
void srs_ui_but::CButDisplay::createProperties()
{
    // Create some properties

    // Get camera.
    rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
    if( panel != 0 )
    {
        Ogre::Camera * camera = panel->getCamera();

        // Set camera to window
        m_camera_window->setCamera( camera );
    }
    else
        ROS_DEBUG( "No render panel... ");
}

/*
 *  Display enabled callback
 */
void srs_ui_but::CButDisplay::onEnable()
{
    m_sceneNode->setVisible( true );
}

/*
 *  Display disabled callback
 */
void srs_ui_but::CButDisplay::onDisable()
{
    m_sceneNode->setVisible( false );
}

/*
 *  Create geometry
 */
bool srs_ui_but::CButDisplay::createGeometry()
{
    // Create manual object
    m_manualObject = scene_manager_->createManualObject( "manual" );

    // Create some geometry
    m_manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    {
        float lSize( 1.0f );
        float cp( 1.0f * lSize );
        float cm( -1.0f * lSize );

        // Insert some vertices
        m_manualObject->position(cm, cp, cm);// a vertex
        m_manualObject->colour(Ogre::ColourValue(0.0f,1.0f,0.0f,1.0f));
        m_manualObject->position(cp, cp, cm);// a vertex
        m_manualObject->colour(Ogre::ColourValue(1.0f,1.0f,0.0f,1.0f));
        m_manualObject->position(cp, cm, cm);// a vertex
        m_manualObject->colour(Ogre::ColourValue(1.0f,0.0f,0.0f,1.0f));
        m_manualObject->position(cm, cm, cm);// a vertex
        m_manualObject->colour(Ogre::ColourValue(0.0f,0.0f,0.0f,1.0f));

        m_manualObject->position(cm, cp, cp);// a vertex
        m_manualObject->colour(Ogre::ColourValue(0.0f,1.0f,1.0f,1.0f));
        m_manualObject->position(cp, cp, cp);// a vertex
        m_manualObject->colour(Ogre::ColourValue(1.0f,1.0f,1.0f,1.0f));
        m_manualObject->position(cp, cm, cp);// a vertex
        m_manualObject->colour(Ogre::ColourValue(1.0f,0.0f,1.0f,1.0f));
        m_manualObject->position(cm, cm, cp);// a vertex
        m_manualObject->colour(Ogre::ColourValue(0.0f,0.0f,1.0f,1.0f));

        // Create triangles - behind and front
        m_manualObject->triangle(0,1,2);
        m_manualObject->triangle(2,3,0);
        m_manualObject->triangle(4,6,5);
        m_manualObject->triangle(6,4,7);

        // Top and bottom
        m_manualObject->triangle(0,4,5);
        m_manualObject->triangle(5,1,0);
        m_manualObject->triangle(2,6,7);
        m_manualObject->triangle(7,3,2);

        // Left and right face
        m_manualObject->triangle(0,7,4);
        m_manualObject->triangle(7,0,3);
        m_manualObject->triangle(1,5,6);
        m_manualObject->triangle(6,2,1);
    }
    m_manualObject->end();

    m_manualObject->setDynamic( false );

    // Attach manual object to the scene
    m_sceneNode->attachObject( m_manualObject );

    return true;
}

void srs_ui_but::CButDisplay::destroyGeometry()
{
    // Destroy manual object
    if( m_manualObject != 0 )
        scene_manager_->destroyManualObject( m_manualObject );

    // Destroy scene
    if( m_sceneNode != 0 )
        scene_manager_->destroySceneNode(m_sceneNode->getName());
}

void srs_ui_but::CButDisplay::update (float wall_dt, float ros_dt)
{
    rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
    if( panel == 0 )
    {
        ROS_DEBUG( "No render panel... ");
    }

/*    Ogre::Camera * camera = panel->getCamera();

    if( camera == 0 )
    {
        ROS_DEBUG( "No camera ");
    }*/
}

/**
  * View controller has changed signal slot
  */
void srs_ui_but::CButDisplay::onViewControllerChange( rviz::ViewController * c )
{
}

