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

#include <srs_env_model_ui/but_rviz_display/octomap_display.h>
#include <srs_env_model_ui/topics_list.h>

#include <OGRE/OgreSceneManager.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

/*
 *  Constructor
 */
srs_env_model_ui::COctomapDisplay::COctomapDisplay(const std::string & name,rviz::VisualizationManager * manager)
    : Display( name, manager )
    , m_sceneNode( NULL )
    , m_manualObject( NULL )
{
	// Get node handle
/*	ros::NodeHandle private_nh("~");

	// Set parameters
	private_nh.param("xyz", m_xyz, m_xyz);*/

    // Connect to the controller changed signal
    vis_manager_->getViewControllerTypeChangedSignal().connect( boost::bind(&COctomapDisplay::onViewControllerChange, this, _1 ) );

    // Get scene node
//    m_sceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

    rviz::WindowManagerInterface * wi( manager->getWindowManager() );

    if( wi != 0 )
    {
    	// Create octomap control pane
    	m_ocmap_window = new COctomapControlPane( wi->getParentWindow(), _T("Octomap control"), wi );

    	if( m_ocmap_window != 0 )
    	{
    		m_ocmap_window->fixedFrameChanged( fixed_frame_ );
    		m_ocmap_window->targetFrameChanged( target_frame_ );

    		wi->addPane( "Octomap control", m_ocmap_window );
    		wi->showPane( m_ocmap_window );
    		ROS_DEBUG("Octomap control pane added...");

    	}
    }
    else
    {
        ROS_DEBUG( "No window manager, no panes :(" );
    }
}

/*
 *  Destructor
 */
srs_env_model_ui::COctomapDisplay::~COctomapDisplay()
{
    // Destroy all geometry
    destroyGeometry();
}

/**
  *
  */
void srs_env_model_ui::COctomapDisplay::targetFrameChanged()
{
    m_ocmap_window->targetFrameChanged( target_frame_ );
}

/**
  *
  */
void srs_env_model_ui::COctomapDisplay::fixedFrameChanged()
{
    m_ocmap_window->fixedFrameChanged( fixed_frame_ );

    ROS_INFO("Target frame: %s", target_frame_.c_str());
}


/**
  *
  */
void srs_env_model_ui::COctomapDisplay::createProperties()
{
    // Create some properties
/*    m_property_position = property_manager_->createProperty<rviz::StringProperty>( "XYZ: ", property_prefix_,
                                  boost::bind( &COctomapDisplay::getCameraPositionString, this),
                                  boost::bind( &COctomapDisplay::setCameraPositionString, this, _1),
                                  parent_category_ );
    if( m_property_position.expired() )
    {
        ROS_INFO("Property expired...");
    }

    // Set help text
    setPropertyHelpText(m_property_position, "XYZ desc...");*/
}

/*
 *  Display enabled callback
 */
void srs_env_model_ui::COctomapDisplay::onEnable()
{
//    m_sceneNode->setVisible( true );

//    m_ocmap_window->Show(true);
}

/*
 *  Display disabled callback
 */
void srs_env_model_ui::COctomapDisplay::onDisable()
{
//    m_sceneNode->setVisible( false );

//    m_ocmap_window->Show(false);
}

/*
 *  Create geometry
 */
bool srs_env_model_ui::COctomapDisplay::createGeometry()
{
    // Create manual object
/*    m_manualObject = scene_manager_->createManualObject( "manual" );

    // Create some geometry
    m_manualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
    {
        ...
    }
    m_manualObject->end();

    m_manualObject->setDynamic( false );

    // Attach manual object to the scene
    m_sceneNode->attachObject( m_manualObject );*/

    return true;
}

void srs_env_model_ui::COctomapDisplay::destroyGeometry()
{
    // Destroy manual object
/*    if( m_manualObject != 0 )
        scene_manager_->destroyManualObject( m_manualObject );

    // Destroy scene
    if( m_sceneNode != 0 )
        scene_manager_->destroySceneNode(m_sceneNode->getName());*/
}

//! Update display
void srs_env_model_ui::COctomapDisplay::update (float wall_dt, float ros_dt)
{
/*    rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
    if( panel == 0 )
    {
        ROS_DEBUG( "No render panel... ");
    }*/
}

/**
    View controller has changed signal slot
  */
void srs_env_model_ui::COctomapDisplay::onViewControllerChange( rviz::ViewController * c )
{
}
