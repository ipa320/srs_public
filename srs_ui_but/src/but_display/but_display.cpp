/******************************************************************************
 * \file
 *
 * $Id: but_display.cpp 396 2012-03-29 12:24:03Z spanel $
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

#define CAMERA_POSITION_TOPIC_NAME std::string("/rviz_camera_position")

/*
 *  Constructor
 */
CButDisplay::CButDisplay(const std::string & name,rviz::VisualizationManager * manager)
    : Display( name, manager )
    , m_manualObject( 0 )
    , m_listener( this )
    , m_child_window( 0 )
    , m_dialog_window( 0 )
    , m_controls_window( 0 )
	, m_cameraPositionPublisherName( CAMERA_POSITION_TOPIC_NAME )
    , m_latchedTopics( false )
{
	// Get node handle
	ros::NodeHandle private_nh("~");

	// Set parameters
	private_nh.param("camera_position_publisher_name", m_cameraPositionPublisherName, m_cameraPositionPublisherName);

    // Connect to the controller changed signal
    vis_manager_->getViewControllerTypeChangedSignal().connect( boost::bind(&CButDisplay::onViewControllerChange, this, _1 ) );

    // Get scene node
    m_sceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // Try to connect camera listener
    connectListener();

    // Try to create, add and show example window

    rviz::WindowManagerInterface * wi( manager->getWindowManager() );

    if( wi != 0 )
    {
        
        

        // Arm manipulation controls
        m_armmanipulation_window = new CArmManipulationControls( wi->getParentWindow(), wxT("Manual arm navigation"), wi);

        if( m_armmanipulation_window != 0 )
        {
            std::cerr << "Adding to the window manager..." << std::endl;
            wi->addPane( "Manual arm manipulation", m_armmanipulation_window );
            wi->showPane( m_armmanipulation_window );
            std::cerr << "Added..." << std::endl;
        }
        
    }else{
        std::cerr << "No window manager, no panes :( " << std::endl;
    }

    // Create publisher
    this->m_cameraPositionPub = private_nh.advertise< srs_env_model_msgs::RVIZCameraPosition >(
    		m_cameraPositionPublisherName, 100, m_latchedTopics);

}

/*
 *  Destructor
 */
CButDisplay::~CButDisplay()
{
    // Destroy all geometry
    destroyGeometry();

    // Disconnect listener
    m_listener.disconnect();
}

/**

  */
void CButDisplay::createProperties()
{

    // Create some properties
    rviz::CategoryPropertyWPtr category = property_manager_->createCategory( "Camera position", property_prefix_, parent_category_ );

    if( category.expired() )
    {
        std::cerr << "Pointer expired..." << std::endl;
        return;
    }
    m_property_position = property_manager_->createProperty<rviz::StringProperty>( "Position: ", property_prefix_,
                                                      boost::bind( &CButDisplay::getCameraPositionString, this),
                                                      boost::bind( &CButDisplay::setCameraPositionString, this, _1),
                                                          parent_category_ );
    //m_property_position.lock();

    if( m_property_position.expired() )
    {
        std::cerr << "Property expired... " << std::endl;
    }

    // Set help text
    setPropertyHelpText(m_property_position, "Current camera position.");
}

/*
 *  Display enablet callback
 */
void CButDisplay::onEnable()
{
    m_sceneNode->setVisible( true );
}

/*
 *  Display disabled callback
 */
void CButDisplay::onDisable()
{
    m_sceneNode->setVisible( false );
}

/*
 *  Create geometry
 */
bool CButDisplay::createGeometry()
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

void CButDisplay::destroyGeometry()
{
    // Destroy manual object
    if( m_manualObject != 0 )
        scene_manager_->destroyManualObject( m_manualObject );

    // Destroy scene
    if( m_sceneNode != 0 )
        scene_manager_->destroySceneNode(m_sceneNode->getName());
}

//! Update display
void CButDisplay::update (float wall_dt, float ros_dt)
{
    rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
    if( panel == 0 )
    {
        ROS_DEBUG( "No render panel... ");
    }

    Ogre::Camera * camera = panel->getCamera();

    if( camera == 0 )
    {
        ROS_DEBUG( "No camera ");
    }
}

/**
    View controller has changed signal slot
  */
void CButDisplay::onViewControllerChange( rviz::ViewController * c )
{
    connectListener();
}

/**
Connect listener
*/
void CButDisplay::connectListener()
{
    // Try to get camera and add listener
    rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
    if( panel == 0 )
    {
        ROS_DEBUG( "No render panel... ");
    }

    Ogre::Camera * camera = panel->getCamera();

    m_listener.connect( camera );
}

//! Property has changed
void CButDisplay::propertyPositionChanged()
{
    if( ! m_property_position.expired() )
    {
        propertyChanged( m_property_position );

        // Publish changes
        if( m_latchedTopics || m_cameraPositionPub.getNumSubscribers() > 0 )
        {
        	// Store start time
        	ros::Time rostime = ros::Time::now();

        	// Prepare header
        	m_cameraPositionMsg.header.frame_id = target_frame_;
        	m_cameraPositionMsg.header.stamp = rostime;

        	// Get camera
        	Ogre::Camera & camera( m_listener.getCamera() );

        	// Fill message data
        	Ogre::Vector3 position( camera.getPosition() );
        	m_cameraPositionMsg.position.x = position.x;
        	m_cameraPositionMsg.position.y = position.y;
        	m_cameraPositionMsg.position.z = position.z;

        	Ogre::Vector3 direction( camera.getDirection() );
        	m_cameraPositionMsg.direction.x = direction.x;
        	m_cameraPositionMsg.direction.y = direction.y;
        	m_cameraPositionMsg.direction.z = direction.z;


        	Ogre::Quaternion orientation( camera.getOrientation() );
        	m_cameraPositionMsg.orientation.w = orientation.w;
        	m_cameraPositionMsg.orientation.x = orientation.x;
        	m_cameraPositionMsg.orientation.y = orientation.y;
        	m_cameraPositionMsg.orientation.z = orientation.z;

        	// Publish message
        	m_cameraPositionPub.publish( m_cameraPositionMsg );

        }
    }
}

/**
  Camera listener - constructor
  */
CButDisplay::CNotifyCameraListener::CNotifyCameraListener( CButDisplay * display )
    : m_position( 0.0f, 0.0f, 0.0f )
    , m_orientation( 0.0, 0.0, 0.0, 0.0 )
    , m_camera( 0 )
    , m_display( display )
{

}

/**
  Camera listener - destructor
  */
CButDisplay::CNotifyCameraListener::~CNotifyCameraListener()
{
    disconnect();
}

/**
  Camera Listener - callback
  */
void CButDisplay::CNotifyCameraListener::cameraPreRenderScene(Ogre::Camera *cam)
{

    Ogre::Vector3 position( cam->getPosition() );
    Ogre::Quaternion orientation ( cam->getOrientation() );

    if( hasMoved( position, orientation ) )
    {
      //cam->getCullingFrustum();

        // callback
        changedCB( position, orientation );

        // Update stored position
        m_position = position;
        m_orientation = orientation;
    }
}

/**
    Camera listener - Connect to the camera
  */
void CButDisplay::CNotifyCameraListener::connect( Ogre::Camera * camera )
{
    // Old connection
    disconnect();

    // Some problem here?
    if( camera == 0 )
        return;

    // Connect listener to the camera
    camera->addListener( this );
    m_camera = camera;
}


/**
    Camera Listener - Test if camera position and orientation has changed
  */
bool CButDisplay::CNotifyCameraListener::hasMoved( const Ogre::Vector3 & position, const Ogre::Quaternion & orientation )
{
#define far( x ) ( abs(x) > 0.001f )

    // test position
    if( far( position.x - m_position.x ) || far( position.y - m_position.y ) || far( position.z - position.z ) )
    {
        return true;
    }

    // Test orientation
    for( int i = 0; i < 4; ++i)
        if( far( orientation[i] - m_orientation[i] ))
        {
            return true;
        }

    return false;
}

/**
  Camera Listener - camera has moved
  */
void CButDisplay::CNotifyCameraListener::changedCB( const Ogre::Vector3 & position, const Ogre::Quaternion & orientation )
{
    m_display->propertyPositionChanged();
}

/**
  Camera Listener - disconnect
  */
void CButDisplay::CNotifyCameraListener::disconnect()
{
    if( m_camera != 0 )
    {
        // Disconnect from the old camera
        m_camera->removeListener( this );
        m_camera = 0;
    }
}

/**
  Camera Listener - get camera position string
  */
const std::string CButDisplay::getCameraPositionString()
{
    std::stringstream ss;
    Ogre::Vector3 position( m_listener.getCameraPosition() );
    ss << position.x << ", " << position.y << ", " << position.z;

    return ss.str();
}

/**
  Camera Listener - Set property string - camera position
  */
void CButDisplay::setCameraPositionString( const std::string & str )
{
    std::cerr << "Set str called" << std::endl;
}


