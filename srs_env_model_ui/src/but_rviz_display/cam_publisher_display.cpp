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
 * Date: 12/12/2012
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

#include <srs_env_model_ui/but_rviz_display/cam_publisher_display.h>
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
srs_env_model_ui::CCameraPublisherDisplay::CCameraPublisherDisplay(const std::string & name,rviz::VisualizationManager * manager)
    : Display( name, manager )
    , m_sceneNode( NULL )
    , m_manualObject( NULL )
    , m_listener( this )
    , m_cameraPositionPublisherName( CAMERA_POSITION_TOPIC_NAME )
    , m_latchedTopics( false )
{
	// Get node handle
	ros::NodeHandle private_nh("~");

	// Set parameters
	private_nh.param("camera_position_publisher_name", m_cameraPositionPublisherName, m_cameraPositionPublisherName);

    // Connect to the controller changed signal
    vis_manager_->getViewControllerTypeChangedSignal().connect( boost::bind(&CCameraPublisherDisplay::onViewControllerChange, this, _1 ) );

    // Get scene node
//    m_sceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // Try to connect camera listener
    connectListener();

//    rviz::WindowManagerInterface * wi( manager->getWindowManager() );

    // Create publisher
    this->m_cameraPositionPub = private_nh.advertise< srs_env_model_msgs::RVIZCameraPosition >(
    		m_cameraPositionPublisherName, 1, m_latchedTopics);
//    this->m_cameraPositionPub = private_nh.advertise< srs_env_model_msgs::RVIZCameraPosition >(
//    		m_cameraPositionPublisherName, 100, m_latchedTopics);
}

/*
 *  Destructor
 */
srs_env_model_ui::CCameraPublisherDisplay::~CCameraPublisherDisplay()
{
    // Destroy all geometry
    destroyGeometry();

    // Disconnect listener
    m_listener.disconnect();
}

/**
  *
  */
void srs_env_model_ui::CCameraPublisherDisplay::targetFrameChanged()
{
}

/**
  *
  */
void srs_env_model_ui::CCameraPublisherDisplay::fixedFrameChanged()
{
    m_listener.setCamFrameId( target_frame_ );

    ROS_INFO("Target frame: %s", target_frame_.c_str());
}

/**
  *
  */
void srs_env_model_ui::CCameraPublisherDisplay::createProperties()
{
    // Create some properties
/*    rviz::CategoryPropertyWPtr category = property_manager_->createCategory( "Camera Position", property_prefix_, parent_category_ );
    if( category.expired() )
    {
        ROS_INFO("Pointer expired...");
        return;
    }*/
    m_property_position = property_manager_->createProperty<rviz::StringProperty>( "Camera Position (not editable)", property_prefix_,
                              boost::bind( &CCameraPublisherDisplay::getCameraPositionString, this),
                              rviz::StringProperty::Setter(),
                              parent_category_ );
    //m_property_position.lock();

    if( m_property_position.expired() )
    {
        ROS_INFO("Property expired... ");
    }

    // Set help text
    setPropertyHelpText(m_property_position, "Current camera position.");
}

/*
 *  Display enabled callback
 */
void srs_env_model_ui::CCameraPublisherDisplay::onEnable()
{
//    m_sceneNode->setVisible( true );
}

/*
 *  Display disabled callback
 */
void srs_env_model_ui::CCameraPublisherDisplay::onDisable()
{
//    m_sceneNode->setVisible( false );
}

/*
 *  Create geometry
 */
bool srs_env_model_ui::CCameraPublisherDisplay::createGeometry()
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

void srs_env_model_ui::CCameraPublisherDisplay::destroyGeometry()
{
    // Destroy manual object
/*    if( m_manualObject != 0 )
        scene_manager_->destroyManualObject( m_manualObject );

    // Destroy scene
    if( m_sceneNode != 0 )
        scene_manager_->destroySceneNode(m_sceneNode->getName());*/
}

//! Update display
void srs_env_model_ui::CCameraPublisherDisplay::update (float wall_dt, float ros_dt)
{
    rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
    if( panel == 0 )
    {
        ROS_DEBUG( "No render panel... ");
    }

    Ogre::Camera * camera = panel->getCamera();

    if( camera == 0 )
    {
        ROS_DEBUG( "No camera...");
    }
}

/**
    View controller has changed signal slot
  */
void srs_env_model_ui::CCameraPublisherDisplay::onViewControllerChange( rviz::ViewController * c )
{
    connectListener();
}

/**
Connect listener
*/
void srs_env_model_ui::CCameraPublisherDisplay::connectListener()
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
void srs_env_model_ui::CCameraPublisherDisplay::propertyPositionChanged()
{
    if( !m_property_position.expired() )
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
  Camera Listener - get camera position string
  */
const std::string srs_env_model_ui::CCameraPublisherDisplay::getCameraPositionString()
{
    std::stringstream ss;
    Ogre::Vector3 position( m_listener.getCameraPosition() );
    ss << position.x << ", " << position.y << ", " << position.z;

    return ss.str();
}

/**
  Camera Listener - Set property string - camera position
  */
void srs_env_model_ui::CCameraPublisherDisplay::setCameraPositionString( const std::string & str )
{
    ROS_DEBUG("Set camera position called unexpectedly!");
}




/**
  Camera listener - constructor
  */
srs_env_model_ui::CNotifyCameraListener::CNotifyCameraListener( srs_env_model_ui::CCameraPublisherDisplay * display )
    : m_position( 0.0f, 0.0f, 0.0f )
    , m_orientation( 0.0, 0.0, 0.0, 0.0 )
    , m_camera( 0 )
    , m_display( display )
// Majkl: changed to test if it is better to publish TF transf. to the /map or /base_link
//    , m_camFrameId( "/map" )
    , m_camFrameId( "/base_link" )
{

}

/**
  Camera listener - destructor
  */
srs_env_model_ui::CNotifyCameraListener::~CNotifyCameraListener()
{
    disconnect();
}

/**
  Camera Listener - callback
  */
void srs_env_model_ui::CNotifyCameraListener::cameraPreRenderScene(Ogre::Camera *cam)
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

    // Publish tf transform
    static tf::TransformBroadcaster br;

    tf::Transform transform;
	transform.setOrigin( tf::Vector3( -position.z, -position.x, position.y ) );
	transform.setRotation( tf::Quaternion(-orientation.z, -orientation.x, orientation.y, orientation.w ) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), m_camFrameId, RVIZ_TF_NAME ));

}

/**
    Camera listener - Connect to the camera
  */
void srs_env_model_ui::CNotifyCameraListener::connect( Ogre::Camera * camera )
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
bool srs_env_model_ui::CNotifyCameraListener::hasMoved( const Ogre::Vector3 & position, const Ogre::Quaternion & orientation )
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
void srs_env_model_ui::CNotifyCameraListener::changedCB( const Ogre::Vector3 & position, const Ogre::Quaternion & orientation )
{
    m_display->propertyPositionChanged();


}

/**
  Camera Listener - disconnect
  */
void srs_env_model_ui::CNotifyCameraListener::disconnect()
{
    if( m_camera != 0 )
    {
        // Disconnect from the old camera
        m_camera->removeListener( this );
        m_camera = 0;
    }
}
