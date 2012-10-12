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

#include "but_projection.h"

// Ros includes
#include <ros/package.h>

// Ogre includes
#include <OGRE/OgreSceneManager.h>
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"
#include "OGRE/OgreCompositorManager.h"
#include <ogre_tools/initialization.h>
#include "OgreHardwarePixelBuffer.h"

#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>

// Rviz includes
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/frame_manager.h>
#include <rviz/validate_floats.h>

#include <wx/filedlg.h>

// Ros image message
#include <sensor_msgs/Image.h>

// Std includes
#include <sstream>

#define DEFAULT_IMAGE_INPUT_TOPIC_NAME std::string("/cam3d/rgb/image_raw")
#define DEFAULT_DEPTH_IMAGE_TOPIC_NAME std::string("/srs_ui_but/depth/image")
#define CAMERA_INFO_TOPIC_NAME std::string("/cam3d/rgb/camera_info")
#define DEFAULT_PUBLISHING_PERIOD double(0.1)

/**
 * Material listener constructor
 */
/*
srs_ui_but::CMaterialListener::CMaterialListener( const std::string & materialName, const std::string & groupName, const std::string & schemeName )
: m_schemeName( schemeName )
{
	std::cerr << "Listener constructor 1" << std::endl;

    // Create material
   // m_materialPtr = Ogre::MaterialManager::getSingleton().getByName( materialName, groupName );

    // Test if success
    if( m_materialPtr.get() == 0 )
        {
        std::cerr << "Cannot get material..." << std::endl;
        }
    else
        {
        // Load it
    	//m_materialPtr->load();

        // Write info
        std::cerr << "Material " << groupName << ":" << materialName << " loaded..." << std::endl;
        std::cerr << "Num of techniques: " << m_materialPtr->getNumTechniques() << std::endl;
        m_materialPtr->getTechnique(0);
        }

    std::cerr << "Listener constructor 1 done" << std::endl;
}
*/

/**
 * Material listener constructor
 */
srs_ui_but::CMaterialListener::CMaterialListener( Ogre::Material * material, const std::string & schemeName )
: m_materialPtr( material )
, m_schemeName( schemeName )
, m_bDistanceChanged( true )
, m_testingDistance( 1.0 )
{
	return;

//    	std::cerr << "Listener constructor 2" << std::endl;

    // Test if success
    if( m_materialPtr == 0 )
        {
        std::cerr << "Cannot get material..." << std::endl;
        }
    else
        {
        // Load it
    	//m_materialPtr->load();

        // Write info
//        std::cerr << "Material assigned..." << std::endl;
 //       std::cerr << "Num of techniques: " << m_materialPtr->getNumTechniques() << std::endl;
        m_materialPtr->getTechnique(0);
        }

//    	std::cerr << "Listener constructor 2 done" << std::endl;
}

/**
 * Material listener destructor
 */
srs_ui_but::CMaterialListener::~CMaterialListener()
{
//	m_materialPtr->removeAllTechniques();
//	m_materialPtr->unload();
//	m_materialPtr.setNull();
}

/**
 * Scheme not found event handler - return stored material technique
 */
Ogre::Technique *srs_ui_but::CMaterialListener::handleSchemeNotFound(unsigned short, const Ogre::String& schemeName, Ogre::Material*mat, unsigned short, const Ogre::Renderable*)
{
    if (schemeName == m_schemeName)
        {

    	// Change shader settings?
    	if( m_bDistanceChanged )
    	{
    		m_bDistanceChanged = false;


    		Ogre::GpuProgramParametersSharedPtr paramsFP( m_materialPtr->getBestTechnique()->getPass(0)->getFragmentProgramParameters() );

			if( paramsFP->_findNamedConstantDefinition("testedDistance"))
			{
				paramsFP->setNamedConstant( "testedDistance", m_testingDistance );
			}
			else
			{
				std::cerr << "Named constant not found: projectionMatrix " << std::endl;
			}

    	}

        //LogManager::getSingleton().logMessage(">> adding glow to material: "+mat->getName());
        return m_materialPtr->getBestTechnique();
        }
    return NULL;
}

/**
 * Set tested distance
 */
void srs_ui_but::CMaterialListener::setTestedDistance( float distance )
{
	m_bDistanceChanged = true;
	m_testingDistance = distance;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/**
 * Constructor with given external ros texture
 */
srs_ui_but::CProjectionData::CProjectionData( Ogre::SceneManager * manager, const ros::NodeHandle &nh, const std::string & materialName, const std::string & groupName )
: m_texW( 512 )
, m_texH( 512 )
, m_mode( TM_ROS )
, m_manager( manager )
{
 //   std::cerr << "CProjectionData constructor" << std::endl;

    // Create frustum and projector node
    m_frustum = new Ogre::Frustum();
    m_frustum->setNearClipDistance( 0.8 );

    m_projectorNode = manager->getRootSceneNode()->createChildSceneNode("DecalProjectorNode");
    m_projectorNode->attachObject(m_frustum);
    m_projectorNode->setPosition(0,5,0);


    // Create ros textures
    m_textureRosRGB = new tRosTextureRGB( nh, "rgb_texture", "rgb8" );
    assert( m_textureRosRGB != 0 );
    m_textureRosDepth = new tRosTextureDepth( nh, "depth_texture", DEFAULT_DEPTH_IMAGE_TOPIC_NAME, "32FC4" );
    assert( m_textureRosDepth != 0 );

    // Create and init material
    m_materialPtr = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(materialName, groupName);
    m_materialPtr->load();
//    std::cerr << "Num of techniques: " << m_materialPtr->getNumTechniques() << std::endl;
    m_materialPtr->getTechnique(0)->setLightingEnabled(false);

    Ogre::Pass *pass = m_materialPtr->getTechnique(0)->getPass(0);

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);
    pass->setLightingEnabled(false);

    // Connect rgb texture and material
    m_texState = pass->createTextureUnitState(m_textureRosRGB->getTexture()->getName());
    m_texState->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    m_texState->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);

    Ogre::LayerBlendOperationEx op = Ogre::LBX_MODULATE;
    m_texState->setColourOperationEx(op, Ogre::LBS_TEXTURE, Ogre::LBS_TEXTURE);

    // Connect Depth texture and material
    m_texState = pass->createTextureUnitState(m_textureRosDepth->getTexture()->getName());
    m_texState->setDesiredFormat( Ogre::PF_FLOAT32_RGB );
    m_texState->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    m_texState->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);

    m_texState->setColourOperationEx(op, Ogre::LBS_TEXTURE, Ogre::LBS_TEXTURE);

    updateMatrices();

//    std::cerr << "CProjectionData constructor done" << std::endl;


}

/**
 * Destructor
 */

//! Destructor
srs_ui_but::CProjectionData::~CProjectionData()
{
//	std::cerr << "CProjection data destructor start." << std::endl;
//	m_material->unload();
	//Ogre::MaterialManager::getSingleton().remove( m_material->getHandle() );
//	Ogre::MaterialManager::getSingleton().remove( m_material->getName() );

//	m_projectorNode->detachAllObjects();
	m_manager->destroySceneNode( m_projectorNode );

	// Remove all
	delete m_textureRosDepth;
	delete m_textureRosRGB;
//	delete m_projectorNode; m_projectorNode = 0;
	delete m_frustum;

//	std::cerr << "CProjection data destructor end." << std::endl;
}

/**
 * Set frustrum size
 */
void srs_ui_but::CProjectionData::setFrustrumSize( Ogre::Vector2 size )
{
    if( size.y == 0 )
        return; // wrong size

    if( m_frustrumSize != size )
        {
        m_frustrumSize = size;

        m_frustum->setAspectRatio( size.x / size.y );

        m_frustum->setOrthoWindowHeight( size.y );
        }

    // Update projection matrix
    updateMatrices();
}

/**
 * Set ros rgb texture topic
 */
void srs_ui_but::CProjectionData::setRGBTextureTopic( const std::string & topic )
{
    if( m_mode == TM_ROS && m_textureRosRGB != 0 )
        {
        m_textureRosRGB->setTopic( topic );
        }
}

/**
 * Set ros depth texture topic
 */
void srs_ui_but::CProjectionData::setDepthTextureTopic( const std::string & topic )
{
    if( m_mode == TM_ROS && m_textureRosDepth != 0 )
        {
    	m_textureRosDepth->setTopic( topic );

 //       std::cerr << std::endl << "Depth texture topic set: " << m_textureRosDepth->getTopic() << std::endl;
        }
}

/**
 * Clear texture
 */
void srs_ui_but::CProjectionData::clear()
{
    if( m_textureRosRGB != 0 )
        m_textureRosRGB->clear();

    if( m_textureRosDepth != 0 )
    	m_textureRosDepth->clear();
}


//! Update material projection matrix parameter
void srs_ui_but::CProjectionData::updateMatrices()
{

	if( m_materialPtr.get() == 0 )
		return;
/*
	Ogre::Matrix4 wt;

//	std::cerr << "SetProjectionMatrix" << std::endl;

	// Get camera to world matrix
	{
		tf::StampedTransform tfTransform;

		// Get transform
		try {
			// Transformation - to, from, time, waiting time
			m_tfListener.waitForTransform("/map", "/head_cam3d_link",
						ros::Time(0), ros::Duration(5));

			m_tfListener.lookupTransform("/map", "/head_cam3d_link",
					ros::Time(0), tfTransform);

		} catch (tf::TransformException& ex) {
			ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		//	PERROR( "Transform error.");
			return;
		}

		Eigen::Matrix4f trMatrix;

		// Get transformation matrix
		pcl_ros::transformAsMatrix(tfTransform, trMatrix);	// Sensor TF to defined base TF

		for( int i = 0; i < 4; ++i)
			for( int j = 0; j < 4; ++j)
			{
				wt[i][j] = trMatrix(i, j);
			}
	}
*/




	Ogre::Matrix4 pm( Ogre::Matrix4::CLIPSPACE2DTOIMAGESPACE * m_frustum->getProjectionMatrixWithRSDepth() );

	// Compute projection matrix. CLIPSPACE2DTOIMAGESPACE: Useful little matrix which takes 2D clipspace {-1, 1} to {0,1} and inverts the Y.
	Ogre::Matrix4 fm( pm * m_frustum->getViewMatrix() );


//	m_frustum->getWorldTransforms( &wt );

//	std::cerr << "WT:" << std::endl << wt << std::endl;

	m_textureRosDepth->setMatrix( pm  );
	//m_textureRosDepth->setCameraModel( m_camera_model );


	// Set vertex program parameters
	Ogre::GpuProgramParametersSharedPtr paramsVP( m_materialPtr->getTechnique(0)->getPass(0)->getVertexProgramParameters() );

	if( paramsVP->_findNamedConstantDefinition("texViewProjMatrix"))
	{
		paramsVP->setNamedConstant( "texViewProjMatrix", fm );
	}
	else
	{
		std::cerr << "Named constant not found..." << std::endl;
	}

	if( paramsVP->_findNamedConstantDefinition("cameraPosition"))
	{
		Ogre::Vector4 position( m_cameraPosition[0], m_cameraPosition[1], m_cameraPosition[2], 1.0 );
		paramsVP->setNamedConstant( "cameraPosition", position );
	}
	else
	{
		std::cerr << "Named constant not found: cameraPosition " << std::endl;
	}

	if( paramsVP->_findNamedConstantDefinition("cameraPlane"))
		{
			Ogre::Vector4 position( m_cameraEquation[0], m_cameraEquation[1], m_cameraEquation[2], m_cameraEquation[3] );
			paramsVP->setNamedConstant( "cameraPlane", position );
		}
		else
		{
			std::cerr << "Named constant not found: cameraPlane " << std::endl;
		}

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *  Constructor
 */
srs_ui_but::CButProjection::CButProjection(const std::string & name,rviz::VisualizationManager * manager)
: Display( name, manager )
, m_pane( 0 )
, m_projectionData( 0 )
, m_camera_info_topic( CAMERA_INFO_TOPIC_NAME )
, m_ml(0)
, m_bMLConnected( false )
{
//	std::cerr << "CButProjection::CButProjection S" << std::endl;

    // Create and connect pane
    rviz::WindowManagerInterface * wi( manager->getWindowManager() );

    if( wi != 0 )
        {
        // Arm manipulation controls
        m_pane = new CControllPane( wi->getParentWindow(), wxT("Material test"), wi);

        if( m_pane != 0 )
            {
            wi->addPane( "Material test", m_pane );
            wi->showPane( m_pane );

            // Connect signal
            m_pane->getSigChckBox().connect( boost::bind( &CButProjection::onPublishStateChanged, this, _1) );
            m_pane->getSigSave().connect( boost::bind( &CButProjection::onSave, this, _1 ) );
            }

        }else{
            std::cerr << "No window manager, no panes :( " << std::endl;
        }


    // Get node handle
    ros::NodeHandle private_nh("/");

    // Set parameters

    // Get image input topic names
    private_nh.param("rgb_image_input_topic", m_imageRGBInputTopicName, DEFAULT_IMAGE_INPUT_TOPIC_NAME );
    private_nh.param("depth_image_input_topic", m_imageDepthInputTopicName, DEFAULT_DEPTH_IMAGE_TOPIC_NAME );

    // Get timer period
    private_nh.param("publishig_period", m_timerPeriod, DEFAULT_PUBLISHING_PERIOD );

    // Get scene node
    m_sceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // Add camera info subscriber
    m_ciSubscriber = new message_filters::Subscriber< sensor_msgs::CameraInfo >(private_nh, m_camera_info_topic, 10 );

    //*m_ciSubscriber = private_nh.subscribe( m_camera_info_topic, 10, &srs_ui_but::CButProjection::cameraInfoCB, this );


    // initializing of listeners for tf transformation and tf message filter
    m_tf_cam_info_Listener = new tf::TransformListener();
    m_camInfoTransformFilter = new tf::MessageFilter<sensor_msgs::CameraInfo>(*m_ciSubscriber,
                                    *m_tf_cam_info_Listener, "/map", 10);

    m_camInfoTransformFilter->registerCallback( boost::bind( &srs_ui_but::CButProjection::cameraInfoCB, this, _1 ));

    // Create geometry
    createGeometry(private_nh);

    // Create publishing timer
    m_timer = private_nh.createTimer( ros::Duration(m_timerPeriod), boost::bind( &srs_ui_but::CButProjection::onTimerPublish, this, _1) );

    m_timer.start();

 //   std::cerr << "CButProjection::CButProjection E" << std::endl;

}

/*
 *  Destructor
 */
srs_ui_but::CButProjection::~CButProjection()
{

//	std::cerr << "CButProjection destructor start." << std::endl;

	lockRender();

	m_timer.stop();

	// Unsubscribe from topics
	unsubscribe();
	m_ciSubscriber->unsubscribe();

    // Destroy all geometry
    destroyGeometry();

    // Destroy materials
    removeMaterials();

    unlockRender();

//    std::cerr << "CButProjection destructor end." << std::endl;

}


/*
 *  Display enabled callback
 */
void srs_ui_but::CButProjection::onEnable()
{
    m_sceneNode->setVisible( true );
    connectML( true );
}

/*
 *  Display disabled callback
 */
void srs_ui_but::CButProjection::onDisable()
{
    m_sceneNode->setVisible( false );
    connectML( false );
}

/**
 * Create properties
 */
void srs_ui_but::CButProjection::createProperties()
{
    m_rgb_topic_property = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "RGB image Topic", property_prefix_, boost::bind( &CButProjection::getRgbTopic, this ),
            boost::bind( &CButProjection::setRgbTopic, this, _1 ), parent_category_, this );

    m_depth_topic_property = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Depth image Topic", property_prefix_, boost::bind( &CButProjection::getDepthTopic, this ),
               boost::bind( &CButProjection::setDepthTopic, this, _1 ), parent_category_, this );

    m_distance_property = property_manager_->createProperty<rviz::FloatProperty>( "Tested distance", property_prefix_, boost::bind( &CButProjection::getTestedDistance, this ),
				boost::bind( &CButProjection::setTestedDistance, this, _1), parent_category_, this );

    // Set distance limits
    rviz::FloatPropertyPtr distp( m_distance_property.lock());
    distp->setMin( 0.0 ); distp->setMax( 10.0 );

    setPropertyHelpText(m_rgb_topic_property, "sensor_msgs::Image topic to subscribe to.");
    rviz::ROSTopicStringPropertyPtr topic_prop = m_rgb_topic_property.lock();
    topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Image>());

    // Set topics
    setRgbTopic( m_imageRGBInputTopicName );
    setDepthTopic( m_imageDepthInputTopicName );

}

/**
 * Subscribe texture to topic
 */
void srs_ui_but::CButProjection::subscribe()
{
	if( m_projectionData != 0 )
	{
		m_projectionData->setRGBTextureTopic(m_imageRGBInputTopicName);
		m_projectionData->setDepthTextureTopic(m_imageDepthInputTopicName);
	}

    if ( !isEnabled() )
        {
        return;
        }


}

/**
 * Unsubscribe from topics
 */
void srs_ui_but::CButProjection::unsubscribe()
{
	if( m_projectionData != 0 )
	{
		m_projectionData->setRGBTextureTopic("");
		m_projectionData->setDepthTextureTopic("");
	}
}

/**
 *
 */
void srs_ui_but::CButProjection::setRgbTopic( const std::string& topic )
{
    unsubscribe();

    m_imageRGBInputTopicName = topic;
    clear();

    subscribe();

    propertyChanged(m_rgb_topic_property);
}

/**
 *
 */
void srs_ui_but::CButProjection::setDepthTopic( const std::string& topic )
{
    unsubscribe();

    m_imageDepthInputTopicName = topic;
    clear();

    subscribe();

    propertyChanged(m_depth_topic_property);
}

/**
 * Clear texture, set status
 */
void srs_ui_but::CButProjection::clear()
{
	if( m_projectionData != 0 )
	{
		m_projectionData->clear();
	}

    setStatus(rviz::status_levels::Warn, "Image", "No Image received");
}

/**
 * Create used materials
 */
void srs_ui_but::CButProjection::createMaterials(Ogre::Camera * camera)
{
//	std::cerr << "CButProjection::createMaterials S" << std::endl;


 //   std::cerr << "1" << std::endl;

    // Load materials
//    Ogre::String nameOfResourceGroup( "MaterialGroup1" );
    {
        // Create resource group
        Ogre::ResourceGroupManager& lRgMgr = Ogre::ResourceGroupManager::getSingleton();
//        lRgMgr.createResourceGroup(nameOfResourceGroup);

 //       std::cerr << "2" << std::endl;

        // Get path
        std::string package_path( ros::package::getPath("srs_ui_but") );

        ogre_tools::V_string paths;
        Ogre::String resource_path(package_path + "/src/but_display/materials");

 //       std::cerr << "3" << std::endl;

//        std::cerr << "Materials path: " << resource_path.c_str() << std::endl;

//        std::cerr << "Exisist: " << lRgMgr.resourceGroupExists("srs_ui_but") << std::endl;

        if( ! lRgMgr.resourceGroupExists("srs_ui_but"))
        {
 //       	std::cerr << "Creating resource group: srs_ui_but" << std::endl;
            lRgMgr.createResourceGroup("srs_ui_but");
        }

//        std::cerr << "4" << std::endl;

        if( ! lRgMgr.isResourceGroupInitialised("srs_ui_but") )
        {
 //       	std::cerr << "Initializing resource group: srs_ui_but" << std::endl;
            lRgMgr.addResourceLocation(resource_path, "FileSystem", "srs_ui_but");
        	lRgMgr.initialiseResourceGroup("srs_ui_but");
        }

        if( ! lRgMgr.isResourceGroupLoaded("srs_ui_but") )
        {
 //       	std::cerr << "Loading resource group: srs_ui_but" << std::endl;
            lRgMgr.addResourceLocation(resource_path, "FileSystem", "srs_ui_but");
        	lRgMgr.loadResourceGroup("srs_ui_but");
        }


 //       std::cerr << "5" << std::endl;

//        std::cerr << "Loaded materials: " << std::endl;
/*

        // Get material manager
        Ogre::MaterialManager& lMaterialManager = Ogre::MaterialManager::getSingleton();

        // List loaded materials
        Ogre::ResourceManager::ResourceMapIterator materialIterator = lMaterialManager.getResourceIterator();

        // Write materials
        int count(0);
        while (materialIterator.hasMoreElements())
            {

            Ogre::String name( (static_cast<Ogre::MaterialPtr>(materialIterator.peekNextValue()))->getName() );

            std::cerr << name << std::endl;

            materialIterator.moveNext();
            count++;
            }

        std::cerr << "Num of materials: " << count << std::endl;
    //*/
    }


    // Load compositor
    {
        if( Ogre::CompositorManager::getSingleton().addCompositor(camera->getViewport(), "zTestedProjection") == 0 )
            {
            std::cerr << "COMPOSITOR FAILED TO LOAD." << std::endl;
            }
        else
            {
            Ogre::CompositorManager::getSingleton().setCompositorEnabled(camera->getViewport(), "zTestedProjection", true);

//            std::cerr << "Creating Projection data" << std::endl;

            //! Create material
            m_projectionData = new CProjectionData( scene_manager_, update_nh_, "tested_projection", "srs_ui_but" );

//            std::cerr << "Projection data done. Running CMaterialListener" << std::endl;

            // Create material listener
            m_ml = new CMaterialListener( m_projectionData->getMaterialPtr(), "myscheme" );

            connectML( true );

			}
    }
//*/
//    std::cerr << "CButProjection::createMaterials E" << std::endl;
}

//! Remove materials
void srs_ui_but::CButProjection::removeMaterials()
{
	connectML(false);

//	std::cerr << "Removing compositior" << std::endl;

	// Get camera and discard compositor.
	rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
	if( panel != 0 )
	{
		Ogre::Camera * camera = panel->getCamera();

		Ogre::CompositorManager::getSingleton().setCompositorEnabled(camera->getViewport(), "zTestedProjection", false);
		Ogre::CompositorManager::getSingleton().removeCompositor( camera->getViewport(), "zTestedProjection" );
	}


//	std::cerr << m_projectionData->getMaterialPtr() << std::endl;
//	std::cerr << "Removing listener" << std::endl;
    delete m_ml;

 //   std::cerr << m_projectionData->getMaterialPtr() << std::endl;
//    std::cerr << "Removing projection data "  << std::endl;
    if( m_projectionData != 0 )
    {
    	delete m_projectionData;
    }

 //   std::cerr << m_projectionData->getMaterialPtr() << std::endl;

//    std::cerr << "Destroying resource groups." << std::endl;

//	Ogre::String nameOfResourceGroup( "MaterialGroup1" );
	Ogre::ResourceGroupManager& lRgMgr = Ogre::ResourceGroupManager::getSingleton();
	//Ogre::MaterialManager &mMgr = Ogre::MaterialManager::getSingleton();

//	std::cerr << "RG: srs_ui_but. Status: " << lRgMgr.isResourceGroupInGlobalPool( "srs_ui_but" ) << ", " << lRgMgr.isResourceGroupInitialised("srs_ui_but") << ", " << lRgMgr.isResourceGroupLoaded("srs_ui_but") << std::endl;

//	lRgMgr.unloadResourceGroup("srs_ui_but");
	lRgMgr.destroyResourceGroup("srs_ui_but");
//	lRgMgr.clearResourceGroup(nameOfResourceGroup);

//	std::cerr << "RG: " << nameOfResourceGroup << std::endl;

//	lRgMgr.destroyResourceGroup( nameOfResourceGroup );


}


void srs_ui_but::CButProjection::connectML( bool bConnect )
{
	if( bConnect == m_bMLConnected || m_ml == 0 )
		return;

	if( bConnect )
	{
		Ogre::MaterialManager::getSingleton().addListener( m_ml );
//		std::cerr << "AAA: Listener added..." << std::endl;
	}
	else
	{
		Ogre::MaterialManager::getSingleton().removeListener( m_ml );
//		std::cerr << "AAA: Listener removed..." << std::endl;
	}

		m_bMLConnected = bConnect;
}

/*
 *  Create geometry
 */
bool srs_ui_but::CButProjection::createGeometry(const ros::NodeHandle & nh)
{
//	std::cerr << "CButProjection::createGeometry S" << std::endl;

    // Get camera.
    rviz::RenderPanel * panel = vis_manager_->getRenderPanel();
    if( panel == 0 )
        {
        ROS_DEBUG( "No render panel... ");
        return false;
        }

    Ogre::Camera * camera = panel->getCamera();

    // Create rtt texture
    //m_textureWithRtt = new CRosRttTexture( 512, 512, camera );

    /*
    Ogre::String lNameOfTheMesh = "MeshCube";
    {
        // Here, I create a 3D element, by using the interface of ManualObject.
        // ManualObject is very close to the opengl old simple way to specify geometry.
        // There are other interfaces (Hardwarebuffers), you can check the ogremanual fo them and wiki.
        // For each vertex I will provide positions and attributes (normal, vertex color, texture coordinates...).
        // Then for each primitive (given its type : triangle, line, line strip etc...),
        // I give the corresponding group of vertex index.
        Ogre::ManualObject* lManualObject = NULL;
        {
            // The manualObject creation requires a name.
            Ogre::String lManualObjectName = "CubeWithAxes";
            lManualObject = scene_manager_->createManualObject(lManualObjectName);

            // Always tell if you want to update the 3D (vertex/index) later or not.
            bool lDoIWantToUpdateItLater = false;
            lManualObject->setDynamic(lDoIWantToUpdateItLater);

            // Here I create a cube in a first part with triangles, and then axes (in red/green/blue).

            // BaseWhiteNoLighting is the name of a material that already exist inside Ogre.
            // Ogre::RenderOperation::OT_TRIANGLE_LIST is a kind of primitive.
            float lSize = 5.0f;
            lManualObject->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            {
                float cp = 1.0f * lSize ;
                float cm = -1.0f * lSize;

                lManualObject->position(cm, cp, cm);// a vertex
                lManualObject->colour(Ogre::ColourValue(0.0f,1.0f,0.0f,1.0f));
                lManualObject->textureCoord(0.0, 1.0);
                lManualObject->position(cp, cp, cm);// a vertex
                lManualObject->colour(Ogre::ColourValue(1.0f,1.0f,0.0f,1.0f));
                lManualObject->textureCoord(1.0, 1.0);
                lManualObject->position(cp, cm, cm);// a vertex
                lManualObject->colour(Ogre::ColourValue(1.0f,0.0f,0.0f,1.0f));
                lManualObject->textureCoord(1.0, 0.0);
                lManualObject->position(cm, cm, cm);// a vertex
                lManualObject->colour(Ogre::ColourValue(0.0f,0.0f,0.0f,1.0f));
                lManualObject->textureCoord(0.0, 0.0);

                lManualObject->position(cm, cp, cp);// a vertex
                lManualObject->colour(Ogre::ColourValue(0.0f,1.0f,1.0f,1.0f));
                lManualObject->textureCoord(0.0, 1.0);
                lManualObject->position(cp, cp, cp);// a vertex
                lManualObject->colour(Ogre::ColourValue(1.0f,1.0f,1.0f,1.0f));
                lManualObject->textureCoord(1.0, 1.0);
                lManualObject->position(cp, cm, cp);// a vertex
                lManualObject->colour(Ogre::ColourValue(1.0f,0.0f,1.0f,1.0f));
                lManualObject->textureCoord(1.0, 0.0);
                lManualObject->position(cm, cm, cp);// a vertex
                lManualObject->colour(Ogre::ColourValue(0.0f,0.0f,1.0f,1.0f));
                lManualObject->textureCoord(0.0, 0.0);

                // face behind / front
                lManualObject->triangle(0,1,2);
                lManualObject->triangle(2,3,0);
                lManualObject->triangle(4,6,5);
                lManualObject->triangle(6,4,7);

                // face top / down
                lManualObject->triangle(0,4,5);
                lManualObject->triangle(5,1,0);
                lManualObject->triangle(2,6,7);
                lManualObject->triangle(7,3,2);

                // face left / right
                lManualObject->triangle(0,7,4);
                lManualObject->triangle(7,0,3);
                lManualObject->triangle(1,5,6);
                lManualObject->triangle(6,2,1);
            }
            lManualObject->end();
            // Here I have finished my ManualObject construction.
            // It is possible to add more begin()-end() thing, in order to use
            // different rendering operation types, or different materials in 1 ManualObject.
        }

        lManualObject->convertToMesh(lNameOfTheMesh, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        // Now I can create several entities using that mesh
        int lNumberOfEntities = 1;
        for(int iter = 0; iter < lNumberOfEntities; ++iter)
            {
            Ogre::Entity* lEntity = scene_manager_->createEntity(lNameOfTheMesh);
            // Now I attach it to a scenenode, so that it becomes present in the scene.
            Ogre::SceneNode* lNode = scene_manager_->getRootSceneNode()->createChildSceneNode();
            lNode->attachObject(lEntity);
            // I move the SceneNode so that it is visible to the camera.
            float lPositionOffset = float(1+ iter * 5.0) - (float(lNumberOfEntities));
            lNode->translate(lPositionOffset, lPositionOffset, -10.0f);
            }
    }

    // Create entity on screen

    Ogre::Entity* lAdditionalEntity = scene_manager_->createEntity( lNameOfTheMesh );
    // lAdditionalEntity->setMaterialName( m_textureWithRtt->getMaterialName() );
    Ogre::SceneNode* lVisibleOnlyByFirstCam = scene_manager_->getRootSceneNode()->createChildSceneNode();
    lVisibleOnlyByFirstCam->attachObject( lAdditionalEntity );
    lVisibleOnlyByFirstCam->setPosition( 1.5,-0.5,-5);
    //*/

    // Create and initialize materials
    createMaterials( camera );

 //   std::cerr << "CButProjection::createGeometry E" << std::endl;

    return true;
}

void srs_ui_but::CButProjection::destroyGeometry()
{
    // Destroy scene
    if( m_sceneNode != 0 )
        scene_manager_->destroySceneNode(m_sceneNode->getName());
}

//! Update display
void srs_ui_but::CButProjection::update (float wall_dt, float ros_dt)
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

    try
    {
    	if( m_projectionData != 0 )
    	{
    		m_projectionData->update();
    	}
    }
    catch (UnsupportedImageEncoding& e)
    {
        setStatus(rviz::status_levels::Error, "Image", e.what());
    }
}

/*
 * Timer publishing callback function
 */

void srs_ui_but::CButProjection::onTimerPublish(const ros::TimerEvent&)
{

    //	m_projectionData->fillTexture( 255 - m_c, 128 + m_c, m_c );
    //	++m_c;
    /*
  if( m_bPublish && (m_camCastPublisher.getNumSubscribers() > 0 ) && m_textureWithRtt->hasData() )
  {
    // Publish image
    m_camCastPublisher.publish( m_textureWithRtt->getImage() );
  }
     */
}


/**
 * On publishing start/stop
 */
void srs_ui_but::CButProjection::onPublishStateChanged(bool state)
{

    if( state )
        {
        m_timer.start();
        }
    else
        m_timer.stop();
}

/**
 * On save screenshot
 */
void srs_ui_but::CButProjection::onSave( std::string filename )
{
    // m_textureWithRtt->saveImage( filename );
}


void srs_ui_but::CButProjection::cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info)
{
	//   std::cerr << "Camera callback. Frame id: " << cam_info->header.frame_id << std::endl;

	boost::mutex::scoped_lock( m_cameraInfoLock );

    // Get camera info
    ROS_DEBUG("OctMapPlugin: Set camera info: %d x %d\n", cam_info->height, cam_info->width);
    if( !m_camera_model.fromCameraInfo(*cam_info) )
        return;

    m_camera_size = m_camera_model.fullResolution();

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    vis_manager_->getFrameManager()->getTransform(cam_info->header, position, orientation);
  //  const cv::Mat_<double> pm( m_camera_model.projectionMatrix() );

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

    // Get z-axis
    Ogre::Vector3 normal( orientation.zAxis() );
    normal.normalise();

    // Compute camera plane equation
    Ogre::Vector4 equation( normal.x, normal.y, normal.z, -normal.dotProduct( position) );


    float width = cam_info->width;
    float height = cam_info->height;

    // Possibly malformed camera info...
    if( width == 0.0 || height == 0.0 )
        return;

    double fx = cam_info->P[0];
    double fy = cam_info->P[5];
/*
     Ogre::Radian fovy( 2.0*atan(height / (2.0 * fy)) );

    if( fovy != fovy)
    	return; // NAN

    double aspect_ratio = width / height;

    if( aspect_ratio != aspect_ratio )
    	return; //NaN
*/
    // Add the camera's translation relative to the left camera (from P[3]);
    // Tx = -1*(P[3] / P[0])
    double tx = -1.0 * (cam_info->P[3] / fx);
    Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
    position = position + (right * tx);

//    std::cerr << right * tx << std::endl;

    double ty = -1 * (cam_info->P[7] / fy);
    Ogre::Vector3 down = orientation * Ogre::Vector3::UNIT_Y;
    position = position + (down * ty);

    if( !rviz::validateFloats( position ))
    {
    	return;
    }


    if( m_projectionData != 0 )
    {
		m_projectionData->setProjectorPosition( position );
		m_projectionData->setProjectorOrientation( orientation );

		// f.setFOVy( fovX );
//		m_projectionData->setFOVy( fovy );
//		m_projectionData->setAspectRatio( aspect_ratio );

		m_projectionData->setCameraModel( *cam_info );
		m_projectionData->setProjectorEquation( equation );
		m_projectionData->updateMatrices();
    }
}

///////////////////////////////////////////////////////////////////////////////
const int ID_CHECKBOX(100);
const int ID_SAVE_BUTTON(101);

srs_ui_but::CButProjection::CControllPane::CControllPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi): wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
, m_wmi( wmi )
{
    // Create controls
    m_button = new wxButton(this, ID_SAVE_BUTTON, wxT("Save screenshot"), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT);

    m_chkb = new wxCheckBox(this, ID_CHECKBOX, wxT("Publish images"),
            wxPoint(20, 20));
    m_chkb->SetValue( true );

    // Create layout
    wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
    this->SetSizer(sizer);

    wxSizer *hsizer = new wxBoxSizer(wxHORIZONTAL);
    hsizer->Add(m_chkb, ID_CHECKBOX, wxALIGN_LEFT);
    hsizer->Add(m_button, ID_SAVE_BUTTON, wxALIGN_RIGHT);

    sizer->Add(hsizer, 0, wxALIGN_LEFT);

    sizer->SetSizeHints(this);


}

///////////////////////////////////////////////////////////////////////////////

/**
On checkbox toggle
 */
void srs_ui_but::CButProjection::CControllPane::OnChckToggle(wxCommandEvent& event)
{
    // Call signal
    m_sigCheckBox( m_chkb->GetValue() );
}

///////////////////////////////////////////////////////////////////////////////

/**
On quit command event handler
 */
void srs_ui_but::CButProjection::CControllPane::OnSave(wxCommandEvent& event)
{
    wxFileDialog fileDlg(this, _("Choose file to save"), wxEmptyString, _("screenshot.png"), _("*.*"), wxFD_SAVE | wxFD_OVERWRITE_PROMPT );

    wxString filename, directory;

    // Pause timer
    if(m_chkb->GetValue())
        m_sigCheckBox( false );

    if( fileDlg.ShowModal() == wxID_OK )
        {
        filename = fileDlg.GetFilename();
        directory = fileDlg.GetDirectory();

        std::string path(std::string( wxString(directory + wxFileName::GetPathSeparator() + filename).mb_str() ) );

        m_sigSave( path );
        }

    // Start timer
    if(m_chkb->GetValue())
        m_sigCheckBox( true );


}

/**
 *  Set testing distance - property callback
 */
void srs_ui_but::CButProjection::setTestedDistance( float distance )
{
	m_ml->setTestedDistance( distance );
}

/**
 * Get testing distance - property callback
 */
float srs_ui_but::CButProjection::getTestedDistance()
{
	return m_ml->getTestedDistance();
}


///////////////////////////////////////////////////////////////////////////////
BEGIN_EVENT_TABLE(srs_ui_but::CButProjection::CControllPane, wxPanel)
EVT_BUTTON(ID_SAVE_BUTTON,  srs_ui_but::CButProjection::CControllPane::OnSave)
EVT_CHECKBOX(ID_CHECKBOX, srs_ui_but::CButProjection::CControllPane::OnChckToggle )
END_EVENT_TABLE()
