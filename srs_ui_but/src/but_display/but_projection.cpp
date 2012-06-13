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

// Rviz includes
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>
#include <rviz/frame_manager.h>

#include <wx/filedlg.h>

// Ros image message
#include <sensor_msgs/Image.h>

// Std includes
#include <sstream>

#define DEFAULT_IMAGE_INPUT_TOPIC_NAME std::string("/cam3d/rgb/image_raw")
#define CAMERA_INFO_TOPIC_NAME std::string("/cam3d/camera_info")
#define DEFAULT_PUBLISHING_PERIOD double(0.1)

/**
 * Material listener constructor
 */
srs_ui_but::CMaterialListener::CMaterialListener( const std::string & materialName, const std::string & groupName, const std::string & schemeName )
: m_schemeName( schemeName )
{
    // Create material
    m_materialPtr = Ogre::MaterialManager::getSingleton().getByName( materialName, groupName );

    // Test if success
    if( m_materialPtr.get() == 0 )
        {
        std::cerr << "Cannot get material..." << std::endl;
        }
    else
        {
        // Load it
        m_materialPtr->load();

        // Write info
        std::cerr << "Material " << groupName << ":" << materialName << " loaded..." << std::endl;
        std::cerr << "Num of techniques: " << m_materialPtr->getNumTechniques() << std::endl;
        m_materialPtr->getTechnique(0);
        }
}

/**
 * Material listener constructor
 */
srs_ui_but::CMaterialListener::CMaterialListener( Ogre::Material * material, const std::string & schemeName )
: m_materialPtr( material )
, m_schemeName( schemeName )
{
    //	std::cerr << "Listener constructor" << std::endl;

    // Test if success
    if( m_materialPtr.get() == 0 )
        {
        std::cerr << "Cannot get material..." << std::endl;
        }
    else
        {
        // Load it
        m_materialPtr->load();

        // Write info
        std::cerr << "Material assigned..." << std::endl;
        std::cerr << "Num of techniques: " << m_materialPtr->getNumTechniques() << std::endl;
        m_materialPtr->getTechnique(0);
        }

    //	std::cerr << "Listener constructor done" << std::endl;
}


/**
 * Scheme not found event handler - return stored material technique
 */
Ogre::Technique *srs_ui_but::CMaterialListener::handleSchemeNotFound(unsigned short, const Ogre::String& schemeName, Ogre::Material*mat, unsigned short, const Ogre::Renderable*)
{
    if (schemeName == m_schemeName)
        {
        //LogManager::getSingleton().logMessage(">> adding glow to material: "+mat->getName());
        //		std::cerr << "gettech " << m_materialPtr.get() << std::endl;
        return m_materialPtr->getBestTechnique();
        }
    return NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Constructor
 */
srs_ui_but::CProjectionData::CProjectionData( Ogre::SceneManager * manager, const std::string & materialName, const std::string & groupName )
: m_texW( 512 )
, m_texH( 512 )
, m_mode( TM_INTERNAL )
, m_textureRos( 0 )
{
    std::cerr << "CProjectionData constructor" << std::endl;


    // Create and init frustrum
    m_frustum = new Ogre::Frustum();
    m_projectorNode = manager->getRootSceneNode()->createChildSceneNode("DecalProjectorNode");
    m_projectorNode->attachObject(m_frustum);
    m_projectorNode->setPosition(0,0,0);

    //	m_frustrum->setProjectionType(Ogre::PT_ORTHOGRAPHIC);
    //	m_frustrum->setOrthoWindowHeight(100);


    // Create and init texture
    m_textureInternal = Ogre::TextureManager::getSingleton().createManual("ProjectionTexture",
            groupName,
            Ogre::TEX_TYPE_2D, // Type
            m_texW,	// Width
            m_texH,	// Height
            1, 		// Depth
            0,		// Number of mipmaps
            Ogre::PF_R8G8B8A8,	// Pixel format
            Ogre::TU_DYNAMIC_WRITE_ONLY); // Usage

    // Fill our texture with some color...
    fillTexture( 255, 128, 128 );

    // Create and init material
    m_material = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(materialName, groupName);
    Ogre::Pass *pass = m_material->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);
    pass->setLightingEnabled(false);

    Ogre::TextureUnitState *texState = pass->createTextureUnitState("ProjectionTexture");
    texState->setProjectiveTexturing(true, m_frustum);
    texState->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    texState->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);

    Ogre::LayerBlendOperationEx op = Ogre::LBX_MODULATE;
    texState->setColourOperationEx(op, Ogre::LBS_TEXTURE, Ogre::LBS_TEXTURE);

    std::cerr << "CProjectionData constructor done" << std::endl;
}

/**
 * Constructor with given external ros texture
 */
srs_ui_but::CProjectionData::CProjectionData( Ogre::SceneManager * manager, const ros::NodeHandle &nh, const std::string & materialName, const std::string & groupName )
: m_texW( 512 )
, m_texH( 512 )
, m_mode( TM_ROS )
{
    std::cerr << "CProjectionData constructor" << std::endl;


    // Create and init frustrum
    m_frustum = new Ogre::Frustum();
    m_projectorNode = manager->getRootSceneNode()->createChildSceneNode("DecalProjectorNode");
    m_projectorNode->attachObject(m_frustum);
    m_projectorNode->setPosition(0,3,0);


    // Create ros texture
    m_textureRos = new rviz::CRosTexture( nh );
    assert( m_textureRos != 0 );

    // Fill our texture with some color...
    fillTexture( 255, 128, 128 );

    // Create and init material
    m_material = (Ogre::MaterialPtr)Ogre::MaterialManager::getSingleton().getByName(materialName, groupName);
    m_material->getTechnique(0)->setLightingEnabled(false);
    Ogre::Pass *pass = m_material->getTechnique(0)->createPass();

    pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
    pass->setDepthBias(1);
    pass->setLightingEnabled(false);

    Ogre::TextureUnitState *texState = pass->createTextureUnitState(m_textureRos->getTexture()->getName());
    texState->setProjectiveTexturing(true, m_frustum);
    texState->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);
    texState->setTextureFiltering(Ogre::FO_POINT, Ogre::FO_LINEAR, Ogre::FO_NONE);

    Ogre::LayerBlendOperationEx op = Ogre::LBX_MODULATE;
    texState->setColourOperationEx(op, Ogre::LBS_TEXTURE, Ogre::LBS_TEXTURE);

    std::cerr << "CProjectionData constructor done" << std::endl;
}

/**
 * Destructor
 */

//! Destructor
srs_ui_but::CProjectionData::~CProjectionData()
{

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
}

/**
 * Fill internal texture with color
 */
void srs_ui_but::CProjectionData::fillTexture( unsigned char r, unsigned char g, unsigned char b )
{
    if( m_mode != TM_INTERNAL )
        return;

    // Get the pixel buffer
    Ogre::HardwarePixelBufferSharedPtr pixelBuffer = m_textureInternal->getBuffer();

    // Lock the pixel buffer and get a pixel box
    pixelBuffer->lock(Ogre::HardwareBuffer::HBL_NORMAL); // for best performance use HBL_DISCARD!
    const Ogre::PixelBox& pixelBox = pixelBuffer->getCurrentLock();

    Ogre::uint8* pDest = static_cast<Ogre::uint8*>(pixelBox.data);

    // Fill in some pixel data. This will give a semi-transparent blue,
    // but this is of course dependent on the chosen pixel format.
    for (size_t j = 0; j < m_texH; j++)
        for(size_t i = 0; i < m_texW; i++)
            {
            // transparent rectangle around texture
            if( i == 0 || j == 0 || i == m_texW - 1 || j == m_texH -1 )
                {
                *pDest++ = 0; // B
                *pDest++ = 0; // G
                *pDest++ = 0; // R
                *pDest++ = 0;

                continue;
                }

            if( i != j)
                {
                *pDest++ = b; // B
                *pDest++ = g; // G
                *pDest++ = r; // R
                *pDest++ = 255;
                }
            else
                {
                *pDest++ = 0; // B
                *pDest++ = 0; // G
                *pDest++ = 0; // R
                *pDest++ = 255;

                }
            }

    // Unlock the pixel buffer
    pixelBuffer->unlock();
}

/**
 * Set ros texture topic
 */
void srs_ui_but::CProjectionData::setTextureTopic( const std::string & topic )
{
    if( m_mode == TM_ROS && m_textureRos != 0 )
        {
        m_textureRos->setTopic( topic );
        }
}

/**
 * Clear texture
 */
void srs_ui_but::CProjectionData::clear()
{
    if( m_textureRos != 0 )
        m_textureRos->clear();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 *  Constructor
 */
srs_ui_but::CButProjection::CButProjection(const std::string & name,rviz::VisualizationManager * manager)
: Display( name, manager )
, m_camera_info_topic( CAMERA_INFO_TOPIC_NAME )
{
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

    // Get image input topic name
    private_nh.param("image_input_topic", m_imageInputTopicName, DEFAULT_IMAGE_INPUT_TOPIC_NAME );

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
}

/*
 *  Destructor
 */
srs_ui_but::CButProjection::~CButProjection()
{
    // Destroy all geometry
    destroyGeometry();

}


/*
 *  Display enabled callback
 */
void srs_ui_but::CButProjection::onEnable()
{
    m_sceneNode->setVisible( true );
}

/*
 *  Display disabled callback
 */
void srs_ui_but::CButProjection::onDisable()
{
    m_sceneNode->setVisible( false );
}

/**
 * Create properties
 */
void srs_ui_but::CButProjection::createProperties()
{
    m_rgb_topic_property = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Image Topic", property_prefix_, boost::bind( &CButProjection::getRgbTopic, this ),
            boost::bind( &CButProjection::setRgbTopic, this, _1 ), parent_category_, this );

    setPropertyHelpText(m_rgb_topic_property, "sensor_msgs::Image topic to subscribe to.");
    rviz::ROSTopicStringPropertyPtr topic_prop = m_rgb_topic_property.lock();
    topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::Image>());

    setRgbTopic( m_imageInputTopicName );

}

/**
 * Subscribe texture to topic
 */
void srs_ui_but::CButProjection::subscribe()
{
    m_projectionData->setTextureTopic(m_imageInputTopicName);

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
    m_projectionData->setTextureTopic("");
}

/**
 *
 */
void srs_ui_but::CButProjection::setRgbTopic( const std::string& topic )
{
    unsubscribe();

    m_imageInputTopicName = topic;
    clear();

    subscribe();

    propertyChanged(m_rgb_topic_property);
}

/**
 * Clear texture, set status
 */
void srs_ui_but::CButProjection::clear()
{
    m_projectionData->clear();

    setStatus(rviz::status_levels::Warn, "Image", "No Image received");
}

/**
 * Create used materials
 */
void srs_ui_but::CButProjection::createMaterials(Ogre::Camera * camera)
{
    // Get material manager
    Ogre::MaterialManager& lMaterialManager = Ogre::MaterialManager::getSingleton();

    // Load materials
    Ogre::String nameOfResourceGroup( "MaterialGroup1" );
    {
        // Create resource group
        Ogre::ResourceGroupManager& lRgMgr = Ogre::ResourceGroupManager::getSingleton();
        lRgMgr.createResourceGroup(nameOfResourceGroup);

        // Get path
        std::string package_path( ros::package::getPath("srs_ui_but") );

        ogre_tools::V_string paths;
        Ogre::String resource_path(package_path + "/src/but_display/materials");

        std::cerr << "Materials path: " << resource_path.c_str() << std::endl;

        // List loaded materials
        Ogre::ResourceManager::ResourceMapIterator materialIterator = lMaterialManager.getResourceIterator();

        lRgMgr.createResourceGroup("srs_ui_but");
        lRgMgr.addResourceLocation(resource_path, "FileSystem", "srs_ui_but");
        lRgMgr.initialiseResourceGroup("srs_ui_but");
        lRgMgr.loadResourceGroup("srs_ui_but");

        std::cerr << "Loaded materials: " << std::endl;
        int count(0);
        while (materialIterator.hasMoreElements())
            {

            Ogre::String name( (static_cast<Ogre::MaterialPtr>(materialIterator.peekNextValue()))->getName() );

            std::cerr << name << std::endl;

            materialIterator.moveNext();
            count++;
            }

        std::cerr << "Num of materials: " << count << std::endl;
    }

    // Load compositor
    {
        if( Ogre::CompositorManager::getSingleton().addCompositor(camera->getViewport(), "DepthMap") == 0 )
            {
            std::cout << "COMPOSITOR FAILED TO LOAD." << std::endl;
            }
        else
            {
            Ogre::CompositorManager::getSingleton().setCompositorEnabled(camera->getViewport(), "DepthMap", true);

            //! Create material
            m_projectionData = new CProjectionData( scene_manager_, update_nh_, "material1", "srs_ui_but" );
            m_projectionData->setFrustrumSize( Ogre::Vector2( 100, 200 ) );

            // Create material listener
            CMaterialListener *ml = new CMaterialListener( m_projectionData->getMaterialPtr(), "myscheme" );
            //CMaterialListener *ml = new CMaterialListener( "depth", "srs_ui_but", "myscheme" );
            Ogre::MaterialManager::getSingleton().addListener(ml);
            }
    }
}

/*
 *  Create geometry
 */
bool srs_ui_but::CButProjection::createGeometry(const ros::NodeHandle & nh)
{
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
    /*/

    // Create and initialize materials
    createMaterials( camera );

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
        m_projectionData->update();
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

    // Get camera info
    ROS_DEBUG("OctMapPlugin: Set camera info: %d x %d\n", cam_info->height, cam_info->width);
    if( !m_camera_model.fromCameraInfo(*cam_info) )
        return;

 //   std::cerr << "Camera callback. Frame id: " << cam_info->header.frame_id << std::endl;
    m_camera_size = m_camera_model.fullResolution();

    // Compute projection matrix
/*
    // Get pinhole projection matrix reference
    const cv::Mat_<double> & pmcv( m_camera_model.intrinsicMatrix() );


    // Convert matrix to ogre
    Ogre::Matrix4 pm;
    for( int j = 0; j < 3; ++j )
        {
        for( int i = 0; i < 3; ++i )
            {
            double v = pmcv( i, j );
            std::cerr << v << ", ";
                pm[i][j] = pmcv(i, j);
            }
        std::cerr << std::endl;
        }
/*
    if( pm.isAffine() )
        {
        std::cerr << "Affine matrix..." << std::endl;
        // Set custom matrix to frustum
        m_projectionData->getFrustum().setCustomViewMatrix(true, pm );
        }

    Ogre::Frustum & f( m_projectionData->getFrustum() );
*/
    m_projectionData->setFrustrumSize( Ogre::Vector2( m_camera_size.width, m_camera_size.height ));

 //   std::cerr << "Size: " << m_camera_size.width << ", " << m_camera_size.height << std::endl;


    // 5 vertices of view frustum and corresponding points
    // transformed to /map frame
    geometry_msgs::PointStamped tl, tr, bl, br, camera;
    geometry_msgs::PointStamped tl_map, tr_map, bl_map, br_map, camera_map;

    // retrieve transform for display rotation
    tf::StampedTransform cameraToWorldTf;
    try {
            m_tf_cam_info_Listener->waitForTransform("/map",
                            cam_info->header.frame_id, cam_info->header.stamp,
                            ros::Duration(0.2));
            m_tf_cam_info_Listener->lookupTransform("/map",
                            cam_info->header.frame_id, cam_info->header.stamp,
                            cameraToWorldTf);
    }
    // In case of absence of transformation path
    catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM("Camera info transform error: " << ex.what()
                            << ", quitting callback");
            return;
    }

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    vis_manager_->getFrameManager()->getTransform(cam_info->header, position, orientation, false);

    // convert vision (Z-forward) frame to ogre frame (Z-out)
    orientation = orientation * Ogre::Quaternion(Ogre::Degree(180), Ogre::Vector3::UNIT_X);

   // Ogre::Quaternion orientation( cameraToWorldTf.getRotation().x(), cameraToWorldTf.getRotation().y(), cameraToWorldTf.getRotation().z(), cameraToWorldTf.getRotation().w() );
//    Ogre::Vector3 position( cameraToWorldTf.getOrigin().x(), cameraToWorldTf.getOrigin().y(), cameraToWorldTf.getOrigin().z());



//    std::cerr << "Position: " << position << std::endl;
//    std::cerr << "Orientation: " << orientation << std::endl;



    Ogre::Frustum & f( m_projectionData->getFrustum() );

    //  f.setFOVy( atan( m_camera_size / (2.0 * camera_info)) );

    Ogre::Radian fovX( 57.0 * 3.14159 / 180.0 );
    Ogre::Radian fovY( 43.0 * 3.14159 / 180.0 );

    float width = cam_info->width;
    float height = cam_info->height;

    // Possibly malformed camera info...
    if( width == 0.0 || height == 0.0 )
        return;

    double fx = cam_info->P[0];
    double fy = cam_info->P[5];
    Ogre::Radian fovy( 2*atan(height / (2 * fy)) );
    double aspect_ratio = width / height;

    // Add the camera's translation relative to the left camera (from P[3]);
    // Tx = -1*(P[3] / P[0])
    double tx = -1 * (cam_info->P[3] / fx);
    Ogre::Vector3 right = orientation * Ogre::Vector3::UNIT_X;
    position = position + (right * tx);

    m_projectionData->setProjectorPosition( position );
    m_projectionData->setProjectorOrientation( orientation );

    // f.setFOVy( fovX );
    f.setFOVy( fovy );
    f.setAspectRatio( aspect_ratio );
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



///////////////////////////////////////////////////////////////////////////////
BEGIN_EVENT_TABLE(srs_ui_but::CButProjection::CControllPane, wxPanel)
EVT_BUTTON(ID_SAVE_BUTTON,  srs_ui_but::CButProjection::CControllPane::OnSave)
EVT_CHECKBOX(ID_CHECKBOX, srs_ui_but::CButProjection::CControllPane::OnChckToggle )
END_EVENT_TABLE()
