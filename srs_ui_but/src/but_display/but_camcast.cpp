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

#include "but_camcast.h"

// Ogre includes
#include <OGRE/OgreSceneManager.h>
#include "OGRE/OgreWindowEventUtilities.h"
#include "OGRE/OgreManualObject.h"
#include "OGRE/OgreEntity.h"

// Rviz includes
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/window_manager_interface.h>

#include <wx/filedlg.h>

// Ros image message
#include <sensor_msgs/Image.h>

// Std includes
#include <sstream>

#define CAMERA_SCREENCAST_TOPIC_NAME std::string("rviz_cam_rgb")
#define DEFAULT_PUBLISHING_PERIOD double(0.1)

/*
 *  Constructor
 */
CButCamCast::CButCamCast(const std::string & name,rviz::VisualizationManager * manager)
: Display( name, manager )
, m_bPublish(true)
{
  // Create and connect pane
  rviz::WindowManagerInterface * wi( manager->getWindowManager() );

  if( wi != 0 )
  {
    // Arm manipulation controls
    m_pane = new CControllPane( wi->getParentWindow(), wxT("Screencasts manager"), wi);

    if( m_pane != 0 )
    {
      wi->addPane( "Screencasts manager", m_pane );
      wi->showPane( m_pane );

      // Connect signal
      m_pane->getSigChckBox().connect( boost::bind( &CButCamCast::onPublishStateChanged, this, _1) );
      m_pane->getSigSave().connect( boost::bind( &CButCamCast::onSave, this, _1 ) );
    }

  }else{
    std::cerr << "No window manager, no panes :( " << std::endl;
  }


  // Get node handle
  ros::NodeHandle private_nh("/");

  // Set parameters

  // Get camera screencasting topic name
  private_nh.param("rviz_screencast_topic_name", m_camCastPublisherName, CAMERA_SCREENCAST_TOPIC_NAME);

  // Get timer period
  private_nh.param("publishig_period", m_timerPeriod, DEFAULT_PUBLISHING_PERIOD );

  // Get scene node
  m_sceneNode = scene_manager_->getRootSceneNode()->createChildSceneNode();

  // Create publisher
  this->m_camCastPublisher = private_nh.advertise< sensor_msgs::Image >(m_camCastPublisherName, 100, false);

  // Create geometry
  createGeometry(private_nh);

  // Create publishing timer
  m_timer = private_nh.createTimer( ros::Duration(m_timerPeriod), boost::bind( &CButCamCast::onTimerPublish, this, _1) );

  // Set publishing parameters
  m_textureWithRtt->setFrame("/map");

}

/*
 *  Destructor
 */
CButCamCast::~CButCamCast()
{
  // Destroy all geometry
  destroyGeometry();

}


/*
 *  Display enablet callback
 */
void CButCamCast::onEnable()
{
  m_sceneNode->setVisible( true );
}

/*
 *  Display disabled callback
 */
void CButCamCast::onDisable()
{
  m_sceneNode->setVisible( false );
}

/*
 *  Create geometry
 */
bool CButCamCast::createGeometry(const ros::NodeHandle & nh)
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
  m_textureWithRtt = new rviz::CRosRttTexture( 512, 512, camera );

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
      float lSize = 0.7f;
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
    int lNumberOfEntities = 5;
    for(int iter = 0; iter < lNumberOfEntities; ++iter)
    {
      Ogre::Entity* lEntity = scene_manager_->createEntity(lNameOfTheMesh);
      // Now I attach it to a scenenode, so that it becomes present in the scene.
      Ogre::SceneNode* lNode = scene_manager_->getRootSceneNode()->createChildSceneNode();
      lNode->attachObject(lEntity);
      // I move the SceneNode so that it is visible to the camera.
      float lPositionOffset = float(1+ iter * 2) - (float(lNumberOfEntities));
      lNode->translate(lPositionOffset, lPositionOffset, -10.0f);
    }
  }

  // Create entity on screen

  Ogre::Entity* lAdditionalEntity = scene_manager_->createEntity( lNameOfTheMesh );
  lAdditionalEntity->setMaterialName( m_textureWithRtt->getMaterialName() );
  Ogre::SceneNode* lVisibleOnlyByFirstCam = scene_manager_->getRootSceneNode()->createChildSceneNode();
  lVisibleOnlyByFirstCam->attachObject( lAdditionalEntity );
  lVisibleOnlyByFirstCam->setPosition( 1.5,-0.5,-5);
*/
  return true;
}

void CButCamCast::destroyGeometry()
{
  // Destroy scene
  if( m_sceneNode != 0 )
    scene_manager_->destroySceneNode(m_sceneNode->getName());
}

//! Update display
void CButCamCast::update (float wall_dt, float ros_dt)
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

/*
 * Timer publishing callback function
 */

void CButCamCast::onTimerPublish(const ros::TimerEvent&)
{


  if( m_bPublish && (m_camCastPublisher.getNumSubscribers() > 0 ) && m_textureWithRtt->hasData() )
  {
    // Publish image
    m_camCastPublisher.publish( m_textureWithRtt->getImage() );
  }
}

/**
* On publishing start/stop
*/
void CButCamCast::onPublishStateChanged(bool state)
{
  if( state )
    m_timer.start();
  else
    m_timer.stop();
}

/**
* On save screenshot
*/
void CButCamCast::onSave( std::string filename )
{
  m_textureWithRtt->saveImage( filename );
}


///////////////////////////////////////////////////////////////////////////////
const int ID_CHECKBOX(100);
const int ID_SAVE_BUTTON(101);

CButCamCast::CControllPane::CControllPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi): wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
, m_wmi( wmi )
{
  // Create controls
  m_button = new wxButton(this, ID_SAVE_BUTTON, wxT("Save screenshot"));

  m_chkb = new wxCheckBox(this, ID_CHECKBOX, wxT("Publish images"),
      wxPoint(20, 20));
  m_chkb->SetValue( true );

  // Create layout
  wxSizer *sizer = new wxBoxSizer(wxVERTICAL);
  this->SetSizer(sizer);
  wxSizer *hsizer = new wxBoxSizer(wxHORIZONTAL);

  sizer->Add(m_button, ID_SAVE_BUTTON, wxALIGN_RIGHT);
  sizer->Add(hsizer, 0, wxALIGN_LEFT);

  hsizer->Add(m_chkb);

  sizer->SetSizeHints(this);

}

///////////////////////////////////////////////////////////////////////////////

/**
On checkbox toggle
*/
void CButCamCast::CControllPane::OnChckToggle(wxCommandEvent& event)
{
  // Call signal
  m_sigCheckBox( m_chkb->GetValue() );
}

///////////////////////////////////////////////////////////////////////////////

/**
On quit command event handler
*/
void CButCamCast::CControllPane::OnSave(wxCommandEvent& event)
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
BEGIN_EVENT_TABLE(CButCamCast::CControllPane, wxPanel)
EVT_BUTTON(ID_SAVE_BUTTON,  CButCamCast::CControllPane::OnSave)
EVT_CHECKBOX(ID_CHECKBOX, CButCamCast::CControllPane::OnChckToggle )
END_EVENT_TABLE()
