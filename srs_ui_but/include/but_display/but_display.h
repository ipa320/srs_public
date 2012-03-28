/**
 * $Id: but_display.h 321 2012-03-09 13:39:19Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#ifndef BUT_DISPLAY_H
#define BUT_DISPLAY_H

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
#include "but_examplepane.h"
#include "but_arm_manipulation.h"
#include "srs_env_model_msgs/RVIZCameraPosition.h"


class CButDisplay:public rviz::Display
{
public:
    class CNotifyCameraListener : public Ogre::Camera::Listener
    {
    public:
        //! Constructor
        CNotifyCameraListener( CButDisplay * display );

        //! Destructor
        ~CNotifyCameraListener();

        //! Pre render notification
        virtual void 	cameraPreRenderScene (Ogre::Camera *cam);

        //! Connect to the camera
        void connect( Ogre::Camera * camera );

        //! Disconnect
        void disconnect();

        //! Get current camera position
        const Ogre::Vector3 & getCameraPosition(){ return m_position; }

        Ogre::Camera & getCamera() { return *m_camera; }

    protected:
        //! Test if camera position and orientation has changed
        bool hasMoved( const Ogre::Vector3 & position, const Ogre::Quaternion & orientation );

        //! Do what is needed when orientation or position has changed
        virtual void changedCB( const Ogre::Vector3 & position, const Ogre::Quaternion & orientation );


    protected:
        //! Old camera position
        Ogre::Vector3 m_position;

        //! Old camera orientation
        Ogre::Quaternion m_orientation;

        //! Old camera
        Ogre::Camera * m_camera;

        //! Owner display
        CButDisplay * m_display;
    };

public:
    //!Constructor
    CButDisplay(const std::string & name,rviz::VisualizationManager * manager);

    //!Destructor
    ~CButDisplay();

    //OverridesfromDisplay
    virtual void targetFrameChanged(){}
    virtual void fixedFrameChanged(){}
    virtual void createProperties();

    //! Update display
    virtual void update (float wall_dt, float ros_dt);

protected:
    //overridesfromDisplay
    virtual void onEnable();
    virtual void onDisable();

    //! Create geometry (includes scene node initialization)
    bool createGeometry();

    //! Destroy geometry
    void destroyGeometry();

    //! View controller has changed signal slot
    void onViewControllerChange( rviz::ViewController * c );

    //! Connect listener
    void connectListener();

    //! Property has changed
    void propertyPositionChanged();

    //! Get property string - camera position
    const std::string getCameraPositionString();

    //! Set property string - camera position
    void setCameraPositionString( const std::string & str );

protected:
    //! Scene node
    Ogre::SceneNode * m_sceneNode;

    //! Geometry manual object
    Ogre::ManualObject * m_manualObject;

    //! Camera listener
    CNotifyCameraListener m_listener;

    //! Position display property
    rviz::StringPropertyWPtr m_property_position;

    //! Simple window example
    CExamplePanel * m_child_window;

    //! Simple dialog example
    CExampleDialog * m_dialog_window;

    //! Controls window example
    CExamplePanelControls * m_controls_window;
    
    //! Controls window example
    CArmManipulationControls * m_armmanipulation_window;

    //! Camera position publisher name
    std::string m_cameraPositionPublisherName;

    //! Camera position message
    srs_env_model_msgs::RVIZCameraPosition m_cameraPositionMsg;

    //! Camera position publisher
    ros::Publisher m_cameraPositionPub;

    //! Enable/disable publishing
    bool m_latchedTopics;


    friend class CNotifyCameraListener;

};//classCBasicPlugin


#endif // BUT_DISPLAY_H
