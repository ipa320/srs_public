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

#ifndef BUT_COB_DISPLAY_H
#define BUT_COB_DISPLAY_H

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

#include "but_arm_manipulation.h"


class CButCobDisplay : public rviz::Display
{
public:
    class CNotifyCameraListener : public Ogre::Camera::Listener
    {
    public:
        //! Constructor
        CNotifyCameraListener( CButCobDisplay * display );

        //! Destructor
        ~CNotifyCameraListener();

        //! Pre render notification
        virtual void cameraPreRenderScene (Ogre::Camera *cam);

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
        CButCobDisplay * m_display;

        //! Publishing timer
        ros::Timer m_timer;
    };

public:
    //!Constructor
    CButCobDisplay(const std::string & name,rviz::VisualizationManager * manager);

    //!Destructor
    ~CButCobDisplay();

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

    //! Controls window example
    CArmManipulationControls * m_armmanipulation_window;

    friend class CNotifyCameraListener;

};//classCBasicPlugin


#endif // BUT_COB_DISPLAY_H

