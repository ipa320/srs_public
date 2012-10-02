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
#pragma once
#ifndef BUT_PROJECTION_H
#define BUT_PROJECTION_H

// ROS includes
#include <message_filters/subscriber.h>

// Rviz includes
#include <rviz/display.h>
#include <rviz/view_controller.h>
#include "rviz/properties/forwards.h"
#include "rviz/properties/property.h"
#include "rviz/properties/edit_enum_property.h"
#include "rviz/properties/property_manager.h"
//#include "rviz/image/ros_image_texture.h"
#include "rviz/properties/forwards.h"
#include <image_geometry/pinhole_camera_model.h>

// OGRE includes
#include <OGRE/OgreSceneNode.h>
#include <OgreManualObject.h>
#include <OgreCamera.h>
#include "OGRE/OgreMaterialManager.h"
#include <OGRE/OgreTexture.h>

// Std includes
#include <string>

// Wx includes
#include <wx/wx.h>

// Local includes
#include "ros_rtt_texture.h"
#include "but_rostexture.h"
#include "but_depthtexture.h"

namespace rviz
{
    class WindowManagerInterface;
}

namespace srs_ui_but
{

    /*********************************************************************************************************************/

    /**
     * Used material listener
     */
    class CMaterialListener : public Ogre::MaterialManager::Listener
    {
    protected:
        //! Material pointer
        Ogre::Material * m_materialPtr;

        //! Tested scheme name
        std::string m_schemeName;

        //! Is listener not paused?
        bool m_bNotPaused;
    public:
        //! Constructor
        //CMaterialListener( const std::string & materialName, const std::string & groupName, const std::string & schemeName );

        //! Constructor
        CMaterialListener( Ogre::Material * material, const std::string & schemeName );

        //! Destructor
        ~CMaterialListener();

        //! Pause/run listener
        void setPaused( bool bPaused ) { m_bNotPaused = !bPaused; }

        //! Scheme not found event handler - return stored material technique
        Ogre::Technique *handleSchemeNotFound(unsigned short, const Ogre::String& schemeName, Ogre::Material*mat, unsigned short, const Ogre::Renderable*);
    }; // class CMaterialListener

    /*********************************************************************************************************************/

    /**
     * Projection used data
     */
    class CProjectionData
    {
    	//typedef rviz::CRosTexture tRosTexture;
    	typedef CRosTopicTexture tRosTextureRGB;

    	//! Depth texture type
    	typedef CRosDepthTexture tRosTextureDepth;

    protected:
        enum ETextureMode
        {
            TM_INTERNAL, // Create and use internal ogre texture
            TM_ROS		 // Use ros texture (typically connected to some topic)
        };

    public:
        //! Constructor - with given texture
        CProjectionData( Ogre::SceneManager * manager, const ros::NodeHandle &nh, const std::string & materialName, const std::string & groupName = Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );

        //! Destructor
        virtual ~CProjectionData();

        //! Get material pointer
        Ogre::Material * getMaterialPtr() { return m_materialPtr.get(); }

        //! Update frustrum aspect ratio
        void setFrustrumSize( Ogre::Vector2 size );

        //! Set texture size
        void setTextureSize( int width, int height );

        //! Set ros texture topic
        void setRGBTextureTopic( const std::string & topic );

        //! Set input depth texture topic
        void setDepthTextureTopic( const std::string & topic );

        //! Get texture width
        int getTextureWidth() { return m_texW; }

        //! Get texture height
		int getTextureHeight() { return m_texH; }

        //! Clear texture
        void clear();

        //! Updata ros texture
        void update() { if( m_textureRosRGB != 0 ) m_textureRosRGB->update(); }

         //! Set projector position
        void setProjectorPosition( const Ogre::Vector3 & v ){ m_projectorNode->setPosition(v); }

        //! Set projecting direction
        void setProjectorOrientation( const Ogre::Quaternion & q ) { m_projectorNode->setOrientation( q ); }

        //! Update material projection matrix parameter
        void updateMatrices();

        //! Set FOVy
        void setFOVy( Ogre::Radian fovy ) { m_frustum->setFOVy( fovy );  }

        //! Set aspect ratio
        void setAspectRatio( Ogre::Real ar ) { m_frustum->setAspectRatio( ar ); }

    	//! Get camera model reference
    	void setCameraModel(const sensor_msgs::CameraInfo& msg) { m_textureRosDepth->setCameraModel(msg); }



    protected:
        //! Used material
        Ogre::MaterialPtr m_materialPtr;

        //! Old frustrum size
        Ogre::Vector2 m_frustrumSize;

        //! Texture sizes
        size_t m_texW, m_texH;

        //! Current mode
        ETextureMode m_mode;

        //! Ros textures
        tRosTextureRGB * m_textureRosRGB;
        tRosTextureDepth* m_textureRosDepth;

        //! Projector node
        Ogre::SceneNode * m_projectorNode;

        //! Frustum node
        Ogre::Frustum * m_frustum;

        //! Transform listener
        tf::TransformListener m_tfListener;

        //! Texture unit state
        Ogre::TextureUnitState * m_texState;

        //! Whole data locking mutex
        boost::mutex m_lock;

        //! Sotred scene manager pointer
        Ogre::SceneManager * m_manager;


    }; // class CProjectionData

    /*********************************************************************************************************************/


    /**
     * Display class
     */
    class CButProjection : public rviz::Display
    {
    public:

        /**
         * Display control WX panel
         */
        class CControllPane : public wxPanel
        {
        public:
            /// Checkbox state changed signal type
            typedef boost::signal< void (bool) > tSigCheckboxState;

            /// Save screenshot signal type
            typedef boost::signal< void ( std::string ) > tSigSave;

        public:
            /// Constructor
            CControllPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );

            /// On checkbox toggle
            void OnChckToggle(wxCommandEvent& event);

            //! On save screenshot button
            virtual void OnSave(wxCommandEvent& event);

            //! Get checkbox state changed signal
            tSigCheckboxState & getSigChckBox(){ return m_sigCheckBox; }

            //! Get save signal
            tSigSave & getSigSave(){ return m_sigSave; }

        protected:
            //! stored window manager interface pointer
            rviz::WindowManagerInterface * m_wmi;

            //! Chcekbox
            wxCheckBox * m_chkb;

            //! Button
            wxButton * m_button;

            //! Checkbox state changed signal
            tSigCheckboxState m_sigCheckBox;

            /// Save screenshot signal
            tSigSave m_sigSave;
        private:
            DECLARE_EVENT_TABLE()
        }; // class CControllPane

    public:
        //!Constructor
        CButProjection(const std::string & name,rviz::VisualizationManager * manager);

        //!Destructor
        ~CButProjection();

        //OverridesfromDisplay
        virtual void targetFrameChanged(){}
        virtual void fixedFrameChanged(){}
        virtual void createProperties();

        //! Get rgb image input topic
        const std::string& getRgbTopic() { return m_imageRGBInputTopicName; }

        //! Set rgb image input topic
        void setRgbTopic(const std::string& topic);

        //! Get depth image input topic
		const std::string& getDepthTopic() { return m_imageDepthInputTopicName; }

		//! Set depth image input topic
		void setDepthTopic(const std::string& topic);

        //! Update display
        virtual void update (float wall_dt, float ros_dt);

    protected:
        //overridesfromDisplay
        virtual void onEnable();
        virtual void onDisable();

        //! Subscribe to image topics
        void subscribe();

        //! Unsubscribe from image topics
        void unsubscribe();

        //! Create geometry (includes scene node initialization)
        bool createGeometry(const ros::NodeHandle & nh);

        //! Destroy geometry
        void destroyGeometry();

        //! Set timer period
        //void setTimerPeriod(float period);

        //! Timer publishing callback function
        void onTimerPublish(const ros::TimerEvent&);

        //! On publishing start/stop
        void onPublishStateChanged(bool state);

        /// On screenshot save signal
        void onSave( std::string filename );

        //! Material part - get material manager, load groups etc.
        void createMaterials(Ogre::Camera * camera);

        //! Remove materials
        void removeMaterials();

        //! Clear texture
        void clear();

        /// Camera info callback
        void cameraInfoCB(const sensor_msgs::CameraInfo::ConstPtr &cam_info);

        //! Connect/disconnect listener
        void connectML( bool bConnect );

    protected:
        //! Scene node
        Ogre::SceneNode * m_sceneNode;

        //! Image input topic name
        std::string m_imageRGBInputTopicName, m_imageDepthInputTopicName;

        //! Publishing timer
        ros::Timer m_timer;

        //! Rgb topic property
        rviz::ROSTopicStringPropertyWPtr m_rgb_topic_property;

        //! Depth image topic property
        rviz::ROSTopicStringPropertyWPtr m_depth_topic_property;

        //! Timer period
        double m_timerPeriod;

        //! Display pane
        CControllPane * m_pane;

        //! Projection data
        CProjectionData * m_projectionData;

        /// Camera info topic name
        std::string m_camera_info_topic;

        // TF transformation listener for camera info message
        tf::TransformListener *m_tf_cam_info_Listener;

        /// Camera info subscriber
        message_filters::Subscriber< sensor_msgs::CameraInfo > *  m_ciSubscriber;

        /// Transform filter
        tf::MessageFilter<sensor_msgs::CameraInfo> * m_camInfoTransformFilter;

        /// Camera model
        image_geometry::PinholeCameraModel m_camera_model;

        /// Camera size
        cv::Size m_camera_size;

        /// Camera info lock
        boost::mutex m_cameraInfoLock;

        //! Material listener object
        CMaterialListener * m_ml;

        //! Is material listener connected
        bool m_bMLConnected;

    };//class CButCamCast

} // namespace srs_ui_but

#endif // BUT_PROJECTION_H


