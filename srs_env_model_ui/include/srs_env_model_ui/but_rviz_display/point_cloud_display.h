/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Michal Spanel (spanel@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 3/12/2012
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
#ifndef BUT_POINTCLOUD_H
#define BUT_POINTCLOUD_H

#include "rviz/default_plugin/point_cloud_base.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/visualization_manager.h"
#include "rviz/frame_manager.h"

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreCamera.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreMaterialManager.h>

#include "ogre_tools/point_cloud.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <ros/time.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <deque>
#include <queue>
#include <vector>

namespace srs_env_model_ui
{

/**
 * \class CButPointCloud
 * \brief Displays a point cloud of type sensor_msgs::PointCloud
 *
 * By default it will assume channel 0 of the cloud is an intensity value, and will color them by intensity.
 * If you set the channel's name to "rgb", it will interpret the channel as an integer rgb value, with r, g and b
 * all being 8 bits.
 */

class CButPointCloud : public rviz::PointCloudBase
{
public:
    CButPointCloud( const std::string& name, rviz::VisualizationManager* manager );
    ~CButPointCloud();

    // Overrides from Display
    virtual void createProperties();
    virtual void targetFrameChanged();
    virtual void fixedFrameChanged();
    virtual void update(float wall_dt, float ros_dt);

    /**
      * Set the incoming PointCloud topic
      * @param topic The topic we should listen to
      */
    void setTopic( const std::string& topic );
    const std::string& getTopic() { return topic_; }

    /**
     * @brief camera_topic property setter
     * @param topic new image topic
     */
    void setCamInfoTopic(const std::string& topic);
    const std::string& getCamInfoTopic() { return caminfo_topic_; }

    /**
     * @brief property setter
     */
    void setDrawViewFrustum(bool value);
    bool getDrawViewFrustum() { return draw_view_frustum_; }

    /**
     * @brief property setter
     */
    void setViewFrustumDepth(float value);
    float getViewFrustumDepth() { return view_frustum_depth_; }

    /**
     * @brief property setter
     */
    void setCullViewFrustum(bool value);
    bool getCullViewFrustum() { return cull_view_frustum_; }

    /**
     * @brief position property getter
     */
    Ogre::Vector3 getPosition() { return position_; }

    /**
     * @brief orientation property getter
     */
    Ogre::Quaternion getOrientation() { return orientation_; }

protected:
    /**
     *  @brief Create view frustum geometry (includes scene node initialization)
     */
    bool createVFGeometry();

    /**
     *  @brief Destroy view frustum geometry
     */
    void destroyVFGeometry();

    /**
     * @brief Overriden from Display
     */
    virtual void onEnable();

    /**
     * @brief Overriden from Display
     */
    virtual void onDisable();

    /**
      * \brief Subscribes to the topic set by setTopic()
      */
    void subscribe();

    /**
      * \brief Unsubscribes from the current topic
      */
    void unsubscribe();

    /**
      * \brief ROS callback for an incoming point cloud message
      */
    void incomingCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

    /**
      * \brief ROS callback for an incoming camera info
      */
    void incomingCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& info);

    /**
     * @brief Makes sure the polygon position/orientation informations are related to correct frame
     */
    void transformCam();

    /**
     * \brief Counts angle parameters of view frustum of a camera specified in given
     * CameraInfo message and sets internal variables elev_d, elev_u, steer_l and steer_r
     */
    void countCameraParams(const sensor_msgs::CameraInfoConstPtr& camInfo);

protected:
    //! The PointCloud topic set by setTopic()
    std::string topic_;

    //! The CameraInfo topic set by setCamInfoTopic()
    std::string caminfo_topic_;
    bool caminfo_subscribed_;

    // New caminfo mutex
    boost::mutex caminfo_mutex_;

    // last arrived CameraInfo message
    sensor_msgs::CameraInfo::ConstPtr caminfo_;

    // camera parameters
    float elev_d_, steer_r_, elev_u_, steer_l_;

    // view frustum points
    Ogre::Vector3 vf_points_[5];

    //! Scene node
    Ogre::SceneNode * vf_scene_node_;

    //! Geometry manual object
    Ogre::ManualObject * vf_manual_object_;

    // Material
    Ogre::MaterialPtr vf_material_;

    //! Subscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> caminfo_sub_;

    //! Message filter (we only want point cloud 2 messages)
    tf::MessageFilter<sensor_msgs::PointCloud2> tf_filter_;
    tf::MessageFilter<sensor_msgs::CameraInfo> caminfo_tf_filter_;

    //! rviz property pointer
    rviz::ROSTopicStringPropertyWPtr topic_property_;
    rviz::ROSTopicStringPropertyWPtr caminfo_topic_property_;

    // rendering polygon position and corresponding display property
    Ogre::Vector3 position_;
    rviz::Vector3PropertyWPtr position_property_;

    // rendering polygon orientation and corresponding display property
    Ogre::Quaternion orientation_;
    rviz::QuaternionPropertyWPtr orientation_property_;

    // draw view frustum property
    bool draw_view_frustum_;
    rviz::BoolPropertyWPtr draw_view_frustum_property_;

    // cull point cloud in the view frustum property
    float view_frustum_depth_;
    rviz::FloatPropertyWPtr view_frustum_depth_property_;

    // cull point cloud in the view frustum property
    bool cull_view_frustum_;
    rviz::BoolPropertyWPtr cull_view_frustum_property_;

}; // class CButPointCloud

} // namespace srs_ui_but


#endif // BUT_POINTCLOUD_H
