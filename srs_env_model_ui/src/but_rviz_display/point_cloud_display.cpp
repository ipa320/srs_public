/******************************************************************************
 * \file
 *
 * $Id: but_point_cloud.cpp 825 2012-05-23 14:29:07Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
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

#include <srs_env_model_ui/but_rviz_display/point_cloud_display.h>

#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"
#include "rviz/window_manager_interface.h"

#include <ros/time.h>
#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include "ogre_tools/point_cloud.h"

#include <Eigen/Dense>


namespace srs_env_model_ui
{

/**
  Constructor
  */
CButPointCloud::CButPointCloud( const std::string& name, rviz::VisualizationManager* manager )
    : PointCloudBase( name, manager )
    , caminfo_subscribed_( false )
    , vf_scene_node_( 0 )
    , vf_manual_object_( 0 )
    , tf_filter_(*manager->getTFClient(), "", 10, threaded_nh_)
    , caminfo_tf_filter_(*manager->getTFClient(), "", 10, threaded_nh_)
    , draw_view_frustum_( false )
    , view_frustum_depth_( 3.0 )
    , cull_view_frustum_( false )
{
    // Connect to the input - point cloud advertiser
    tf_filter_.connectInput(sub_);

    // Connect incoming point cloud callback
    tf_filter_.registerCallback(&CButPointCloud::incomingCloudCallback, this);

    // Connect transform filter
    vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);

    // Camera info
    caminfo_tf_filter_.connectInput(caminfo_sub_);
    caminfo_tf_filter_.registerCallback(&CButPointCloud::incomingCamInfoCallback, this);
    vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(caminfo_tf_filter_, this);

    createVFGeometry();
}

/**
  Destructor
  */
CButPointCloud::~CButPointCloud()
{
    // Unsubscribe from topics
    unsubscribe();

    // Clear target frame filter
    tf_filter_.clear();
    caminfo_tf_filter_.clear();

    destroyVFGeometry();
}

bool CButPointCloud::createVFGeometry()
{
    static const char * VF_NAME = "Camera View";
    static const rviz::Color VF_COLOR(1.0, 1.0, 0.0);

    // Scene node to draw view frustum geometry
    vf_scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

    // Set material
    vf_material_ = Ogre::MaterialManager::getSingleton().create( VF_NAME, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME );
    vf_material_->setCullingMode( Ogre::CULL_NONE );
    vf_material_->getTechnique(0)->setLightingEnabled(true);
    vf_material_->getTechnique(0)->setAmbient( VF_COLOR.r_, VF_COLOR.g_, VF_COLOR.b_ );
    vf_material_->getTechnique(0)->setDiffuse( VF_COLOR.r_, VF_COLOR.g_, VF_COLOR.b_, 1.0 );
    vf_material_->setSceneBlending( Ogre::SBT_TRANSPARENT_ALPHA );

    // Create manual object
    vf_manual_object_ = scene_manager_->createManualObject( VF_NAME );
    vf_manual_object_->setDynamic( true );

    vf_manual_object_->begin( vf_material_->getName(), Ogre::RenderOperation::OT_LINE_LIST );
        vf_manual_object_->position(0, 0, 0);
        vf_manual_object_->position(0, 0, 0);
        vf_manual_object_->position(0, 0, 0);
        vf_manual_object_->position(0, 0, 0);
    vf_manual_object_->end();

    vf_scene_node_->attachObject( vf_manual_object_ );

    vf_scene_node_->setVisible(false);

    return true;
}

void CButPointCloud::destroyVFGeometry()
{
    // Destroy manual object
    if( vf_manual_object_ != 0 )
    {
        scene_manager_->destroyManualObject( vf_manual_object_ );
        vf_manual_object_ = NULL;
    }

    // Destroy scene
    if( vf_scene_node_ != 0 )
    {
        scene_manager_->destroySceneNode( vf_scene_node_->getName() );
    }
}

/**
  Set input topic
  */
void CButPointCloud::setTopic( const std::string& topic )
{
    // Topic is the same - do nothing
    if( topic == topic_ )
        return;

    // Unsubscribe if subscribed
    unsubscribe();

    // Set topic name
    topic_ = topic;

    // Reset internal state
    reset();

    // Subscribe to the new topic
    subscribe();

    // Change rviz property to the new name
    propertyChanged(topic_property_);

    // Redraw
    causeRender();
}

/**
  Set input topic
  */
void CButPointCloud::setCamInfoTopic( const std::string& topic )
{
    // Topic is the same - do nothing
    if( topic == caminfo_topic_ )
        return;

    // Unsubscribe if subscribed
    unsubscribe();

    // Set topic name
    caminfo_topic_ = topic;

    // Reset internal state
    reset();

    // Subscribe to the new topic
    subscribe();

    // Change rviz property to the new name
    propertyChanged(caminfo_topic_property_);

    // Redraw
    causeRender();
}

/**
  Display enabled
  */
void CButPointCloud::onEnable()
{
    PointCloudBase::onEnable();

    if( draw_view_frustum_ && vf_scene_node_ != 0 )
    {
        vf_scene_node_->setVisible(true);
    }

    subscribe();
}

/**
  Display disabled
  */
void CButPointCloud::onDisable()
{
    unsubscribe();

    tf_filter_.clear();
    caminfo_tf_filter_.clear();

    if( vf_scene_node_ != 0 )
    {
        vf_scene_node_->setVisible(false);
    }

    PointCloudBase::onDisable();
}

/**
  Subscribe to the topic
  */
void CButPointCloud::subscribe()
{
    if( !isEnabled() )
    {
        // Not enabled - do nothing
        return;
    }

    // Subscribe to the topic
    sub_.subscribe(threaded_nh_, topic_, 2);
    if( !caminfo_topic_.empty() )
    {
        caminfo_sub_.subscribe(threaded_nh_, caminfo_topic_, 2);
        caminfo_subscribed_ = true;
    }
}

/**
  Unsubscribe from the topic
  */
void CButPointCloud::unsubscribe()
{
    sub_.unsubscribe();

    if( caminfo_subscribed_ )
    {
        caminfo_subscribed_ = false;
        caminfo_sub_.unsubscribe();
    }
}

/**
  Incoming point cloud data callback
  */
void CButPointCloud::incomingCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
//    std::cerr << "Incomming PC CB" << std::cerr;

    // Filter any nan values out of the cloud.  Any nan values that make it through to PointCloudBase
    // will get their points put off in lala land, but it means they still do get processed/rendered
    // which can be a big performance hit
    sensor_msgs::PointCloud2Ptr filtered(new sensor_msgs::PointCloud2);
    int32_t xi = rviz::findChannelIndex(cloud, "x");
    int32_t yi = rviz::findChannelIndex(cloud, "y");
    int32_t zi = rviz::findChannelIndex(cloud, "z");

    if (xi == -1 || yi == -1 || zi == -1)
    {
        return;
    }

    // Get cloud offset
    const uint32_t xoff = cloud->fields[xi].offset;
    const uint32_t yoff = cloud->fields[yi].offset;
    const uint32_t zoff = cloud->fields[zi].offset;

    // Get point step
    const uint32_t point_step = cloud->point_step;

    // Get point count
    const size_t point_count = cloud->width * cloud->height;

    // Set this size
    filtered->data.resize(cloud->data.size());
    if (point_count == 0)
    {
        return; // No data, no bussiness
    }

    // VIEW FRUSTUM filtering
    bool culling_enabled = false;
    Eigen::Vector3f n[4], cam;
    float d[4], max_depth = 10.0f;
    if( caminfo_ && cull_view_frustum_ )
    {
        boost::mutex::scoped_lock lock(caminfo_mutex_);

        if( caminfo_ && cull_view_frustum_ )
        {
            culling_enabled = true;
            max_depth = view_frustum_depth_;

            // Transform points defining the view frustum into the correct frame
            geometry_msgs::PointStamped point, tl, tr, bl, br, camera;
            point.header.frame_id = caminfo_->header.frame_id;
            point.header.stamp = caminfo_->header.stamp;

            try {
                tf::TransformListener * listener = vis_manager_->getFrameManager()->getTFClient();
                //listener->waitForTransform( cloud->header.frame_id,
                //                            caminfo_->header.frame_id,
                //                            ros::Time(0),
                //                            ros::Duration(2.0)
                //                            );

                point.point.x = vf_points_[0].x;
                point.point.y = vf_points_[0].y;
                point.point.z = vf_points_[0].z;
                listener->transformPoint( cloud->header.frame_id, point, camera );

                point.point.x = vf_points_[1].x;
                point.point.y = vf_points_[1].y;
                point.point.z = vf_points_[1].z;
                listener->transformPoint( cloud->header.frame_id, point, br );

                point.point.x = vf_points_[2].x;
                point.point.y = vf_points_[2].y;
                point.point.z = vf_points_[2].z;
                listener->transformPoint( cloud->header.frame_id, point, tr );

                point.point.x = vf_points_[3].x;
                point.point.y = vf_points_[3].y;
                point.point.z = vf_points_[3].z;
                listener->transformPoint( cloud->header.frame_id, point, bl );

                point.point.x = vf_points_[4].x;
                point.point.y = vf_points_[4].y;
                point.point.z = vf_points_[4].z;
                listener->transformPoint( cloud->header.frame_id, point, tl );
            }
            catch( tf::TransformException ex )
            {
                ROS_ERROR("%s",ex.what());
                culling_enabled = false;
            }

            // Calculate parameters of the plane
            if( culling_enabled )
            {
                Eigen::Vector3f points[4];
                points[0] = Eigen::Vector3f(float(bl.point.x), float(bl.point.y), float(bl.point.z));
                points[1] = Eigen::Vector3f(float(tl.point.x), float(tl.point.y), float(tl.point.z));
                points[2] = Eigen::Vector3f(float(tr.point.x), float(tr.point.y), float(tr.point.z));
                points[3] = Eigen::Vector3f(float(br.point.x), float(br.point.y), float(br.point.z));

                cam = Eigen::Vector3f(float(camera.point.x), float(camera.point.y), float(camera.point.z));

                for( int i = 0; i < 4; ++i )
                {
                    int i2 = (i + 1) % 4;
                    Eigen::Vector3f v1(points[i] - cam);
                    Eigen::Vector3f v2(points[i2] - cam);
                    n[i] = v1.cross(v2);
                    d[i] = -n[i].dot(cam);
                }
            }
        }
    }

    uint32_t output_count = 0;
    const uint8_t* ptr = &cloud->data.front();

    // Filter incoming points
    for (size_t i = 0; i < point_count; ++i, ptr += point_step )
    {
        float x = *reinterpret_cast<const float*>(ptr + xoff);
        float y = *reinterpret_cast<const float*>(ptr + yoff);
        float z = *reinterpret_cast<const float*>(ptr + zoff);

        // Is this point valid?
        if (rviz::validateFloats(Ogre::Vector3(x, y, z)))
        {
            if( culling_enabled )
            {
                Eigen::Vector3f p(x, y, z);
                float e1 = n[0].dot(p) + d[0];
                float e2 = n[1].dot(p) + d[1];
                float e3 = n[2].dot(p) + d[2];
                float e4 = n[3].dot(p) + d[3];
                if( e1 > 0.0f && e2 > 0.0f && e3 > 0.0f && e4 > 0.0f )
                {
                    p -= cam;
                    if( p.norm() < max_depth )
                    {
                        continue;
                    }
                }
            }

            // Copy point
            memcpy(&filtered->data.front() + (output_count * point_step), ptr, point_step);
            ++output_count;
        }
    }

    // Copy additional information
    filtered->header = cloud->header;
    filtered->fields = cloud->fields;
    filtered->data.resize(output_count * point_step);
    filtered->height = 1;
    filtered->width = output_count;
    filtered->is_bigendian = cloud->is_bigendian;
    filtered->point_step = point_step;
    filtered->row_step = output_count;

    // TODO - should not be filtered array resized, if some invalid points was dropped?

    // Create output message
    addMessage(filtered);
}

/**
  Incoming camera info callback
  */
void CButPointCloud::incomingCamInfoCallback(const sensor_msgs::CameraInfoConstPtr& camInfo)
{
//    ROS_INFO("CameraInfo received");
    {
        boost::mutex::scoped_lock lock(caminfo_mutex_);

        caminfo_ = camInfo;
        countCameraParams( caminfo_ );

        // 5 vertices defining the view frustum
        // camera position
        vf_points_[0].x = 0.0;
        vf_points_[0].y = 0.0;
        vf_points_[0].z = 0.0;

        // bottom right point
        vf_points_[1].x = -steer_l_ * view_frustum_depth_;
        vf_points_[1].y = -elev_u_ * view_frustum_depth_;
        vf_points_[1].z = view_frustum_depth_;

        // top right point
        vf_points_[2].x = -steer_l_ * view_frustum_depth_;
        vf_points_[2].y = +elev_d_ * view_frustum_depth_;
        vf_points_[2].z = view_frustum_depth_;

        // bottom left point
        vf_points_[3].x = +steer_r_ * view_frustum_depth_;
        vf_points_[3].y = -elev_u_ * view_frustum_depth_;
        vf_points_[3].z = view_frustum_depth_;

        // top left point
        vf_points_[4].x = +steer_r_ * view_frustum_depth_;
        vf_points_[4].y = +elev_d_ * view_frustum_depth_;
        vf_points_[4].z = view_frustum_depth_;
    }
}

/**
 * Counts angle parameters of view frustum of one camera specified in given
 * CameraInfo message and sets internal variables elev_d, elev_u, steer_l and steer_r
 */
void CButPointCloud::countCameraParams(const sensor_msgs::CameraInfoConstPtr& camInfo)
{
    // do other distortion models have the same camera matrices?
    if( camInfo->distortion_model != "plumb_bob" )
    {
        ROS_ERROR("Unknown distortion model in CameraInfo message.\\"
                  "Optimized only for plumb bob distortion model."
                  );
    }

    unsigned int height = 0;
    unsigned int width = 0;

    // focal lengths and principal point coordinates
    float fx, fy, cx, cy;

    height = camInfo->height;
    width = camInfo->width;

    // getting essential parameters from intrinsic camera matrix
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    fx = camInfo->K[0];
    fy = camInfo->K[4];

    if( fx == 0 || fy == 0 )
    {
        ROS_ERROR("Uncalibrated camera, unable to count view frustum parameters.");
    }

    cx = camInfo->K[2];
    cy = camInfo->K[5];

    // counting view frustum parameters from camera parameters
    elev_d_ = cy / fy;
    steer_r_ = cx / fx;

    elev_u_ = (height - cy) / fy;
    steer_l_ = (width - cx) / fx;

//    ROS_INFO("elev_d = %f", elev_d_);
//    ROS_INFO("elev_u = %f", elev_u_);
//    ROS_INFO("steer_l = %f", steer_l_);
//    ROS_INFO("steer_r = %f", steer_r_);
}


void CButPointCloud::update(float wall_dt, float ros_dt)
{
    PointCloudBase::update(wall_dt, ros_dt);

//    ROS_INFO("Update called");

    if( caminfo_ )
    {
        boost::mutex::scoped_lock lock(caminfo_mutex_);

        if( caminfo_ )
        {
            // Transform scene node to link position
            if( vis_manager_->getFrameManager()->getTransform(caminfo_->header, position_, orientation_) )
            {
                vf_scene_node_->setVisible( draw_view_frustum_ );
                vf_scene_node_->setPosition(position_);
                vf_scene_node_->setOrientation(orientation_);

                propertyChanged(orientation_property_);
                propertyChanged(position_property_);

                vf_scene_node_->detachAllObjects();

                // Update the view frustum
                vf_manual_object_->beginUpdate(0);
                vf_manual_object_->setMaterialName( 0, vf_material_->getName() );
                vf_manual_object_->position(vf_points_[0].x, vf_points_[0].y, vf_points_[0].z);
                vf_manual_object_->position(0.5f * (vf_points_[1].x + vf_points_[2].x),
                                            0.5f * (vf_points_[1].y + vf_points_[2].y),
                                            0.5f * (vf_points_[1].z + vf_points_[2].z)
                                            );
                vf_manual_object_->position(vf_points_[0].x, vf_points_[0].y, vf_points_[0].z);
                vf_manual_object_->position(0.5f * (vf_points_[3].x + vf_points_[4].x),
                                            0.5f * (vf_points_[3].y + vf_points_[4].y),
                                            0.5f * (vf_points_[3].z + vf_points_[4].z)
                                            );
                vf_manual_object_->end();

                vf_scene_node_->attachObject( vf_manual_object_ );

                //ROS_INFO("Geometry updated");
            }
        }
    }
}


void CButPointCloud::transformCam()
{
    if( caminfo_ )
    {
        boost::mutex::scoped_lock lock(caminfo_mutex_);

        if( caminfo_ )
        {
            if( vis_manager_->getFrameManager()->getTransform(caminfo_->header, position_, orientation_) )
            {
                // transform geometry to its real position and size
                // (transforming scene node, lines are the only object attached)
                vf_scene_node_->setPosition(position_);
                vf_scene_node_->setOrientation(orientation_);

                propertyChanged(orientation_property_);
                propertyChanged(position_property_);
            }
        }
    }
}

/**
  What to do if target frame was changed...
  */
void CButPointCloud::targetFrameChanged()
{
}

/**
  What to do if fixed frame was changed...
  */
void CButPointCloud::fixedFrameChanged()
{
    // Change filter target frame
    tf_filter_.setTargetFrame( fixed_frame_ );
    caminfo_tf_filter_.setTargetFrame( fixed_frame_ );

    transformCam();

    PointCloudBase::fixedFrameChanged();
}

/**
  Create rviz properties
  */
void CButPointCloud::createProperties()
{
    // Add subscribed topic property
    topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &CButPointCloud::getTopic, this ),
                                                                                boost::bind( &CButPointCloud::setTopic, this, _1 ), parent_category_, this );
    // Add helper text
    setPropertyHelpText(topic_property_, "sensor_msgs::PointCloud2 topic to subscribe to.");

    // Set property message type - this information is used by rviz to filter automatically filled listbox
    rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
    topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::PointCloud2>());


    // Add subscribed topic property
    caminfo_topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty>( "Camera Info Topic", property_prefix_, boost::bind( &CButPointCloud::getCamInfoTopic, this ),
                                                                                boost::bind( &CButPointCloud::setCamInfoTopic, this, _1 ), parent_category_, this );
    // Add helper text
    setPropertyHelpText(caminfo_topic_property_, "sensor_msgs::CameraInfo topic to subscribe to.");

    // Set property message type - this information is used by rviz to filter automatically filled listbox
    rviz::ROSTopicStringPropertyPtr caminfo_topic_prop = caminfo_topic_property_.lock();
    caminfo_topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::CameraInfo>());


    // View frustum
    draw_view_frustum_property_ = property_manager_->createProperty<rviz::BoolProperty> (
            "Show View Frustum", property_prefix_, boost::bind(
            &CButPointCloud::getDrawViewFrustum, this), boost::bind(
            &CButPointCloud::setDrawViewFrustum, this, _1), parent_category_,
            this);
    setPropertyHelpText(
            draw_view_frustum_property_,
            "Controls weather the view frustum is visualized or not.");

    view_frustum_depth_property_ = property_manager_->createProperty<rviz::FloatProperty> (
            "View Frustum Depth", property_prefix_, boost::bind(
            &CButPointCloud::getViewFrustumDepth, this), boost::bind(
            &CButPointCloud::setViewFrustumDepth, this, _1), parent_category_,
            this);
    setPropertyHelpText(
            view_frustum_depth_property_,
            "Length of the view frustum lines.");


    // View frustum culling
    cull_view_frustum_property_ = property_manager_->createProperty<rviz::BoolProperty> (
            "Cull View Frustum", property_prefix_, boost::bind(
            &CButPointCloud::getCullViewFrustum, this), boost::bind(
            &CButPointCloud::setCullViewFrustum, this, _1), parent_category_,
            this);
    setPropertyHelpText(
            cull_view_frustum_property_,
            "Controls weather the points inside the view frustum are removed or not.");


    // Polygon position
    position_property_ = property_manager_->createProperty<rviz::Vector3Property> (
            "Position", property_prefix_, boost::bind(
            &CButPointCloud::getPosition, this),
            rviz::Vector3Property::Setter(), parent_category_, this);
    setPropertyHelpText(position_property_,
            "Position of the camera. (not editable)");

    // Polygon orientation
    orientation_property_ = property_manager_->createProperty<rviz::QuaternionProperty> (
            "Orientation", property_prefix_, boost::bind(
            &CButPointCloud::getOrientation, this),
            rviz::QuaternionProperty::Setter(), parent_category_, this);
    setPropertyHelpText(orientation_property_,
            "Orientation of the camera. (not editable)");


    // Base class properties should be done too...
    PointCloudBase::createProperties();
}


void CButPointCloud::setDrawViewFrustum(bool value)
{
    // set internal variable
    draw_view_frustum_ = value;
    if( vf_scene_node_ != 0 )
    {
        ROS_INFO("DrawViewFrustum = %d", int(draw_view_frustum_));

        vf_scene_node_->setVisible( draw_view_frustum_ );
    }

    propertyChanged(draw_view_frustum_property_);

    causeRender();
}


void CButPointCloud::setViewFrustumDepth(float value)
{
    // set internal variable
    view_frustum_depth_ = value;

    propertyChanged(view_frustum_depth_property_);

    causeRender();
}


void CButPointCloud::setCullViewFrustum(bool value)
{
    // set internal variable
    cull_view_frustum_ = value;

    propertyChanged(cull_view_frustum_property_);

    causeRender();
}


} // namespace rviz
