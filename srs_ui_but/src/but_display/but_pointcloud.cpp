/******************************************************************************
 * \file
 *
 * $Id: but_pointcloud.cpp 396 2012-03-29 12:24:03Z spanel $
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

#include "but_pointcloud.h"
#include "rviz/default_plugin/point_cloud_transformers.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <ros/time.h>
#include "ogre_tools/point_cloud.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace rviz
{

/**
  Constructor
  */
CButPointCloud::CButPointCloud( const std::string& name, VisualizationManager* manager )
    : PointCloudBase( name, manager )
    , tf_filter_(*manager->getTFClient(), "", 10, threaded_nh_)
{
    // Connect to the input - point cloud advertiser
    tf_filter_.connectInput(sub_);

    // Connect incoming point cloud callback
    tf_filter_.registerCallback(&CButPointCloud::incomingCloudCallback, this);

    // Connect transform filter
    vis_manager_->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
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
  Display enabled
  */
void CButPointCloud::onEnable()
{
    PointCloudBase::onEnable();

    subscribe();
}

/**
  Display disabled
  */
void CButPointCloud::onDisable()
{
    unsubscribe();
    tf_filter_.clear();

    PointCloudBase::onDisable();
}

/**
  Subscribe to the topic
  */
void CButPointCloud::subscribe()
{
    if ( !isEnabled() )
    {
        // Not enabled - do nothing
        return;
    }

    // Subscribe to the topic
    sub_.subscribe(threaded_nh_, topic_, 2);
}

/**
  Unsubscribe from the topic
  */
void CButPointCloud::unsubscribe()
{
    sub_.unsubscribe();
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
    int32_t xi = findChannelIndex(cloud, "x");
    int32_t yi = findChannelIndex(cloud, "y");
    int32_t zi = findChannelIndex(cloud, "z");

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

    uint32_t output_count = 0;
    const uint8_t* ptr = &cloud->data.front();

    // Filter incoming points
    for (size_t i = 0; i < point_count; ++i)
    {
        float x = *reinterpret_cast<const float*>(ptr + xoff);
        float y = *reinterpret_cast<const float*>(ptr + yoff);
        float z = *reinterpret_cast<const float*>(ptr + zoff);

        // Is this point valid?
        if (validateFloats(Ogre::Vector3(x, y, z)))
        {
            // Copy point
            memcpy(&filtered->data.front() + (output_count * point_step), ptr, point_step);
            ++output_count;
        }

        // Input array pointer
        ptr += point_step;
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


    PointCloudBase::fixedFrameChanged();
}

/**
  Create rviz properties
  */
void CButPointCloud::createProperties()
{
    // Add subscribed topic property
    topic_property_ = property_manager_->createProperty<ROSTopicStringProperty>( "Topic", property_prefix_, boost::bind( &CButPointCloud::getTopic, this ),
                                                                                boost::bind( &CButPointCloud::setTopic, this, _1 ), parent_category_, this );
    // Add helper text
    setPropertyHelpText(topic_property_, "sensor_msgs::PointCloud2 topic to subscribe to.");

    // Set property message type - this information is used by rviz to filter automatically filled listbox
    ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
    topic_prop->setMessageType(ros::message_traits::datatype<sensor_msgs::PointCloud2>());

    // Base class properties should be done too...
    PointCloudBase::createProperties();
}

} // namespace rviz
