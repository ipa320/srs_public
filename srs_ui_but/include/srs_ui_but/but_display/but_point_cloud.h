/******************************************************************************
 * \file
 *
 * $Id: but_point_cloud.h 839 2012-05-24 11:44:20Z spanel $
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
#ifndef BUT_POINTCLOUD_H
#define BUT_POINTCLOUD_H

#include "rviz/default_plugin/point_cloud_base.h"
#include "rviz/helpers/color.h"
#include "rviz/properties/forwards.h"

#include "ogre_tools/point_cloud.h"
#include "sensor_msgs/PointCloud2.h"

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <deque>
#include <queue>
#include <vector>

namespace srs_ui_but
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

    /**
   * Set the incoming PointCloud topic
   * @param topic The topic we should listen to
   */
    void setTopic( const std::string& topic );
    const std::string& getTopic() { return topic_; }

protected:
    virtual void onEnable();
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

protected:
    ///< The PointCloud topic set by setTopic()
    std::string topic_;

    //! Subscriber
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;

    //! Message filter (we only want point cloud 2 messages)
    tf::MessageFilter<sensor_msgs::PointCloud2> tf_filter_;

    //! rviz property pointer
    rviz::ROSTopicStringPropertyWPtr topic_property_;
}; // class CButPointCloud

} // namespace srs_ui_but


#endif // BUT_POINTCLOUD_H
