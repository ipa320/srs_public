/******************************************************************************
 * \file
 *
 * $Id: but_server.h 617 2012-04-16 13:45:44Z stancl $
 *
 * Modified by dcgm-robotics@FIT group
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
 * 
 * This code is derived from the OctoMap server provided by A. Hornung.
 * Please, see the original comments below.
 */

/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/**
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SERVER_H
#define SERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

//#include <sensor_msgs/CameraInfo.h>
//#include <std_srvs/Empty.h>

//#include <pcl/point_types.h>


//#include <pcl/io/pcd_io.h>


#include <but_server/plugins/CMapPlugin.h>
#include <but_server/plugins/PointCloudPlugin.h>
#include <but_server/plugins/OctoMapPlugin.h>
#include <but_server/plugins/CollisionObjectPlugin.h>
#include <but_server/plugins/Map2DPlugin.h>
#include <but_server/plugins/IMarkersPlugin.h>
#include <but_server/plugins/MarkerArrayPlugin.h>
#include <but_server/plugins/LimitedPointCloudPlugin.h>

// Old interactive markers plugin used for testing
#include <but_server/plugins/OldIMarkersPlugin.h>

/**
  BUT dynamic scene server class.
  */
class CButServer{
public:


public:
    //! Type of the used pointcloud
    typedef pcl::PointCloud<pcl::PointXYZ> tPCLPointCloud;

    //! Constructor - load file
    CButServer(const std::string& filename= "");

    //! Destructor
    virtual ~CButServer();

    /// Reset server and all plugins
    void reset();

protected:
    //! Publish all
    void publishAll(const ros::Time& rostime = ros::Time::now());

    /**
 * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
 * @param key
 * @return
 */
    bool isSpeckleNode(const octomap::OcTreeKey& key) const;

    //! On octomap data changed
    void onOcMapDataChanged( const srs::tButServerOcMap & mapdata );

    /// On reset service call
    bool onReset(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response){ reset(); return true; }

protected:

    /// Node handle
    ros::NodeHandle m_nh;

    bool m_latchedTopics;

    //! Every N-th frame should be processed when including point-cloud.
    int m_numPCFramesProcessed;

    //! Current frame counter
    int m_frameCounter;

    //======================================================================================================
    // Services

    /// Reset service
    ros::ServiceServer m_serviceReset;


    //======================================================================================================
    // Plugins


    /// All plugins vector type
    typedef std::vector<srs::CServerPluginBase * > tVecPlugins;

    /// All plugins
    tVecPlugins m_plugins;

    /// Call all plugins function
#define FOR_ALL_PLUGINS( X ) { for( tVecPlugins::iterator p = m_plugins.begin(); p != m_plugins.end(); ++p ){ (*p)->X; } }
#define FOR_ALL_PLUGINS_PARAM( X, Y ) { for( tVecPlugins::iterator p = m_plugins.begin(); p != m_plugins.end(); ++p ){ (*p)->X(Y); } }

    /// Collision map
    srs::SCMapPluginHolder< srs::COctoMapPlugin > m_plugCMapHolder;

    /// Incoming depth points cloud
    srs::SPointCloudPluginHolder< srs::COctoMapPlugin > m_plugInputPointCloudHolder,
                                    /// Output depth points cloud - whole octomap
                                                        m_plugOcMapPointCloudHolder;
    /// Visible points point cloud
    srs::SLimitedPointCloudPluginHolder< srs::COctoMapPlugin > m_plugVisiblePointCloudHolder;

    /// Octo map plugin
    srs::COctoMapPlugin m_plugOctoMap;

    /// Collision object plugin
    srs::SCollisionObjectPluginHolder< srs::COctoMapPlugin > m_plugCollisionObjectHolder;

    /// 2D map plugin
    srs::SMap2DPluginHolder< srs::COctoMapPlugin > m_plugMap2DHolder;

    /// Interactive markers server plugin
    srs::CIMarkersPlugin * m_plugIMarkers;

    /// Marker array publisher plugin
    srs::SMarkerArrayHolder< srs::COctoMapPlugin > m_plugMarkerArrayHolder;

    /// Old interactive markers plugin
    srs::COldIMarkersPlugin * m_plugOldIMarkers;

    /// Use old interactive server plugin?				TODO: Remov this when new is will be finished
    bool m_bUseOldIMP;

};




#endif // SERVER_H
