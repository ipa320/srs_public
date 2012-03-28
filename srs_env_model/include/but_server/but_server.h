/**
 * $Id: but_server.h 281 2012-03-05 14:50:43Z stancl $
 *
 * Modified by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * This code is derived from the OctoMap server provided by A. Hornung.
 * Please, see the original comments below.
 *
 */

/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
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

protected:
    std_msgs::ColorRGBA heightMapColor(double h) const;

    /// Node handle
    ros::NodeHandle m_nh;

    /// Publishers
    ros::Publisher 	  m_markerPub
					, m_binaryMapPub
					, m_visiblePartPub;			//! Visible part of the scene publisher


    /// Subscriber - camera information
    //message_filters::Subscriber<sensor_msgs::CameraInfo> * m_camInfoSub;

    /// Subscriber - camera position
    //message_filters::Subscriber<srs_ui_but::RVIZCameraPosition> m_cameraPositionSub;

    //! Message filter - camera info
    //tf::MessageFilter<sensor_msgs::CameraInfo>* m_tfCameraInfoSub;

    ros::ServiceServer m_octomapService, m_clearBBXService, m_resetService;
    tf::TransformListener m_tfListener;


    bool m_useHeightMap;
    std_msgs::ColorRGBA m_color;
    double m_colorFactor;

    bool m_latchedTopics;

    double m_occupancyMinZ;
    double m_occupancyMaxZ;

    /// Should speckles be filtered?
    bool m_filterSpeckles;

    /// Connection settings
    std::string m_ocupiedCellsPublisher,
                m_octomapBinaryPublisher,
                m_cameraPositionSubscriber,
                m_worldFrameId,
                m_baseFrameId,
                m_visiblePartPublisher;

    /// Sensor to base transform matrix
    Eigen::Matrix4f m_sensorToBaseTM;

    /// Base to world transform matrix
    Eigen::Matrix4f m_baseToWorldTM;

    //! Every N-th frame should be processed when including point-cloud.
    int m_numPCFramesProcessed;

    //! Current frame counter
    int m_frameCounter;


    //======================================================================================================
    // Plugins

    /// Collision map
    srs::CCMapPlugin m_plugCMapPub;

    /// Incoming depth points cloud
    srs::CPointCloudPlugin m_plugInputPointCloud,
    /// Output depth points cloud - whole octomap
						m_plugOcMapPointCloud,
	/// Visible points point cloud
						m_plugVisiblePointCloud;

    /// Octo map plugin
    srs::COctoMapPlugin m_plugOctoMap;

    /// Collision object plugin
    srs::CCollisionObjectPlugin m_plugCollisionObject;

    /// 2D map plugin
    srs::CMap2DPlugin m_plugMap2D;

    /// Interactive markers server plugin
    srs::CIMarkersPlugin m_plugIMarkers;

};




#endif // SERVER_H
