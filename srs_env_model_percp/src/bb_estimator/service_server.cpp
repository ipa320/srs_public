/******************************************************************************
 * \file
 *
 * $Id: bb_estimator_server.cpp 742 2012-04-25 15:18:28Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 25.4.2012 (version 6.0)
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

/**
 * Description:
 * This server advertises service ("/bb_estimator/estimate_bb") performing bounding
 * box estimation. The input/output is defined in Estimate.srv file.
 *
 * There are two variants of subscription - to be the service able to work also
 * with a simulation of Care-o-Bot (subscription variant #2 is meant to be used
 * with simulation, which does not produce a depth map so it must be created from
 * a point cloud).
 *
 * Node parameters:
 *  Parameters for subscription variant #1:
 *  bb_sv1_depth_topic - topic with Image messages containing depth information
 *  bb_sv1_camInfo_topic - topic with CameraInfo messages
 *
 *  Parameters for subscription variant #2:
 *  bb_sv2_pointCloud_topic - topic with PointCloud2 messages containing Point Cloud
 *  bb_sv2_camInfo_topic - topic with CameraInfo messages
 *
 *  Other parameters:
 *  bb_outliers_percent - Percentage of furthest points from mean considered
 *                        as outliers when calculating statistics of ROI
 *  bb_scene_frame_id - Frame Id in which the BB coordinates are returned
 *------------------------------------------------------------------------------
 */

#include <srs_env_model_percp/bb_estimator/funcs.h>
#include <srs_env_model_percp/services_list.h>
#include <srs_env_model_percp/topics_list.h>

// Definition of the service for BB estimation
#include "srs_env_model_percp/EstimateBB.h"
#include "srs_env_model_percp/EstimateBBAlt.h"
#include "srs_env_model_percp/EstimateRect.h"
#include "srs_env_model_percp/EstimateRectAlt.h"

#include <algorithm>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


namespace srs_env_model_percp
{

// Cache
message_filters::Cache<Image> depthCache; // Cache of Image messages
message_filters::Cache<PointCloud2> pointCloudCache; // Cache of PointCloud2 messages
message_filters::Cache<CameraInfo> camInfoCache; // Cache of CameraInfo messages

// Frame IDs
std::string camFrameId; // Camera frame id (will be obtained automatically)
std::string sceneFrameId; // Scene (world) frame id

// Subscription variants
int subVariant = SV_NONE;

// TF listener
tf::TransformListener *tfListener;

// Percentage of furthest points from mean considered as outliers when
// calculating statistics of ROI
int outliersPercent = outliersPercentDefault;

// The required maximum ratio of sides length (the longer side is at maximum
// sidesRatio times longer than the shorter one)
double sidesRatio = sidesRatioDefault;

// Modes of bounding box estimation
//----------
// They differ in interpretation of the specified 2D region of interest (ROI).

// MODE1 = The ROI corresponds to projection of BB front face and the BB is
//         rotated to fit the viewing frustum (representing the back-projection
//         of the ROI) in such way, that the BB front face is perpendicular
//         to the frustum's center axis.
//         (BB can be non-parallel with all axis.)

// MODE2 = In the ROI is contained the whole projection of BB.
//         (BB is parallel with all axis.)

// MODE3 = The ROI corresponds to projection of BB front face.
//         (BB is parallel with all axis.)
int estimationMode = 1;


/*==============================================================================
 * Bounding box estimation service.
 *
 * @param req  Request of type EstimateBB.
 * @param res  Response of type EstimateBB.
 */
bool estimateBB_callback(srs_env_model_percp::EstimateBB::Request  &req,
                         srs_env_model_percp::EstimateBB::Response &res
                         )
{
    if( DEBUG )
    {
        std::cout << "EstimateBB service called:" 
            << " p1.x = " << req.p1[0]
            << ", p1.y = " << req.p1[1]
            << ", p2.x = " << req.p2[0]
            << ", p2.y = " << req.p2[1]
            << std::endl;
    }

    // Estimate the bounding box
    //--------------------------------------------------------------------------

    // Vertices (corners) defining the bounding box.
    // Each is noted using this template:
    // bb{L=left,R=right}{B=bottom,T=top}{F=front,B=back}
    // (This notation provides more readable code, in comparison with
    // e.g. an array with 8 items.)
    Point3f bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB;

    if( !estimateBB(req.header.stamp,
                    req.p1, req.p2, req.mode,
                    bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB) )
    {
        return false;
    }

    // Calculate also Pose of the bounding box
    //--------------------------------------------------------------------------
    tf::Quaternion q;
    Point3f p, s;

    if( !estimateBBPose(bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB,
                        p, q, s) )
    {
        return false;
    }
    
    // Set response
    //--------------------------------------------------------------------------
    res.p1[0] = bbLBF.x; res.p1[1] = bbLBF.y; res.p1[2] = bbLBF.z;
    res.p2[0] = bbRBF.x; res.p2[1] = bbRBF.y; res.p2[2] = bbRBF.z;
    res.p3[0] = bbRTF.x; res.p3[1] = bbRTF.y; res.p3[2] = bbRTF.z;
    res.p4[0] = bbLTF.x; res.p4[1] = bbLTF.y; res.p4[2] = bbLTF.z;
    res.p5[0] = bbLBB.x; res.p5[1] = bbLBB.y; res.p5[2] = bbLBB.z;
    res.p6[0] = bbRBB.x; res.p6[1] = bbRBB.y; res.p6[2] = bbRBB.z;
    res.p7[0] = bbRTB.x; res.p7[1] = bbRTB.y; res.p7[2] = bbRTB.z;
    res.p8[0] = bbLTB.x; res.p8[1] = bbLTB.y; res.p8[2] = bbLTB.z;
    
    res.pose.position.x = p.x;
    res.pose.position.y = p.y;
    res.pose.position.z = p.z;
    
    res.pose.orientation.x = q.x();
    res.pose.orientation.y = q.y();
    res.pose.orientation.z = q.z();
    res.pose.orientation.w = q.w();
    
    res.scale.x = s.x;
    res.scale.y = s.y;
    res.scale.z = s.z;
    
    // Log request timestamp
    //--------------------------------------------------------------------------
    ROS_INFO("Request timestamp: %d.%d", req.header.stamp.sec, req.header.stamp.nsec);

    return true;
}


/*==============================================================================
 * Bounding box estimation alternative service.
 *
 * @param req  Request of type EstimateBBAlt.
 * @param res  Response of type EstimateBBAlt.
 */
bool estimateBBAlt_callback(srs_env_model_percp::EstimateBBAlt::Request  &req,
                            srs_env_model_percp::EstimateBBAlt::Response &res
                            )
{
    // Estimate the bounding box
    //--------------------------------------------------------------------------

    // Vertices (corners) defining the bounding box.
    // Each is noted using this template:
    // bb{L=left,R=right}{B=bottom,T=top}{F=front,B=back}
    // (This notation provides more readable code, in comparison with
    // e.g. an array with 8 items.)
    Point3f bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB;

    if( !estimateBB(req.header.stamp,
                    req.p1, req.p2, req.mode,
                    bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB) )
    {
        return false;
    }

    // Calculate also Pose of the bounding box
    //--------------------------------------------------------------------------
    tf::Quaternion q;
    Point3f p, s;

    if( !estimateBBPose(bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB,
                        p, q, s) )
    {
        return false;
    }
    
    // Set response
    //--------------------------------------------------------------------------   
    res.pose.position.x = p.x;
    res.pose.position.y = p.y;
    res.pose.position.z = p.z - 0.5f * s.z;
    
    res.pose.orientation.x = q.x();
    res.pose.orientation.y = q.y();
    res.pose.orientation.z = q.z();
    res.pose.orientation.w = q.w();
    
    res.bounding_box_lwh.x = 0.5f * s.x;
    res.bounding_box_lwh.y = 0.5f * s.y;
    res.bounding_box_lwh.z = s.z;
    
    // Log request timestamp
    //--------------------------------------------------------------------------
    ROS_INFO("Request timestamp: %d.%d", req.header.stamp.sec, req.header.stamp.nsec);

    return true;
}


/*==============================================================================
 * Rectangle estimation service.
 *
 * @param req  Request of type EstimateRect.
 * @param res  Response of type EstimateRect.
 */
bool estimateRect_callback(srs_env_model_percp::EstimateRect::Request  &req,
                           srs_env_model_percp::EstimateRect::Response &res
                           )
{
    // Vertices (corners) defining the bounding box.
    // Each is noted using this template:
    // bb{L=left,R=right}{B=bottom,T=top}{F=front,B=back}
    // (This notation provides more readable code, in comparison with
    // e.g. an array with 8 items.)
    Point3f bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB;

    // Vector with pointers to the BB vertices (to be able to iterate over them)
    vector<Point3f *> bbVertices(8);
    bbVertices[0] = &bbLBF;
    bbVertices[1] = &bbRBF;
    bbVertices[2] = &bbRTF;
    bbVertices[3] = &bbLTF;
    bbVertices[4] = &bbLBB;
    bbVertices[5] = &bbRBB;
    bbVertices[6] = &bbRTB;
    bbVertices[7] = &bbLTB;
    
    // Calculate coordinates of corners of the bounding box relative to its center
    bbLBF.x = -0.5f * req.scale.x;
    bbLBF.y = -0.5f * req.scale.y;
    bbLBF.z = -0.5f * req.scale.z;

    bbLTF.x = -0.5f * req.scale.x;
    bbLTF.y = -0.5f * req.scale.y;
    bbLTF.z =  0.5f * req.scale.z;

    bbRBF.x =  0.5f * req.scale.x;
    bbRBF.y = -0.5f * req.scale.y;
    bbRBF.z = -0.5f * req.scale.z;

    bbRTF.x =  0.5f * req.scale.x;
    bbRTF.y = -0.5f * req.scale.y;
    bbRTF.z =  0.5f * req.scale.z;
    
    bbRBB.x =  0.5f * req.scale.x;
    bbRBB.y =  0.5f * req.scale.y;
    bbRBF.z = -0.5f * req.scale.z;

    bbRTB.x =  0.5f * req.scale.x;
    bbRTB.y =  0.5f * req.scale.y;
    bbRTB.z =  0.5f * req.scale.z;

    bbLBB.x = -0.5f * req.scale.x;
    bbLBB.y =  0.5f * req.scale.y;
    bbRBF.z = -0.5f * req.scale.z;

    bbLTB.x = -0.5f * req.scale.x;
    bbLTB.y =  0.5f * req.scale.y;
    bbLTB.z =  0.5f * req.scale.z;

    // Apply the rotation and translation stored in the Pose parameter
    tf::Quaternion q(req.pose.orientation.x,
                   req.pose.orientation.y, 
                   req.pose.orientation.z, 
                   req.pose.orientation.w);
    btTransform trMat(q);
    for( int i = 0; i < (int)bbVertices.size(); i++ )
    {
        btVector3 res = trMat * btVector3(bbVertices[i]->x, bbVertices[i]->y, bbVertices[i]->z);
        bbVertices[i]->x = res.x() + req.pose.position.x;
        bbVertices[i]->y = res.y() + req.pose.position.y;
        bbVertices[i]->z = res.z() + req.pose.position.z;
    }
    
    // Estimate the rectangle
    point2_t p1, p2;
    if( !estimateRect(req.header.stamp,
                      bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB,
                      p1, p2) )
    {
        return false;
    }
    
    // Set response
    //--------------------------------------------------------------------------   
    res.p1[0] = p1[0];
    res.p1[1] = p1[1];
    res.p2[0] = p2[0];
    res.p2[1] = p2[1];
    
    // Log request timestamp
    //--------------------------------------------------------------------------
    ROS_INFO("Request timestamp: %d.%d", req.header.stamp.sec, req.header.stamp.nsec);   

    return true;
}


/*==============================================================================
 * Rectangle estimation service.
 *
 * @param req  Request of type EstimateRectAlt.
 * @param res  Response of type EstimateRectAlt.
 */
bool estimateRectAlt_callback(srs_env_model_percp::EstimateRectAlt::Request  &req,
                              srs_env_model_percp::EstimateRectAlt::Response &res
                              )
{
    // Vertices (corners) defining the bounding box.
    // Each is noted using this template:
    // bb{L=left,R=right}{B=bottom,T=top}{F=front,B=back}
    // (This notation provides more readable code, in comparison with
    // e.g. an array with 8 items.)
    Point3f bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB;

    // Vector with pointers to the BB vertices (to be able to iterate over them)
    vector<Point3f *> bbVertices(8);
    bbVertices[0] = &bbLBF;
    bbVertices[1] = &bbRBF;
    bbVertices[2] = &bbRTF;
    bbVertices[3] = &bbLTF;
    bbVertices[4] = &bbLBB;
    bbVertices[5] = &bbRBB;
    bbVertices[6] = &bbRTB;
    bbVertices[7] = &bbLTB;
    
    // Calculate coordinates of corners of the bounding box relative to its center
    bbLBF.x = -req.bounding_box_lwh.x;
    bbLBF.y = -req.bounding_box_lwh.y;
    bbLBF.z =  0.0f;

    bbLTF.x = -req.bounding_box_lwh.x;
    bbLTF.y = -req.bounding_box_lwh.y;
    bbLTF.z =  req.bounding_box_lwh.z;

    bbRBF.x =  req.bounding_box_lwh.x;
    bbRBF.y = -req.bounding_box_lwh.y;
    bbRBF.z =  0.0f;

    bbRTF.x =  req.bounding_box_lwh.x;
    bbRTF.y = -req.bounding_box_lwh.y;
    bbRTF.z =  req.bounding_box_lwh.z;
    
    bbRBB.x =  req.bounding_box_lwh.x;
    bbRBB.y =  req.bounding_box_lwh.y;
    bbRBF.z =  0.0f;

    bbRTB.x =  req.bounding_box_lwh.x;
    bbRTB.y =  req.bounding_box_lwh.y;
    bbRTB.z =  req.bounding_box_lwh.z;

    bbLBB.x = -req.bounding_box_lwh.x;
    bbLBB.y =  req.bounding_box_lwh.y;
    bbRBF.z =  0.0f;

    bbLTB.x = -req.bounding_box_lwh.x;
    bbLTB.y =  req.bounding_box_lwh.y;
    bbLTB.z =  req.bounding_box_lwh.z;

    // Apply the rotation and translation stored in the Pose parameter
    tf::Quaternion q(req.pose.orientation.x,
                   req.pose.orientation.y, 
                   req.pose.orientation.z, 
                   req.pose.orientation.w);
    btTransform trMat(q);
    for( int i = 0; i < (int)bbVertices.size(); i++ )
    {
        btVector3 res = trMat * btVector3(bbVertices[i]->x, bbVertices[i]->y, bbVertices[i]->z);
        bbVertices[i]->x = res.x() + req.pose.position.x;
        bbVertices[i]->y = res.y() + req.pose.position.y;
        bbVertices[i]->z = res.z() + req.pose.position.z;
    }
    
    // Estimate the rectangle
    point2_t p1, p2;
    if( !estimateRect(req.header.stamp,
                      bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB,
                      p1, p2) )
    {
        return false;
    }
    
    // Set response
    //--------------------------------------------------------------------------   
    res.p1[0] = p1[0];
    res.p1[1] = p1[1];
    res.p2[0] = p2[0];
    res.p2[1] = p2[1];
    
    // Log request timestamp
    //--------------------------------------------------------------------------
    ROS_INFO("Request timestamp: %d.%d", req.header.stamp.sec, req.header.stamp.nsec);   

    return true;
}


/*==============================================================================
 * Adds messages to cache (for synchronized subscription variant #1) and obtains
 * the corresponding transformation.
 *
 * @param depth  Message with depth image.
 * @param camInfo  Message with camera information.
 */
void sv1_callback(const sensor_msgs::ImageConstPtr &depth,
                const sensor_msgs::CameraInfoConstPtr &camInfo)
{
    if(subVariant == SV_NONE) {
        subVariant = SV_1;
        
        // Set size of cache
        depthCache.setCacheSize(CACHE_SIZE);
        camInfoCache.setCacheSize(CACHE_SIZE);
    }
    else if(subVariant != SV_1) {
        return;
    }

    depthCache.add(depth);
    camInfoCache.add(camInfo);
    
    if(DEBUG) {
        // Prints the timestamp of the received frame
        ROS_INFO("Received frame timestamp: %d.%d",
                 depth->header.stamp.sec, depth->header.stamp.nsec);
    }
}


/*==============================================================================
 * Adds messages to cache (for synchronized subscription variant #2) and obtains
 * the corresponding transformation.
 *
 * @param pointCloud  Message with point cloud.
 * @param camInfo  Message with camera information.
 */
void sv2_callback(const sensor_msgs::PointCloud2ConstPtr &pointCloud,
                const sensor_msgs::CameraInfoConstPtr &camInfo)
{
    if(subVariant == SV_NONE) {
        subVariant = SV_2;
        
        // Set size of cache
        pointCloudCache.setCacheSize(CACHE_SIZE);
        camInfoCache.setCacheSize(CACHE_SIZE);
    }
    else if(subVariant != SV_2) {
        return;
    }

    pointCloudCache.add(pointCloud);
    camInfoCache.add(camInfo);
    
    if(DEBUG) {
        // Prints the timestamp of the received frame
        ROS_INFO("Received frame timestamp: %d.%d",
                 pointCloud->header.stamp.sec, pointCloud->header.stamp.nsec);
    }
}

}


/*==============================================================================
 * Main function.
 */
int main(int argc, char **argv)
{
	using namespace srs_env_model_percp;

    // ROS initialization (the last argument is the name of the node)
    ros::init(argc, argv, "bb_estimator_server");
    
    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle n;
    
    // Create a TF listener
    tfListener = new tf::TransformListener();
    
    // Get parameters from the parameter server
    // (the third parameter of function param is the default value)
    //--------------------------------------------------------------------------
    std::string sv1_depthTopic, sv1_camInfoTopic;
    n.param("bb_sv1_depth_topic", sv1_depthTopic, sv1_depthTopicDefault);
    n.param("bb_sv1_camInfo_topic", sv1_camInfoTopic, sv1_camInfoTopicDefault);
    
    std::string sv2_pointCloudTopic, sv2_camInfoTopic;
    n.param("bb_sv2_pointCloud_topic", sv2_pointCloudTopic, sv2_pointCloudTopicDefault);
    n.param("bb_sv2_camInfo_topic", sv2_camInfoTopic, sv2_camInfoTopicDefault);
    
    n.param("bb_outliers_percent", outliersPercent, outliersPercentDefault);
    n.param("bb_scene_frame_id", sceneFrameId, sceneFrameIdDefault);
    
    // Subscription and synchronization of messages
    // TODO: Create the subscribers dynamically and unsubscribe the unused
    // subscription variant.
    //--------------------------------------------------------------------------
    // Subscription variant #1
    message_filters::Subscriber<Image> sv1_depth_sub(n, sv1_depthTopic, 1);
    message_filters::Subscriber<CameraInfo> sv1_camInfo_sub(n, sv1_camInfoTopic, 1);
    
    typedef sync_policies::ApproximateTime<Image, CameraInfo> sv1_MySyncPolicy;
	Synchronizer<sv1_MySyncPolicy> sv1_sync(sv1_MySyncPolicy(QUEUE_SIZE),
	    sv1_depth_sub, sv1_camInfo_sub);
	sv1_sync.registerCallback(boost::bind(&sv1_callback, _1, _2));
	
	// Subscription variant #2
    message_filters::Subscriber<PointCloud2> sv2_pointCloud_sub(n, sv2_pointCloudTopic, 1);
    message_filters::Subscriber<CameraInfo> sv2_camInfo_sub(n, sv2_camInfoTopic, 1);
    
    typedef sync_policies::ApproximateTime<PointCloud2, CameraInfo> sv2_MySyncPolicy;
	Synchronizer<sv2_MySyncPolicy> sv2_sync(sv2_MySyncPolicy(QUEUE_SIZE),
	    sv2_pointCloud_sub, sv2_camInfo_sub);
	sv2_sync.registerCallback(boost::bind(&sv2_callback, _1, _2));
    
    // Create and advertise this service over ROS
    //--------------------------------------------------------------------------
    ros::ServiceServer service = n.advertiseService(EstimateBB_SRV, estimateBB_callback);
    ros::ServiceServer serviceAlt = n.advertiseService(EstimateBBAlt_SRV, estimateBBAlt_callback);
    ros::ServiceServer serviceRect = n.advertiseService(EstimateRect_SRV, estimateRect_callback);
    ros::ServiceServer serviceRectAlt = n.advertiseService(EstimateRectAlt_SRV, estimateRectAlt_callback);
    ROS_INFO("Ready.");
    
    // Enters a loop, calling message callbacks
    ros::spin();

    return 0;
}

