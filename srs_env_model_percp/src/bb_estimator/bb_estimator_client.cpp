/******************************************************************************
 * \file
 *
 * $Id: bb_estimator_client.cpp 699 2012-04-20 12:41:14Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 02.03.2012 (version 5.0)
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
 * This client demonstrates the function of service "/bb_estimator/estimate_bb"
 * performing bounding box estimation.
 *
 * There are two variants of subscription - to be the service able to work also
 * with a simulation of Care-o-Bot (subscription variant #2 is meant to be used
 * with simulation, which does not produce a depth map so it must be created from
 * a point cloud).
 *
 * Node parameters:
 *  Parameters for subscription variant #1:
 *  bb_sv1_rgb_topic - topic with Image messages containing RGB information
 *  bb_sv1_depth_topic - topic with Image messages containing depth information
 *  bb_sv1_camInfo_topic - topic with CameraInfo messages
 *
 *  Parameters for subscription variant #2:
 *  bb_sv2_rgb_topic - topic with Image messages containing RGB information
 *  bb_sv2_pointCloud_topic - topic with PointCloud2 messages containing Point Cloud
 *  bb_sv2_camInfo_topic - topic with CameraInfo messages
 *
 *  Other parameters:
 *  bb_scene_frame_id - Frame Id in which the BB coordinates are returned
 *
 * Manual:
 * - Use your mouse to specify a region of interest (ROI) in the window with
 *   input video. You will get another window with the visualization of the
 *   resulting bounding box.
 * - Press "D" key to switch between display modes (RGB or DEPTH data is displayed)
 * - Press "E" key to switch between estimation modes:
 *      MODE1 = The ROI corresponds to projection of BB front face and the BB is 
 *         rotated to fit the viewing frustum (representing the back-projection
 *         of the ROI) in such way, that the BB front face is perpendicular
 *         to the frustum's center axis.
 *         (BB can be non-parallel with all axis.)
 *
 *      MODE2 = In the ROI is contained the whole projection of BB.
 *         (BB is parallel with all axis.)
 *
 *      MODE3 = The ROI corresponds to projection of BB front face.
 *         (BB is parallel with all axis.)
 *------------------------------------------------------------------------------
 */

#include "ros/ros.h"
#include "bb_estimator/services_list.h"
#include "srs_env_model_percp/EstimateBB.h" // Definition of the service for BB estimation
#include "srs_interaction_primitives/AddBoundingBox.h"
#include "srs_interaction_primitives/ChangePose.h"
#include "srs_interaction_primitives/ChangeScale.h"

#include <algorithm>
#include <cstdlib>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/cache.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


// Global variables
//------------------------------------------------------------------------------
const bool DEBUG = false; // If true, verbose outputs are written to console.

// Size of waiting queue for messages to arrive and complete their "set"
const int QUEUE_SIZE = 10;

// Default topics to subscribe from:
// Topics for subscription variant #1
const std::string sv1_rgbTopicDefault("/cam3d/rgb/image_raw");
const std::string sv1_depthTopicDefault("/cam3d/depth/image_raw");
const std::string sv1_camInfoTopicDefault("/cam3d/depth/camera_info");

// Topics for subscription variant #2
const std::string sv2_rgbTopicDefault("/cam3d/rgb/image_raw");
const std::string sv2_pointCloudTopicDefault("/cam3d/depth/points");
const std::string sv2_camInfoTopicDefault("/cam3d/camera_info");

// Frame IDs
std::string camFrameId; // Camera frame id (will be obtained automatically)
std::string sceneFrameId; // Scene (world) frame id
const std::string sceneFrameIdDefault("/map"); // Default scene (world) frame id

// Subscription variants
enum subVariantsEnum {SV_NONE=0, SV_1, SV_2};
int subVariant = SV_NONE;

bool isWaitingForResponse = false; // Client is waiting for response from server
bool isBBCalculated = false; // Indicates, if there is any BB already calculated
bool isBBPrimitiveCreated = false; // Indicates, if there is any BB primitive already created
                                   // (for visualization using interactive markers)
int estimationMode = 1; // Estimation mode (modes are defined in BBEstimatorServer.cpp)
Point2i mouseDown; // Position of mouse click
Point2i roiP1, roiP2; // Two diagonally opposite corners of ROI
vector<Point3f> bbVertices(8); // Corners of the estimated BB
geometry_msgs::Pose bbPose; // Pose of the estimated BB
geometry_msgs::Vector3 bbScale; // Scale of the estimated BB

// Names of windows
string inputVideoWinName("Input Video (use your mouse to specify a ROI)");
string bbWinName("Bounding Box");

// Bounding box colors
Scalar bbColorFront(0, 0, 255); // Color of the front side edges of the box
Scalar bbColor(255, 0, 0); // Color of the rest of the edges

// Transformation from world to camera coordinates
tf::StampedTransform worldToSensorTf;

// Display mode enumeration
enum displayModesEnum {RGB, DEPTH};
int displayMode = RGB;

// Client for comunication with bb_estimator_server
ros::ServiceClient bbEstimateClient;

// Client for the add_bounding_box, change_pose and change_scale service
ros::ServiceClient bbAddClient, bbChangePoseClient, bbChangeScaleClient;

// Current and request images
// currentX = the last received image of type X
// requestX = the image of type X, which was current in the time of request
// (currentImage and requestImage are references to corresponding Rgb or Depth
// images - according to the selected display mode)
Mat currentImage, requestImage;
Mat currentRgb, requestRgb, currentDepth, requestDepth;
Mat currentCamK, requestCamK;

// TF listener
tf::TransformListener *tfListener;


/*==============================================================================
 * Visualization of bounding box in a special window.
 */
void visualizeInImage()
{    
    // If the image does not have 3 channels => convert it (we want to visualize
    // the bounding box in color, thus we need 3 channels)
    Mat img3ch;
    if(requestImage.channels() != 3) {
        cvtColor(requestImage, img3ch, CV_GRAY2RGB, 3);
    }
    else {
        requestImage.copyTo(img3ch);
    }
    
    // Transform the resulting BB vertices (in world coordinates)
    // to camera coordinates
    //--------------------------------------------------------------------------
    vector<Point3f> bbVerticesCam(8); // Corners of estimated BB in camera coords
    
    tf::Transformer t;
    t.setTransform(worldToSensorTf);
    
    for(int i = 0; i < (int)bbVertices.size(); i++) {
    
        tf::Stamped<tf::Point> vertex;
        vertex.frame_id_ = sceneFrameId;
    
        vertex.setX(bbVertices[i].x);
        vertex.setY(bbVertices[i].y);
        vertex.setZ(bbVertices[i].z);
        t.transformPoint(camFrameId, vertex, vertex);
        bbVerticesCam[i].x = vertex.getX();
        bbVerticesCam[i].y = vertex.getY();
        bbVerticesCam[i].z = vertex.getZ();
    }
    
    // Perspective projection of the bounding box on the image plane:
    // world (camera) coords (3D) => image coords (2D)
    //
    // The world coordinate system is identical with the camera coordinate system
    // => Tx = Ty = 0 and R is identity => P[1:3,1:3] = K, where P is the camera
    // matrix and K is the intrinsic camera matrix.
    // (http://www.ros.org/wiki/image_pipeline/CameraInfo)
    //
    // The lines below implement these formulas:
    // x = X / Z * f_x + c_x
    // y = Y / Z * f_y + c_y
    // where [X,Y,Z] is a 3D point and [x,y] its perspective projection.
    //--------------------------------------------------------------------------
    double bb3DVerticesArray[8][3];
    for(int i = 0; i < (int)bbVertices.size(); i++) {
        bb3DVerticesArray[i][0] = bbVerticesCam[i].x / (double)bbVerticesCam[i].z;
        bb3DVerticesArray[i][1] = bbVerticesCam[i].y / (double)bbVerticesCam[i].z;
        bb3DVerticesArray[i][2] = 1;
    }
    Mat bb3DVertices = Mat(8, 3, CV_64F, bb3DVerticesArray);

    // requestCamK is the intrinsic camera matrix (containing f_x, f_y, c_x, c_y)
    Mat bb2DVertices = requestCamK * bb3DVertices.t();
    bb2DVertices = bb2DVertices.t();
    
    // Define edges of the bounding box
    //--------------------------------------------------------------------------
    // Edges are represented by connections between vertices calculated above
    int edges[12][2] = {
        {0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 5},
        {2, 6}, {3, 7}, {4, 5}, {5, 6}, {6, 7}, {7, 4}
    };
    
    // Draw the bounding box
    //--------------------------------------------------------------------------
    for(int i = 11; i >= 0; i--) {
        int j = edges[i][0];
        int k = edges[i][1];
        line(img3ch,
            Point((int)bb2DVertices.at<double>(j, 0), (int)bb2DVertices.at<double>(j, 1)),
            Point((int)bb2DVertices.at<double>(k, 0), (int)bb2DVertices.at<double>(k, 1)),
            (i < 4) ? bbColorFront : bbColor, 2);
    }
    
    // Show the image with visualized bounding box
    imshow(bbWinName, img3ch);
}


/*==============================================================================
 * Visualization of bounding box using interactive markers.
 */
void visualizeWithMarkers()
{   
    std::string bbPrimitiveName("estimated_bb");

    // No BB primitive has been created yet => create it
    //--------------------------------------------------------------------------
    if(!isBBPrimitiveCreated) {

        // Set parameters to the new bounding box
        srs_interaction_primitives::AddBoundingBox bbAddSrv;
        
        bbAddSrv.request.name = bbPrimitiveName;
        bbAddSrv.request.frame_id = sceneFrameId;
        bbAddSrv.request.description = "Bounding Box";
        
        bbAddSrv.request.pose = bbPose;
        bbAddSrv.request.scale = bbScale;
        
        bbAddSrv.request.color.r = 1.0;
        bbAddSrv.request.color.g = 0.0;
        bbAddSrv.request.color.b = 1.0;
        bbAddSrv.request.color.a = 1.0;

        // Call service with specified parameters
        bbAddClient.call(bbAddSrv);
        
        isBBPrimitiveCreated = true;
    }
    
    // The BB primitive has already been created => modify it
    //--------------------------------------------------------------------------
    else {
        // Change pose of the BB
        //--------------------
        srs_interaction_primitives::ChangePose bbChangePoseSrv;
        bbChangePoseSrv.request.name = bbPrimitiveName;
        bbChangePoseSrv.request.pose = bbPose;

        // Call service with specified parameters
        bbChangePoseClient.call(bbChangePoseSrv);
        
        // Change scale of the BB
        //--------------------
        srs_interaction_primitives::ChangeScale bbChangeScaleSrv;
        bbChangeScaleSrv.request.name = bbPrimitiveName;
        bbChangeScaleSrv.request.scale = bbScale;

        // Call service with specified parameters
        bbChangeScaleClient.call(bbChangeScaleSrv);
    }
}


/*==============================================================================
 * Visualization of bounding box.
 */
void visualize()
{
    visualizeInImage();
    visualizeWithMarkers();
}


/*==============================================================================
 * Sets the images (current and request one) based on the selected display mode.
 */
void setImages()
{  
    // RGB mode
    if(displayMode == RGB) {
        currentImage = currentRgb;
        requestImage = requestRgb;
    }
    
    // DEPTH mode
    else {
        currentImage = currentDepth;
        requestImage = requestDepth;
    }
}


/*==============================================================================
 * Redraw windows according to the selected display mode.
 */
void redrawWindows()
{
    // Set the current and request images based on the selected display mode
    setImages();
    
    // Show the current image
    imshow(inputVideoWinName, currentImage);
    
    // Show the bounding box (if there is any calculated already)
    if(isBBCalculated) {
        visualize();
    }
}


/*==============================================================================
 * Send request for bounding box calculation.
 *
 * @param p1  A corner of the region of interest.
 * @param p1  A corner of the region of interest - diagonally opposite to p1.
 */
void sendRequest(Point2i p1, Point2i p2)
{
    // Return if the input points are empty
    if(p1 == Point2i() || p2 == Point2i()) {
        return;
    }

    isWaitingForResponse = true;

    // Create the request
    //--------------------------------------------------------------------------
    // Instantiate the autogenerated service class, and assign values into
    // its request member
    srs_env_model_percp::EstimateBB srv;
    srv.request.p1[0] = p1.x;
    srv.request.p1[1] = p1.y;
    srv.request.p2[0] = p2.x;
    srv.request.p2[1] = p2.y;
    srv.request.mode = estimationMode;
    
    // When using simulated Clock time, now() returns time 0 until first message
    // has been received on /clock topic => wait for that.
    ros::Time reqTime;
    do {
        reqTime = ros::Time::now();
    } while(reqTime.sec == 0);
    srv.request.header.stamp = reqTime;
    
    // Obtain the corresponding transformation from world to camera coordinate system
    // (for later visualization in an image)
    //--------------------------------------------------------------------------
    try {
        tfListener->waitForTransform(sceneFrameId, camFrameId, reqTime, ros::Duration(0.2));
        tfListener->lookupTransform(sceneFrameId, camFrameId, reqTime, worldToSensorTf);
    }
    catch(tf::TransformException& ex) {
        string errorMsg = String("Transform error: ") + ex.what();
        ROS_ERROR("%s", errorMsg.c_str());
        return;
    }
    
    // Send request and obtain response (the bounding box coordinates)
    //--------------------------------------------------------------------------    
    // Call the service (calls are blocking, it will return once the call is done)
    if(bbEstimateClient.call(srv)) {
        isWaitingForResponse = false;
        isBBCalculated = true;
        
        srs_env_model_percp::EstimateBB::Response res = srv.response;
        
        // Save the response (in world coordinates)
        //----------------------------------------------------------------------
        bbVertices[0] = Point3f(res.p1[0], res.p1[1], res.p1[2]);
        bbVertices[1] = Point3f(res.p2[0], res.p2[1], res.p2[2]);
        bbVertices[2] = Point3f(res.p3[0], res.p3[1], res.p3[2]);
        bbVertices[3] = Point3f(res.p4[0], res.p4[1], res.p4[2]);
        bbVertices[4] = Point3f(res.p5[0], res.p5[1], res.p5[2]);
        bbVertices[5] = Point3f(res.p6[0], res.p6[1], res.p6[2]);
        bbVertices[6] = Point3f(res.p7[0], res.p7[1], res.p7[2]);
        bbVertices[7] = Point3f(res.p8[0], res.p8[1], res.p8[2]);
        
        bbPose = res.pose;
        bbScale = res.scale;
        
        // Show the resulting bounding box    
        visualize();
        
        // Log request
        //----------------------------------------------------------------------
        std::cout << "----------" << std::endl;
        ROS_INFO("ROI (request): [%ld, %ld], [%ld, %ld]",
                (long int)srv.request.p1[0], (long int)srv.request.p1[1],
                (long int)srv.request.p2[0], (long int)srv.request.p2[1]);
        
        // Log response
        //----------------------------------------------------------------------
        std::stringstream ss;
        ss << "Bounding box (response):\n";
        for(int i = 0; i < (int)bbVertices.size(); i++) {
            ss << "[" << bbVertices[i].x << ","
                      << bbVertices[i].y << ","
                      << bbVertices[i].z << "] ";
            if(i == 3) ss << "\n";
        }
        ss << "Mode: " << estimationMode;
        ROS_INFO("%s", ss.str().c_str());
    }
    else {
        isWaitingForResponse = false;
        std::string errMsg = "Failed to call service " + BB_ESTIMATOR_Estimate_SRV + ".";
        ROS_ERROR("%s", errMsg.c_str());
    }
}

/*==============================================================================
 * Scales depth values (for visualization purposes).
 *
 * The provided depth is typically in CV_16SC1 datatype.
 * Because we do not expect to have any negative depth value, we
 * can convert the datatype to unsigned integers, which is sufficient
 * for visualization purposes + more OpenCV functions can work with this
 * datatype (e.g. cvtColor used in visualize function).
 * The depth values are scaled to fit the range <0, 255>.
 */
void scaleDepth()
{
    // Do not consider large ("infinity") values
    Mat infMask;
    if(subVariant == SV_1) {
        infMask = currentDepth > 20000; // Pixels
    }
    else if(subVariant == SV_2) {
        infMask = currentDepth > 20; // Meters
    }
    
    // Get mask of known values
    Mat knownMask = (infMask == 0);

    double maxDepth;
    minMaxLoc(currentDepth, NULL, &maxDepth, NULL, NULL, knownMask);
    currentDepth.convertTo(currentDepth, CV_8U, 255.0 / maxDepth);
    
    // Set color of "infinity" values
    currentDepth.setTo(255, infMask);
}

/*==============================================================================
 * Processes messages from synchronized subscription variant #1.
 *
 * @param rgb  Message with rgb image.
 * @param depth  Message with depth image.
 * @param camInfo  Message with camera information.
 */
void sv1_processSubMsgs(const sensor_msgs::ImageConstPtr &rgb,
                    const sensor_msgs::ImageConstPtr &depth,
                    const sensor_msgs::CameraInfoConstPtr &camInfo)
{
    if(subVariant == SV_NONE) {
        subVariant = SV_1;
    }
    else if(subVariant != SV_1) {
        return;
    }

    // Get the intrinsic camera matrix of our camera
    Mat K = Mat(3, 3, CV_64F, (double *)camInfo->K.data());
    K.copyTo(currentCamK);
    
    // Save frame id
    camFrameId = camInfo->header.frame_id;

    // Get rgb and depth image from the messages
    //--------------------------------------------------------------------------
    try {
        currentRgb = cv_bridge::toCvCopy(rgb)->image;
        currentDepth = cv_bridge::toCvCopy(depth)->image;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // Scales depth values (for visualization purposes).
    scaleDepth();
    
    // Set the current and request images based on the selected display mode
    setImages();
    
    // Show the current image
    //--------------------------------------------------------------------------
    imshow(inputVideoWinName, currentImage);
    
    if(DEBUG) {
        // Prints the timestamp of the received frame
        ROS_INFO("Received frame timestamp: %d.%d",
                 rgb->header.stamp.sec, rgb->header.stamp.nsec);
    }
}


/*==============================================================================
 * Processes messages from synchronized subscription variant #2.
 *
 * @param rgb  Message with rgb image.
 * @param pointCloud  Message with point cloud.
 * @param camInfo  Message with camera information.
 */
void sv2_processSubMsgs(const sensor_msgs::ImageConstPtr &rgb,
                    const sensor_msgs::PointCloud2ConstPtr &pointCloud,
                    const sensor_msgs::CameraInfoConstPtr &camInfo)
{
    if(subVariant == SV_NONE) {
        subVariant = SV_2;
    }
    else if(subVariant != SV_2) {
        return;
    }

    // Get the intrinsic camera matrix of our camera
    Mat K = Mat(3, 3, CV_64F, (double *)camInfo->K.data());
    K.copyTo(currentCamK);
    
    // Save frame id
    camFrameId = camInfo->header.frame_id;

    // Convert the sensor_msgs/PointCloud2 data to depth map
    //--------------------------------------------------------------------------
    // At first convert to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*pointCloud, cloud);
    
    currentDepth = Mat(cloud.height, cloud.width, CV_32F);
    
    for(int y = 0; y < (int)cloud.height; y++) {
        for(int x = 0; x < (int)cloud.width; x++) {
            currentDepth.at<float>(y, x) = cloud.points[y * cloud.width + x].z;
        }
    }

    // Get rgb and depth image from the Image messages
    //--------------------------------------------------------------------------
    try {
        currentRgb = cv_bridge::toCvCopy(rgb)->image;
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    // The image / point cloud coming from COB is flipped around X and Y axis
    //flip(currentRgb, currentRgb, -1);
    //flip(currentDepth, currentDepth, -1);
    
    // Scales depth values (for visualization purposes).
    scaleDepth();
    
    // Set the current and request images based on the selected display mode
    setImages();
    
    // Show the current image
    //--------------------------------------------------------------------------
    imshow(inputVideoWinName, currentImage);
    
    if(DEBUG) {
        // Prints the timestamp of the received frame
        ROS_INFO("Received frame timestamp: %d.%d",
                 rgb->header.stamp.sec, rgb->header.stamp.nsec);
    }
}


/*==============================================================================
 * Mouse event handler.
 *
 * @param event  Mouse event to be handled.
 * @param x  x-coordinate of the event.
 * @param y  y-coordinate of the event.
 */
void onMouse(int event, int x, int y, int flags, void *param)
{
    switch(event) {
    
        // Mouse DOWN (start-point of the ROI)
        case CV_EVENT_LBUTTONDOWN:
            mouseDown.x = x;
            mouseDown.y = y;
            break;
            
        // Mouse UP (end-point of the ROI)
        case CV_EVENT_LBUTTONUP:
            // Save the current images as the request ones
            currentImage.copyTo(requestImage);
            currentRgb.copyTo(requestRgb);
            currentDepth.copyTo(requestDepth);
            currentCamK.copyTo(requestCamK);
            
            roiP1 = Point2i(mouseDown.x, mouseDown.y);
            roiP2 = Point2i(x, y);
  
            // Send request for bounding box calculation
            // (if there is no pending one already)
            if(!isWaitingForResponse) {
                sendRequest(roiP1, roiP2);
            }
            break;
    }
}


/*==============================================================================
 * Main function
 */
int main(int argc, char **argv)
{
    // ROS initialization (the last argument is the name of the node)
    ros::init(argc, argv, "bb_estimator_client");
    
    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle n;
    
    // Create a client for the /bb_estimator/estimate_bb service
    bbEstimateClient = n.serviceClient<srs_env_model_percp::EstimateBB>(BB_ESTIMATOR_Estimate_SRV);
    
    // Create clients for the add_bounding_box, change_pose and change_scale services
    // (for manipulation with a visualization of BB)
    bbAddClient = n.serviceClient<srs_interaction_primitives::AddBoundingBox>("/but_interaction_primitives/add_bounding_box");
    bbChangePoseClient = n.serviceClient<srs_interaction_primitives::ChangePose>("/but_interaction_primitives/change_pose");
    bbChangeScaleClient = n.serviceClient<srs_interaction_primitives::ChangeScale>("/but_interaction_primitives/change_scale");
    
    // Create a TF listener
    tfListener = new tf::TransformListener();
    
    // Get parameters from the parameter server
    // (the third parameter of function param is the default value)
    //--------------------------------------------------------------------------
    std::string sv1_rgbTopic, sv1_depthTopic, sv1_camInfoTopic;
    n.param("bb_sv1_rgb_topic", sv1_rgbTopic, sv1_rgbTopicDefault);
    n.param("bb_sv1_depth_topic", sv1_depthTopic, sv1_depthTopicDefault);
    n.param("bb_sv1_camInfo_topic", sv1_camInfoTopic, sv1_camInfoTopicDefault);
    
    std::string sv2_rgbTopic, sv2_pointCloudTopic, sv2_camInfoTopic;
    n.param("bb_sv2_rgb_topic", sv2_rgbTopic, sv2_rgbTopicDefault);
    n.param("bb_sv2_pointCloud_topic", sv2_pointCloudTopic, sv2_pointCloudTopicDefault);
    n.param("bb_sv2_camInfo_topic", sv2_camInfoTopic, sv2_camInfoTopicDefault);
    
    n.param("bb_scene_frame_id", sceneFrameId, sceneFrameIdDefault);

    // Subscription and synchronization of messages
    // TODO: Create the subscribers dynamically and unsubscribe the unused
    // subscription variant.
    //--------------------------------------------------------------------------
    // Subscription variant #1
    message_filters::Subscriber<Image> sv1_rgb_sub(n, sv1_rgbTopic, 1);
    message_filters::Subscriber<Image> sv1_depth_sub(n, sv1_depthTopic, 1);
    message_filters::Subscriber<CameraInfo> sv1_camInfo_sub(n, sv1_camInfoTopic, 1);
        
    typedef sync_policies::ApproximateTime<Image, Image, CameraInfo> sv1_MySyncPolicy;
	Synchronizer<sv1_MySyncPolicy> sv1_sync(sv1_MySyncPolicy(QUEUE_SIZE),
	    sv1_rgb_sub, sv1_depth_sub, sv1_camInfo_sub);
	sv1_sync.registerCallback(boost::bind(&sv1_processSubMsgs, _1, _2, _3));
	
	// Subscription variant #2
	message_filters::Subscriber<Image> sv2_rgb_sub(n, sv2_rgbTopic, 1);
    message_filters::Subscriber<PointCloud2> sv2_pointCloud_sub(n, sv2_pointCloudTopic, 1);
    message_filters::Subscriber<CameraInfo> sv2_camInfo_sub(n, sv2_camInfoTopic, 1);
    
    typedef sync_policies::ApproximateTime<Image, PointCloud2, CameraInfo> sv2_MySyncPolicy;
	Synchronizer<sv2_MySyncPolicy> sv2_sync(sv2_MySyncPolicy(QUEUE_SIZE),
	    sv2_rgb_sub, sv2_pointCloud_sub, sv2_camInfo_sub);
	sv2_sync.registerCallback(boost::bind(&sv2_processSubMsgs, _1, _2, _3));

    // Create a window to show the incoming video and set its mouse event handler
    //--------------------------------------------------------------------------
    namedWindow(inputVideoWinName.c_str(), CV_WINDOW_AUTOSIZE);
    cvSetMouseCallback(inputVideoWinName.c_str(), onMouse);

    ROS_INFO("Ready.");

    // Enters a loop
    //--------------------------------------------------------------------------
    while(ros::ok()) {
        char key = waitKey(10); // Process window events (i.e. also mouse events)
        
        // If the key D was pressed -> change the display mode
        if(key == 'd' || key == 'D') {
            displayMode = (displayMode == RGB) ? DEPTH : RGB;
            redrawWindows(); // Redraw windows (with input video and bounding box)
        }
        // If the key E was pressed -> change the estimation mode
        else if(key == 'e' || key == 'E') {
            estimationMode = (estimationMode == 3) ? 1 : (estimationMode + 1);
            ROS_INFO("Estimation mode %d activated.", estimationMode);
            sendRequest(roiP1, roiP2);
        }
        
        ros::spinOnce(); // Call all the message callbacks waiting to be called
    }

    return 0;
}

