/**
 * $Id: bb_estimator_server.cpp 138 2012-01-12 23:56:08Z xhodan04 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Date: 11.01.2012 (version 4.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 * This server advertises service "bb_estimate" performing bounding box estimation.
 * The input/output is defined in estimateBB.srv file.
 *
 * There are two variants of subscription - to be able to work with the newest
 * and also older versions of Care-o-Bot (subscription variant #2 is for COB 3-3
 * which doesn't provide a depth map so it must be created from a point cloud).
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
 *------------------------------------------------------------------------------
 */

#include "ros/ros.h"
#include "srs_env_model_percp/EstimateBB.h" // Definition of the service for BB estimation
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

using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;


// Global variables
//------------------------------------------------------------------------------
const bool DEBUG = false; // If true, verbose outputs are written to console.

// Cache
const int CACHE_SIZE = 10;
message_filters::Cache<Image> depthCache; // Cache of Image messages
message_filters::Cache<PointCloud2> pointCloudCache; // Cache of PointCloud2 messages
message_filters::Cache<CameraInfo> camInfoCache; // Cache of CameraInfo messages

// Size of waiting queue for messages to arrive and complete their "set"
const int QUEUE_SIZE = 10;

// Default topics to subscribe from:
// Topics for subscription variant #1
const std::string sv1_depthTopicDefault("/cam3d/depth/image_raw");
const std::string sv1_camInfoTopicDefault("/cam3d/depth/camera_info");

// Topics for subscription variant #2
const std::string sv2_pointCloudTopicDefault("/cam3d/depth/points");
const std::string sv2_camInfoTopicDefault("/cam3d/camera_info");

// Subscription variants
enum subVariantsEnum {SV_NONE=0, SV_1, SV_2};
int subVariant = SV_NONE;

// Percentage of furthest points from mean considered as outliers when
// calculating statistics of ROI
int outliersPercent;
const int outliersPercentDefault = 10;

// The required maximum ratio of sides length (the longer side is at maximum
// sidesRatio times longer than the shorter one)
double sidesRatio;
const double sidesRatioDefault = 5;

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
enum estimationModeEnum{MODE1 = 1, MODE2, MODE3};
int estimationMode;

// The number of pixels per meter for Kinect IR camera
// focal length = 0.00473 [m], 589.367 [pixels] => 589.3667 / 0.00473 = 124601.839
// Ref:
// http://www.isprs.org/proceedings/XXXVIII/5-W12/Papers/ls2011_submission_40.pdf
// http://www.ros.org/wiki/kinect_calibration/technical#Lens_distortion_and_focal_length
//const float kinectIRPixelsPerMeter = 124601.839;


/*==============================================================================
 * Back perspective projection of a 2D point with a known depth:
 * 2D point in image coords + known depth => 3D point in world (camera) coords
 * (Result of back projection of an image point is typically a line. However,
 * in our case we know also its depth (from the depth map) and thus we can
 * determine its location on that line and so get a single 3D point.)
 *
 * The world coordinate system is in our case identical with the camera
 * coordinate system => Tx = Ty = 0 and R is identity => P[1:3,1:3] = K,
 * where P is the camera matrix and K is the intrinsic camera matrix.
 * (http://www.ros.org/wiki/image_pipeline/CameraInfo)
 *
 * This function implements these formulas:
 * X = x * Z / f_x
 * Y = y * Z / f_y
 * where [x,y] is an image point (with (0,0) in the middle of the image)
 * and [X,Y,Z] its perspective back projection in the given depth.
 *
 * @param p  2D image point.
 * @param z  Depth of the back projected 3D point.
 * @param fx  Focal length w.r.t. axis X.
 * @param fy  Focal length w.r.t. axis Y.
 * @return  The perspective back projection in the given depth.
 */
Point3f backProject(Point2i p, float z, float fx, float fy)
{
    return Point3f((p.x * z) / fx, (p.y * z) / fy, z);
}


/*==============================================================================
 * Calculation of statistics (mean, standard deviation, min and max).
 *
 * @param m  The matrix with depth information from which the statistics will
 *           be calculated (it is assumed that the unknown values are represented
 *           by zero).
 * @param mean  The calculated mean of m.
 * @param stdDev  The calculated standard deviation of m.
 * @return  True if the statistics was calculated. False if the statistics could
 *          not be calculated, because there is no depth information available
 *          in the specified ROI.
 */
bool calcStats(Mat &m, float *mean, float *stdDev)
{
    // Get the mask of known values (the unknown are represented by 0, the
    // known by 255)
    Mat negMask = m <= 0; // Negative values
    Mat infMask = m > 20000; // "Infinite" values (more than 20 meters)
    Mat knownMask = ((negMask + infMask) == 0);

    // Mean and standard deviation
    Scalar meanS, stdDevS;
    float meanValue, stdDevValue;
    meanStdDev(m, meanS, stdDevS, knownMask);
    meanValue = meanS[0];
    stdDevValue = stdDevS[0];
    
    // When all the knownMask elements are zeros, the function meanStdDev returns
    // roiMean == roiStdDev == 0
    if(meanValue == 0 && stdDevValue == 0) {
        ROS_WARN("No depth information available in the region of interest");
        return false;
    }
    
    // Number of known values and outliers to be ignored
    int knownCount = sum(knownMask * (1.0/255.0))[0];
    int outliersCount = (knownCount * outliersPercent) / 100;
    
    // Just for sure check if there will be some known values left after removal
    // of outliers.
    if((knownCount - outliersCount) > 0) {
        // Values of m relative to the mean value
        Mat mMeanRel = abs(m - meanValue);
        
        // Find and ignore the given percentage of outliers (the furthest points
        // from the mean value)
        Point2i maxLoc;
        for(int i = 0; i < outliersCount; i++) {
            minMaxLoc(mMeanRel, NULL, NULL, NULL, &maxLoc, knownMask);
            knownMask.at<uchar>(maxLoc.y, maxLoc.x) = 0;
        }
        
        // Mean and standard deviation (now ignoring the outliers)
        meanStdDev(m, meanS, stdDevS, knownMask);
        meanValue = meanS[0];
        stdDevValue = stdDevS[0];
    }
    
    *mean = meanValue;
    *stdDev = stdDevValue;


    if(DEBUG) {
        // Min and max
        double min, max;
        minMaxLoc(m, &min, &max, 0, 0, knownMask);

        // Print the calculated statistics
        std::cout << "DEPTH STATISTICS "
                  << "- Mean: " << *mean << ", StdDev: " << *stdDev
                  << ", Min: " << min << ", Max: " << max << std::endl;
    }
    
    return true;
}


/*==============================================================================
 * Calculation of distances from origin to the BB front and back face vertices.
 * (It is used in MODE1.)
 *
 * @param m  The matrix with distance information (distance from origin).
 * @param fx  Focal length w.r.t. X axis.
 * @param fy  Focal length w.r.t. Y axis.
 * @param roiLB  Left-bottom corner of ROI.
 * @param roiRT  Right-top corner of ROI.
 * @param d1  Caclulated distance from origin to the BB front face.
 * @param d2  Caclulated distance from origin to the BB back face.
 * @return  True if the distance values were calculated. False if not (due to
 *          missing depth information in function calcStats).
 */
bool calcNearAndFarFaceDistance(Mat &m, float fx, float fy, Point2i roiLB,
                                Point2i roiRT, float *d1, float *d2)
{

	// Get mean and standard deviation
	float mean, stdDev;
	if(!calcStats(m, &mean, &stdDev)) {
		return false;
	}

	// Distance from origin to the front and back face vertices
	*d1 = mean - stdDev;
	*d2 = mean + stdDev;

	// Check if all BB vertices are in the front of the image plane (only the
	// vertices of the front face can be behind (= closer to origin),
	// so it is enough to check them). A vector from origin to the depth = f
	// in direction of each of the front face vertices is constructed.
	// If the length l of such vector is bigger than the distance d1
	// (= distance from origin to the front face vertices), it means that
	// the corresponding vertex is behind the image plane, thus set d1 to l
	// (the vertex is moved to depth = f). This is done for all front face
	// vertices...
    float f = (fx + fy) / 2.0;
	
	Point3f vecLBFf = backProject(roiLB, f, fx, fy);
	float vecLBFlen = norm(vecLBFf);
	if(vecLBFlen > *d1 || *d1 < 0) *d1 = vecLBFlen;

	Point3f vecRBFf = backProject(Point2i(roiRT.x, roiLB.y), f, fx, fy);
	float vecRBFlen = norm(vecRBFf);
	if(vecRBFlen > *d1) *d1 = vecRBFlen;

	Point3f vecRTFf = backProject(roiRT, f, fx, fy);
	float vecRTFlen = norm(vecRTFf);
	if(vecRTFlen > *d1) *d1 = vecRTFlen;

	Point3f vecLTFf = backProject(Point2i(roiLB.x, roiRT.y), f, fx, fy);
	float vecLTFlen = norm(vecLTFf);
	if(vecLTFlen > *d1) *d1 = vecLTFlen;
	
	return true;
}


/*==============================================================================
 * Calculation of depth of near and far face of BB (perpendicular with all axis).
 * (It is used in MODE2 and MODE3.)
 *
 * @param m  The matrix with depth information.
 * @param f  Focal length.
 * @param z1  Caclulated depth of the near BB face.
 * @param z2  Caclulated depth of the far BB face.
 * @return  True if the depth values were calculated. False if not (due to
 *          missing depth information in function calcStats).
 */
bool calcNearAndFarFaceDepth(Mat &m, float f, float *z1, float *z2)
{    
    // Get statistics of depth in ROI
    float mean, stdDev;
    if(!calcStats(m, &mean, &stdDev)) {
       return false;
    }
    
    // The near and far faces of BB are positioned in the same distance
    // (given by the depth standard deviation) from the depth mean.
    *z1 = mean - stdDev; // Depth of the near BB face
    *z2 = mean + stdDev; // Depth of the far BB face
        
    // There cannot be any object whose depth is smaller than the focal length
    // => if we have obtained such value, set the value to be equal to f.
    *z1 = (*z1 > f) ? *z1 : f;
    
    return true;
}


/*==============================================================================
 * Bounding box estimation.
 *
 * @param req  Request of type estimateBB.
 * @param res  Response of type estimateBB.
 */
bool estimateBB(srs_env_model_percp::EstimateBB::Request  &req,
                srs_env_model_percp::EstimateBB::Response &res)
{
    // Set estimation mode. If it is not specified in the request or the value
    // is not valid => set MODE1 (== 1) as default estimation mode.
    estimationMode = req.mode;
    if(estimationMode == 0 || estimationMode > 3) {
        estimationMode = 1;
    }
    
    Mat depthMap;

    // Read the messages from cache (the latest ones before the request timestamp)
    //--------------------------------------------------------------------------
    sensor_msgs::CameraInfoConstPtr camInfo = camInfoCache.getElemBeforeTime(req.header.stamp);
    if(camInfo == 0) {
        ROS_ERROR("Cannot calculate the bounding box. "
                  "No frames were obtained before the request time.");
        return false;
    }
    
    // Subscription variant #1 - there is an Image message with a depth map
    //----------------------
    if(subVariant == SV_1) {
        sensor_msgs::ImageConstPtr depth = depthCache.getElemBeforeTime(req.header.stamp);

        // Get depth from the message
        try {
            cv_bridge::CvImagePtr cvDepth;
            cvDepth = cv_bridge::toCvCopy(depth);
            cvDepth->image.convertTo(depthMap, CV_32F);
        }
        catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return false;
        }
    }
    
    // Subscription variant #2 - the depth map must be created from a point clound
    //----------------------
    else if(subVariant == SV_2) {
        sensor_msgs::PointCloud2ConstPtr pointCloud = pointCloudCache.getElemBeforeTime(req.header.stamp);
    
        // Convert the sensor_msgs/PointCloud2 data to depth map
        // At first convert to pcl/PointCloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg (*pointCloud, cloud);
        
        depthMap = Mat(cloud.height, cloud.width, CV_32F);
        
        for(int y = 0; y < (int)cloud.height; y++) {
            for(int x = 0; x < (int)cloud.width; x++) {
                float z = cloud.points[y * cloud.width + x].z;
                if(cvIsNaN(z)) z = 0;
                depthMap.at<float>(y, x) = z;
            }
        }
        
        // The point cloud coming from COB is flipped around X and Y axis
        flip(depthMap, depthMap, -1);
        
        // Convert the depth coming in meters to pixels
        depthMap *= 1000.0;
    }
    else {
        ROS_ERROR("Unknown subscription variant!");
        return false;
    }
    
    // Width and height of the depth image
    int width = depthMap.cols;
    int height = depthMap.rows;

    // Get the coordinates of the specified ROI (Region of Interest)
    //--------------------------------------------------------------------------
    // Determine the left-bottom and the right-top ROI corner.
    // It is assumed that the origin (0,0) is in the top-left image corner.
    Point2i roiLBi; // Left-bottom corner of ROI (in image coordinates)
    Point2i roiRTi; // Right-top corner of ROI (in image coordinates)
    
    if(req.p1[0] < req.p2[0]) {roiLBi.x = req.p1[0]; roiRTi.x = req.p2[0];}
    else {roiLBi.x = req.p2[0]; roiRTi.x = req.p1[0];}
    
    if(req.p1[1] < req.p2[1]) {roiRTi.y = req.p1[1]; roiLBi.y = req.p2[1];}
    else {roiRTi.y = req.p2[1]; roiLBi.y = req.p1[1];}
    
    // Consider only the part of the ROI which is within the image
    roiLBi.x = min(max(roiLBi.x, 0), width);
    roiLBi.y = min(max(roiLBi.y, 0), height);
    roiRTi.x = min(max(roiRTi.x, 0), width);
    roiRTi.y = min(max(roiRTi.y, 0), height);
    
    // Get depth information in the ROI
    Mat roi = Mat(depthMap, Rect(roiLBi.x, roiRTi.y,
                                       roiRTi.x - roiLBi.x, roiLBi.y - roiRTi.y));

    // Get the intrinsic camera parameters (needed for back-projection)
    //--------------------------------------------------------------------------
    Mat K = Mat(3, 3, CV_64F, (double *)camInfo->K.data());
    float cx = (float)K.at<double>(0, 2);
    float cy = (float)K.at<double>(1, 2);
    float fx = (float)K.at<double>(0, 0);
    float fy = (float)K.at<double>(1, 1);
    float f = (fx + fy) / 2.0;
    
    if(fx == 0 || fy == 0 || cx == 0 || cy == 0) {
        ROS_ERROR("Intrinsic camera parameters are undefined.");
    }
    
    // Transform the corners of ROI from image coordinates to coordinates
    // with center in the middle of the image
    // (Notice that the Y axis is flipped! = going downwards)
    //--------------------------------------------------------------------------
    Point2i roiLB(roiLBi.x - cx, roiLBi.y - cy);
    Point2i roiRT(roiRTi.x - cx, roiRTi.y - cy);
    
    // Vertices (corners) defining the bounding box.
    // Each is noted using this template:
    // bb{L=left,R=right}{B=bottom,T=top}{F=front,B=back}
    // (This notation provides more readable code, in comparison with
    // e.g. an array with 8 items.)
    Point3f bbLBF, bbRBF, bbRTF, bbLTF, bbLBB, bbRBB, bbRTB, bbLTB;
    
    // MODE #1
    //--------------------------------------------------------------------------
    if(estimationMode == MODE1) {
        
        // Convert depth to distance from origin (0,0,0) (= optical center).
        Mat roiD(roi.size(), CV_32F);
        for(int i = 0; i < roi.rows; i++) {
            for(int j = 0; j < roi.cols; j++) {
                int z = roi.at<float>(i, j);
                Point3f P = backProject(Point2i(roiLB.x + i, roiRT.y + j), z, fx, fy);
                
                // Get the distance from the origin
                roiD.at<float>(i, j) = norm(P);
            }
        }
        
        // Get distance from origin to the front and back face vertices
        float d1, d2;
        if(!calcNearAndFarFaceDistance(roiD, fx, fy, roiLB, roiRT, &d1, &d2)) {
            return false;
        }
        
        // Front-face vertices of BB
        // Each of them is obtained by construction of a vector from origin with
        // length = d1 in direction of the corresponding ROI corner.
        //----------------------------------------------------------------------
        Point3f vecLBF = backProject(roiLB, 1, fx, fy);
        bbLBF = vecLBF * (d1 / (float)norm(vecLBF));
        
        Point3f vecRBF = backProject(Point2i(roiRT.x, roiLB.y), 1, fx, fy);
        bbRBF = vecRBF * (d1 / (float)norm(vecRBF));
        
        Point3f vecRTF = backProject(roiRT, 1, fx, fy);
        bbRTF = vecRTF * (d1 / (float)norm(vecRTF));
        
        Point3f vecLTF = backProject(Point2i(roiLB.x, roiRT.y), 1, fx, fy);
        bbLTF = vecLTF * (d1 / (float)norm(vecLTF));
        
        // Back-face vertices of BB
        // These vertices are obtained by translation of the corresponding
        // front face vertices by vector vecFFtoBF, which is the vector from the
        // intersection of the viewing frustum axis (A) and the circle with radius
        // d1 to the intersection of A and the circle with radius d2.
        //----------------------------------------------------------------------
        // Calculate an unit vector pointing to the center of the front face of BB
        Point3f vecMid = bbLBF + bbRBF + bbRTF + bbLTF;
        vecMid *= 1.0 / (float)norm(vecMid);
        
        // Calculate a vector from the front face to the back face (perpendicular
        // to them). We know that this vector must have the same direction as
        // vecMid, we only don't know its magnitude. We can express the magnitude
        // (k) from this equation: d2 = |vecLBF + k*vecMid|, which is describing
        // the geometry of our problem. After expressing k we get a quadratic
        // equation: a*k^2 + b*k + c = 0, where:
        // a = vecMid.x*vecMid.x + vecMid.y*vecMid.y + vecMid.z*vecMid.z = 1
        // b = 2*(bbLBF.x*vecMid.x + bbLBF.y*vecMid.y + bbLBF.z*vecMid.z)
        // c = d1*d1 - d2*d2
        // Discriminant D is always positive, because c <= 0, since d2 >= d1, so
        // we know that the equation has at least one solution. We do not
        // want to flip the direction of vecMid, thus we can observe that we are
        // interested only in the positiove solution (the bigger one).
        float b = 2.0*(bbLBF.x*vecMid.x + bbLBF.y*vecMid.y + bbLBF.z*vecMid.z);
        float c = d1*d1 - d2*d2;
        float D = b*b - 4.0*c;
        float k = (-b + sqrt(D)) / 2.0;
        Point3f vecFFtoBF = k * vecMid;
        
        // Back-face vertices
        bbLBB = bbLBF + vecFFtoBF;
        bbRBB = bbRBF + vecFFtoBF;
        bbRTB = bbRTF + vecFFtoBF;
        bbLTB = bbLTF + vecFFtoBF;
    }
    
    // MODE #2
    //--------------------------------------------------------------------------
    // Only bbLBF and bbRTF are calculated in this mode. The rest of the BB
    // vertices is expressed using these points (it is possible because the
    // resulting BB is parallel with all axis in this mode).
    else if(estimationMode == MODE2) {
        
        // Get depth of near and far face of BB
        float z1, z2;
        if(!calcNearAndFarFaceDepth(roi, f, &z1, &z2)) {
            return false;
        }

        // If necessary, flip axis (this is done to unify the calculation in
        // different situations). There can appear these situations (#1 and #2
        // can appear simultaneously):
        // 1) The X coordinates of both specified ROI corners are positive
        // => we flip them to be negative (the resulting 3D points will be then
        // flipped back). We want that, because we want bbLBF to be on the border
        // of viewing frustum - calculation for this case is implemented.
        // 2) The Y coordinates of both specified ROI corners are negative
        // => we flip them to be positive (the resulting 3D points will be then
        // flipped back). We want that, because we want bbRTB to be on the border
        // of viewing frustum - calculation for this case is implemented.
        // 3) If one X (Y) coordinate is negative and one positive, we know
        // that the X (Y) coordinates of the back BB face projection won't
        // be outside the range given by the X (Y) coordinates of the front BB
        // face projection, thus the calculation is the same as in MODE #3 in
        // this situation.
        //-----------------------------
        // Copy the corners of ROI (because they may be modified to unify the
        // calculation)
        Point2i roiLBm = roiLB;
        Point2i roiRTm = roiRT;
        
        bool flippedX = false;
        bool flippedY = false;
        
        // If both corners of ROI have positive X-coordinate => flip X-axis.
        if(roiLBm.x > 0) {
            roiLBm.x = -roiRT.x;
            roiRTm.x = -roiLB.x;
            flippedX = true;
        }
        
        // If both corners of ROI have negative Y-coordinate => flip Y-axis.
        if(roiLBm.y < 0) {
            roiLBm.y = -roiRT.y;
            roiRTm.y = -roiLB.y;
            flippedY = true;
        }
        
        // X coordinates of BB vertices
        //-----------------------------
        if(roiLBm.x <= 0 && roiRTm.x >= 0) {
            bbLBF.x = (roiLBm.x * z1) / fx;
            bbRTB.x = (roiRTm.x * z1) / fx;
        }
        else {
            bbLBF.x = (roiLBm.x * z1) / fx;
            bbRTB.x = (roiRTm.x * z2) / fx;
        
            // Degenerate case #1 - the back-projection of the left ROI corner
            // is on the right of the back-projection of the right ROI corner
            // => set X-coord of both to the middle value.
            if(bbLBF.x > bbRTB.x) {
                float midX = (bbLBF.x + bbRTB.x) / 2.0;
                bbLBF.x = midX;
                bbRTB.x = midX;
                z1 = (midX * fx) / roiLBm.x;
                z2 = (midX * fx) / roiRTm.x;
            }
        }
        
        // Y coordinates of BB vertices
        //-----------------------------
        if(roiLBm.y >= 0 && roiRTm.y <= 0) {
            bbLBF.y = (roiLBm.y * z1) / fy;
            bbRTB.y = (roiRTm.y * z1) / fy;
        }
        else {
            bbLBF.y = (roiLBm.y * z1) / fy;
            bbRTB.y = (roiRTm.y * z2) / fy;
            
            // Degenerate case #2 - the back-projection of the bottom ROI corner
            // is on the top of the back-projection of the top ROI corner.
            // => set Y-coord of both to the middle value
            if(bbLBF.y < bbRTB.y) {
                float midY = (bbLBF.y + bbRTB.y) / 2.0;
                bbLBF.y = midY;
                bbRTB.y = midY;
                z1 = (midY * fy) / roiLBm.y;
                z2 = (midY * fy) / roiRTm.y;
                
                // Adjust X coordinates to the new depth
                if(roiLBm.x <= 0 && roiRTm.x >= 0) {
                    bbLBF.x = (roiLBm.x * z1) / fx;
                    bbRTB.x = (roiRTm.x * z1) / fx;
                }
                else {
                    bbLBF.x = (roiLBm.x * z1) / fx;
                    bbRTB.x = (roiRTm.x * z2) / fx;
                }
            }
        }
        
        // Z coordinates of BB vertices
        //-----------------------------
        bbLBF.z = z1;
        bbRTB.z = z2;
        
        // If any axis was flipped => flip back
        if(flippedX) {
            float bbLBFx = bbLBF.x;
            bbLBF.x = -bbRTB.x;
            bbRTB.x = -bbLBFx;
        }
        if(flippedY) {
            float bbLBFy = bbLBF.y;
            bbLBF.y = -bbRTB.y;
            bbRTB.y = -bbLBFy;
        }
        
        // Get the remaining vertices (in this mode the BB is perpendicular
        // with all axis, so we can use the coordinates which have already
        // been calculated).
        bbRBF = Point3f(bbRTB.x, bbLBF.y, bbLBF.z);
        bbRTF = Point3f(bbRTB.x, bbRTB.y, bbLBF.z);
        bbLTF = Point3f(bbLBF.x, bbRTB.y, bbLBF.z);
        bbLBB = Point3f(bbLBF.x, bbLBF.y, bbRTB.z);
        bbRBB = Point3f(bbRTB.x, bbLBF.y, bbRTB.z);
        bbLTB = Point3f(bbLBF.x, bbRTB.y, bbRTB.z);
    }
    
    // MODE #3
    //--------------------------------------------------------------------------
    // Only bbLBF and bbRTF are calculated in this mode. The rest of the BB
    // vertices is expressed using these points (it is possible because the
    // resulting BB is parallel with all axis in this mode).
    else if(estimationMode == MODE3) {
            
        // Get depth of near and far face of BB
        float z1, z2;
        if(!calcNearAndFarFaceDepth(roi, f, &z1, &z2)) {
            return false;
        }
    
        // Left-bottom-front vertex of BB
        bbLBF = Point3f((roiLB.x * z1) / fx, (roiLB.y * z1) / fy, z1);
        
        // Right-top-back vertex of BB
        // X and Y coordinates of the back projected point is obtained using
        // the near depth, but its Z coordinate is set to the further
        // depth (this is so to ensure perpendicularity of the bounding box).
        bbRTB = Point3f((roiRT.x * z1) / fx, (roiRT.y * z1) / fy, z2);
        
        // Get the remaining vertices (in this mode the BB is perpendicular
        // with all axis, so we can use the coordinates which have already
        // been calculated).
        bbRBF = Point3f(bbRTB.x, bbLBF.y, bbLBF.z);
        bbRTF = Point3f(bbRTB.x, bbRTB.y, bbLBF.z);
        bbLTF = Point3f(bbLBF.x, bbRTB.y, bbLBF.z);
        bbLBB = Point3f(bbLBF.x, bbLBF.y, bbRTB.z);
        bbRBB = Point3f(bbRTB.x, bbLBF.y, bbRTB.z);
        bbLTB = Point3f(bbLBF.x, bbRTB.y, bbRTB.z);
    }
    
    // If the the depth map was already in pixels, round the resulting coordinates
    // to return the whole numbers
    if(subVariant == SV_1) {
        bbLBF.x = cvRound(bbLBF.x); bbLBF.y = cvRound(bbLBF.y); bbLBF.y = cvRound(bbLBF.y);
        bbRBF.x = cvRound(bbRBF.x); bbRBF.y = cvRound(bbRBF.y); bbRBF.y = cvRound(bbRBF.y);
        bbRTF.x = cvRound(bbRTF.x); bbRTF.y = cvRound(bbRTF.y); bbRTF.y = cvRound(bbRTF.y);
        bbLTF.x = cvRound(bbLTF.x); bbLTF.y = cvRound(bbLTF.y); bbLTF.y = cvRound(bbLTF.y);
        bbLBB.x = cvRound(bbLBB.x); bbLBB.y = cvRound(bbLBB.y); bbLBB.y = cvRound(bbLBB.y);
        bbRBB.x = cvRound(bbRBB.x); bbRBB.y = cvRound(bbRBB.y); bbRBB.y = cvRound(bbRBB.y);
        bbRTB.x = cvRound(bbRTB.x); bbRTB.y = cvRound(bbRTB.y); bbRTB.y = cvRound(bbRTB.y);
        bbLTB.x = cvRound(bbLTB.x); bbLTB.y = cvRound(bbLTB.y); bbLTB.y = cvRound(bbLTB.y);
    }
    // If we obtained the depth map from point clound from Kinect (whose coordinates
    // are in meter), convert it back to meters.
    else if(subVariant == SV_2) {
        /*
        bbLBF *= 1.0 / kinectIRPixelsPerMeter;
        bbRBF *= 1.0 / kinectIRPixelsPerMeter;
        bbRTF *= 1.0 / kinectIRPixelsPerMeter;
        bbLTF *= 1.0 / kinectIRPixelsPerMeter;
        bbLBB *= 1.0 / kinectIRPixelsPerMeter;
        bbRBB *= 1.0 / kinectIRPixelsPerMeter;
        bbRTB *= 1.0 / kinectIRPixelsPerMeter;
        bbLTB *= 1.0 / kinectIRPixelsPerMeter;
        */
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
    
    // Log request timestamp
    //--------------------------------------------------------------------------
    ROS_INFO("Request timestamp: %d.%d", req.header.stamp.sec, req.header.stamp.nsec);

    return true;
}


/*==============================================================================
 * Adds messages to cache (for synchronized subscription variant #1).
 *
 * @param depth  Message with depth image.
 * @param camInfo  Message with camera information.
 */
void sv1_addToCache(const sensor_msgs::ImageConstPtr &depth,
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
 * Adds messages to cache (for synchronized subscription variant #2).
 *
 * @param pointCloud  Message with point cloud.
 * @param camInfo  Message with camera information.
 */
void sv2_addToCache(const sensor_msgs::PointCloud2ConstPtr &pointCloud,
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


/*==============================================================================
 * Main function.
 */
int main(int argc, char **argv)
{
    // ROS initialization (the last argument is the name of the node)
    ros::init(argc, argv, "bb_estimator_server");
    
    // NodeHandle is the main access point to communications with the ROS system
    ros::NodeHandle n;
    
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
	sv1_sync.registerCallback(boost::bind(&sv1_addToCache, _1, _2));
	
	// Subscription variant #2
    message_filters::Subscriber<PointCloud2> sv2_pointCloud_sub(n, sv2_pointCloudTopic, 1);
    message_filters::Subscriber<CameraInfo> sv2_camInfo_sub(n, sv2_camInfoTopic, 1);
    
    typedef sync_policies::ApproximateTime<PointCloud2, CameraInfo> sv2_MySyncPolicy;
	Synchronizer<sv2_MySyncPolicy> sv2_sync(sv2_MySyncPolicy(QUEUE_SIZE),
	    sv2_pointCloud_sub, sv2_camInfo_sub);
	sv2_sync.registerCallback(boost::bind(&sv2_addToCache, _1, _2));
    
    // Create and advertise this service over ROS
    //--------------------------------------------------------------------------
    ros::ServiceServer service = n.advertiseService("bb_estimate", estimateBB);   
    ROS_INFO("Ready.");
    
    // Enters a loop, calling message callbacks
    ros::spin();

    return 0;
}

