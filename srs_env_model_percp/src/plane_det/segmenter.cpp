/**
 * $Id: segmenter.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: 11.01.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 * A module gets depth map data and segments them
 * For use please start with -h param
 *
 */


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/CvBridge.h>

//PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/filters/voxel_grid.h>

//better opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>


#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <float.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "plane_det/filtering.h"
#include "plane_det/normals.h"


using namespace std;
using namespace cv;
using namespace pcl;
using namespace sensor_msgs;
using namespace message_filters;




ros::Publisher n_pub;

ros::Publisher region_image;
ros::Publisher deviation_image;
ros::Publisher triangle_pub;

#define REGIONS_DEPTH 		but_scenemodel::WatershedType::DepthDiff
#define REGIONS_NORMAL 		but_scenemodel::WatershedType::NormalDiff
#define REGIONS_COMBINED 	but_scenemodel::WatershedType::Combined
#define REGIONS_PREDICTOR 	but_scenemodel::WatershedType::PredictorDiff
#define REGIONS_TILE	 	16
#define DEFAULT_REGIONS REGIONS_COMBINED
#define DEFAULT_MAXDEPTH 3000

int typeRegions = DEFAULT_REGIONS;
unsigned short maxDepth = DEFAULT_MAXDEPTH;

bool initParams( int argc, char** argv )
{
	if (argc == 2 && strcmp(argv[1], "-h")==0)
	{
		std::cerr << "Siple help for segmenter node:" << std::endl;
		std::cerr << "==============================" << std::endl << std::endl;

		std::cerr << "Parameter overview:" << std::endl;
		std::cerr << "-------------------" << std::endl;
		std::cerr << "    -type [TYPE]:" << std::endl;
		std::cerr << "        Parameter type specifies type of segmenting method" << std::endl;
		std::cerr << "        depth - Segmenter uses only depth information" << std::endl;
		std::cerr << "        normal - Segmenter uses only normal information (delegates also std deviation image)" << std::endl;
		std::cerr << "        combined - Segmenter uses combined depth and normal information (delegates also std deviation image)" << std::endl;
		std::cerr << "        predictor - Segmenter uses predictor plane algorithm" << std::endl;
		std::cerr << "        tile - Segmenter uses tiling plane segmentation (delegates also std deviation image)" << std::endl;
		std::cerr << "        Default is combined" << std::endl;
		std::cerr << "    -maxdepth [NUM]:" << std::endl;
		std::cerr << "        Number how deep point should be considered for computation (in milimeters)" << std::endl;
		std::cerr << "        Default is 3000" << std::endl << std::endl;
		std::cerr << "Exaples of startup:" << std::endl;
		std::cerr << "-------------------" << std::endl;
		std::cerr << "    rosrun but_scenemodel segmenter" << std::endl;
		std::cerr << "    rosrun but_scenemodel segmenter -maxdepth 2500" << std::endl;
		std::cerr << "    rosrun but_scenemodel segmenter -type normal" << std::endl;
		std::cerr << "    rosrun but_scenemodel segmenter -type predictor -maxdepth 2500" << std::endl;
		std::cerr << "    rosrun but_scenemodel segmenter -maxdepth 2500 - type tile" << std::endl;
		return false;
	}
	if (argc <= 5 && argc % 2 != 0)
		if (argc > 1)
		{
			int expected = (argc-1)/2;
			for (int i = 1; i < argc; i += 2)
			{
				if (strcmp(argv[i], "-type")==0)
				{
					if (strcmp(argv[i+1], "depth")==0)
					{
						typeRegions = REGIONS_DEPTH;
						--expected;
					}
					else if (strcmp(argv[i+1], "normal")==0)
					{
						typeRegions = REGIONS_NORMAL;
						--expected;
					}
					else if (strcmp(argv[i+1], "combined")==0)
					{
						typeRegions = REGIONS_COMBINED;
						--expected;
					}
					else if (strcmp(argv[i+1], "predictor")==0)
					{
						typeRegions = REGIONS_PREDICTOR;
						--expected;
					}
					else if (strcmp(argv[i+1], "tile")==0)
					{
						typeRegions = REGIONS_TILE;
						--expected;
					}
				}
				if (strcmp(argv[i], "-maxdepth")==0)
				{
					if ((maxDepth = atoi(argv[i+1])) > 0)
						--expected;
				}
			}
			if (expected == 0)
				return true;
			else return false;
		}
		else
			return true;

	else
		return false;
}

void callback( const sensor_msgs::ImageConstPtr& dep, const CameraInfoConstPtr& cam_info)
{
	ros::Time begin = ros::Time::now();
	//  Debug info
	std::cerr << "Recieved frame..." << std::endl;
	std::cerr << "Cam info: fx:" << cam_info->K[0] << " fy:" << cam_info->K[4] << " cx:" << cam_info->K[2] <<" cy:" << cam_info->K[5] << std::endl;
	std::cerr << "Depth image h:" << dep->height << " w:" << dep->width << " e:" << dep->encoding << " " << dep->step << endl;
	std::cerr << "=========================================================" << endl;

	//get image from message
	sensor_msgs::CvBridge bridge;
	cv::Mat depth = bridge.imgMsgToCv( dep );
	double min, max;
	Mat xxx = Mat(depth.size(), CV_32FC1);
	minMaxLoc(depth, &min, &max);
	depth.convertTo(xxx, CV_32F, 1.0/(max-min), -min);


	Mat depth2;

	for (int i = 0; i < depth.rows; ++i)
		for (int j = 0; j < depth.cols; ++j)
		{
			if (depth.at<unsigned short>(i, j) > maxDepth)
				depth.at<unsigned short>(i, j) = 0;
		}
	using namespace but_scenemodel;
	but_scenemodel::Regions reg;


	if (typeRegions == REGIONS_DEPTH)
	{
		reg.watershedRegions(depth, cam_info, WatershedType::DepthDiff, 1, 2, 20);
	}
	if (typeRegions == REGIONS_NORMAL)
	{
		reg.watershedRegions(depth, cam_info, WatershedType::NormalDiff);
		reg.computeStatistics(0.3);
		minMaxLoc(reg.m_stddeviation, &min, &max);
		reg.m_stddeviation.convertTo(depth, CV_16U, 255.0/(max-min), -min);

		deviation_image.publish(bridge.cvToImgMsg(&IplImage(depth)));
	}
	if (typeRegions == REGIONS_COMBINED)
	{
		reg.watershedRegions(depth, cam_info, WatershedType::Combined);
		reg.computeStatistics(0.3);
		minMaxLoc(reg.m_stddeviation, &min, &max);
		reg.m_stddeviation.convertTo(depth, CV_16U, 255.0/(max-min), -min);
		minMaxLoc(reg.m_regionMatrix, &min, &max);

		deviation_image.publish(bridge.cvToImgMsg(&IplImage(depth)));
	}
	if (typeRegions == REGIONS_PREDICTOR)
	{
		reg.watershedRegions(depth, cam_info, WatershedType::PredictorDiff);
	}
	if (typeRegions == REGIONS_TILE)
	{
		reg.independentTileRegions(depth, cam_info);
		reg.computeStatistics(0.3);
		minMaxLoc(reg.m_stddeviation, &min, &max);
		reg.m_stddeviation.convertTo(depth, CV_16U, 255.0/(max-min), -min);
		deviation_image.publish(bridge.cvToImgMsg(&IplImage(depth)));
	}



	minMaxLoc(reg.m_regionMatrix, &min, &max);
	reg.m_regionMatrix.convertTo(depth, CV_16U, 255.0/(max-min), -min);
	region_image.publish(bridge.cvToImgMsg(&IplImage(depth)));

	ros::Time end = ros::Time::now();
	std::cout << "Computation time: " << (end - begin).toNSec()/1000000.0 << "ms" << std::endl;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "depth_segmenter");
	if (!initParams( argc, argv ))
	{
		std::cerr << "Bad command line parameters, bailing out..." << std::endl;
		return 1;
	}
	else
	{
		std::cerr << "Application is running on following configuration:" << std::endl;
		if (typeRegions == REGIONS_DEPTH)
			std::cerr << "Segmenting algorithm: Depth only" << std::endl;
		if (typeRegions == REGIONS_NORMAL)
			std::cerr << "Segmenting algorithm: Normals only" << std::endl;
		if (typeRegions == REGIONS_COMBINED)
			std::cerr << "Segmenting algorithm: Combined normals and depth" << std::endl;
		if (typeRegions == REGIONS_PREDICTOR)
			std::cerr << "Segmenting algorithm: Plane prediction algorithm" << std::endl;
		if (typeRegions == REGIONS_TILE)
			std::cerr << "Segmenting algorithm: Tiling algorithm" << std::endl;

		std::cerr << "Max depth is set to: " << maxDepth << std::endl;
		if (typeRegions == REGIONS_DEPTH || typeRegions == REGIONS_PREDICTOR)
			std::cerr << "WARNING - deviation picture is not distributed" << std::endl;
	}
	ros::NodeHandle n;

	// subscribe depth info
	message_filters::Subscriber<Image> depth_sub(n, "/cam3d/depth/image_raw", 1);
	message_filters::Subscriber<CameraInfo> info_sub_depth(n, "/cam3d/depth/camera_info", 1);

	// sync images
	typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), depth_sub, info_sub_depth);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	region_image = n.advertise<Image>("/segmented/region_image", 1);
	deviation_image = n.advertise<Image>("/segmented/deviation_image", 1);

	std::cerr << "Image segmenter initialized and listening..." << std::endl;
	ros::spin();

	return 1;
}
