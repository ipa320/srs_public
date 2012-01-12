/**
 * $Id: sceneModel.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: dd.mm.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *
 */

#include "plane_det/sceneModel.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "plane_det/filtering.h"
#include "plane_det/parameterSpace.h"

using namespace pcl;
using namespace cv;

namespace but_scenemodel
{

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SceneModel::SceneModel(Mat &depth, const CameraInfoConstPtr& cam_info, Normals &normals) : 	scene_cloud(new PointCloud<PointXYZRGB>),
											current_hough_cloud(new PointCloud<PointXYZI>)
{
	int maxi = normals.m_points.rows;
	int maxj = normals.m_points.cols;
	Vec3f point;
	Vec4f plane;
	Regions reg(&normals);
	reg.watershedRegions(depth, cam_info, WatershedType::DepthDiff, 1, 2, 20);
	double min, max;
	minMaxLoc(reg.m_regionMatrix, &min, &max);
	double indexFactor = 1.0;

	ParameterSpace space(-M_PI, M_PI, -20.0, 20.0, 512, 400);
	std::cout << "Parameter space size: " << space.m_size / 1000000.0 << " MB" << std::endl;
	std::cout << "Parameter shift step: " << space.m_shiftStep << std::endl;
	std::cout << "Parameter angle step: " << space.m_angleStep << std::endl;

	ParameterSpace gauss(-5 * space.m_angleStep, 5 * space.m_angleStep, -5 * space.m_shiftStep, 5 * space.m_shiftStep, 11, 11);
	std::cout << "Gauss space size: " << gauss.m_size / 1000000.0 << " MB" << std::endl;
	std::cout << "Gauss shift step: " << gauss.m_shiftStep << std::endl;
	std::cout << "Gauss angle step: " << gauss.m_angleStep << std::endl;

	gauss.generateGaussIn(0.04, 0.15);

	if (max !=0)
		indexFactor /= (double)max;
	//PointCloud<PointXYZINormal>::Ptr aux(new PointCloud<PointXYZINormal>);

//	for (int i = 0; i < depth.rows; ++i)
//			for (int j = 0; j < depth.cols; ++j)
//			{
//				if (depth.at<unsigned short>(i, j) > 3000)
//					depth.at<unsigned short>(i, j) = 0;
//			}

	for (int i = 0; i < maxi; ++i)
		for (int j = 0; j < maxj; ++j)
		{
			point = normals.m_points.at<Vec3f>(i, j);
			plane = normals.m_planes.at<Vec4f>(i, j);
			//pcl::Ve
			//if (point[2] < 3.0)
			{
			// signed angle atan2(b.y,b.x) - atan2(a.y,a.x)
			// angle on XZ plane with X
			float a1, a2;
			ParameterSpace::toAngles(plane[0], plane[1], plane[2], a1, a2);


			PointXYZRGB rgbpoint(255, 255, 255);
			if (reg.m_regionMatrix.at<int>(i, j) > 0)
				rgbpoint.rgb = indexFactor *reg.m_regionMatrix.at<int>(i, j);
			rgbpoint.x = a1;
			rgbpoint.y = a2;
			rgbpoint.z = plane[3];

			rgbpoint.x = point[0];
			rgbpoint.y = point[1];
			rgbpoint.z = point[2];
			scene_cloud->push_back(rgbpoint);

			int i, j, k;
			space.getIndex(a1, a2, plane[3], i, j, k);
			space.addVolume(gauss, i, j, k);
			}
		}

		max_plane = space.findMaxima(planes);


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Control visualisaation - uncoment to see HT space
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//		min = 99999999999.0;
//		max = -99999999999.0;
//		for (int shift = 0; shift < space.m_shiftSize; shift += 1)
//		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
//		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
//		{
//			float value = space(angle1, angle2, shift);
//			if (value < min) min = value;
//			if (value > max) max = value;
//
////			if (space(angle1, angle2, shift) > 0.0)
////			{
////			PointXYZI pt;
////			pt.x = (float)angle1;
////			pt.y = (float)angle2;
////			pt.z = (float)shift;
////			pt.intensity = space(angle1, angle2, shift);
////			current_hough_cloud->push_back(pt);
////			}
//		}
//
//		for (int shift = 0; shift < space.m_shiftSize; shift += 1)
//		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
//		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
//		{
//			space(angle1, angle2, shift) = 255.0*((space(angle1, angle2, shift) - min) / (max - min));
//		}
//
//		cv::Mat image = cv::Mat::zeros(cvSize(space.m_angleSize, space.m_angleSize), CV_32FC1);
//		int shiftview = space.m_shiftSize/2;
//		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
//		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
//		{
//			image.at<float>(angle1, angle2) = space(angle1, angle2, shiftview);
//		}
//
//
//
//		//create a new window & display the image
//		cvNamedWindow("Smile", 1);
//		cvShowImage("Smile", &IplImage(image));
//		std::cout << "Viewing shift = " << shiftview << std::endl;
//		//wait for key to close the window
//		int key = 0;
//		while(1)
//		{
//		    key = cvWaitKey();
//		    key &= 0x0000ffff;
//		    std::cout << key << std::endl;
//		    if(key==27 || key == 0xffff) break;
//
//		    switch(key)
//		    {
//		        case 'a':
//		        	if (shiftview < space.m_shiftSize-1)
//		        	{
//		        		++shiftview;
//		        		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
//		        		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
//		        		{
//		        			image.at<float>(angle1, angle2) = space(angle1, angle2, shiftview);
//		        		}
//		        	}
//		        	cvShowImage("Smile", &IplImage(image));
//		        	std::cout << "Viewing shift = " << shiftview << "/" << space.m_shiftSize-1 << std::endl;
//		            break;
//		        case 'z':
//		        	if (shiftview > 0)
//		        	{
//		        		--shiftview;
//		        		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
//		        		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
//		        		{
//		        			image.at<float>(angle1, angle2) = space(angle1, angle2, shiftview);
//		        		}
//		        	}
//		        	cvShowImage("Smile", &IplImage(image));
//		        	std::cout << "Viewing shift = " << shiftview << "/" << space.m_shiftSize-1 << std::endl;
//		            break;
//		    }
//		}
//
//
//		cvDestroyWindow( "Smile" );

;

}
}
