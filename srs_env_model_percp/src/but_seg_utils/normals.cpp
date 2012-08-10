/******************************************************************************
 * \file
 *
 * $Id: normals.cpp 694 2012-04-20 10:24:24Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 0.8)
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
 *	 Contains necessary classes for normal estimation from point clouds and height maps
 */

// OpenCV
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>

// ROS
#include <ros/ros.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <Eigen/StdVector>

#include <srs_env_model_percp/but_seg_utils/normals.h>

// std
#include <stdlib.h>
#include <stdio.h>
#include <time.h>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>

using namespace sensor_msgs;
using namespace std;
using namespace cv;


namespace srs_env_model_percp
{

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor - computes real point positions (in scene coordinates) and normals and initiates all variables
// @param points Input CV_16UC depth matrix (raw input from kinect)
// @param cam_info Camera info message (ROS)
// @param normalType Type of normal computation method (NormalType enum)
// @see NormalType()
// @param neighborhood Neighborhood from which normals are computed
// @param threshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
// @param outlierThreshold Outlier threshold for least trimmed squares regression (max error between point depth and proposed plane)
// @param iter Maximum RANSAC iterations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Normals::Normals(cv::Mat &points, const CameraInfoConstPtr& cam_info, int normalType, int neighborhood, float threshold, float outlierThreshold, int iter):
													m_points(points.size(), CV_32FC3),
													m_planes(points.size(), CV_32FC4)
{

	m_cam_info = (CameraInfoConstPtr)cam_info;

	float aux;
	Vec3f nullvector(0.0, 0.0, 0.0);
	Vec4f nullvector4(0.0, 0.0, 0.0, 0.0);

	if (normalType == NormalType::PCL)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>());
		// ... fill point cloud...

		cloud->width = points.cols;
		cloud->height = points.rows;
		cloud->points.resize (cloud->width * cloud->height);

		for (int i = 0; i < points.rows; ++i)
		for (int j = 0; j < points.cols; ++j)
		{



			if ((aux = points.at<unsigned short>(i, j)) != 0)
			{
				Vec3f realPoint;
				realPoint[2] = aux/1000.0;
				realPoint[0] = ( (j - cam_info->K[2]) * realPoint[2] / cam_info->K[0] );
				realPoint[1] = ( (i - cam_info->K[5]) * realPoint[2] / cam_info->K[4] );
				cloud->operator()(j, i).x = realPoint[0];
				cloud->operator()(j, i).y = realPoint[1];
				cloud->operator()(j, i).z = realPoint[2];

				m_points.at<Vec3f>(i, j) = realPoint;
			}
			else
			{
				m_points.at<Vec3f>(i, j) = nullvector;
				cloud->operator()(j, i).x = 0.0;
				cloud->operator()(j, i).y = 0.0;
				cloud->operator()(j, i).z = 0.0;
			}
		}

		// Estimate normals
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

		pcl::PointCloud<pcl::Normal> normals;

		ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
		ne.setDepthDependentSmoothing(true);
		ne.setMaxDepthChangeFactor(threshold);
		//ne.setRectSize(10, 10);
		ne.setNormalSmoothingSize((float)(neighborhood*2+1));
		ne.setInputCloud(cloud);
		ne.compute(normals);

		for (int i = 0; i < points.rows; ++i)
		for (int j = 0; j < points.cols; ++j)
		{
			Vec3f realPoint = m_points.at<Vec3f>(i, j);
			if (realPoint != nullvector)
			{
				Vec4f normal;
				if (normals(j, i).normal_x == normals(j, i).normal_x)
				{
					normal[0] = normals(j, i).normal_x;
					normal[1] = normals(j, i).normal_y;
					normal[2] = normals(j, i).normal_z;
					if (normal[2] < 0)
					{
						normal[0] *= -1.0;
						normal[1] *= -1.0;
						normal[2] *= -1.0;
					}
					normal[3] = -(normal[0]*realPoint[0]+normal[1]*realPoint[1]+normal[2]*realPoint[2]);
					m_planes.at<Vec4f>(i, j) = normal;
				}
				else
					m_planes.at<Vec4f>(i, j) = nullvector4;
			}
			else
			{
				m_planes.at<Vec4f>(i, j) = nullvector4;
			}
		}
	}
	else
	{
		for (int i = 0; i < points.rows; ++i)
		for (int j = 0; j < points.cols; ++j)
		{
			if ((aux = points.at<unsigned short>(i, j)) != 0)
			{
				Vec3f realPoint;
				realPoint[2] = aux/1000.0;
				realPoint[0] = ( (j - cam_info->K[2]) * realPoint[2] / cam_info->K[0] );
				realPoint[1] = ( (i - cam_info->K[5]) * realPoint[2] / cam_info->K[4] );
				m_points.at<Vec3f>(i, j) = realPoint;
			}
			else
			{
				m_points.at<Vec3f>(i, j) = nullvector;
			}
		}

		if (normalType & NormalType::DIRECT)
			{
				for (int i = 0; i < points.rows; ++i)
					for (int j = 0; j < points.cols; ++j)
					{
						Vec3f realPoint = m_points.at<Vec3f>(i, j);
						if (realPoint != nullvector)
						{
							Vec4f normal = getNormal(i, j, neighborhood, threshold);
							m_planes.at<Vec4f>(i, j) = normal;
						}
						else
						{
							m_planes.at<Vec4f>(i, j) = nullvector4;
						}
					}
			}
		else if (normalType & NormalType::LSQ)
			{
				for (int i = 0; i < points.rows; ++i)
					for (int j = 0; j < points.cols; ++j)
					{
						Vec3f realPoint = m_points.at<Vec3f>(i, j);
						if (realPoint != nullvector)
						{
							Vec4f normal = getNormalLSQ(i, j, neighborhood, threshold);
							m_planes.at<Vec4f>(i, j) = normal;
						}
						else
						{
							m_planes.at<Vec4f>(i, j) = nullvector4;
						}
					}
			}
		else if (normalType & NormalType::LSQAROUND)
			{
				for (int i = 0; i < points.rows; ++i)
					for (int j = 0; j < points.cols; ++j)
					{
						Vec3f realPoint = m_points.at<Vec3f>(i, j);
						if (realPoint != nullvector)
						{
							Vec4f normal= getNormalLSQAround(i, j, neighborhood, threshold);
							m_planes.at<Vec4f>(i, j) = normal;
						}
						else
							m_planes.at<Vec4f>(i, j) = nullvector4;
					}
			}
		else if (normalType & NormalType::LTS)
			{
				for (int i = 0; i < points.rows; ++i)
					for (int j = 0; j < points.cols; ++j)
					{
						Vec3f realPoint = m_points.at<Vec3f>(i, j);
						if (realPoint != nullvector)
						{
							Vec4f normal = getNormalLTS(i, j, neighborhood, threshold, outlierThreshold, iter);
							m_planes.at<Vec4f>(i, j) = normal;
						}
						else
						{
							m_planes.at<Vec4f>(i, j) = nullvector4;
						}
					}
			}
		else if (normalType & NormalType::LTSAROUND)
			{
				for (int i = 0; i < points.rows; ++i)
					for (int j = 0; j < points.cols; ++j)
					{
						Vec3f realPoint = m_points.at<Vec3f>(i, j);
						if (realPoint != nullvector)
						{
							Vec4f normal = getNormalLTSAround(i, j, neighborhood, threshold, outlierThreshold, iter);
							m_planes.at<Vec4f>(i, j) = normal;
						}
						else
						{
							m_planes.at<Vec4f>(i, j) = nullvector4;
						}
					}
			}
	}

	//int n_bins = 16;
	//if (n_bins)
	//{
	//	m_quantbins = n_bins;
	// initQuantVectors(m_quantbins, m_quantVectors);
	//}

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor - computes real point positions (in scene coordinates) and normals and initiates all variables
// @param pointcloud Input point cloud
// @param threshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
// @param neighborhood Neighborhood from which normals are computed
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Normals::Normals(pcl::PointCloud<pcl::PointXYZ> &pointcloud, float threshold, int neighborhood):
													m_points(cvSize(pointcloud.width, pointcloud.height), CV_32FC3),
													m_planes(cvSize(pointcloud.width, pointcloud.height), CV_32FC4)
{
		// ... fill point cloud...

	Vec3f nullvector(0.0, 0.0, 0.0);
	Vec4f nullvector4(0.0, 0.0, 0.0, 0.0);

	//pcl::PointCloud<pcl::PointXYZ> cloud2;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new cloud2);

	for(int y = 0; y < (int)pointcloud.height; ++y)
	for(int x = 0; x < (int)pointcloud.width; ++x)
	{
//		if (pointcloud.at(x, y).z > 0.001)
//		{
				Vec3f realPoint;
				realPoint[0] = pointcloud.at(x, y).x;
				realPoint[1] = pointcloud.at(x, y).y;
				realPoint[2] = pointcloud.at(x, y).z;

				m_points.at<Vec3f>(y, x) = realPoint;
//			}
//			else
//			{
//				m_points.at<Vec3f>(y, x) = nullvector;
//				pointcloud.at(x, y).x = 0.0;
//				pointcloud.at(x, y).y = 0.0;
//				pointcloud.at(x, y).z = 0.0;
//			}
	}
	///////////////////////////////////////////////////////////////
//	// estimate normals using LSQ
//	for (int i = 0; i < m_points.rows; ++i)
//		for (int j = 0; j < m_points.cols; ++j)
//		{
//			Vec3f realPoint = m_points.at<Vec3f>(i, j);
//			if (realPoint != nullvector)
//			{
//				Vec4f normal= getNormalLSQAround(i, j, 4, 0.2);
//				m_planes.at<Vec4f>(i, j) = normal;
//			}
//				else
//				m_planes.at<Vec4f>(i, j) = nullvector4;
//		}
///////////////////////////////////////////////////////////////
		// Estimate normals
		pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		pcl::PointCloud<pcl::Normal> normals;

		ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
		ne.setDepthDependentSmoothing(true);
		ne.setMaxDepthChangeFactor(threshold);
		//ne.setRectSize(10, 10);
		ne.setNormalSmoothingSize((float)(neighborhood*2+1));
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pointcloud.makeShared();
		ne.setInputCloud(cloud);
		ne.compute(normals);

		for (int i = 0; i < m_points.rows; ++i)
		for (int j = 0; j < m_points.cols; ++j)
		{
			Vec3f realPoint = m_points.at<Vec3f>(i, j);

			Vec4f normal;
			if (normals(j, i).normal_x == normals(j, i).normal_x &&
				normals(j, i).normal_y == normals(j, i).normal_y &&
				normals(j, i).normal_z == normals(j, i).normal_z)
			{
				normal[0] = normals(j, i).normal_x;
				normal[1] = normals(j, i).normal_y;
				normal[2] = normals(j, i).normal_z;

//					if (normal[2] < 0)
//					{
//						normal[0] *= -1.0;
//						normal[1] *= -1.0;
//						normal[2] *= -1.0;
//					}
				normal[3] = -(normal[0]*realPoint[0]+normal[1]*realPoint[1]+normal[2]*realPoint[2]);
				m_planes.at<Vec4f>(i, j) = normal;
			}
			else
				m_planes.at<Vec4f>(i, j) = nullvector4;
		}
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function computes normal for point (i, j) using direct computation (mean of surrounding triangles)
// @param i row index
// @param j column index
// @param step Neighborhood to compute with
// @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Vec4f Normals::getNormal(int i, int j, int step, float depthThreshold)
{
	int size = (step*2)*4;

	cv::Vec<float, 4> normalVec;
		normalVec[0] = 0;
		normalVec[1] = 0;
		normalVec[2] = 0;
		normalVec[3] = 0;

	if (i < step || j < step || i >= m_points.rows-step || j >= m_points.cols-step)
			return normalVec;

	cv::Vec3f aux;
	cv::Vec3f center;

	Vec3f realPoint = m_points.at<Vec3f>(i, j);
	center[0] = realPoint[0];
	center[1] = realPoint[1];
	center[2] = realPoint[2];

	std::vector<Vec3f> around;


	// neighbourhood
	int x = -step;
	int y = -step;
	int plusX = 1;
	int plusY = 0;

	Vec3f centroid;
	centroid[0] = 0;
	centroid[1] = 0;
	centroid[2] = 0;

	for (int index = 0; index < size; ++index)
	{
		realPoint = m_points.at<Vec3f>(i+x, j+y);
		if (abs(center[2]-realPoint[2]) < depthThreshold)
		{
			centroid += realPoint;
			around.push_back(realPoint - center);
		}

		// move next pixel around
		nextStep(step, x, y, plusX, plusY);

	}
	size = around.size();
	centroid[0] /= size;
	centroid[1] /= size;
	centroid[2] /= size;

	for (int index = 0; index < size; ++index)
	{
		int second = ((index+1) % size);
		aux = around[index].cross(around[second]);
		normalVec = normalVec + Vec4f(aux[0], aux[1], aux[2], 0.0);
	}

	normalVec[3] = -(centroid[0]*normalVec[0] + centroid[1]*normalVec[1] + centroid[2]*normalVec[2]);

	float norm = sqrt(normalVec[0]*normalVec[0] + normalVec[1]*normalVec[1] + normalVec[2]*normalVec[2]);
	if (norm != 0)
	{
		if (normalVec[2] < 0)
		{
			normalVec[0] = normalVec[0] / -norm;
			normalVec[1] = normalVec[1] / -norm;
			normalVec[2] = normalVec[2] / -norm;
			normalVec[3] = normalVec[3] / -norm;
		}
		else
		{
			normalVec[0] = normalVec[0] / norm;
			normalVec[1] = normalVec[1] / norm;
			normalVec[2] = normalVec[2] / norm;
			normalVec[3] = normalVec[3] / norm;
		}
	}

	return normalVec;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function computes normal for point (i, j) using least squares regression
// @param i row index
// @param j column index
// @param step Neighborhood to compute with
// @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Vec4f Normals::getNormalLSQ(int i, int j, int step, float depthThreshold)
{
	cv::Vec<float, 4> normalVec;
			normalVec[0] = 0;
			normalVec[1] = 0;
			normalVec[2] = 0;
			normalVec[3] = 0;

		if (i < step || j < step || i >= m_points.rows-step || j >= m_points.cols-step)
		return normalVec;

	int iMax = i+step;
	int jMax = j+step;

	std::vector<Vec3f> points;

	Vec3f center = m_points.at<Vec3f>(i, j);
	Vec3f point;
	for (int x = i-step; x <= iMax; ++x)
		for (int y = j-step; y <= jMax; ++y)
		{
			point = m_points.at<Vec3f>(x, y);
			if (abs(center[2]-point[2]) < depthThreshold)
			{
				points.push_back(point);
			}
		}

	if (points.size() < 3)
		return normalVec;

	Plane<float> plane = LeastSquaresPlane(points);

	return Vec4f(plane.a, plane.b, plane.c, plane.d);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function computes normal for point (i, j) using least squares regression with using only outer neighborhood ring
// @param i row index
// @param j column index
// @param step Neighborhood to compute with
// @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Vec4f Normals::getNormalLSQAround(int i, int j, int step, float depthThreshold)
{
	cv::Vec<float, 4> normalVec;
			normalVec[0] = 0;
			normalVec[1] = 0;
			normalVec[2] = 0;
			normalVec[3] = 0;

	if (i < step || j < step || i >= m_points.rows-step || j >= m_points.cols-step)
	return normalVec;

	std::vector<Vec3f> points;

	Vec3f center = m_points.at<Vec3f>(i, j);
	Vec3f point;
	int x = -step;
	int y = -step;
	int plusX = 1;
	int plusY = 0;
	int size = (step*2)*4;

	for (int index = 0; index < size; ++index)
	{
		point = m_points.at<Vec3f>(i+x, j+y);
		if (abs(center[2]-point[2]) < depthThreshold)
		{
			points.push_back(point);
		}

		// move next pixel around
		nextStep(step, x, y, plusX, plusY);

	}

	if (points.size() < 3)
		return normalVec;

	Plane<float> plane = LeastSquaresPlane(points);

	return Vec4f(plane.a, plane.b, plane.c, plane.d);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function computes normal for point (i, j) using least trimmed squares regression
// @param i row index
// @param j column index
// @param step Neighborhood to compute with
// @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
// @param outlierThreshold Threshold for marking outliers in RANSAC (maximum difference from proposed plane)
// @param maxIter Maximum RANSAC iterations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Vec4f Normals::getNormalLTS(int i, int j, int step, float depthThreshold, float outlierThreshold, int maxIter)
{
	cv::Vec<float, 4> normalVec;
			normalVec[0] = 0;
			normalVec[1] = 0;
			normalVec[2] = 0;
			normalVec[3] = 0;

	if (i < step || j < step || i >= m_points.rows-step || j >= m_points.cols-step)
			return normalVec;

	int iMax = i+step;
	int jMax = j+step;

	Plane<float> best(0.0, 0.0, 0.0, 0.0);
	float bestScore = 9999999.0;
	Plane<float> candidate(0.0, 0.0, 0.0, 0.0);
	Vec3f center = m_points.at<Vec3f>(i, j);
	Vec3f point;
	for (int cnt = 0; cnt < maxIter; ++cnt)
	{
		std::vector<Vec3f> points;
		for (int x = i-step; x <= iMax; ++x)
			for (int y = j-step; y <= jMax; ++y)
			{
				point = m_points.at<Vec3f>(x, y);
				if (rand() > RAND_MAX/2 && abs(center[2]-point[2]) < depthThreshold)
				{
					points.push_back(point);
				}
			}

		candidate = LeastSquaresPlane(points);

		float score = 0.0;
		float count = 0.0;
		float aux;

		for (int x = i-step; x < iMax; ++x)
			for (int y = j-step; y < jMax; ++y)
			{
				aux = candidate.distance(m_points.at<Vec3f>(x, y));
				if (aux < outlierThreshold)
				{
					score += aux;
					count += 1.0;
				}
			}

		score /= count;

		if (bestScore > score)
		{
			bestScore = score;
			best = candidate;
		}
	}

	return Vec4f(best.a, best.b, best.c, best.d);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function computes normal for point (i, j) using least trimmed squares regression with using only outer neighborhood ring
// @param i row index
// @param j column index
// @param step Neighborhood to compute with
// @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
// @param outlierThreshold Threshold for marking outliers in RANSAC (maximum difference from proposed plane)
// @param maxIter Maximum RANSAC iterations
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
cv::Vec4f Normals::getNormalLTSAround(int i, int j, int step, float depthThreshold, float outlierThreshold, int maxIter)
{
	cv::Vec<float, 4> normalVec;
			normalVec[0] = 0;
			normalVec[1] = 0;
			normalVec[2] = 0;
			normalVec[3] = 0;

	if (i < step || j < step || i >= m_points.rows-step || j >= m_points.cols-step)
			return normalVec;

	Plane<float> best(0.0, 0.0, 0.0, 0.0);
	float bestScore = 9999999.0;
	Plane<float> candidate(0.0, 0.0, 0.0, 0.0);
	Vec3f center = m_points.at<Vec3f>(i, j);
	Vec3f point;
	int size = (step*2)*4;
	for (int cnt = 0; cnt < maxIter; ++cnt)
	{
		std::vector<Vec3f> points;

		int x = -step;
		int y = -step;
		int plusX = 1;
		int plusY = 0;

		for (int index = 0; index < size; ++index)
		{
			point = m_points.at<Vec3f>(i+x, j+y);
			if (rand() > RAND_MAX/2 && abs(center[2]-point[2]) < depthThreshold)
				points.push_back(point);

			// move next pixel around
			nextStep(step, x, y, plusX, plusY);

		}

		candidate = LeastSquaresPlane(points);

		float score = 0.0;
		float count = 0.0;
		float aux;

		x = -step;
		y = -step;
		plusX = 1;
		plusY = 0;

		for (int index = 0; index < size; ++index)
		{
			aux = candidate.distance(m_points.at<Vec3f>(i+x, j+y));
			if (aux < outlierThreshold)
			{
				score += aux;
				count += 1.0;
			}
			// move next pixel around
			nextStep(step, x, y, plusX, plusY);
		}

		score /= count;

		if (bestScore > score)
		{
			bestScore = score;
			best = candidate;
		}
	}

	return Vec4f(best.a, best.b, best.c, best.d);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Function computes plane using least squares regression from given point set
// @param points Vector of Vec3f points
// @return Plane object
// @see Plane()
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
Plane<float> Normals::LeastSquaresPlane(std::vector<cv::Vec3f> &points)
{
	if (points.size() == 0) return Plane<float>(0,0,0,0);
	cv::Vec3f centroid(0.0, 0.0, 0.0);
	Eigen::Matrix3f tensor = Eigen::Matrix3f::Zero();

	//////////////////////////////////////////
	// first, compute a centroid (mean value)
	cv::Vec3f current = *points.begin();
	for (unsigned int i = 0; i < points.size(); ++i)
		centroid += points[i];
	centroid[0] /= points.size();
	centroid[1] /= points.size();
	centroid[2] /= points.size();


	//////////////////////////////////////////
	// second step, fill a tenzor
	double dx, dy, dz;
	for (unsigned int i = 0; i < points.size(); ++i)
	{
		dx = centroid[0] - points[i][0];
		dy = centroid[1] - points[i][1];
		dz = centroid[2] - points[i][2];

		tensor(0, 0) += dx*dx;
		tensor(0, 1) += dx*dy;
		tensor(0, 2) += dx*dz;
		tensor(1, 0) += dy*dx;
		tensor(1, 1) += dy*dy;
		tensor(1, 2) += dy*dz;
		tensor(2, 0) += dz*dx;
		tensor(2, 1) += dz*dy;
		tensor(2, 2) += dz*dz;
	}

	//////////////////////////////////////////
	// compute eigenvectors and eigenvalues
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(tensor);

	Eigen::Matrix3f eigenVectors;
	Eigen::Vector3f eigenValues;

	eigenVectors = solver.eigenvectors();
	eigenValues = solver.eigenvalues();
	Plane<float> plane(eigenVectors(0, 0), eigenVectors(1, 0), eigenVectors(2, 0), -(centroid[0]*eigenVectors(0,0) + centroid[1]*eigenVectors(1,0) + centroid[2]*eigenVectors(2,0)));

	//////////////////////////////////////////
	// Normalize
	if (plane.norm != 0)
	{
		plane.a /= plane.norm;
		plane.b /= plane.norm;
		plane.c /= plane.norm;
		plane.d /= plane.norm;
		plane.norm = 1;
	}
	if (plane.c < 0)
	{
		plane.a *= -1.0;
		plane.b *= -1.0;
		plane.c *= -1.0;
		plane.d *= -1.0;
	}


	return plane;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function for "Around" functions - sets next point on outer ring
// @param step Maximum distance from center (neighborhood)
// @param x Current x offset
// @param y Current y offset
// @param plusX Next x shift
// @param plusY Next y shift
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Normals::nextStep(int step, int &x, int &y, int &plusX, int &plusY)
{
	if (x == -step && y == -step)
	{
		plusX = 1;
		plusY = 0;
	}
	if (x == step && y == -step)
	{
		plusX = 0;
		plusY = 1;
	}
	if (x == step && y == step)
	{
		plusX = -1;
		plusY = 0;
	}
	if (x == -step && y == step)
	{
		plusX = 0;
		plusY = -1;
	}
	x += plusX;
	y += plusY;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function which initializes quantization vectors (not used, TODO)
// @param n_bins Number of quantization bins
// @param vec Vector of computed quantization vectors
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Normals::initQuantVectors(int n_bins, std::vector<cv::Vec3f> &vec)
{
	float twoPI = (2.0 * M_PI);

	float step = twoPI/n_bins;
	float sinPi4 = sin(M_PI_4);
	for (float i = 0; i < twoPI; i+= step)
	{
		vec.push_back(cv::Vec3f( 	cos(i)*sinPi4,
								sin(i)*sinPi4,
								sinPi4
		));
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Helper function which returns bin index for given vector (not used, TODO)
// @param vec Vector of computed quantization vectors
// @param vector Vector whos bin we are computing
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int Normals::getQuantVector(std::vector<cv::Vec3f> &vec, cv::Vec3f &vector)
{
	unsigned int size = vec.size();
	unsigned int minimum = 10;
	float minimalVal = 9999.0;
	for (unsigned int i = 0; i < size; ++i)
	{
		float angle = acos(	vec[i][0] * vector[0] +
							vec[i][1] * vector[1] +
							vec[i][2] * vector[2]);
		if (angle < minimalVal)
		{
			minimalVal = angle;
			minimum = i;
		}
	}
	return minimum;

}

}

