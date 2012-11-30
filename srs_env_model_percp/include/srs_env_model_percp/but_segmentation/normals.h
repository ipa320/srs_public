﻿/******************************************************************************
 * \file
 *
 * $Id: normals.h 693 2012-04-20 09:22:39Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 1.0)
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

#pragma once
#ifndef BUT_SEG_UTILS_NORMALS_H
#define BUT_SEG_UTILS_NORMALS_H

// Opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

// ROS
#include <sensor_msgs/CameraInfo.h>

// Eigen
#include <Eigen/Core>

// PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace but_plane_detector
{
	/**
	 * Type of normal computation enum
	 */
	class NormalType
	{
		public:
			/**
			 * Direct - normals are computed as mean normal vector of adjacent tris
			 */
			static const int DIRECT = 1;
			/**
			 * Least squares regression
			 */
			static const int LSQ = 2;
			/**
			 * Least squares regression with taking only outer ring of neighborhood
			 */
			static const int LSQAROUND = 4;
			/**
			 * Least trimmed squares regression
			 */
			static const int LTS = 8;
			/**
			 * Least trimmed squares regression with taking only outer ring of neighborhood
			 */
			static const int LTSAROUND = 16;

			/**
			 * PCL based integral image normals
			 */
			static const int PCL = 32;
	};

	/**
	 * Class encapsulating plane equation and some plane speciffic computations
	 */
	template <typename Scalar>
	class Plane
	{
		public:
			/**
			 * Constructor - saves equation and computes norm of normal vector
			 * @param A Ax + by + cz + d = 0
			 * @param B ax + By + cz + d = 0
			 * @param C ax + by + Cz + d = 0
			 * @param D ax + by + cz + D = 0
			 */
			Plane(Scalar A, Scalar B, Scalar C, Scalar D)
			{
				a = A;
				b = B;
				c = C;
				d = D;
				norm = sqrt(a*a + b*b + c*c);

				if (norm != 0)
				{
					a /= norm;
					b /= norm;
					c /= norm;
					d /= norm;
				}
			}

			/**
			 * Returns a distance of point and this plane
			 * @param pt Point for distance computation
			 */
			Scalar distance(cv::Vec<Scalar, 3> pt)
			{
				Scalar val = a*pt[0] + b*pt[1] + c*pt[2] + d;
				return (val > 0? val : -val);
			}

			/**
			 * Returns if this plane is similar to the other one with taking into account angle error and shift error
			 * @param plane Plane to be compared with
			 * @param angleErr Error threshold in angle of normals
			 * @param shiftErr Error threshold in shift (D coefficient)
			 */
			bool isSimilar(Plane &plane, Scalar angleErr, Scalar shiftErr)
			{
				Scalar angle = acos(((a * plane.a) + (b * plane.b) + (c * plane.c)));
				Scalar xd1 = plane.d - d;
				Scalar xd2 = d - plane.d;
				if ((angle < angleErr  && (xd1 > 0 ? xd1 : - xd1) < shiftErr) || (angle > M_PI - angleErr && (xd2 > 0 ? xd2 : - xd2) < shiftErr))
					return true;
				else
					return false;
			}

			/**
			 * Returns a quaternion rotation of this  plane normal from vector (1, 0, 0)
			 * @param x X value of quaternion
			 * @param y Y value of quaternion
			 * @param z Z value of quaternion
			 * @param w W value of quaternion
			 */
			void getQuaternionRotation(Scalar &x, Scalar &y, Scalar &z, Scalar &w)
			{
				Eigen::Vector3f current(a, b, c);
				Eigen::Vector3f target(1.0, 0.0, 0.0);

				Eigen::Quaternion<float> q;
				q.setFromTwoVectors(current, target);
//
//				Scalar nx = b*1 - c*0;
//				Scalar ny = c*0 - a*1;
//				Scalar nz = a*0 - b*0;
//
//				Scalar length = sqrt(nx*nx + ny*ny + nz*nz);
//				nx /= length;
//				ny /= length;
//				nz /= length;
//
//
//				Scalar sign = ((nx * 1 + ny * 0 + nz * 0) < 0) ? -1:1;
//				Scalar angle = acos(a*0 + b*0 + c*1) * sign;
//
//				Eigen::Quaternion<Scalar> q;
//				q = Eigen::AngleAxis<Scalar>(-angle, Eigen::Matrix<Scalar, 3, 1>(nx, ny, nz));

				x = q.x();
				y = q.y();
				z = q.z();
				w = q.w();
			}

			void getInverseQuaternionRotation(Scalar &x, Scalar &y, Scalar &z, Scalar &w)
			{
				Eigen::Vector3f current(a, b, c);
				Eigen::Vector3f target(1.0, 0.0, 0.0);

				Eigen::Quaternion<float> q;
				q.setFromTwoVectors(target, current);


//				Scalar nx = b*1 - c*0;
//				Scalar ny = c*0 - a*1;
//				Scalar nz = a*0 - b*0;
//
//				Scalar length = sqrt(nx*nx + ny*ny + nz*nz);
//				nx /= length;
//				ny /= length;
//				nz /= length;
//
//				Scalar sign = ((nx * 1 + ny * 0 + nz * 0) < 0) ? -1:1;
//				Scalar angle = acos(a*0 + b*0 + c*1) * sign;
//
//				Eigen::Quaternion<Scalar> q;
//				q = Eigen::AngleAxis<Scalar>(angle, Eigen::Matrix<Scalar, 3, 1>(nx, ny, nz));

				x = q.x();
				y = q.y();
				z = q.z();
				w = q.w();
			}

			void getQuaternionRotation(Scalar &x, Scalar &y, Scalar &z, Scalar &w, cv::Vec<Scalar, 3> &normal)
			{
				Eigen::Vector3f current(a, b, c);
				Eigen::Vector3f target(normal[0], normal[1], normal[2]);

				Eigen::Quaternion<float> q;
				q.setFromTwoVectors(current, target);

//				Scalar nx = b*normal[2] - c*normal[1];
//				Scalar ny = c*normal[0] - a*normal[2];
//				Scalar nz = a*normal[1] - b*normal[0];
//
//				Scalar length = sqrt(nx*nx + ny*ny + nz*nz);
//				nx /= length;
//				ny /= length;
//				nz /= length;
//
//
//				Scalar sign = ((nx * 1 + ny * 0 + nz * 0) < 0) ? -1:1;
//				Scalar angle = acos(a*0 + b*0 + c*1) * sign;
//
//				Eigen::Quaternion<Scalar> q;
//				q = Eigen::AngleAxis<Scalar>(-angle, Eigen::Matrix<Scalar, 3, 1>(nx, ny, nz));

				x = q.x();
				y = q.y();
				z = q.z();
				w = q.w();
			}

			void getQuaternionRotationInverse(Scalar &x, Scalar &y, Scalar &z, Scalar &w, cv::Vec<Scalar, 3> &normal)
			{
				Eigen::Vector3f current(a, b, c);
				Eigen::Vector3f target(normal[0], normal[1], normal[2]);

				Eigen::Quaternion<float> q;
				q.setFromTwoVectors(target, current);

//				Scalar nx = b*normal[2] - c*normal[1];
//				Scalar ny = c*normal[0] - a*normal[2];
//				Scalar nz = a*normal[1] - b*normal[0];
//
//				Scalar length = sqrt(nx*nx + ny*ny + nz*nz);
//				nx /= length;
//				ny /= length;
//				nz /= length;
//
//				Scalar sign = ((nx * 1 + ny * 0 + nz * 0) < 0) ? -1:1;
//				Scalar angle = acos(a*0 + b*0 + c*1) * sign;
//
//				Eigen::Quaternion<Scalar> q;
//				q = Eigen::AngleAxis<Scalar>(angle, Eigen::Matrix<Scalar, 3, 1>(nx, ny, nz));

				x = q.x();
				y = q.y();
				z = q.z();
				w = q.w();
			}

			/**
			 * Ax + by + cz + d = 0
			 */
			Scalar a;

			/**
			 * ax + By + cz + d = 0
			 */
			Scalar b;

			/**
			 * ax + by + Cz + d = 0
			 */
			Scalar c;

			/**
			 * Plane normal length
			 */
			Scalar norm;

			/**
			 * ax + by + cz + D = 0
			 */
			Scalar d;
	};

	/**
	 * Class encapsulating normal computation from depth image
	 */
	class Normals
	{
		public:
			/**
			 * Constructor - computes real point positions (in scene coordinates) and normals and initiates all variables
			 * @param points Input CV_16UC depth matrix (raw input from kinect)
			 * @param cam_info Camera info message (ROS)
			 * @param normalType Type of normal computation method (NormalType enum)
			 * @see NormalType()
			 * @param neighborhood Neighborhood from which normals are computed
			 * @param threshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
			 * @param outlierThreshold Outlier threshold for least trimmed squares regression (max error between point depth and proposed plane)
			 * @param iter Maximum RANSAC iterations
			 */
			Normals(cv::Mat &points, const sensor_msgs::CameraInfoConstPtr& cam_info, int normalType = NormalType::PCL, int neighborhood = 4,
																		 float threshold = 0.2, float outlierThreshold = 0.02, int iter = 3);

			/**
			 * Constructor - computes real point positions (in scene coordinates) and normals and initiates all variables
			 * @param pointcloud Point cloud
			 * @param threshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
			 * @param neighborhood Neighborhood from which normals are computed
			 */

			Normals(pcl::PointCloud<pcl::PointXYZ> &pointcloud, int normalType = NormalType::PCL, int neighborhood = 4,
					 float threshold = 0.2, float outlierThreshold = 0.02, int iter = 3);

			/**
			 * Function computes normal for point (i, j) using direct computation (mean of surrounding triangles)
			 * @param i row index
			 * @param j column index
			 * @param step Neighborhood to compute with
			 * @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
			 */
			cv::Vec4f getNormal(int i, int j, int step, float depthThreshold);

			/**
			 * Function computes normal for point (i, j) using least trimmed squares regression
			 * @param i row index
			 * @param j column index
			 * @param step Neighborhood to compute with
			 * @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
			 * @param outlierThreshold Threshold for marking outliers in RANSAC (maximum difference from proposed plane)
			 * @param maxIter Maximum RANSAC iterations
			 */
			cv::Vec4f getNormalLTS(int i, int j, int step, float depthThreshold, float outlierThreshold, int maxIter);

			/**
			 * Function computes normal for point (i, j) using least trimmed squares regression with using only outer neighborhood ring
			 * @param i row index
			 * @param j column index
			 * @param step Neighborhood to compute with
			 * @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
			 * @param outlierThreshold Threshold for marking outliers in RANSAC (maximum difference from proposed plane)
			 * @param maxIter Maximum RANSAC iterations
			 */
			cv::Vec4f getNormalLTSAround(int i, int j, int step, float depthThreshold, float outlierThreshold, int maxIter);

			/**
			 * Function computes normal for point (i, j) using least squares regression
			 * @param i row index
			 * @param j column index
			 * @param step Neighborhood to compute with
			 * @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
			 */
			cv::Vec4f getNormalLSQ(int i, int j, int step, float depthThreshold);

			/**
			 * Function computes normal for point (i, j) using least squares regression with using only outer neighborhood ring
			 * @param i row index
			 * @param j column index
			 * @param step Neighborhood to compute with
			 * @param depthThreshold Threshold for depth difference outlier marking (if depth of neighbor is greater than this threshold, point is skipped)
			 */
			cv::Vec4f getNormalLSQAround(int i, int j, int step, float depthThreshold);

			/**
			 * Function computes plane using least squares regression from given point set
			 * @param points Vector of Vec3f points
			 * @return Plane object
			 * @see Plane()
			 */
			static Plane<float> LeastSquaresPlane(std::vector<cv::Vec3f> &points);

			/**
			 * Computed real points - scene coordinates (in meters)
			 */
			cv::Mat m_points;

			/**
			 * Computed plane equation approximations (so the normals also) for each point
			 */
			cv::Mat m_planes;

			/**
			 * Saved camera info (ROS)
			 */
			sensor_msgs::CameraInfoConstPtr m_cam_info;

			/**
			 * Helper if vector quantization is on - not used in this version (TODO)
			 */
			std::vector<cv::Vec3f> m_quantVectors;

			/**
			 * Helper if vector quantization is on - not used in this version (TODO)
			 */
			int m_quantbins;

		private:

			/**
			 * Helper function for "Around" functions - sets next point on outer ring
			 * @param step Maximum distance from center (neighborhood)
			 * @param x Current x offset
			 * @param y Current y offset
			 * @param plusX Next x shift
			 * @param plusY Next y shift
			 */
			void nextStep(int step, int &x, int &y, int &plusX, int &plusY);

			/**
			 * Helper function which initializes quantization vectors (not used, TODO)
			 * @param n_bins Number of quantization bins
			 * @param vec Vector of computed quantization vectors
			 */
			void initQuantVectors(int n_bins, std::vector<cv::Vec3f> &vec);

			/**
			 * Helper function which returns bin index for given vector (not used, TODO)
			 * @param vec Vector of computed quantization vectors
			 * @param vector Vector whos bin we are computing
			 */
			unsigned int getQuantVector(std::vector<cv::Vec3f> &vec, cv::Vec3f &vector);
	}; // class

} // but_scenemodel

#endif //BUT_SEG_UTILS_NORMALS_H
