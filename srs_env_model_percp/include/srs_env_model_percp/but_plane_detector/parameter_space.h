/******************************************************************************
 * \file
 *
 * $Id: parameterSpace.h 693 2012-04-20 09:22:39Z ihulik $
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
 *	 Class encapsulating Parameter space (i.e. 3D Hough grid)
 *
 *	 Contains methods for construction / maxima search / adding of volumes etc.
 */

#pragma once
#ifndef BUT_PLANE_DET_PARAMSPACE_H
#define BUT_PLANE_DET_PARAMSPACE_H

// Opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

// ROS
#include <sensor_msgs/CameraInfo.h>

#include <but_segmentation/normals.h>


namespace srs_env_model_percp
{

	#define DEFAULT_ANGLE_STEP 5
	#define DEFAULT_SHIFT_STEP 5

	/**
	 * Class encapsulating a Hough parameter space
	 */
	class ParameterSpace
	{
		public:
			typedef but_plane_detector::Plane<float> tPlane;
			typedef std::vector<tPlane, Eigen::aligned_allocator<tPlane> > tPlanes;

		public:
			/**
			 * Constructor - creates and allocates a space (angle X angle X shift)
			 * @param anglemin Minimal angle in space
			 * @param anglemax Maximal angle in space
			 * @param zmin Minimal shift in space
			 * @param zmax Maximal shift in space
			 * @param angleRes Angle resolution (step)
			 * @param shiftRes Shift resolution (step)
			 */
			ParameterSpace(double anglemin, double anglemax, double zmin, double zmax, double angleRes = DEFAULT_ANGLE_STEP, double shiftRes = DEFAULT_SHIFT_STEP);

			/**
			 * Destructor
			 */
			~ParameterSpace();

			/**
			 * Finds maximas in this and saves them as planes to given vector
			 * @param indices Found planes
			 */
			int findMaxima(tPlanes &indices);

			/**
			 * Generates a Gauss curve into this space (centered)
			 * @param angleSigma sigma in angle directions
			 * @param shiftSigma sigma in shift direction
			 */
			void generateGaussIn(double angleSigma, double shiftSigma);

			/**
			 * Adds a second volume to this with offset
			 * @param second Second volume to be added
			 * @param angle1 Angle 1 offset
			 * @param angle2 Angle 2 offset
			 * @param shift Shift offset
			 */
			void addVolume(ParameterSpace &second, int angle1, int angle2, int shift);

			/**
			 * Converts index in parameter space into angle value
			 * @param index Angle axis index
			 */
			double getAngle(int index);

			/**
			 * Converts index in parameter space into shift value
			 * @param index Shift axis index
			 */
			double getShift(int irndex);

			/**
			 * Returns a value saved at given coordinates (indices)
			 * @param angle1 First angle coordinate
			 * @param angle2 Second angle coordinate
			 * @param z Shift (d param) coordinate
			 */
			double &operator() (int angle1, int angle2, int z);

			/**
			 * Returns a value saved at given index
			 * @param index Given index
			 */
			double &operator[] (int index);

			/**
			 * Returns a value saved at given values
			 * @param angle1 First angle value
			 * @param angle2 Second angle value
			 * @param z Shift (d param) value
			 */
			double &operator() (double angle1, double angle2, double z);

			/**
			 * Returns a size of this space structure in Bytes
			 */
			int getSize();

			/**
			 * Returns an index from given values
			 * @param angle1 First angle value
			 * @param angle2 Second angle value
			 * @param z Shift (d param) value
			 */
			int getIndex(double angle1, double angle2, double z);

			/**
			 * Converts between values and coordinates
			 * @param angle1 First angle value
			 * @param angle2 Second angle value
			 * @param z Shift (d param) value
			 * @param angle1Index First angle coordinate
			 * @param angle2Index Second angle coordinate
			 * @param shiftIndex (d param) coordinate
			 */
			void getIndex(double angle1, double angle2, double z, int &angle1Index, int &angle2Index, int &shiftIndex);

			/**
			 * Conversion from Euclidian representation of normal (x, y, z) to parametrized (a1, a2)
			 * @param x X vector coordinate
			 * @param y Y vector coordinate
			 * @param z Z vector coordinate
			 * @param a1 First angle
			 * @param a2 Second angle
			 */
			static void toAngles(float x, float y, float z, float &a1, float &a2);

			/**
			 * Conversion from parametrized representation of normal (a1, a2) to Euclidian (x, y, z)
			 * @param x X vector coordinate
			 * @param y Y vector coordinate
			 * @param z Z vector coordinate
			 * @param a1 First angle
			 * @param a2 Second angle
			 */
			static void toEuklid(float a1, float a2, float &x, float &y, float &z);

			/**
			 * Initialized flag
			 */
			bool m_init;
			
			/**
			 * Step of one angle coordinage
			 */
			double m_angleStep;
			
			/**
			 * Step of one shift coordinate
			 */
			double m_shiftStep;
			
			/**
			 * Minimal shift value
			 */
			double m_shiftmin;
			
			/**
			 * Maximal shift value
			 */
			double m_shiftmax;
			
			/**
			 * Minimal angle value
			 */
			double m_anglemin;
			
			/**
			 * Maximal shift value
			 */
			double m_anglemax;
			
			/**
			 * Size of underlying structure
			 */
			int m_size;
			
			/**
			 * Size of angle axis
			 */
			int m_angleSize;
			
			/**
			 * Size of angle axis ^ 2
			 */
			int m_angleSize2;
			
			/**
			 * Shift axis size
			 */
			int m_shiftSize;
			
			/**
			 * Param space matrix
			 */
			cv::Mat m_paramSpace;
			
			/**
			 * Pointer to the Hough space structure
			 */
			double *m_data;
	};
} // but_plane_detector

#endif
