/******************************************************************************
 * \file
 *
 * $Id:$
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

#ifndef ParameterSpaceHierarchy_H
#define ParameterSpaceHierarchy_H

// Opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

// ROS
#include <sensor_msgs/CameraInfo.h>

// but_scenemodel
#include "plane_det/parameterSpace.h"
#include "plane_det/normals.h"

using namespace std;
using namespace sensor_msgs;
using namespace cv;

namespace but_scenemodel
{

	#define DEFAULT_ANGLE_STEP 5
	#define DEFAULT_SHIFT_STEP 5
	#define DEFAULT_BIN_SIZE 16

	/**
	 * Class encapsulating indices in low/high resolution images
	 */
	class IndexStruct
	{
		public:
			/**
			 * Index in low resolution image
			 */
			int lowResolutionIndex;
			
			/**
			 * Index in high resolution image
			 */
			int highResolutionIndex;
			
			/**
			 * Final index in the whole structure
			 */
			int CompleteIndex;
	};
	
	/**
	 * Class encapsulating a Hough parameter space - (angle, angle, shift)
	 * Structure is 2 level hierarchic
	 */
	class ParameterSpaceHierarchy
	{
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
			ParameterSpaceHierarchy(double anglemin, double anglemax, double zmin, double zmax, double angleRes = DEFAULT_ANGLE_STEP, double shiftRes = DEFAULT_SHIFT_STEP);

			/**
			 * Destructor
			 */
			~ParameterSpaceHierarchy();

			/**
			 * Finds maximas in this and saves them as planes to given vector
			 * @param indices Found planes
			 * @returns index of maximal plane
			 */
			int findMaxima(std::vector<Plane<float> > &indices);

			/**
			 * Adds a second volume to this with offset
			 * @param second Second volume to be added
			 * @param angle1 Angle 1 offset
			 * @param angle2 Angle 2 offset
			 * @param shift Shift offset
			 */
			void addVolume(ParameterSpace &second, int angle1, int angle2, int shift);
			
			/**
			 * Adds a second volume to this with offset and multiplied by given factor
			 * @param second Second volume to be added
			 * @param angle1 Angle 1 offset
			 * @param angle2 Angle 2 offset
			 * @param shift Shift offset
			 * @param factor number by which each gauss function will be multiplied
			 */
			void addVolume(ParameterSpace &second, int angle1, int angle2, int shift, float factor);

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
			double get(int angle1, int angle2, int z);
			
			/**
			 * Returns a value saved at given index
			 * @param index Given index
			 */
			double get(int index);
			
			/**
			 * Returns a value saved at given values
			 * @param angle1 First angle value
			 * @param angle2 Second angle value
			 * @param z Shift (d param) value
			 */
			double get(double angle1, double angle2, double z);
			
			/**
			 * Saves a value at given coordinates (indices)
			 * @param angle1 First angle coordinate
			 * @param angle2 Second angle coordinate
			 * @param z Shift (d param) coordinate
			 * @param val Value to be saved
			 */
			void set(int angle1, int angle2, int z, double val);
			
			/**
			 * Saves a value at given index
			 * @param index Given index
			 * @param val Value to be saved
			 */
			void set(int index, double val);

			/**
			 * Saves a value at given values
			 * @param angle1 First angle value
			 * @param angle2 Second angle value
			 * @param z Shift (d param) value
			 * @param val Value to be saved
			 */
			void set(double angle1, double angle2, double z, double val);

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
			IndexStruct getIndex(double angle1, double angle2, double z);
			
			/**
			 * Returns an index from given coordinate indices
			 * @param angle1 First angle coordinate
			 * @param angle2 Second angle coordinate
			 * @param z Shift (d param) coordinate
			 */
			IndexStruct getIndex(int angle1, int angle2, int z);
			
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
			 * Converts an index into axis coordinates
			 * @param i Given index
			 * @param angle1 First angle coordinate
			 * @param angle2 Second angle coordinate
			 * @param z Shift (d param) coordinate
			 */
			void fromIndex(int i, int& angle1, int& angle2, int& z);
		
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
			 * Step of one low resolution angle coordinate
			 */
			double m_angleLoStep;
			
			/**
			 * Step of one low resolution shift coordinate
			 */
			double m_shiftLoStep;
			
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
			 * Size of low reslution array
			 */
			int m_loSize;
			
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
			 * Low resolution angle axis size
			 */
			int m_angleLoSize;
			
			/**
			 * Low resolution angle axis size ^ 2
			 */
			int m_angleLoSize2;
			
			/**
			 * Low resolution shift axis size
			 */
			int m_shiftLoSize;
			
			/**
			 * High resolution space size
			 */
			int m_hiSize;
			
			/**
			 * High resolution space size ^ 2
			 */
			int m_hiSize2;
			
			/**
			 * Param space matrix
			 */
			Mat m_paramSpace;
			
			/**
			 * Pointer to the low resolution structure
			 */
			double **m_dataLowRes;
	};

	/**
	 * Iterator to pass all non null parameter space indices
	 */
	class ParameterSpaceHierarchyFullIterator
	{
		public:
			/**
			 * Constructor - initializes iterator and sets an index to the first.
			 */
			ParameterSpaceHierarchyFullIterator(ParameterSpaceHierarchy *space)
			{
				index = space->m_angleSize*space->m_angleSize*space->m_shiftSize;
				currentI = 0;
				end = false;
				m_space = space;
				max_size = space->m_size;
			}

			/**
			 * Returns a value at the current index
			 */
			double getVal()
			{
				return m_space->get(currentI);
			}

			/**
			 * Sets a value at the current index
			 * @param val Value to be saved
			 */
			void setVal(double val)
			{
				m_space->set(currentI, val);
			}

			/**
			 * Increases an index to the next non null value
			 */
			ParameterSpaceHierarchyFullIterator &operator ++()
			{
				if (m_space->m_dataLowRes[currentI / m_space->m_hiSize] == NULL)
					currentI += m_space->m_hiSize;
				else
				{
					++currentI;
				}

				if (currentI>=max_size) end = true;
				return *this;
			}

			/**
			 * Current index
			 */
			int currentI;
			
			/**
			 * End flag (true if we are out of given array)
			 */
			bool end;
		private:
		
			/**
			 * TODO
			 */ 
			int index;
			
			/**
			 * Max size of given structure
			 */
			int max_size;

			/**
			 * Structure pointer
			 */
			ParameterSpaceHierarchy *m_space;
	};
} // but_scenemodel

#endif
