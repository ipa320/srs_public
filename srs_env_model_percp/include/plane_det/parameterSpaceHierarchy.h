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

//better opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <sensor_msgs/CameraInfo.h>
#include "parameterSpace.h"
#include "normals.h"

using namespace std;
using namespace sensor_msgs;
using namespace cv;

namespace but_scenemodel
{

#define DEFAULT_ANGLE_STEP 5
#define DEFAULT_SHIFT_STEP 5
#define DEFAULT_BIN_SIZE 16

class IndexStruct
{
	public:
		int lowResolutionIndex;
		int highResolutionIndex;
		int CompleteIndex;
};
class ParameterSpaceHierarchy
{
	public:
		/**
		 * Constructor - creates and allocates a space (angle X angle X shift)
		 *
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
		 * TODO
		 * @param indices Found planes
		 */
		int findMaxima(std::vector<Plane<float> > &indices);

		/**
		 * Generates a Gauss curve into this space (centered)
		 * @param angleSigma sigma in angle directions
		 * @param shiftSigma sigma in shift direction
		 */
		//void generateGaussIn(double angleSigma, double shiftSigma);

		/**
		 * Adds a second volume to this with offset
		 * @param second Second volume to be added
		 * @param angle1 Angle 1 offset
		 * @param angle2 Angle 2 offset
		 * @param shift Shift offset
		 */
		void addVolume(ParameterSpace &second, int angle1, int angle2, int shift);
		void addVolume(ParameterSpace &second, int angle1, int angle2, int shift, float factor);

		/**
		 * Converts index in parameter space into angle value
		 */
		double getAngle(int index);

		/**
		 * Converts index in parameter space into shift value
		 */
		double getShift(int irndex);


		double get(int angle1, int angle2, int z);
		void set(int angle1, int angle2, int z, double val);
		double get(int index);
		void set(int index, double val);

		double get(double angle1, double angle2, double z);
		void set(double angle1, double angle2, double z, double val);


		/**
		 * Returns a size of this space
		 */
		int getSize();

		/**
		 * Returns an index from given values
		 */
		IndexStruct getIndex(double angle1, double angle2, double z);
		void fromIndex(int i, int& angle1, int& angle2, int& z);
		IndexStruct getIndex(int angle1, int angle2, int z);

		/**
		 * Returns axis indices from given values
		 */
		void getIndex(double angle1, double angle2, double z, int &angle1Index, int &angle2Index, int &shiftIndex);


		/**
		 * Conversion from Euclidian representation of normal (x, y, z) to parametrized (a1, a2)
		 */
		static void toAngles(float x, float y, float z, float &a1, float &a2);

		/**
		 * Conversion from parametrized representation of normal (a1, a2) to Euclidian (x, y, z)
		 */
		static void toEuklid(float a1, float a2, float &x, float &y, float &z);

		bool m_init;
		double m_angleStep;
		double m_shiftStep;
		double m_angleLoStep;
		double m_shiftLoStep;
		double m_shiftmin;
		double m_shiftmax;
		double m_anglemin;
		double m_anglemax;
		int m_size;
		int m_loSize;
		int m_angleSize;
		int m_angleSize2;
		int m_shiftSize;
		int m_angleLoSize;
		int m_angleLoSize2;
		int m_shiftLoSize;
		int m_hiSize;
		int m_hiSize2;
		Mat m_paramSpace;
		double **m_dataLowRes;
};

class ParameterSpaceHierarchyFullIterator
{
	public:
		ParameterSpaceHierarchyFullIterator(ParameterSpaceHierarchy *space)
		{
			index = space->m_angleSize*space->m_angleSize*space->m_shiftSize;
			currentI = 0;
			end = false;
			m_space = space;
			max_size = space->m_size;
		}

		double getVal()
		{
			return m_space->get(currentI);
		}

		void setVal(double val)
		{
			m_space->set(currentI, val);
		}

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

		int currentI;
		bool end;
	private:
		int index;
		int max_size;

		ParameterSpaceHierarchy *m_space;
};

}

#endif
