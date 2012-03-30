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

#ifndef ParameterSpaceOctomapOCTOMAP_H
#define ParameterSpaceOctomapOCTOMAP_H

//better opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <sensor_msgs/CameraInfo.h>
#include "normals.h"
#include <octomap/OcTree.h>

#include "parameterSpace.h"
using namespace std;
using namespace sensor_msgs;
using namespace cv;

namespace but_scenemodel
{

#define DEFAULT_RESOLUTION 0.01

class ParameterSpaceOctomap
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
		ParameterSpaceOctomap(double resolution = DEFAULT_RESOLUTION);

		/**
		 * Destructor
		 */
		~ParameterSpaceOctomap();

		/**
		 * Finds maximas in this and saves them as planes to given vector
		 * TODO
		 * @param indices Found planes
		 */
		int findMaxima(std::vector<Plane<float> > &indices);


		/**
		 * Adds a second volume to this with offset
		 * @param second Second volume to be added
		 * @param angle1 Angle 1 offset
		 * @param angle2 Angle 2 offset
		 * @param shift Shift offset
		 */
		void addVolume(ParameterSpace &second, float angle1, float angle2, float shift);


		/**
		 * Returns a value from given indices
		 */
		float get(float angle1, float angle2, float z);
		void set(float angle1, float angle2, float z, float val);

		/**
		 * Returns a size of this space
		 */
		int getSize();

		/**
		 * Conversion from Euclidian representation of normal (x, y, z) to parametrized (a1, a2)
		 */
		static void toAngles(float x, float y, float z, float &a1, float &a2);

		/**
		 * Conversion from parametrized representation of normal (a1, a2) to Euclidian (x, y, z)
		 */
		static void toEuklid(float a1, float a2, float &x, float &y, float &z);

		double m_resolution;

		octomap::OcTree m_paramSpace;
};

}

#endif
