/**
 * $Id: ParameterSpaceOctomap.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: 11.01.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
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
