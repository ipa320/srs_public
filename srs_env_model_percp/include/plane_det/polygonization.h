/**
 * $Id: polygonization.h 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: 11.01.2012 (version 0.1)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *	 Class encapsulating region transformation into poly representation
 */


#ifndef POLYGONIZATION_H
#define POLYGONIZATION_H

//better opencv 2
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <sensor_msgs/CameraInfo.h>

using namespace std;
using namespace sensor_msgs;
using namespace cv;

namespace but_scenemodel
{

/**
 * Class encapsulating region transformation into poly representation
 * IN CONSTRUCTION, TODO
 */
class Polygonizer
{
	public:
		void GetPolyRegions(cv::Mat &regionImage);
};
}
#endif
