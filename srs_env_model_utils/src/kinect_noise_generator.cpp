/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by Robo@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 1/28/2012
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

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>



using namespace std;
using namespace sensor_msgs;

ros::Publisher m_pub;
bool add_noise = true;
double nac = 1.0; // noise amount coef
double max_val = 5000.0; // max depth value in mm
double min_val = 500.0;



/******************************************************************************/
/* randn()
 *
 * Normally (Gaussian) distributed random numbers, using the Box-Muller
 * transformation.  This transformation takes two uniformly distributed deviates
 * within the unit circle, and transforms them into two independently
 * distributed normal deviates.  Utilizes the internal rand() function; this can
 * easily be changed to use a better and faster RNG.
 *
 * The parameters passed to the function are the mean and standard deviation of
 * the desired distribution.  The default values used, when no arguments are
 * passed, are 0 and 1 - the standard normal distribution.
 *
 *
 * Two functions are provided:
 *
 * The first uses the so-called polar version of the B-M transformation, using
 * multiple calls to a uniform RNG to ensure the initial deviates are within the
 * unit circle.  This avoids making any costly trigonometric function calls.
 *
 * The second makes only a single set of calls to the RNG, and calculates a
 * position within the unit circle with two trigonometric function calls.
 *
 * The polar version is generally superior in terms of speed; however, on some
 * systems, the optimization of the math libraries may result in better
 * performance of the second.  Try it out on the target system to see which
 * works best for you.  On my test machine (Athlon 3800+), the non-trig version
 * runs at about 3x10^6 calls/s; while the trig version runs at about
 * 1.8x10^6 calls/s (-O2 optimization).
 *
 *
 * Example calls:
 * randn_notrig();	//returns normal deviate with mean=0.0, std. deviation=1.0
 * randn_notrig(5.2,3.0);	//returns deviate with mean=5.2, std. deviation=3.0
 *
 *
 * Dependencies - requires <cmath> for the sqrt(), sin(), and cos() calls, and a
 * #defined value for PI.
 */

/******************************************************************************/
//	"Polar" version without trigonometric calls
double randn_notrig(double mu=0.0, double sigma=1.0) {
	static bool deviateAvailable=false;	//	flag
	static float storedDeviate;			//	deviate from previous calculation
	double polar, rsquared, var1, var2;

	//	If no deviate has been stored, the polar Box-Muller transformation is
	//	performed, producing two independent normally-distributed random
	//	deviates.  One is stored for the next round, and one is returned.
	if (!deviateAvailable) {

		//	choose pairs of uniformly distributed deviates, discarding those
		//	that don't fall within the unit circle
		do {
			var1=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
			var2=2.0*( double(rand())/double(RAND_MAX) ) - 1.0;
			rsquared=var1*var1+var2*var2;
		} while ( rsquared>=1.0 || rsquared == 0.0);

		//	calculate polar tranformation for each deviate
		polar=sqrt(-2.0*log(rsquared)/rsquared);

		//	store first deviate and set flag
		storedDeviate=var1*polar;
		deviateAvailable=true;

		//	return second deviate
		return var2*polar*sigma + mu;
	}

	//	If a deviate is available from a previous call to this function, it is
	//	returned, and the flag is set to false.
	else {
		deviateAvailable=false;
		return storedDeviate*sigma + mu;
	}
}


/******************************************************************************/
//	Standard version with trigonometric calls
#define PI 3.14159265358979323846

double randn_trig(double mu=0.0, double sigma=1.0) {
	static bool deviateAvailable=false;	//	flag
	static float storedDeviate;			//	deviate from previous calculation
	double dist, angle;

	//	If no deviate has been stored, the standard Box-Muller transformation is
	//	performed, producing two independent normally-distributed random
	//	deviates.  One is stored for the next round, and one is returned.
	if (!deviateAvailable) {

		//	choose a pair of uniformly distributed deviates, one for the
		//	distance and one for the angle, and perform transformations
		dist=sqrt( -2.0 * log(double(rand()) / double(RAND_MAX)) );
		angle=2.0 * PI * (double(rand()) / double(RAND_MAX));

		//	calculate and store first deviate and set flag
		storedDeviate=dist*cos(angle);
		deviateAvailable=true;

		//	calcaulate return second deviate
		return dist * sin(angle) * sigma + mu;
	}

	//	If a deviate is available from a previous call to this function, it is
	//	returned, and the flag is set to false.
	else {
		deviateAvailable=false;
		return storedDeviate*sigma + mu;
	}
}

bool isRGBCloud( const sensor_msgs::PointCloud2ConstPtr& cloud )
	{
		sensor_msgs::PointCloud2::_fields_type::const_iterator i, end;

		for( i = cloud->fields.begin(), end = cloud->fields.end(); i != end; ++i )
		{
			if( i->name == "rgb" )
			{
				return true;
			}
		}
		return false;
	}

double add_noise_to_point(double val) {

	double val_noise;
	double sigma; // standard deviation

	if (val!=0.0) {

		sigma = (2.6610e-06*(val*val*nac*nac))  - (1.0787e-03*val*nac) +  2.7011;

		val_noise = randn_notrig(val,sigma); // to milimeters

    if (val_noise < min_val) val_noise = min_val;

		double q = 2.1983;

		val_noise = floor((val_noise-min_val)/q) * q + q/2 + min_val;
		
	} else val_noise = 0;

	return val_noise;

}

void cloudCallback(const PointCloud2ConstPtr& cloud) {
	
	sensor_msgs::PointCloud2 cloud_out;

  if (!isRGBCloud(cloud)) {
  
    ROS_INFO_ONCE("First pointcloud received.");
    
    //if (m_pub.getNumSubscribers() == 0) return;
  
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    pcl::PointCloud<pcl::PointXYZ> pointcloud_n;
    pcl::fromROSMsg (*cloud, pointcloud);
    
    pointcloud_n.header = pointcloud.header;
    
    for (unsigned int i = 0; i < pointcloud.points.size(); ++i) {

		  double val = pointcloud.points[i].z * 1000.0; // depth value in milimeters

		  if (val > max_val) continue;
		  if (val < min_val) continue;

		  double noise_val = add_noise_to_point(val);
		  
		  if (add_noise) pointcloud.points[i].z = noise_val / 1000.0;
		  
		  pointcloud_n.points.push_back(pointcloud.points[i]);

		}

    pcl::toROSMsg(pointcloud_n,cloud_out);


  } else {
  
    ROS_INFO_ONCE("First RGB pointcloud received.");
    
    if (m_pub.getNumSubscribers() == 0) return;
  
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
    pcl::PointCloud<pcl::PointXYZRGB> pointcloud_n;
    pcl::fromROSMsg (*cloud, pointcloud);
    
    pointcloud_n.header = pointcloud.header;
    
    for (unsigned int i = 0; i < pointcloud.points.size(); ++i) {

		  double val = pointcloud.points[i].z * 1000.0; // depth value in milimeters

		  if (val > max_val) continue;
		  if (val < min_val) continue;

		  double noise_val = add_noise_to_point(val);
		  
		  if (add_noise) pointcloud.points[i].z = noise_val / 1000.0;
		  
		  pointcloud_n.points.push_back(pointcloud.points[i]);

		}

	  pcl::toROSMsg(pointcloud_n,cloud_out);
  
  
  }

	
	m_pub.publish(cloud_out);

	
}



int main(int argc, char** argv) {

	ros::init(argc, argv, "add_noise_pcl");

	ROS_INFO("Starting kinect noise generator node...");

	ros::NodeHandle n;

	ros::param::param<bool>("~add_noise",add_noise,true);
	ros::param::param<double>("~noise_amount_coef",nac,1.0);

	ros::Subscriber sub;

	m_pub = n.advertise<sensor_msgs::PointCloud2> ("points_out", 1);

	ros::Rate r(1);

	ros::AsyncSpinner spinner(5);
	spinner.start();

	ROS_INFO("Spinning");

	while(ros::ok()) {

		if (m_pub.getNumSubscribers() != 0) sub = n.subscribe("points_in", 1, cloudCallback);
		else sub.shutdown();

		r.sleep();

	}

	spinner.stop();

}
