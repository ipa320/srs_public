/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/** \author Ioan Sucan */

#include <ros/ros.h>
#include <sstream>
#include "robot_self_filter/self_see_filter.h"
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <pcl/ros/conversions.h>
#include <pcl/filters/voxel_grid.h>

class SelfFilter
{
  public:
	//! PCL cloud type - contains only position
	typedef pcl::PointCloud<pcl::PointXYZ> tCloudXYZ;
	//! PCL cloud type - contains position and rgb data
	typedef pcl::PointCloud<pcl::PointXYZRGB> tCloudRGB;

	//! Self filter types
	typedef filters::SelfFilter<tCloudXYZ> tSelfFilterXYZ;
	typedef filters::SelfFilter<tCloudRGB> tSelfFilterRGB;

  public:
    SelfFilter (void): nh_ ("~")
    {
//    	ROS_ERROR( "MY FILTER VERSION");
      nh_.param<std::string> ("sensor_frame", sensor_frame_, std::string ());
      nh_.param<double> ("subsample_value", subsample_param_, 0.01);

      self_filter_xyz = new  tSelfFilterXYZ(nh_);
      self_filter_rgb = new  tSelfFilterRGB(nh_);

      sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2> (root_handle_, "cloud_in", 1);	
      mn_ = new tf::MessageFilter<sensor_msgs::PointCloud2> (*sub_, tf_, "", 1);

      //mn_ = new tf::MessageNotifier<sensor_msgs::PointCloud2>(tf_, boost::bind(&SelfFilter::cloudCallback, this, _1), "cloud_in", "", 1);
      pointCloudPublisher_ = root_handle_.advertise<sensor_msgs::PointCloud2>("cloud_out", 1);
      std::vector<std::string> frames;
      self_filter_xyz->getSelfMask()->getLinkNames(frames);
      self_filter_rgb->getSelfMask()->getLinkNames(frames);
      if (frames.empty())
      {
        ROS_DEBUG ("No valid frames have been passed into the self filter. Using a callback that will just forward scans on.");
        no_filter_sub_ = root_handle_.subscribe<sensor_msgs::PointCloud2> ("cloud_in", 1, boost::bind(&SelfFilter::noFilterCallback, this, _1));
      }
      else
      {
        ROS_DEBUG ("Valid frames were passed in. We'll filter them.");
        mn_->setTargetFrames (frames);
        mn_->registerCallback (boost::bind (&SelfFilter::cloudCallback, this, _1));
      }
    }
      
    ~SelfFilter (void)
    {
      delete self_filter_xyz;
      delete self_filter_rgb;
      delete mn_;
      delete sub_;
    }

    /**
	 * Test if incomming pointcloud2 has rgb part - parameter driven
	 */
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

  private:
    void 
      noFilterCallback (const sensor_msgs::PointCloud2ConstPtr &cloud)
    {
      pointCloudPublisher_.publish (cloud);
      ROS_DEBUG ("Self filter publishing unfiltered frame");
    }
      
    void cloudCallback (const sensor_msgs::PointCloud2ConstPtr &cloud2)
    {
      ROS_DEBUG ("Got pointcloud that is %f seconds old", (ros::Time::now() - cloud2->header.stamp).toSec ());
      std::vector<int> mask;
      ros::WallTime tm = ros::WallTime::now ();

      sensor_msgs::PointCloud2 out;

      if( !isRGBCloud(cloud2) )
      {
    	tCloudXYZ cloud, cloud_filtered;
		pcl::fromROSMsg (*cloud2, cloud);

		if (subsample_param_ != 0)
		{
		  tCloudXYZ cloud_downsampled;
		  // Set up the downsampling filter
		  grid_xyz.setLeafSize (subsample_param_, subsample_param_, subsample_param_);     // 1cm leaf size
		  grid_xyz.setInputCloud (boost::make_shared <tCloudXYZ> (cloud));
		  grid_xyz.filter (cloud_downsampled);

		  self_filter_xyz->updateWithSensorFrame (cloud_downsampled, cloud_filtered, sensor_frame_);
		}
		else
		{
		  self_filter_xyz->updateWithSensorFrame (cloud, cloud_filtered, sensor_frame_);
		}

		pcl::toROSMsg (cloud_filtered, out);
		double sec = (ros::WallTime::now() - tm).toSec ();
		ROS_DEBUG ("Self filter: reduced %d points to %d points in %f seconds", (int)cloud.points.size(), (int)cloud_filtered.points.size (), sec);

      }
      else
      {
//    	std::cerr << "IS RGB " << std::endl;

 		tCloudRGB cloud, cloud_filtered;
		pcl::fromROSMsg (*cloud2, cloud);

		if (subsample_param_ != 0)
		{
		  tCloudRGB cloud_downsampled;
		  // Set up the downsampling filter
		  grid_rgb.setLeafSize (subsample_param_, subsample_param_, subsample_param_);     // 1cm leaf size
		  grid_rgb.setInputCloud (boost::make_shared <tCloudRGB> (cloud));
		  grid_rgb.filter (cloud_downsampled);

		  self_filter_rgb->updateWithSensorFrame (cloud_downsampled, cloud_filtered, sensor_frame_);
		}
		else
		{
		  self_filter_rgb->updateWithSensorFrame (cloud, cloud_filtered, sensor_frame_);
		}

		pcl::toROSMsg (cloud_filtered, out);
		double sec = (ros::WallTime::now() - tm).toSec ();
		ROS_DEBUG ("Self filter: reduced %d points to %d points in %f seconds", (int)cloud.points.size(), (int)cloud_filtered.points.size (), sec);

      }
      pointCloudPublisher_.publish (out);
    }

    tf::TransformListener                                 tf_;
    //tf::MessageNotifier<sensor_msgs::PointCloud>           *mn_;
    ros::NodeHandle                                       nh_, root_handle_;

    tf::MessageFilter<sensor_msgs::PointCloud2>           *mn_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> *sub_;

    // Two possible self filters
    tSelfFilterXYZ *self_filter_xyz;
    tSelfFilterRGB *self_filter_rgb;

    std::string sensor_frame_;
    double subsample_param_;

    ros::Publisher                                        pointCloudPublisher_;
    ros::Subscriber                                       no_filter_sub_;

    pcl::VoxelGrid<pcl::PointXYZ>                         grid_xyz;
    pcl::VoxelGrid<pcl::PointXYZRGB>					  grid_rgb;
};

int 
  main (int argc, char **argv)
{
  ros::init (argc, argv, "self_filter");

  SelfFilter s;
  ros::spin ();
    
  return (0);
}
