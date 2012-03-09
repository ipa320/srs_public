/*
 * gripper.cpp
 *
 *  Created on: 25.1.2012
 *      Author: lom
 */

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/tools.h>
#include <interactive_markers/menu_handler.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/io/pcd_io.h>
#include <math.h>
#include <float.h>

using namespace std;

tf::MessageFilter<sensor_msgs::PointCloud2> *transform_filter;
tf::TransformListener *tfListener;
ros::Publisher marker_pub;

void getClosest(pcl::PointCloud<pcl::PointXYZ>& pclCloud, tf::Transformer& t, tf::Stamped<btVector3>& pose,float& distance)
{
  float d;
  BOOST_FOREACH (const pcl::PointXYZ& pt, pclCloud.points)
{  tf::Stamped<btVector3> p;
  p.setX(pt.x);
  p.setY(pt.y);
  p.setZ(pt.z);
  p.frame_id_ = "/head_cam3d_link";
  t.transformPoint("/sdh_palm_link", p, p);
  d = sqrt(pow(p.getX(),2)+pow(p.getY(),2)+pow(p.getZ(),2));
  if (d<distance)
  {
    distance=d;
    pose=p;
  }
}
//printf ("\tNejbližší bod: (%f, %f, %f), vzdalenost: %f\n", pose.getX(), pose.getY(), pose.getZ(), distance);
}

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
  /*pcl::PointCloud<pcl::PointXYZ> plcCloud;
   pcl::fromROSMsg(*cloud, plcCloud);

   BOOST_FOREACH (const pcl::PointXYZ& pt, plcCloud.points)
   { printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);}
   //cout << cloud->fields[0] << endl;
   exit(1);*/

  /* ros::Rate r(30);
   while (ros::ok())
   {*/
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "/sdh_palm_link";
  marker.header.stamp = cloud->header.stamp;
  marker.lifetime = ros::Duration();

  marker.ns = "gripper";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;

  geometry_msgs::Point p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker.points.push_back(p);
  /*
   geometry_msgs::PointStamped laser_point;
   laser_point.header.frame_id = "/sdh_palm_link";
   laser_point.header.stamp = ros::Time();
   laser_point.point.x = 10.0;
   laser_point.point.y = 12;
   laser_point.point.z = 6.0;*/

  ROS_INFO("pred transformaci");
  /*tf::TransformListener *tfListener;
   tfListener = new tf::TransformListener();*/
  tf::StampedTransform sensorToWorldTf;
  try
  {
    //ros::Duration(0, 100).sleep();
    tfListener->waitForTransform("/sdh_palm_link", "/head_cam3d_link", cloud->header.stamp, ros::Duration(0.2)/*, ros::Duration(0.1)*/);
    ROS_INFO("po wait");
    tfListener->lookupTransform("/sdh_palm_link", "/head_cam3d_link", cloud->header.stamp, sensorToWorldTf);
  }
  catch (tf::TransformException& ex)
  {
    std::cerr << "Transform error: " << ex.what() << ", quitting callback" << std::endl;
  }
  ROS_INFO("ziskana transformace");
  //tf::StampedTransform sensorToWorldTf; - already specified
  tf::Transformer t;
  t.setTransform(sensorToWorldTf);

  pcl::PointCloud<pcl::PointXYZ> plcCloud;
  pcl::fromROSMsg(*cloud, plcCloud);

  //cout << cloud->fields[0] << endl;
  tf::Stamped<btVector3> pose;
  float distance=FLT_MAX ;
  getClosest(plcCloud, t, pose, distance);

  ROS_INFO("pretransformovano");
  /*   try
   {
   tf::TransformListener listener;
   geometry_msgs::PointStamped base_point;
   listener.transformPoint("/map", laser_point, base_point);

   ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f", laser_point.point.x,
   laser_point.point.y, laser_point.point.z, base_point.point.x, base_point.point.y, base_point.point.z,
   base_point.header.stamp.toSec());
   */
  p.x = pose.getX();
  p.y = pose.getY();
  p.z = pose.getZ();
  marker.points.push_back(p);
  /* }
   catch (tf::TransformException& ex)
   {
   ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
   }*/

  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.4;
  marker_pub.publish(marker);

  /*visualization_msgs::Marker marker2;
   // Set the frame ID and timestamp.  See the TF tutorials for information on these.
   marker2.header.frame_id = "/sdh_palm_link";
   marker2.header.stamp = ros::Time::now();

   marker2.ns = "gripper";
   marker2.id = 0;
   marker2.type = visualization_msgs::Marker::SPHERE;
   marker2.action = visualization_msgs::Marker::ADD;
   marker2.pose.position.x = pose.getX();
   marker2.pose.position.y = pose.getY();
   marker2.pose.position.z = pose.getZ();
   marker2.pose.orientation.x = 0.0;
   marker2.pose.orientation.y = 0.0;
   marker2.pose.orientation.z = 0.0;
   marker2.pose.orientation.w = 1.0;
   marker2.scale.x = 0.5;
   marker2.scale.y = 0.5;
   marker2.scale.z = 0.5;
   marker2.color.r = 0.0f;
   marker2.color.g = 1.0f;
   marker2.color.b = 0.0f;
   marker2.color.a = 1.0;
   marker_pub.publish(marker2);*/

  /*  r.sleep();
   }*/

  visualization_msgs::Marker marker2;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker2.header.frame_id = "/sdh_palm_link";
  marker2.header.stamp = cloud->header.stamp;

  marker2.ns = "gripper";
  marker2.id = 2;
  marker2.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker2.action = visualization_msgs::Marker::ADD;
  marker2.pose.position.x = pose.getX() / 2;
  marker2.pose.position.y = pose.getY() / 2;
  marker2.pose.position.z = pose.getZ() / 2;
  ostringstream text_d;
  text_d << fabs(distance) << "m";
  marker2.text = text_d.str();
  marker2.scale.z = 0.2;
  marker2.color.r = 1.0f;
  marker2.color.g = 0.0f;
  marker2.color.b = 0.0f;
  marker2.color.a = 1.0;
  marker_pub.publish(marker2);

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripper");

  ros::NodeHandle n;
  marker_pub = n.advertise<visualization_msgs::Marker> ("gripper", 10);

  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud(n, "/cam3d/depth/points", 1);

  // init of tf listener
  tfListener = new tf::TransformListener();

  // Specify a message filter - this function only passes through when a transform from point cloud into "/map" is present
  // params: (subscriber, tf listener, destination frame, rate for checking new messages)
  transform_filter = new tf::MessageFilter<sensor_msgs::PointCloud2>(point_cloud, *tfListener, "/map", 1);

  // Register a callback
  transform_filter->registerCallback(boost::bind(&callback, _1));

  ros::spin();
  return 1;
}
