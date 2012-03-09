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

ros::Publisher marker_pub;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker> ("test", 1);
  // %EndTag(INIT)%

  // Set our initial shape type to be a cube
  // %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::MESH_RESOURCE;
  // %EndTag(SHAPE_INIT)%

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    // %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    // %Tag(NS_ID)%
    marker.ns = "basic_shapes";
    marker.id = 0;
    // %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    // %Tag(TYPE)%
    marker.type = shape;
    // %EndTag(TYPE)%

    // Set the marker action.  Options are ADD and DELETE
    // %Tag(ACTION)%
    marker.action = visualization_msgs::Marker::ADD;
    // %EndTag(ACTION)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    // %Tag(POSE)%
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    // %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    // %Tag(SCALE)%
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    // %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
    // %Tag(COLOR)%
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    // %EndTag(COLOR)%

    // %Tag(LIFETIME)%
    marker.lifetime = ros::Duration();
    // %EndTag(LIFETIME)%

    marker.text = "NEJAKY text";
    //marker.mesh_resource = "package://srs_env_model/meshes/unknown_object.dae";
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
    marker.mesh_use_embedded_materials = false;

    // Publish the marker
    // %Tag(PUBLISH)%
    marker_pub.publish(marker);
    // %EndTag(PUBLISH)%


    // Cycle between different shapes
    // %Tag(CYCLE_SHAPES)%
  /*  switch (shape)
    {
      case visualization_msgs::Marker::MESH_RESOURCE:
        shape = visualization_msgs::Marker::SPHERE;
        break;
      case visualization_msgs::Marker::SPHERE:
        shape = visualization_msgs::Marker::ARROW;
        break;
      case visualization_msgs::Marker::ARROW:
        shape = visualization_msgs::Marker::CYLINDER;
        break;
      case visualization_msgs::Marker::CYLINDER:
        shape = visualization_msgs::Marker::CUBE;
        break;
      case visualization_msgs::Marker::CUBE:
        shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
        break;
      case visualization_msgs::Marker::TEXT_VIEW_FACING:
        shape = visualization_msgs::Marker::MESH_RESOURCE;
        break;
    }*/
    // %EndTag(CYCLE_SHAPES)%

    // %Tag(SLEEP_END)%
    r.sleep();
  }

}
