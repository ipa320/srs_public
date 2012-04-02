#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include <fstream>
#include <string>
#include <iostream>
using namespace std;

void legCallback(const sensor_msgs::PointCloud::ConstPtr& people_cloud)
{


        cout << "Opening  file: /home/alex/srs/legdata.txt for writing \n";

	std::fstream testleg("/home/alex/srs/legdata.txt",std::ios::out | std::ios::app);
	if (testleg.is_open()){
        cout << "Writing data from /particle_filt_cloud \n";
	testleg<<"seq: ";
	testleg<<people_cloud->header.seq;
	testleg<<"\n";
	testleg<<"frame_id: ";
	testleg<<people_cloud->header.frame_id;
	testleg<<"\n";
	testleg<<"points: "<<"\n";

	if ((int)people_cloud->points.size()!=0){
		for (int  i=0; i<(int)people_cloud->points.size(); ++i){
			testleg<<" x: ";testleg<<people_cloud->points[i].x;testleg<<"\n";
			testleg<<" y: ";testleg<<people_cloud->points[i].y;testleg<<"\n";
			testleg<<" z: ";testleg<<people_cloud->points[i].z;testleg<<"\n";
		}
	}
        cout << "Closing file: \n\n";
	testleg.close();
	}
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "legListener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber subleg = n.subscribe("/particle_filt_cloud", 10, legCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

