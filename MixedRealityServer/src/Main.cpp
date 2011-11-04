#include <ros/ros.h>
#include "MixedRealityServer/MixedRealityServer.h"
#include "nav_msgs/OccupancyGrid.h"

#include "tf/transform_listener.h"
#include "MixedRealityServer/DrawObject.h"


using namespace MixedRealityServer;

MRServer* server;

void mapCallback(const nav_msgs::OccupancyGrid msg)
{
	IplImage* img =  cvCreateImage(cvSize(msg.info.width,msg.info.height),IPL_DEPTH_8U,1);
	int sz = msg.data.size();
	for(int i = 0; i < sz; i++)
	{
		img->imageData[i] = (msg.data[i] == -1 || msg.data[i] == 0) ? 255 : 0;
	}
	cvConvertImage(img, img, CV_CVTIMG_FLIP);
	server->SetMap(img, msg.info.resolution);
	cvReleaseImage(&img);
}

void controlMRSCallback(MixedRealityServer::DrawObject obj)
{
	server->DrawCommand(obj);
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "MixedRealityServer");

    ros::NodeHandle node_hndl;
    server = new MRServer(node_hndl);
    ros::Subscriber map_sub = node_hndl.subscribe("map", 1, mapCallback);
	ros::Subscriber control_sub = node_hndl.subscribe("control_mrs", 10, controlMRSCallback);
    //ros::Subscriber input_sub = node_hndl.subscribe("interface_recobj", 1, inputCallback);
    
    server->Spin();

    return(0);
}
