/**
 * $Id$
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: 06.02.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#include <but_server/plugins/CollisionObjectPlugin.h>
#include <pcl_ros/transforms.h>


#define COLLISION_OBJECT_PUBLISHER_NAME std::string("butsrv_collision_object")
#define COLLISION_OBJECT_FRAME_ID std::string("/map")


srs::CCollisionObjectPlugin::CCollisionObjectPlugin(const std::string & name)
: srs::CServerPluginBase(name)
, m_publishCollisionObject(true)
, m_coPublisherName(COLLISION_OBJECT_PUBLISHER_NAME)
, m_latchedTopics(false)
, m_coFrameId(COLLISION_OBJECT_FRAME_ID)
{
	m_data = new tData;
	assert( m_data != 0 );
}



srs::CCollisionObjectPlugin::~CCollisionObjectPlugin()
{
}



bool srs::CCollisionObjectPlugin::shouldPublish()
{
	return( m_publishCollisionObject && m_coPublisher.getNumSubscribers() > 0 );
}



void srs::CCollisionObjectPlugin::init(ros::NodeHandle & node_handle)
{
	node_handle.param("collision_object_publisher", m_coPublisherName, COLLISION_OBJECT_PUBLISHER_NAME );
	node_handle.param("collision_object_frame_id", m_coFrameId, COLLISION_OBJECT_FRAME_ID );

	// Create publisher
	m_coPublisher = node_handle.advertise<arm_navigation_msgs::CollisionObject> (m_coPublisherName, 100, m_latchedTopics);
}



void srs::CCollisionObjectPlugin::onPublish(const ros::Time & timestamp)
{
	m_coPublisher.publish(*m_data);
}



void srs::CCollisionObjectPlugin::onFrameStart(const SMapParameters & par)
{
	m_data->header.frame_id = m_coFrameId;
	m_data->header.stamp = par.currentTime;
	m_data->id = "map";

	m_ocFrameId = par.frameId;

	tf::StampedTransform ocToCoTf;

	// Get transform
	try {
		// Transformation - to, from, time, waiting time
		m_tfListener.waitForTransform(m_coFrameId, m_ocFrameId,
				par.currentTime, ros::Duration(2.0)); // orig. 0.2

		m_tfListener.lookupTransform(m_coFrameId, m_ocFrameId,
				par.currentTime, ocToCoTf);

	} catch (tf::TransformException& ex) {
		ROS_ERROR_STREAM("Transform error: " << ex.what() << ", quitting callback");
		return;
	}

	Eigen::Matrix4f ocToPcTM;

	// Get transformation matrix
	pcl_ros::transformAsMatrix(ocToCoTf, ocToPcTM);

	// Disassemble translation and rotation
	m_ocToCoRot  = ocToPcTM.block<3, 3> (0, 0);
	m_ocToCoTrans = ocToPcTM.block<3, 1> (0, 3);

}



void srs::CCollisionObjectPlugin::handleOccupiedNode(const srs::tButServerOcTree::iterator & it, const SMapParameters & mp)
{
	// Transform input point
	Eigen::Vector3f point( it.getX(), it.getY(), it.getZ() );
	point = m_ocToCoRot * point + m_ocToCoTrans;

	// Add shape
	arm_navigation_msgs::Shape shape;
	shape.type = arm_navigation_msgs::Shape::BOX;
	shape.dimensions.resize(3);
	shape.dimensions[0] = shape.dimensions[1] = shape.dimensions[2] = it.getSize();
	m_data->shapes.push_back(shape);

	// Add pose
	geometry_msgs::Pose pose;
	pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
	pose.position.x = point.x();
	pose.position.y = point.y();
	pose.position.z = point.z();
	m_data->poses.push_back(pose);
}



void srs::CCollisionObjectPlugin::handlePostNodeTraversal(const ros::Time & rostime)
{
	invalidate();
}
