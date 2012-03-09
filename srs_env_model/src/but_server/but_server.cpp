/**
 * $Id: but_server.cpp 281 2012-03-05 14:50:43Z stancl $
 *
 * Modified by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 * This code is derived from the OctoMap server provided by A. Hornung.
 * Please, see the original comments below.
 *
 */

/**
 * octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
 * (inspired by the ROS map_saver)
 * @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
 * @see http://octomap.sourceforge.net/
 * License: BSD
 */

/*
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <but_server/but_server.h>
//#include <but_server/ServerTools.h>

#include <sstream>

// Define publishers names
#define MARKERS_PUBLISHER_NAME std::string("butsrv_ocupied_cells_markers")
#define OCTOMAP_BINARY_PUBLISHER_NAME std::string("butsrv_octomap_binary")
#define MAP_PUBLISHER_NAME std::string("but_map")
#define SUBSCRIBER_CAMERA_POSITION std::string("rviz_camera_position")
#define WORLD_FRAME_ID std::string("/map")
#define BASE_FRAME_ID std::string("/base_footprint")
#define NUM_PCFRAMES_PROCESSED int(3)

//! Swap function template
//template <typename tpType> void swap( tpType & x, tpType & y){ tpType b(x); x = y; y = b; }

///////////////////////////////////////////////////////////////////////////////
/**
 Constructor
 */
CButServer::CButServer(const std::string& filename) :
			m_nh(), m_useHeightMap(true), m_colorFactor(0.8),
			m_latchedTopics(false),  m_occupancyMinZ(
					-std::numeric_limits<double>::max()), m_occupancyMaxZ(
					std::numeric_limits<double>::max()), m_filterSpeckles(false),
			m_numPCFramesProcessed(1.0), m_frameCounter(0),
			m_plugCMapPub("CMAP"),
			m_plugInputPointCloud("PCIN"),
			m_plugOcMapPointCloud("PCOC"),
			m_plugVisiblePointCloud("PCVIS"),
			m_plugOctoMap("OCM"),
			m_plugCollisionObject("COB"),
			m_plugMap2D("M2D"),
			m_plugIMarkers("IM")
{
	// Get node handla
	ros::NodeHandle private_nh("~");

	// Set parameters
	private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
	private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
	private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
	private_nh.param("color_factor", m_colorFactor, m_colorFactor);


	private_nh.param("occupancy_min_z", m_occupancyMinZ, m_occupancyMinZ);
	private_nh.param("occupancy_max_z", m_occupancyMaxZ, m_occupancyMaxZ);


	private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);

	// Use every n-th frame when processing incoming point cloud
	private_nh.param("PC_frames_processed", m_numPCFramesProcessed,
			NUM_PCFRAMES_PROCESSED);

	// Topic names
	private_nh.param("ocupied_cells_publisher", m_ocupiedCellsPublisher,
			MARKERS_PUBLISHER_NAME );
	private_nh.param("octomap_binary_publisher", m_octomapBinaryPublisher,
			OCTOMAP_BINARY_PUBLISHER_NAME );

	private_nh.param("camera_position_topic", m_cameraPositionSubscriber,
			SUBSCRIBER_CAMERA_POSITION );

	// In this frame ID will be incomming points transformed and stored
	private_nh.param("world_frame_id", m_worldFrameId, WORLD_FRAME_ID );

	// Get FID to which will be all points transformed
	private_nh.param("base_frame_id", m_baseFrameId, BASE_FRAME_ID );

	double r, g, b, a;

	private_nh.param("color/r", r, 0.0);
	private_nh.param("color/g", g, 0.0);
	private_nh.param("color/b", b, 1.0);
	private_nh.param("color/a", a, 1.0);
	m_color.r = r;
	m_color.g = g;
	m_color.b = b;
	m_color.a = a;

	// Is map static (loaded from file)?
	bool staticMap(filename != "");

	m_latchedTopics = staticMap;
	private_nh.param("latch", m_latchedTopics, m_latchedTopics);

	// Create publishers
	m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray> (
			m_ocupiedCellsPublisher, 100, m_latchedTopics);
	m_binaryMapPub = m_nh.advertise<octomap_ros::OctomapBinary> (
			m_octomapBinaryPublisher, 100, m_latchedTopics);


	// Subscribe to the camera info topic
	// m_camInfoSub = new message_filters::Subscriber<sensor_msgs::CameraInfo> ( m_nh, m_cameraInfoSubscriber, 200 );
	// m_tfCameraInfoSub = new tf::MessageFilter<sensor_msgs::CameraInfo> (*m_camInfoSub, m_tfListener, m_worldFrameId, 100);
	// m_tfCameraInfoSub->registerCallback(boost::bind(&CButServer::cameraInfoCallback, this, _1));



	//=========================================================================
	// Initialize plugins
	m_plugCMapPub.init( private_nh );
	m_plugInputPointCloud.init(private_nh);
	m_plugOcMapPointCloud.init(private_nh, false);
	m_plugOctoMap.init( private_nh );
	m_plugCollisionObject.init( private_nh );
	m_plugMap2D.init(private_nh);
	m_plugIMarkers.init(private_nh);

	// Connect input point cloud input with octomap
	m_plugInputPointCloud.getSigDataChanged().connect( boost::bind( &srs::COctoMapPlugin::insertCloud, &m_plugOctoMap, _1 ));

	// Connect octomap data changed signal with server publish
	m_plugOctoMap.getSigDataChanged().connect( boost::bind( &CButServer::onOcMapDataChanged, this, _1 ));


} // Constructor

///////////////////////////////////////////////////////////////////////////////
/**
 Destructor
 */
CButServer::~CButServer()
{


}



///////////////////////////////////////////////////////////////////////////////

/**
 Publish all data
 */
void CButServer::publishAll(const ros::Time& rostime) {

	// Store start time
	ros::WallTime startTime = ros::WallTime::now();

	// If no data, do nothing
	if (m_plugOctoMap.getSize() <= 1) {
		ROS_WARN("Nothing to publish, octree is empty");
		return;
	}

	// What should be published - get if there are any subscribers...
	bool publishMarkerArray = (m_latchedTopics
			|| m_markerPub.getNumSubscribers() > 0);

	// init markers:
	visualization_msgs::MarkerArray occupiedNodesVis;

	// each array stores all cubes of a different size, one for each depth level:
	occupiedNodesVis.markers.resize(m_plugOctoMap.getTreeDepth() + 1);

	//=========================================================================
	// Plugins frame start


	// Create holders
	typedef srs::CCrawlingPluginHolder< srs::CPointCloudPlugin, srs::COctoMapPlugin > tPCHolder;
	tPCHolder cPCHolder( &m_plugOcMapPointCloud, tPCHolder::ON_START | tPCHolder::ON_OCCUPIED | tPCHolder::ON_STOP );

	typedef srs::CCrawlingPluginHolder< srs::CCMapPlugin, srs::COctoMapPlugin > tCMapHolder;
	tCMapHolder cCMapHolder( &m_plugCMapPub, tCMapHolder::ON_START | tCMapHolder::ON_OCCUPIED | tCMapHolder::ON_STOP );

	typedef srs::CCrawlingPluginHolder< srs::CCollisionObjectPlugin, srs::COctoMapPlugin > tCOHolder;
	tCOHolder cCOHolder( &m_plugCollisionObject, tCOHolder::ON_START | tCOHolder::ON_OCCUPIED | tCOHolder::ON_STOP );

	typedef srs::CCrawlingPluginHolder< srs::CMap2DPlugin, srs::COctoMapPlugin > tM2DHolder;
	tM2DHolder cM2DHolder( &m_plugMap2D, tM2DHolder::ON_START | tM2DHolder::ON_OCCUPIED | tM2DHolder::ON_FREE | tM2DHolder::ON_STOP );

	// Connect what needed
	if( m_plugOcMapPointCloud.shouldPublish() )
	{
		cPCHolder.connect( & m_plugOctoMap );
	}

	if( m_plugCMapPub.shouldPublish() )
	{
		cCMapHolder.connect(& m_plugOctoMap );
	}

	if( m_plugCollisionObject.shouldPublish() )
	{
		cCOHolder.connect(& m_plugOctoMap );
	}

	if( m_plugMap2D.shouldPublish() )
	{
		cM2DHolder.connect(& m_plugOctoMap );
	}

	// Crawl octomap
	m_plugOctoMap.crawl( rostime );

	// Disconnect all
	cPCHolder.disconnect();
	cCMapHolder.disconnect();
	cCOHolder.disconnect();
	cM2DHolder.disconnect();

	// Publish collision object
	if (m_plugCollisionObject.shouldPublish() )
	{
		m_plugCollisionObject.onPublish( rostime );
	}

	// Publish point cloud
	if( m_plugOcMapPointCloud.shouldPublish() )
	{
		m_plugOcMapPointCloud.onPublish( rostime );
	}

	if(m_plugMap2D.shouldPublish())
	{
		m_plugMap2D.onPublish(rostime);
	}

	// Finalize octomap publishing
	if (m_plugOctoMap.shouldPublish())
		m_plugOctoMap.onPublish( rostime );

	//=========================================================================
	// Plugins publishing
	m_plugCMapPub.onPublish( rostime );

	// Compute and show elapsed time
	double total_elapsed = (ros::WallTime::now() - startTime).toSec();
	ROS_DEBUG("Map publishing in CButServer took %f sec", total_elapsed);

}


///////////////////////////////////////////////////////////////////////////////

/**
 Generate color from the height
 */
std_msgs::ColorRGBA CButServer::heightMapColor(double h) const {

	std_msgs::ColorRGBA color;
	color.a = 1.0;
	// blend over HSV-values (more colors)

	double s = 1.0;
	double v = 1.0;

	h -= floor(h);
	h *= 6;
	int i;
	double m, n, f;

	i = floor(h);
	f = h - i;
	if (!(i & 1))
		f = 1 - f; // if i is even
	m = v * (1 - s);
	n = v * (1 - s * f);

	switch (i) {
	case 6:
	case 0:
		color.r = v;
		color.g = n;
		color.b = m;
		break;
	case 1:
		color.r = n;
		color.g = v;
		color.b = m;
		break;
	case 2:
		color.r = m;
		color.g = v;
		color.b = n;
		break;
	case 3:
		color.r = m;
		color.g = n;
		color.b = v;
		break;
	case 4:
		color.r = n;
		color.g = m;
		color.b = v;
		break;
	case 5:
		color.r = v;
		color.g = m;
		color.b = n;
		break;
	default:
		color.r = 1;
		color.g = 0.5;
		color.b = 0.5;
		break;
	}

	return color;
}



/**
 * On octomap data changed
 */
void CButServer::onOcMapDataChanged( const srs::tButServerOcMap & mapdata )
{
	// Publish all data
	publishAll(ros::Time::now() );
}


