/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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
#pragma once
#ifndef SERVER_TOOLS_H_INCLUDED
#define SERVER_TOOLS_H_INCLUDED

#include <srs_env_model/but_server/octonode.h>

#include <boost/signal.hpp>
#include <boost/thread/mutex.hpp>

// Small double number
#define SMALL_DOUBLE double(0.00000001);

//! Absolute value method
template <typename tpType> tpType abs( tpType x ) { return x < 0.0 ? -x : x; }

//! Is absolute value of the given number smaller than SMALL_DOUBLE?
template <typename tpType> bool isSmall( tpType x ) { return abs( x ) < SMALL_DOUBLE; }

//! Is absolute value of the given number greater than SMALL_DOUBLE?
template <typename tpType> bool isGreat( tpType x ) { return abs( x ) > SMALL_DOUBLE; }

namespace srs_env_model
{
	//! ROS octomap type
	typedef octomap::OctomapROS< srs_env_model::EMOcTree > tButServerOcMap;

	//! Define octree type
	typedef tButServerOcMap::OcTreeType tButServerOcTree;

	//! Define node type
	typedef tButServerOcTree::NodeType tButServerOcNode;

	//! Define pcl point type
	typedef pcl::PointXYZRGB tPclPoint;

	//! Define point cloud type
	typedef pcl::PointCloud<tPclPoint> tPointCloud;

	/// All needed octo map parameters and something more...
	struct SMapParameters
	{
	public:
		/// Octomap resolution
		double resolution;

		/// Octomap tree depth
		unsigned treeDepth;

		/// Hit probability
		double probHit;

		/// Miss (free cell) probability
		double probMiss;

		/// Clamping minimum
		double thresMin;

		/// Clamping maximum
		double thresMax;

		/// Occupancy threshold
		double thresOccupancy;

		/// Maximal range of valid sensor data
		double maxRange;

		/// Current octomap size
		unsigned mapSize;

		/// Current stamp time
		ros::Time currentTime;

		/// Map frame id
		std::string frameId;

		/// Map pointer
		const tButServerOcMap * map;

	}; // struct SMapParameters.

	///////////////////////////////////////////////////////////////////////////

	//! Server plugin base class
	class CServerPluginBase
	{

	#define PERROR( x ) std::cerr << "Plugin "<< this->m_name << ": " << x << std::endl;

	public:
		//! Constructor
		CServerPluginBase( const std::string & name ) : m_frame_number( 0 ), m_use_every_nth( 1 ), m_name(name){ }

		//! Initialize plugin - called in server constructor
		virtual void init(ros::NodeHandle & node_handle){}

		//! Called when new scan was inserted and now all can be published
		virtual void onPublish(const ros::Time & timestamp){}

		//! Reset plugin content when reseting whole server
		virtual void reset() {}

		/// Should something be published?
		virtual bool shouldPublish(){ return false; }

		/// Set frame skip
		void setFrameSkip(unsigned long skip){ m_use_every_nth = skip; }

	protected:
		//! Counts frames and checks if node should publish in this frame
		virtual bool useFrame() { return ++m_frame_number % m_use_every_nth == 0; }

	protected:
		//! Current frame number
		unsigned long m_frame_number;

		//! Use every n-th frame (if m_frame_number modulo m_use_every_nth)
		unsigned long m_use_every_nth;

		//! Plugin name
		std::string m_name;

	};

	//! Octomap node crawler policy interface -
	template< class tpNodeType >
	class COctomapCrawlerBase
	{
	public:
		//! Set used octomap frame id and timestamp
		virtual void onFrameStart( const SMapParameters & par )
			{ m_frame_id = par.frameId; m_time_stamp = par.currentTime; }

		//! Handle free node (does nothing here)
		virtual void handleFreeNode(tButServerOcTree::iterator & it, const SMapParameters & mp ){}

		/// hook that is called when traversing all nodes of the updated Octree (does nothing here)
		virtual void handleNode(tButServerOcTree::iterator& it, const SMapParameters & mp) {};

		/// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
		virtual void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapParameters & mp){}

		/// Called when all nodes was visited.
		virtual void handlePostNodeTraversal(const SMapParameters & mp){}

	protected:
		//! Octomap frame_id
		std::string m_frame_id;

		//! Current timestamp
		ros::Time m_time_stamp;

	};

	/// To the octomap attachable plugins envelope
	template< class tpPlugin, class tpOctomapPlugin >
	class CCrawlingPluginHolder
	{
	public:
		/// Connection flags - set them to enable connection of according signal
		enum EConnectionFlags
		{
			ON_START = 1,
			ON_NODE  = 2,
			ON_FREE  = 4,
			ON_OCCUPIED = 8,
			ON_STOP = 16,
			ALL	= ON_START | ON_NODE | ON_FREE | ON_OCCUPIED | ON_STOP
		};
	public:

		/// Creating constructor
		CCrawlingPluginHolder( const std::string & name, int flags )
		: m_plugin( new tpPlugin( name ) )
		, m_source( 0 )
		, m_connected( false )
		, m_flags( flags )
		{
		    assert( m_plugin != 0 );
		    m_bDeletePlugin = true;
		}
		/// Constructor
		CCrawlingPluginHolder( tpPlugin * plugin, int flags, bool deletePlugin = false )
		: m_plugin( plugin )
		, m_source(0)
		, m_connected( false )
		, m_flags(flags)
		{
		    assert( plugin != 0 );
		    m_bDeletePlugin = deletePlugin;
		}

		/// Connecting constructor
		CCrawlingPluginHolder( tpPlugin * plugin, tpOctomapPlugin * source, EConnectionFlags flags, bool deletePlugin = false )
		: m_plugin( plugin )
		, m_source(0)
		, m_connected( false )
		, m_flags(flags)
		{
		    assert( plugin != 0 );
		    connect(source);
		    m_bDeletePlugin = deletePlugin;
		}

		/// Destructor
		virtual ~CCrawlingPluginHolder()
		{
		    disconnect();
		    if( m_bDeletePlugin && m_plugin != 0 )
		        delete m_plugin;
		}

		/// Connect plugin to the data
		void connect( tpOctomapPlugin * source )
		{
			if( m_connected )
				disconnect();

			if( !m_plugin->shouldPublish() )
			    return;

			if( source != 0 )
			{
				m_source = source;

				if( m_flags & ON_START)	m_conStart = m_source->getSigOnStart().connect( boost::bind(&tpPlugin::onFrameStart, m_plugin, _1 ) );
				if( m_flags & ON_NODE)	m_conNode = source->getSigOnNode().connect( boost::bind(&tpPlugin::handleNode, m_plugin, _1, _2 ) );
				if( m_flags & ON_FREE)	m_conFreeNode = source->getSigOnFreeNode().connect( boost::bind(&tpPlugin::handleFreeNode, m_plugin, _1, _2 ) );
				if( m_flags & ON_OCCUPIED)	m_conOccupiedNode = source->getSigOnOccupiedNode().connect( boost::bind(&tpPlugin::handleOccupiedNode, m_plugin, _1, _2 ) );
				if( m_flags & ON_STOP)	m_conStop = m_source->getSigOnPost().connect( boost::bind(&tpPlugin::handlePostNodeTraversal, m_plugin, _1 ) );
				m_connected = true;
			}
		}

		/// Disconnect plugin
		void disconnect()
		{
			if( m_connected && m_source != 0 )
			{
				m_conStart.disconnect();
				m_conNode.disconnect();
				m_conFreeNode.disconnect();
				m_conOccupiedNode.disconnect();
				m_conStop.disconnect();

				m_source = 0;
				m_connected = false;
			}
		}

		/// Publish data if exists
		void publish(const ros::Time & timestamp)
		{
		    if( m_plugin->shouldPublish() )
		        m_plugin->onPublish( timestamp );
		}

		/// Get plugin pointer
		tpPlugin * getPlugin( ) { return m_plugin; }

	protected:
		/// Plugin pointer
		tpPlugin * m_plugin;

		/// Input source pointer
		tpOctomapPlugin * m_source;

		/// Connections
		boost::signals::connection m_conStart, m_conNode, m_conFreeNode, m_conOccupiedNode, m_conStop;

		/// Is plugin connected now?
		bool m_connected;

		/// Used flags
		int m_flags;

		/// Delete plugin data on exit?
		bool m_bDeletePlugin;
	};

	/**
	 * @brief Data holder policy
	 */
	template< class tpDataType >
	class CDataHolderBase
	{
	public:
		/// Data type
		typedef tpDataType tData;

		/// Data has changed signal type
		typedef boost::signal< void (const tData & ) > tSigDataHasChanged;

	public:
		//! Get data reference
		tData & getData(){ return *m_data; }

		//! Get constant data reference
		const tData & getData() const { return *m_data; }

		/// Get data has changed signal
		tSigDataHasChanged & getSigDataChanged() { return m_sigDataChanged; }

		/// Data valid
		virtual bool hasValidData() { return m_data != 0; }

		/// Invalidate data - calls invalid signal
		void invalidate() { if( hasValidData() ) m_sigDataChanged( *m_data ); }


	protected:
		/// Data
		tData * m_data;

		/// Time stamp
		ros::Time m_DataTimeStamp;

		/// Data changed signal
		tSigDataHasChanged m_sigDataChanged;

		/// You can use this mutex to lock data
		boost::mutex m_lockData;
	};

} // namespace srs_env_model


// SERVER_TOOLS_H_INCLUDED
#endif
