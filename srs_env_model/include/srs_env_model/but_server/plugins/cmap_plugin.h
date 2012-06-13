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
#ifndef CMapPubPlugin_H_included
#define CMapPubPlugin_H_included

#include <srs_env_model/but_server/server_tools.h>
#include <srs_env_model/GetCollisionMap.h>
#include <srs_env_model/IsNewCollisionMap.h>

#include <arm_navigation_msgs/CollisionMap.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

namespace srs_env_model
{

    class CCMapPlugin : public CServerPluginBase, public COctomapCrawlerBase<tButServerOcTree::NodeType>, public CDataHolderBase< arm_navigation_msgs::CollisionMap >
    {
    protected:
        typedef COctomapCrawlerBase<tButServerOcTree::NodeType> tOctomapCrawler;

    public:
        //! Constructor
        CCMapPlugin(const std::string & name);

        //! Enable or disable publishing
        void enable( bool enabled ){ m_publishCollisionMap = enabled; }

        //! Initialize plugin - called in server constructor
        virtual void init(ros::NodeHandle & node_handle);

        //! Called when new scan was inserted and now all can be published
        virtual void onPublish(const ros::Time & timestamp);

        //! Set used octomap frame id and timestamp
        virtual void onFrameStart( const SMapParameters & par );

        /// hook that is called when traversing occupied nodes of the updated Octree (does nothing here)
        virtual void handleOccupiedNode(tButServerOcTree::iterator& it, const SMapParameters & mp);

        /// Is something to publish and some subscriber to publish to?
        virtual bool shouldPublish(  );


    protected:
        //! Compare two collision maps and test if they are the same
        bool sameCMaps( arm_navigation_msgs::CollisionMap * map1, arm_navigation_msgs::CollisionMap * map2 );

        //! Test collision point if it is in the collision distance from the robot
        bool isNearRobot( const btVector3 & point, double extent );

        /**
        * @brief Get collision map service call
        *
        * @param req request - caller's map version
        * @param res response - current map and current version
        */
        bool getCollisionMapSrvCallback( srs_env_model::GetCollisionMap::Request & req, srs_env_model::GetCollisionMap::Response & res );

        /**
         * @brief Get true if given timestamp is older then current map time
         * @param req request - caller's map timestamp
         * @param res response - true, if new map and current timestamp
         */
        bool isNewCmapSrvCallback( srs_env_model::IsNewCollisionMap::Request & req, srs_env_model::IsNewCollisionMap::Response & res );

    protected:
        //! Collision map publisher name
        std::string m_cmapPublisherName;

        //! Publisher
        ros::Publisher m_cmapPublisher;

        //! Collision map distance limit
        double m_collisionMapLimitRadius;

        //! Collision map version counter
        long int m_collisionMapVersion;

        //! Robot position in the octomap coordinate system
        tf::Stamped<btVector3> m_robotBasePosition;

        //! Collision map frame id
        std::string m_cmapFrameId;

        /// Collision map message buffer - used to resolve if collision map has changed.
        arm_navigation_msgs::CollisionMap * m_dataBuffer;

        /// Empty collision map - used when callers map id is the same as the current map id
        arm_navigation_msgs::CollisionMap m_dataEmpty;

        //! Is publishing enabled?
        bool m_publishCollisionMap;

        //! Get current collision map service
        ros::ServiceServer m_serviceGetCollisionMap;

        //! Is new cmap service
        ros::ServiceServer m_serviceIsNewCMap;

        //! Transform listener
        tf::TransformListener m_tfListener;

        /// World to collision map transform matrix
        Eigen::Matrix4f m_worldToCMapTM;

        /// World to cmap translation and rotation
        Eigen::Matrix3f m_worldToCMapRot;
        Eigen::Vector3f m_worldToCMapTrans;

        //
        bool m_latchedTopics;

        /// Does need input point to be converted?
        bool m_bConvertPoint;

        /// Current cmap timestamp
        ros::Time m_mapTime;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; // class CCollisionMapPublisher

    /// Declare holder object - partial specialization of the default holder with predefined connection settings
    template< class tpOctomapPlugin >
    struct SCMapPluginHolder : public  CCrawlingPluginHolder< CCMapPlugin, tpOctomapPlugin >
    {
    protected:
        /// Define holder type
        typedef CCrawlingPluginHolder< CCMapPlugin, tpOctomapPlugin > tCMPHolder;

    public:
        /// Create holder
        SCMapPluginHolder( const std::string & name )
        : tCMPHolder(  name,  tCMPHolder::ON_START | tCMPHolder::ON_OCCUPIED )
        {

        }

    }; // struct SMarkerArrayHolder

}

// CCMapPubPlugin_H_included
#endif


