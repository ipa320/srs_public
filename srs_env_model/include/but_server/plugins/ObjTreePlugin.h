/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Jan Gorig (xgorig01@stud.fit.vutbr.cz)
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

#ifndef OBJTREEPLUGIN_H
#define OBJTREEPLUGIN_H

#include <message_filters/subscriber.h>
#include <interactive_markers/interactive_marker_server.h>

#include <but_server/ServerTools.h>
#include <but_interaction_primitives/services_list.h>

#include <srs_env_model/InsertPlane.h>
#include <srs_env_model/InsertPlanes.h>
#include <srs_env_model/ShowObject.h>
#include <srs_env_model/RemoveObject.h>
#include <srs_env_model/ShowObjtree.h>

#include <objtree/octree.h>

namespace srs
{

class CObjTreePlugin : public CServerPluginBase
{
public:
    typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

	/// Constructor
	CObjTreePlugin(const std::string & name);

	/// Destructor
	virtual ~CObjTreePlugin();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);

    virtual void reset();

protected:
    //Services
    bool srvInsertPlane(srs_env_model::InsertPlane::Request &req, srs_env_model::InsertPlane::Response &res);
    bool srvInsertPlanes(srs_env_model::InsertPlanes::Request &req, srs_env_model::InsertPlanes::Response &res);
    bool srvShowObject(srs_env_model::ShowObject::Request &req, srs_env_model::ShowObject::Response &res);
    bool srvRemoveObject(srs_env_model::RemoveObject::Request &req, srs_env_model::RemoveObject::Response &res);
    bool srvShowObjtree(srs_env_model::ShowObjtree::Request &req, srs_env_model::ShowObjtree::Response &res);

    //Methods
    unsigned int insertPlane(const srs_env_model_msgs::PlaneDesc &plane);
    unsigned int insertBoundingBox(const geometry_msgs::Pose &pose, const geometry_msgs::Vector3 &scale);
    void showObject(unsigned int id);
    void removeObject(unsigned int id);
    void showObjtree();

    //Service servers
    ros::ServiceServer m_serviceInsertPlane;
    ros::ServiceServer m_serviceInsertPlanes;
    ros::ServiceServer m_serviceShowObject;
    ros::ServiceServer m_serviceRemoveObject;
    ros::ServiceServer m_serviceShowObjtree;
    
    //Service clients
    ros::ServiceClient m_clientAddPlane;

    ros::Publisher m_markerPub;

    objtree::Octree m_octree;

private:
    void publishLine(visualization_msgs::Marker &lines, float x1, float y1, float z1, float x2, float y2, float z2);
    void publishCube(visualization_msgs::Marker &lines, float x, float y, float z, float w, float h, float d);
    void publishOctree(const std::list<objtree::Box> &nodes);
};

}

#endif

