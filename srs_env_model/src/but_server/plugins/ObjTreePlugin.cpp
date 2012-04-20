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

#include <but_server/plugins/ObjTreePlugin.h>
#include <but_interaction_primitives/Plane.h>

#include <objtree/plane.h>
#include <objtree/filter.h>

#define IM_SERVER_FRAME_ID "/world"
#define IM_SERVER_TOPIC_NAME "BUT_ObjTree_IM_Server"

namespace srs
{

CObjTreePlugin::CObjTreePlugin(const std::string &name)
: CServerPluginBase(name)
{
}

CObjTreePlugin::~CObjTreePlugin()
{
}

void CObjTreePlugin::init(ros::NodeHandle &node_handle)
{
    //Advertise services
    m_serviceInsertPlane = node_handle.advertiseService("insert_plane2", &CObjTreePlugin::srvInsertPlane, this);
    m_serviceInsertPlanes = node_handle.advertiseService("insert_planes", &CObjTreePlugin::srvInsertPlanes, this);
    m_serviceShowObject = node_handle.advertiseService("show_object", &CObjTreePlugin::srvShowObject, this);
    m_serviceRemoveObject = node_handle.advertiseService("remove_object", &CObjTreePlugin::srvRemoveObject, this);
    m_serviceShowObjtree = node_handle.advertiseService("show_objtree", &CObjTreePlugin::srvShowObjtree, this);

    m_clientAddPlane = node_handle.serviceClient<srs_interaction_primitives::AddPlane>(BUT_AddPlane_SRV);

    m_markerPub = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 100);

    printf("ObjTree plugin initialized!\n");
}

void CObjTreePlugin::reset()
{
    m_octree.clear();
    //TODO: Clear existing objects in interactive primitives server
}

bool CObjTreePlugin::srvInsertPlane(srs_env_model::InsertPlane::Request &req, srs_env_model::InsertPlane::Response &res)
{
    res.object_id = insertPlane(req.plane);

    return true;
}

bool CObjTreePlugin::srvInsertPlanes(srs_env_model::InsertPlanes::Request &req, srs_env_model::InsertPlanes::Response &res)
{
    std::vector<srs_env_model_msgs::PlaneDesc>::iterator i;
    std::vector<srs_env_model_msgs::PlaneDesc> &planes(req.plane_array.planes);

    for(i = planes.begin(); i != planes.end(); i++)
    {
        unsigned int id = insertPlane(*i);
        res.object_ids.push_back(id);
    }

    return true;
}

bool CObjTreePlugin::srvShowObject(srs_env_model::ShowObject::Request &req, srs_env_model::ShowObject::Response &res)
{
    showObject(req.object_id);

    return true;
}

bool CObjTreePlugin::srvRemoveObject(srs_env_model::RemoveObject::Request &req, srs_env_model::RemoveObject::Response &res)
{
    removeObject(req.object_id);

    return true;
}

bool CObjTreePlugin::srvShowObjtree(srs_env_model::ShowObjtree::Request &req, srs_env_model::ShowObjtree::Response &res)
{
    showObjtree();

    return true;
}

/*bool CObjTreePlugin::srvListObjects(srs_env_model::ListObjects::Request &req, srs_env_model::ListObjects::Response &res)
{
    listObjects();

    return true;
}*/

unsigned int CObjTreePlugin::insertPlane(const srs_env_model_msgs::PlaneDesc &plane)
{
    printf("insertPlane called\n");

    objtree::Point pos(plane.pose.position.x, plane.pose.position.y, plane.pose.position.z);
    objtree::Vector normal(plane.pose.orientation.x, plane.pose.orientation.y, plane.pose.orientation.z);
    objtree::Point scale(plane.scale.x, plane.scale.y, plane.scale.z);

    return m_octree.insert(new objtree::Plane(pos, normal, scale));
}

unsigned int CObjTreePlugin::insertBoundingBox(const geometry_msgs::Pose &pose, const geometry_msgs::Vector3 &scale)
{
    printf("insertBoundingBox called\n");

    return 0;
}

void CObjTreePlugin::showObject(unsigned int id)
{
    const objtree::Object *object = m_octree.object(id);
    if(!object) return;

    char name[64];
    snprintf(name, sizeof(name), "imn%u", id);

    switch(object->type())
    {
        case objtree::Object::BOUNDING_BOX:
        break;

        case objtree::Object::PLANE:
        {
            objtree::Plane *plane = (objtree::Plane*)object;

            srs_interaction_primitives::AddPlane addPlaneSrv;

            addPlaneSrv.request.frame_id = IM_SERVER_FRAME_ID;
            addPlaneSrv.request.name = name;
            addPlaneSrv.request.description = name;

            addPlaneSrv.request.pose.position.x = plane->pos().x;
            addPlaneSrv.request.pose.position.y = plane->pos().y;
            addPlaneSrv.request.pose.position.z = plane->pos().z;
            addPlaneSrv.request.pose.orientation.x = plane->normal().x;
            addPlaneSrv.request.pose.orientation.y = plane->normal().y;
            addPlaneSrv.request.pose.orientation.z = plane->normal().z;

            addPlaneSrv.request.scale.x = plane->boundingMax().x-plane->boundingMin().x;
            addPlaneSrv.request.scale.y = plane->boundingMax().y-plane->boundingMin().y;
            addPlaneSrv.request.scale.z = plane->boundingMax().z-plane->boundingMin().z;

            addPlaneSrv.request.color.r = 1.0;
            addPlaneSrv.request.color.g = addPlaneSrv.request.color.b = 0.0;
            addPlaneSrv.request.color.a = 1.0;

            m_clientAddPlane.call(addPlaneSrv);
        }
        break;
    }
}

void CObjTreePlugin::removeObject(unsigned int id)
{
    m_octree.removeObject(id);
}

void CObjTreePlugin::showObjtree()
{
    std::set<objtree::Object*> objects;
    std::list<objtree::Box> nodes;
    objtree::FilterZero filter;

    m_octree.nodes(nodes, objects, &filter);

    publishOctree(nodes);

    printf("Number of objects %zd\n", objects.size());
}

//Methods for octree visualization

void CObjTreePlugin::publishLine(visualization_msgs::Marker &lines, float x1, float y1, float z1, float x2, float y2, float z2)
{
    geometry_msgs::Point p;

    p.x = x1;
    p.y = y1;
    p.z = z1;

    lines.points.push_back(p);

    p.x = x2;
    p.y = y2;
    p.z = z2;

    lines.points.push_back(p);
}

void CObjTreePlugin::publishCube(visualization_msgs::Marker &lines, float x, float y, float z, float w, float h, float d)
{
    publishLine(lines, x, y, z, x+w, y, z);
    publishLine(lines, x, y+h, z, x+w, y+h, z);
    publishLine(lines, x, y, z+d, x+w, y, z+d);
    publishLine(lines, x, y+h, z+d, x+w, y+h, z+d);

    publishLine(lines, x, y, z, x, y+h, z);
    publishLine(lines, x+w, y, z, x+w, y+h, z);
    publishLine(lines, x, y, z+d, x, y+h, z+d);
    publishLine(lines, x+w, y, z+d, x+w, y+h, z+d);

    publishLine(lines, x, y, z, x, y, z+d);
    publishLine(lines, x+w, y, z, x+w, y, z+d);
    publishLine(lines, x, y+h, z, x, y+h, z+d);
    publishLine(lines, x+w, y+h, z, x+w, y+h, z+d);
}

void CObjTreePlugin::publishOctree(const std::list<objtree::Box> &nodes)
{
    visualization_msgs::Marker lines;

    lines.header.frame_id = IM_SERVER_FRAME_ID;
    lines.header.stamp = ros::Time::now();
    lines.ns = "objtree";
    lines.action = visualization_msgs::Marker::ADD;
    lines.pose.orientation.w = 1.0f;

    lines.id = 0;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.color.r = lines.color.g = lines.color.b = 0.75f;
    lines.color.a = 1.0f;

    lines.scale.x = 0.03f;

    for(std::list<objtree::Box>::const_iterator i = nodes.begin(); i != nodes.end(); i++)
    {
        publishCube(lines, i->x, i->y, i->z, i->w, i->h, i->d);
    }

    m_markerPub.publish(lines);
}

}
