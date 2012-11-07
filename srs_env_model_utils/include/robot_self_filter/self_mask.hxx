/*
 * Copyright (c) 2008, Willow Garage, Inc.
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#include "robot_self_filter/self_mask.h"
#include <urdf/model.h>
#include <resource_retriever/retriever.h>
#include <geometric_shapes/shape_operations.h>
#include <ros/console.h>
#include <algorithm>
#include <sstream>
#include <climits>
#include <assimp/assimp.hpp>
#include <assimp/aiScene.h>
#include <assimp/aiPostProcess.h>

template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::freeMemory (void)
{
  for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
  {
    if (bodies_[i].body)
	    delete bodies_[i].body;
    if (bodies_[i].unscaledBody)
	    delete bodies_[i].unscaledBody;
  }

  bodies_.clear ();
}


namespace robot_self_filter
{
    static inline btTransform urdfPose2btTransform(const urdf::Pose &pose)
    {
      return btTransform(btQuaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w), btVector3(pose.position.x, pose.position.y, pose.position.z));
    }

    static shapes::Shape* constructShape(const urdf::Geometry *geom)
    {
      ROS_ASSERT(geom);

      shapes::Shape *result = NULL;
      switch (geom->type)
      {
        case urdf::Geometry::SPHERE:
        {
          result = new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(geom)->radius);
          break;
        }
        case urdf::Geometry::BOX:
        {
          urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
          result = new shapes::Box(dim.x, dim.y, dim.z);
          break;
        }
        case urdf::Geometry::CYLINDER:
        {
          result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder*>(geom)->radius,
                  dynamic_cast<const urdf::Cylinder*>(geom)->length);
          break;
        }
        case urdf::Geometry::MESH:
        {
          const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh*>(geom);
          if (!mesh->filename.empty())
          {
            btVector3 scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
            result = shapes::createMeshFromFilename(mesh->filename, &scale);
          } else
            ROS_WARN("Empty mesh filename");
          break;
        }
      default:
        {
          ROS_ERROR("Unknown geometry type: %d", (int)geom->type);
          break;
        }
      }
      return (result);
    }
}

template <typename tpCloudType>
bool robot_self_filter::SelfMask<tpCloudType>::configure(const std::vector<LinkInfo> &links)
{
  // in case configure was called before, we free the memory
  freeMemory();
  sensor_pos_.setValue(0, 0, 0);

  std::string content;
  boost::shared_ptr<urdf::Model> urdfModel;

  if (nh_.getParam("robot_description", content))
  {
    urdfModel = boost::shared_ptr<urdf::Model>(new urdf::Model());
    if (!urdfModel->initString(content))
    {
        ROS_ERROR("Unable to parse URDF description!");
        return false;
    }
  }
  else
  {
    ROS_ERROR("Robot model not found! Did you remap 'robot_description'?");
    return false;
  }

  std::stringstream missing;

  // from the geometric model, find the shape of each link of interest
  // and create a body from it, one that knows about poses and can
  // check for point inclusion
  for (unsigned int i = 0 ; i < links.size() ; ++i)
  {
    const urdf::Link *link = urdfModel->getLink(links[i].name).get();
    if (!link)
    {
      missing << " " << links[i].name;
      continue;
    }

    if (!(link->collision && link->collision->geometry))
    {
        ROS_WARN("No collision geometry specified for link '%s'", links[i].name.c_str());
        continue;
    }

    shapes::Shape *shape = constructShape(link->collision->geometry.get());

    if (!shape)
    {
        ROS_ERROR("Unable to construct collision shape for link '%s'", links[i].name.c_str());
        continue;
    }

    SeeLink sl;
    sl.body = bodies::createBodyFromShape(shape);

    if (sl.body)
    {
      sl.name = links[i].name;

      // collision models may have an offset, in addition to what TF gives
      // so we keep it around
      sl.constTransf = urdfPose2btTransform(link->collision->origin);

      sl.body->setScale(links[i].scale);
      sl.body->setPadding(links[i].padding);
            ROS_DEBUG_STREAM("Self see link name " <<  links[i].name << " padding " << links[i].padding);
      sl.volume = sl.body->computeVolume();
      sl.unscaledBody = bodies::createBodyFromShape(shape);
      bodies_.push_back(sl);
    }
    else
      ROS_WARN("Unable to create point inclusion body for link '%s'", links[i].name.c_str());

    delete shape;
  }

  if (missing.str().size() > 0)
    ROS_WARN("Some links were included for self mask but they do not exist in the model:%s", missing.str().c_str());

  if (bodies_.empty())
    ROS_WARN("No robot links will be checked for self mask");

  // put larger volume bodies first -- higher chances of containing a point
  std::sort(bodies_.begin(), bodies_.end(), SortBodies());

  bspheres_.resize(bodies_.size());
  bspheresRadius2_.resize(bodies_.size());

  for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
    ROS_DEBUG("Self mask includes link %s with volume %f", bodies_[i].name.c_str(), bodies_[i].volume);

  //ROS_INFO("Self filter using %f padding and %f scaling", padd, scale);

  return true;
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::getLinkNames(std::vector<std::string> &frames) const
{
  for (unsigned int i = 0 ; i < bodies_.size() ; ++i)
    frames.push_back(bodies_[i].name);
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::maskContainment(const tpCloudType& data_in, std::vector<int> &mask)
{
  mask.resize(data_in.points.size());
  if (bodies_.empty())
    std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
  else
  {
    assumeFrame(data_in.header.frame_id,data_in.header.stamp);
    maskAuxContainment(data_in, mask);
  }
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::maskIntersection(const tpCloudType& data_in, const std::string &sensor_frame, const double min_sensor_dist,
						   std::vector<int> &mask, const boost::function<void(const btVector3&)> &callback)
{
  mask.resize(data_in.points.size());
  if (bodies_.empty()) {
    std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
  }
  else
  {
    assumeFrame(data_in.header.frame_id, data_in.header.stamp, sensor_frame, min_sensor_dist);
    if (sensor_frame.empty())
        maskAuxContainment(data_in, mask);
    else
        maskAuxIntersection(data_in, mask, callback);
  }
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::maskIntersection(const tpCloudType& data_in, const btVector3 &sensor_pos, const double min_sensor_dist,
						   std::vector<int> &mask, const boost::function<void(const btVector3&)> &callback)
{
  mask.resize(data_in.points.size());
  if (bodies_.empty())
    std::fill(mask.begin(), mask.end(), (int)OUTSIDE);
  else
  {
    assumeFrame(data_in.header.frame_id, data_in.header.stamp, sensor_pos, min_sensor_dist);
    maskAuxIntersection(data_in, mask, callback);
  }
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::computeBoundingSpheres(void)
{
  const unsigned int bs = bodies_.size();
  for (unsigned int i = 0 ; i < bs ; ++i)
  {
    bodies_[i].body->computeBoundingSphere(bspheres_[i]);
    bspheresRadius2_[i] = bspheres_[i].radius * bspheres_[i].radius;
  }
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::assumeFrame(const std::string& frame_id, const ros::Time& stamp, const btVector3 &sensor_pos, double min_sensor_dist)
{
  assumeFrame(frame_id,stamp);
  sensor_pos_ = sensor_pos;
  min_sensor_dist_ = min_sensor_dist;
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::assumeFrame(const std::string& frame_id, const ros::Time& stamp, const std::string &sensor_frame, double min_sensor_dist)
{
  assumeFrame(frame_id,stamp);

  std::string err;
  if(!tf_.waitForTransform(frame_id, sensor_frame, stamp, ros::Duration(.1), ros::Duration(.01), &err)) {
    ROS_ERROR("WaitForTransform timed out from %s to %s after 100ms.  Error string: %s", sensor_frame.c_str(), frame_id.c_str(), err.c_str());
    sensor_pos_.setValue(0, 0, 0);
  }

  //transform should be there
  // compute the origin of the sensor in the frame of the cloud
  try
  {
    tf::StampedTransform transf;
    tf_.lookupTransform(frame_id, sensor_frame, stamp, transf);
    sensor_pos_ = transf.getOrigin();
  }
  catch(tf::TransformException& ex)
  {
    sensor_pos_.setValue(0, 0, 0);
    ROS_ERROR("Unable to lookup transform from %s to %s.  Exception: %s", sensor_frame.c_str(), frame_id.c_str(), ex.what());
  }

  min_sensor_dist_ = min_sensor_dist;
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::assumeFrame(const std::string &frame_id, const ros::Time &stamp)
{
  const unsigned int bs = bodies_.size();

  // place the links in the assumed frame
  for (unsigned int i = 0 ; i < bs ; ++i)
  {
    std::string err;
    if(!tf_.waitForTransform(frame_id, bodies_[i].name, stamp, ros::Duration(.1), ros::Duration(.01), &err)) {
      ROS_ERROR("WaitForTransform timed out from %s to %s after 100ms.  Error string: %s", bodies_[i].name.c_str(), frame_id.c_str(), err.c_str());
    }

    // find the transform between the link's frame and the pointcloud frame
    tf::StampedTransform transf;
    try
    {
      tf_.lookupTransform(frame_id, bodies_[i].name, stamp, transf);
    }
    catch(tf::TransformException& ex)
    {
      transf.setIdentity();
      ROS_ERROR("Unable to lookup transform from %s to %s. Exception: %s", bodies_[i].name.c_str(), frame_id.c_str(), ex.what());
    }

    // set it for each body; we also include the offset specified in URDF
    bodies_[i].body->setPose(transf * bodies_[i].constTransf);
    bodies_[i].unscaledBody->setPose(transf * bodies_[i].constTransf);
  }

  computeBoundingSpheres();
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::maskAuxContainment(const tpCloudType& data_in, std::vector<int> &mask)
{
    const unsigned int bs = bodies_.size();
    const unsigned int np = data_in.points.size();

    // compute a sphere that bounds the entire robot
    bodies::BoundingSphere bound;
    bodies::mergeBoundingSpheres(bspheres_, bound);
    btScalar radiusSquared = bound.radius * bound.radius;

    // we now decide which points we keep
    //#pragma omp parallel for schedule(dynamic)
    for (int i = 0 ; i < (int)np ; ++i)
    {
      btVector3 pt = btVector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
      int out = OUTSIDE;
      if (bound.center.distance2(pt) < radiusSquared)
          for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
        if (bodies_[j].body->containsPoint(pt))
            out = INSIDE;

      mask[i] = out;
    }
}


template <typename tpCloudType>
void robot_self_filter::SelfMask<tpCloudType>::maskAuxIntersection(const tpCloudType& data_in, std::vector<int> &mask, const boost::function<void(const btVector3&)> &callback)
{
  const unsigned int bs = bodies_.size();
  const unsigned int np = data_in.points.size();

  // compute a sphere that bounds the entire robot
  bodies::BoundingSphere bound;
  bodies::mergeBoundingSpheres(bspheres_, bound);
  btScalar radiusSquared = bound.radius * bound.radius;

  //std::cout << "Testing " << np << " points\n";

  // we now decide which points we keep
  //#pragma omp parallel for schedule(dynamic)
  for (int i = 0 ; i < (int)np ; ++i)
  {
    bool print = false;
    //if(i%100 == 0) print = true;
    btVector3 pt = btVector3(data_in.points[i].x, data_in.points[i].y, data_in.points[i].z);
    int out = OUTSIDE;

    // we first check is the point is in the unscaled body.
    // if it is, the point is definitely inside
    if (bound.center.distance2(pt) < radiusSquared)
      for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
        if (bodies_[j].unscaledBody->containsPoint(pt))
        {
          if(print)
          std::cout << "Point " << i << " in unscaled body part " << bodies_[j].name << std::endl;
        out = INSIDE;
        }

        // if the point is not inside the unscaled body,
        if (out == OUTSIDE)
        {
          // we check it the point is a shadow point
          btVector3 dir(sensor_pos_ - pt);
          btScalar  lng = dir.length();
          if (lng < min_sensor_dist_)
          {
            out = INSIDE;
            //std::cout << "Point " << i << " less than min sensor distance away\n";
          }
          else
          {
            dir /= lng;
            std::vector<btVector3> intersections;
            for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
            {
              if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
              {
                if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
                {
                  if (callback)
                    callback(intersections[0]);
                  out = SHADOW;
                  if(print) std::cout << "Point " << i << " shadowed by body part " << bodies_[j].name << std::endl;
                }
             }
		       }
           // if it is not a shadow point, we check if it is inside the scaled body
           if (out == OUTSIDE && bound.center.distance2(pt) < radiusSquared)
             for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
               if (bodies_[j].body->containsPoint(pt))
               {
                 if(print)
                   std::cout << "Point " << i << " in scaled body part " << bodies_[j].name << std::endl;
                 out = INSIDE;
               }
          }
        }
        mask[i] = out;
  }
}


template <typename tpCloudType>
int robot_self_filter::SelfMask<tpCloudType>::getMaskContainment(const btVector3 &pt) const
{
  const unsigned int bs = bodies_.size();
  int out = OUTSIDE;
  for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
    if (bodies_[j].body->containsPoint(pt))
	    out = INSIDE;
  return out;
}


template <typename tpCloudType>
int robot_self_filter::SelfMask<tpCloudType>::getMaskContainment(double x, double y, double z) const
{
  return getMaskContainment(btVector3(x, y, z));
}


template <typename tpCloudType>
int robot_self_filter::SelfMask<tpCloudType>::getMaskIntersection(const btVector3 &pt, const boost::function<void(const btVector3&)> &callback) const
{
  const unsigned int bs = bodies_.size();

  // we first check is the point is in the unscaled body.
  // if it is, the point is definitely inside
  int out = OUTSIDE;
  for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
    if (bodies_[j].unscaledBody->containsPoint(pt))
      out = INSIDE;

  if (out == OUTSIDE)
  {
    // we check it the point is a shadow point
    btVector3 dir(sensor_pos_ - pt);
    btScalar  lng = dir.length();
    if (lng < min_sensor_dist_)
        out = INSIDE;
    else
    {
      dir /= lng;

      std::vector<btVector3> intersections;
      for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
        if (bodies_[j].body->intersectsRay(pt, dir, &intersections, 1))
        {
          if (dir.dot(sensor_pos_ - intersections[0]) >= 0.0)
          {
            if (callback)
              callback(intersections[0]);
            out = SHADOW;
          }
        }

        // if it is not a shadow point, we check if it is inside the scaled body
        for (unsigned int j = 0 ; out == OUTSIDE && j < bs ; ++j)
          if (bodies_[j].body->containsPoint(pt))
            out = INSIDE;
    }
  }
  return (out);
}


template <typename tpCloudType>
int robot_self_filter::SelfMask<tpCloudType>::getMaskIntersection(double x, double y, double z, const boost::function<void(const btVector3&)> &callback) const
{
  return getMaskIntersection(btVector3(x, y, z), callback);
}
