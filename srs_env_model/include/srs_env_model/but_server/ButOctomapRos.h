// $Id: ButOctomapROS.h 1669 2011-06-27 15:58:35Z hornunga@informatik.uni-freiburg.de $

/**
 * OctoMap ROS integration
 *
 * @author A. Hornung, University of Freiburg, Copyright (C) 2011.
 * @see http://www.ros.org/wiki/octomap_ros
 * License: BSD
 */

/*
 * Copyright (c) 2010, A. Hornung, University of Freiburg
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

/**
 * Default ros octomap uses octree as a non pointer attribute. It leads to some problems
 * with loading (load function returns pointer to the new octree and this octree cannot
 * replace stored one) and initialization (default r.o. does not implement declared
 * constructor wit tree as an parameter).
 */

#pragma once
#ifndef BUTOCTOMAP_ROS_H_
#define BUTOCTOMAP_ROS_H_


#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <tf/transform_datatypes.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <octomap/octomap.h>
#include <octomap_ros/conversions.h>

namespace octomap {


  /**
   * @brief ROS wrapper class for OctoMap Octrees, providing the most important
   * functionality with ROS / PCL types.
   * The class is templated over the Octree type. Any OcTree derived from
   * octomap::OccupancyOcTreeBase should work. For most cases, OcTreeROS
   * should work best for occupancy maps.
   *
   */
  template <class OctreeT>
  class ButOctomapROS {
  public:
    ButOctomapROS(double resolution);
    ButOctomapROS(OctreeT * tree);
    virtual ~ButOctomapROS() {}

    /**
     * @brief Integrate a Pointcloud measurement (ROS msg, in global reference frame)
     *
     * @param scan PointCloud2 of measurement endpoints
     * @param origin Origin of the scan. Any type of Point that has x,y,z member (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     * @param dirty whether the tree is left 'dirty' after the update (default: false).
     *   This speeds up the insertion by not updating inner nodes, but you need to call octree.updateInnerOccupancy() when done.
     */
    template <class PointT>
    void insertScan(const sensor_msgs::PointCloud2& scan, const PointT& origin,
        double maxrange=-1., bool pruning=true, bool dirty=false);

    /**
     * @brief Integrate a Pointcloud measurement (in global reference frame)
     *
     * @param scan pcl PointCloud of arbitrary type. Only the members x,y,z of the Points are used.
     * @param origin Origin of the scan. Any type of Point that has x,y,z members (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     *   This speeds up the insertion by not updating inner nodes, but you need to call octree.updateInnerOccupancy() when done.
     */
    template <class PCLPointT, class PointT>
    void insertScan(const pcl::PointCloud<PCLPointT>& scan, const PointT& origin,
        double maxrange=-1., bool pruning=true, bool dirty=false);

    /**
     * @brief Integrate a 3d scan in relative coordinates, transform scan before tree update
     *
     * @param pc Pointcloud (measurement endpoints) relative to frame origin
     * @param sensor_origin origin of sensor relative to frame origin
     * @param frame_origin_trans Translation of reference frame origin, determines transform to be applied to cloud and sensor origin
     * @param frame_origin_rot Rotation (as quaternion) of reference frame origin, determines transform to be applied to cloud and sensor origin
     * @param maxrange maximum range for how long individual beams are inserted (default -1: complete beam)
     * @param pruning whether the tree is (losslessly) pruned after insertion (default: true)
     *   This speeds up the insertion by not updating inner nodes, but you need to call octree.updateInnerOccupancy() when done.
     */
    template <class PCLPointT, class PCLQuaternionT>
    void insertScan(const pcl::PointCloud<PCLPointT>& scan, const PCLPointT& sensor_origin,
        const PCLPointT& frame_origin_trans, const PCLQuaternionT& frame_origin_rot,
        double maxrange=-1., bool pruning=true, bool dirty=false);




    /**
     * @brief Performs raycasting in 3d.
     *
     * A ray is cast from origin with a given direction, the first occupied
     * cell is returned (as center coordinate). If the starting coordinate is already
     * occupied in the tree, this coordinate will be returned as a hit.
     *
     * @param origin Any type of Point that has x,y,z members (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @param direction Any type of Point that has x,y,z members (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @param end Any type of Point that has x,y,z members (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @param ignoreUnknownCells whether unknown cells are ignored. If false (default), the raycast aborts when an unkown cell is hit.
     * @param maxRange Maximum range after which the raycast is aborted (<= 0: no limit, default)
     * @return whether or not an occupied cell was hit
     */
    template <class PointT>
    bool castRay(const PointT& origin, const PointT& direction, PointT& end,
        bool ignoreUnknownCells = false, double maxRange = -1.0) const;

    /**
     * @brief Insert one ray between origin and end into the tree.
     * The last node will be marked as occupied, all others on the ray as free.
     *
     *
     * @param origin Any type of Point that has x,y,z members (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @param end Any type of Point that has x,y,z members (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @param maxRange
     * @return
     */
    template <class PointT>
    bool insertRay(const PointT& origin, const PointT& end, double maxRange = -1.0);


    /**
     * @brief Search for a Node in the octree at a given coordinate.
     * You need to check if a node is found at all (returned pointer != NULL).
     *
     * @param point The searched coordinate. Any type of Point that has x,y,z members (e.g. pcl::PointXYZ or a geometry_msgs::Point) works.
     * @return Pointer to the octree node at the coordinate if it exists, NULL if it doesn't.
     */
    template <class PointT>
    typename OctreeT::NodeType* search(const PointT& point) const;

    typename OctreeT::NodeType* search(const tf::Point& point) const;
    typedef OctreeT OcTreeType; ///< Type of the wrapped octree
    typedef typename OctreeT::NodeType NodeType; ///< Type of the wrapped octree's nodes
    typedef boost::shared_ptr<OctreeT> OctreePtr;

    OctreeT & getTree() { return *octree; }

    void setTree(OctreeT * tree);

  protected:
    OctreePtr octree; ///< the wrapped OctoMap octree
  };

  /**
   * @brief The default octomap::OcTree wrapped in ROS.
   * OcTree provides a 3D occupancy map which stores log-odds in
   * OcTreeNodes of floats.
   *
   */
  typedef ButOctomapROS<OcTree> OcTreeROS;



  //
  // Implementation:
  //

  template <class OctreeT>
  ButOctomapROS<OctreeT>::ButOctomapROS(double resolution)
      : octree( new OctreeT(resolution) )
  {

  }

  template <class OctreeT>
    ButOctomapROS<OctreeT>::ButOctomapROS(OctreeT * tree)
        : octree( tree )
    {

    }

  template <class OctreeT>
      void ButOctomapROS<OctreeT>::setTree(OctreeT * tree)
      {
		  if( tree != 0 )
		  {
			  octree->clear();
			  octree = OctreePtr( tree );
		  }

      }

  template <class OctreeT>
  template <class PCLPointT, class PointT>
  void ButOctomapROS<OctreeT>::insertScan(const pcl::PointCloud<PCLPointT>& scan, const PointT& origin,
      double maxrange, bool pruning, bool dirty){

    Pointcloud pc;
    pointcloudPCLToOctomap(scan, pc);

    octree->insertScan(pc, point3d(origin.x, origin.y, origin.z), maxrange, pruning, dirty);

  }


  template <class OctreeT>
  template <class PCLPointT, class PCLQuaternionT>
  void ButOctomapROS<OctreeT>::insertScan(const pcl::PointCloud<PCLPointT>& scan, const PCLPointT& sensor_origin,
      const PCLPointT& frame_origin_trans, const PCLQuaternionT& frame_origin_rot, double maxrange, bool pruning, bool dirty)
    {

    Pointcloud pc;
    pointcloudPCLToOctomap(scan, pc);

    pose6d frame_origin;
    frame_origin.trans() = point3d(frame_origin_trans.x, frame_origin_trans.y, frame_origin_trans.z);
    frame_origin.rot() = octomath::Quaternion(frame_origin_rot.w(),
            frame_origin_rot.x(), frame_origin_rot.y(), frame_origin_rot.z());
    octree->insertScan(pc, point3d(sensor_origin.x, sensor_origin.y, sensor_origin.z), frame_origin,
        maxrange, pruning, dirty);

  }


  template <class OctreeT>
  template <class PointT>
  void ButOctomapROS<OctreeT>::insertScan(const sensor_msgs::PointCloud2& scan, const PointT& origin,
      double maxrange, bool pruning, bool dirty){


    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(scan, pc);
    insertScan(pc, origin, maxrange, pruning, dirty);

  }

  template <class OctreeT>
  template <class PointT>
  bool ButOctomapROS<OctreeT>::insertRay(const PointT& origin, const PointT& end, double maxRange){

    return octree->insertRay(point3d(origin.x, origin.y, origin.z),
        point3d(end.x, end.y, end.z),maxRange);

  }

  template <class OctreeT>
  template <class PointT>
  bool ButOctomapROS<OctreeT>::castRay(const PointT& origin, const PointT& direction,
      PointT& end, bool ignoreUnknownCells, double maxRange) const{

    point3d ptEnd;
    bool result =  octree->castRay(point3d(origin.x, origin.y, origin.z), point3d(direction.x, direction.y, direction.z),
        ptEnd, ignoreUnknownCells, maxRange);

    end.x = ptEnd.x();
    end.y = ptEnd.y();
    end.z = ptEnd.z();

    return result;
  }


  template <class OctreeT>
  template <class PointT>
  typename OctreeT::NodeType* ButOctomapROS<OctreeT>::search(const PointT& point) const{
    return octree->search(point.x, point.y, point.z);
  }

  template <class OctreeT>
  typename OctreeT::NodeType* ButOctomapROS<OctreeT>::search(const tf::Point& point) const{
    return octree->search(point.x(), point.y(), point.z());
  }

}

#endif /* BUTButOctomapROS_H_ */

