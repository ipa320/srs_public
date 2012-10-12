/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
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

#include <srs_assisted_arm_navigation/assisted_arm_navigation/arm_manip_node.h>

using namespace srs_assisted_arm_navigation;
using namespace planning_scene_utils;
using namespace srs_assisted_arm_navigation_msgs;


std::string CArmManipulationEditor::add_coll_obj_bb(std::string name, geometry_msgs::PoseStamped pose, geometry_msgs::Point bb_lwh, bool coll=false) {

  std::string ret = "";

  ROS_INFO("Trying to add BB of detected object (%s) to collision map",name.c_str());

  stringstream ss;

  ss << coll_obj_det.size();

  std::string oname = name + "_" + ss.str();

  std_msgs::ColorRGBA color;

  color.r = 0;
  color.g = 0;
  color.b = 1.0;
  color.a = 0.25;

  bool transf = false;

  geometry_msgs::PoseStamped mpose;

  if (pose.header.frame_id != collision_objects_frame_id_) {

	  /// We have to transform object to /base_link system
	  ros::Time now = ros::Time::now();
	  //geometry_msgs::PoseStamped pose_transf;

	  pose.header.stamp = now; /// we need transformation for current time!

	  std::string target = collision_objects_frame_id_;

	  ROS_INFO("Trying to transform detected object BB from %s to %s frame",pose.header.frame_id.c_str(),target.c_str());

	  ROS_INFO("Waiting for transformation between %s and %s",pose.header.frame_id.c_str(),target.c_str());

	  try {

			  if (tfl_->waitForTransform(target, pose.header.frame_id, now, ros::Duration(2.0))) {

				tfl_->transformPose(target,pose,mpose);

			  } else {

				pose.header.stamp = ros::Time(0);
				tfl_->transformPose(target,pose,mpose);
				ROS_WARN("Using latest transform available, may be wrong.");

			  }

			  transf = true;

		 }

		  // In case of absence of transformation path
		  catch(tf::TransformException& ex){
			 std::cerr << "Transform error: " << ex.what() << std::endl;
			 transf = false;
		  }



  } else {

	  ROS_INFO("BB of detected object is already in correct frame_id (%s). No need to transform.",pose.header.frame_id.c_str());
	  mpose = pose;
	  transf=true;

  }

  if (transf) {

  		 mpose.pose.position.z +=  bb_lwh.z/2;

  		 ret = createCollisionObject(oname,
  							   mpose.pose,
  							   PlanningSceneEditor::Box,
  							   (bb_lwh.x)*inflate_bb_,
  							   (bb_lwh.y)*inflate_bb_,
  							   (bb_lwh.z)*inflate_bb_,
  							   color);

  		 ROS_INFO("Coll. obj. name=%s, id=%s has been created.",oname.c_str(),ret.c_str());


  	   } // transf
  	   else {

  		 ROS_ERROR("Error on transforming - cannot add object %s to scene!",oname.c_str());

  	   }


   if (coll) {

	   ROS_INFO("Allowing collisions of gripper with object: %s (NOT IMPLEMENTED YET)", name.c_str());
	   // TODO

   }



  return ret;

}

std::string CArmManipulationEditor::add_coll_obj_attached(double x, double y, double z, double scx, double scz) {

  std::string ret = "";
  ros::Time now = ros::Time::now();

  std::string target = collision_objects_frame_id_;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "/arm_7_link"; //sdh_palm_link
  //pose.header.frame_id = "sdh_palm_link"; // pokus
  pose.header.stamp = now;
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  ROS_INFO("Trying to add coll. obj., frame=%s, x=%f,y=%f,z=%f",pose.header.frame_id.c_str(),x,y,z);

  bool transf = false;

  ROS_INFO("Waiting for transformation between %s and %s",pose.header.frame_id.c_str(),target.c_str());

  try {

        if (tfl_->waitForTransform(target, pose.header.frame_id, now, ros::Duration(2.0))) {

          tfl_->transformPose(target,pose,pose);

        } else {

          pose.header.stamp = ros::Time(0);
          tfl_->transformPose(target,pose,pose);
          ROS_WARN("Using latest transform available, may be wrong.");

        }

        transf = true;

   }

    // In case of absence of transformation path
    catch(tf::TransformException& ex){
       std::cerr << "Transform error: " << ex.what() << std::endl;
       transf = false;
    }


    if (transf) {

      ROS_INFO("Transformation succeeded");

      stringstream ss;

      ss << coll_obj_attached_id.size();

      std::string name = "gripper_co_" + ss.str();

      std_msgs::ColorRGBA color;

      color.r = 1.0;
      color.g = 0;
      color.b = 0;
      color.a = 0.25;

      ret = createCollisionObject(name,
                            pose.pose,
                            PlanningSceneEditor::Cylinder,
                            scx,
                            0,
                            scz,
                            color);

      ROS_INFO("Coll. obj. name=%s, id=%s",name.c_str(),ret.c_str());

      if (ret!="") {

        ROS_INFO("Attaching object to robot.");

        attachCollisionObject(ret,
                              "arm_7_link",
                              links_);

        //changeToAttached(ret);

      } else {

        ROS_ERROR("Gripper collision object was not created successfully");

      }

    } // if transf


    return ret;

}


