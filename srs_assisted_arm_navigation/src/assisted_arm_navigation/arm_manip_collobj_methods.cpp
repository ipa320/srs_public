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


std::string CArmManipulationEditor::add_coll_obj_bb(t_det_obj &obj) {

  std::string ret = "";

  ROS_INFO("Trying to add BB of detected object (%s) to collision map",obj.name.c_str());

  stringstream ss;

  ss << coll_obj_det.size();

  std::string oname = obj.name + "_" + ss.str();

  std_msgs::ColorRGBA color;

  color.r = 0;
  color.g = 0;
  color.b = 1.0;
  color.a = 0.25;

  bool transf = false;

  geometry_msgs::PoseStamped mpose;

  // TODO BUG -> problem with attached object, after executed trajectory, it stays on original place !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  std::string target = collision_objects_frame_id_;

  /*if (attached) {

	  // attached objects are stored in end effector frame -> we have to deal with them correctly
	  //MotionPlanRequestData& data = motion_plan_map_[getMotionPlanRequestNameFromId(mpr_id)];
	  target = end_eff_link_;

  }*/


  if (obj.pose.header.frame_id != target) {

	  /// We have to transform object to /base_link system
	  ros::Time now = ros::Time::now();
	  //geometry_msgs::PoseStamped pose_transf;

	  obj.pose.header.stamp = now; /// we need transformation for current time!

	  //std::string target = collision_objects_frame_id_;

	  ROS_INFO("Trying to transform detected object BB from %s to %s frame",obj.pose.header.frame_id.c_str(),target.c_str());

	  ROS_INFO("Waiting for transformation between %s and %s",obj.pose.header.frame_id.c_str(),target.c_str());

	  try {

			  if (tfl_->waitForTransform(target, obj.pose.header.frame_id, now, ros::Duration(2.0))) {

				tfl_->transformPose(target,obj.pose,mpose);

			  } else {

				  obj.pose.header.stamp = ros::Time(0);
				tfl_->transformPose(target,obj.pose,mpose);
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

	  ROS_INFO("BB of detected object is already in correct frame_id (%s). No need to transform.",obj.pose.header.frame_id.c_str());
	  mpose = obj.pose;
	  transf=true;

  }

  if (transf) {

  		 mpose.pose.position.z +=  obj.bb_lwh.z/2;

  		 ret = createCollisionObject(oname,
  							   mpose.pose,
  							   PlanningSceneEditor::Box,
  							   (obj.bb_lwh.x)*inflate_bb_,
  							   (obj.bb_lwh.y)*inflate_bb_,
  							   (obj.bb_lwh.z)*inflate_bb_,
  							   color,
  							   coll_objects_selectable_);



  		 if (obj.attached) {

  			attachCollisionObject(ret, end_eff_link_, links_);
  			ROS_INFO("Attached coll. obj. name=%s, id=%s has been created.",oname.c_str(),ret.c_str());

  		 } else {

  			ROS_INFO("Coll. obj. name=%s, id=%s has been created.",oname.c_str(),ret.c_str());

  		 }


  	   } // transf
  	   else {

  		 ROS_ERROR("Error on transforming - cannot add object %s to scene!",oname.c_str());

  	   }


   if (obj.allow_collision) {

	   ROS_INFO("Allowing collisions of gripper with object: %s (NOT IMPLEMENTED YET)", obj.name.c_str());
	   // TODO

   }

   // TODO handle somehow situation for more objects !!!!!
   // pregrasps are not allowed for attached objects (no sense there...)
   if (obj.allow_pregrasps && (!obj.attached) && ros::service::exists("/interaction_primitives/clickable_positions",true)) {

	   srs_interaction_primitives::ClickablePositions srv;

	   std_msgs::ColorRGBA c;
	   c.r = 0.0;
	   c.g = 0.0;
	   c.b = 1.0;
	   c.a = 0.6;

	   srv.request.color = c;
	   srv.request.frame_id = world_frame_;

	   std::vector<geometry_msgs::Point> points;

	   double offset = 0.3;

	   geometry_msgs::Point tmp;

	   tmp.x = mpose.pose.position.x + offset;
	   tmp.y = mpose.pose.position.y;
	   tmp.z = mpose.pose.position.z;

	   points.push_back(tmp);

	   tmp.x = mpose.pose.position.x - offset;
	   tmp.y = mpose.pose.position.y;
	   tmp.z = mpose.pose.position.z;

	   points.push_back(tmp);

	   tmp.x = mpose.pose.position.x;
	   tmp.y = mpose.pose.position.y + offset;
	   tmp.z = mpose.pose.position.z;

	   points.push_back(tmp);

	   tmp.x = mpose.pose.position.x;
	   tmp.y = mpose.pose.position.y - offset;
	   tmp.z = mpose.pose.position.z;

	   points.push_back(tmp);


	   srv.request.positions = points;
	   srv.request.radius = 0.05;
	   srv.request.topic_suffix = obj.name;

	   if (ros::service::call("/interaction_primitives/clickable_positions",srv)) {

		   obj.topic_name = srv.response.topic;

		   sub_click_ = nh_.subscribe(obj.topic_name,1,&CArmManipulationEditor::subClick,this);


	   } else {

		   ROS_ERROR("Can't add clickable positions for object (%s).",obj.name.c_str());
		   obj.topic_name = "";

	   }


   }


  return ret;

}

void CArmManipulationEditor::subClick(const srs_interaction_primitives::PositionClickedConstPtr &msg) {

	// TODO extract object name from topic to get object position...
	std::string topic = sub_click_.getTopic();

	sub_click_.shutdown();

	if (inited && !planned_) {


		ArmNavMovePalmLink::Request req;
		ArmNavMovePalmLink::Response resp;

		req.sdh_palm_link_pose.pose.position.x = msg->position.x;
		req.sdh_palm_link_pose.pose.position.y = msg->position.y;
		req.sdh_palm_link_pose.pose.position.z = msg->position.z;



		// TODO consider position of robot and position of point somehow...
		req.sdh_palm_link_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI/2.0,0.0,M_PI/2.0);

		req.sdh_palm_link_pose.header.frame_id = world_frame_;
		req.sdh_palm_link_pose.header.stamp = ros::Time::now();

		ArmNavMovePalmLink(req,resp);

	} else {

		ROS_WARN("Ops... We can't move gripper IM right now.");

	}

}

std::string CArmManipulationEditor::add_coll_obj_attached(double x, double y, double z, double scx, double scz) {

  std::string ret = "";
  ros::Time now = ros::Time::now();

  std::string target = collision_objects_frame_id_;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = aco_link_; //sdh_palm_link
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
                            color,
                            coll_objects_selectable_);

      ROS_INFO("Coll. obj. name=%s, id=%s",name.c_str(),ret.c_str());

      if (ret!="") {

        ROS_INFO("Attaching object to robot.");

        attachCollisionObject(ret,
                              aco_link_,
                              links_);

        //changeToAttached(ret);

      } else {

        ROS_ERROR("Gripper collision object was not created successfully");

      }

    } // if transf


    return ret;

}


