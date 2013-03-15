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

#include "srs_user_tests/bb_overlap.h"

using namespace srs_user_tests;


BBOverlap::BBOverlap() {

			ros::NodeHandle nh("~");

			id_.pub = nh.advertise<visualization_msgs::Marker>("ideal_bb_pose",1);
			gripper_pub_ = nh.advertise<visualization_msgs::Marker>("gripper_target_pose",1);


			srv_move_ = nh.advertiseService("move_bb", &BBOverlap::moveX,this);

			im_.pose_rec = false;
			im_.scale_rec = false;
			
			gr_suc_ = false;
			bb_suc_ = false;

			ros::param::param("~publish_debug_markers",publish_debug_markers_,true);

			if (publish_debug_markers_) {

			  id_.points_pub = nh.advertise<visualization_msgs::Marker>("ideal_bb_points",1);
			  im_.points_pub = nh.advertise<visualization_msgs::Marker>("actual_bb_points",1);
			  points_proc_pub_ = nh.advertise<visualization_msgs::Marker>("proc_points",1);

			}

			ros::param::param("~bb/lwh/x",id_.bb.lwh.x,0.0);
			ros::param::param("~bb/lwh/y",id_.bb.lwh.y,0.0);
			ros::param::param("~bb/lwh/z",id_.bb.lwh.z,0.0);

			ros::param::param("~bb/position/x",id_.bb.pose.position.x,0.0);
			ros::param::param("~bb/position/y",id_.bb.pose.position.y,0.0);
			ros::param::param("~bb/position/z",id_.bb.pose.position.z,0.0);

			ros::param::param("~bb/orientation/x",id_.bb.pose.orientation.x,0.0);
			ros::param::param("~bb/orientation/y",id_.bb.pose.orientation.y,0.0);
			ros::param::param("~bb/orientation/z",id_.bb.pose.orientation.z,0.0);
			ros::param::param("~bb/orientation/w",id_.bb.pose.orientation.w,1.0);

			ros::param::param("~gripper/position/x",gripper_pose_.position.x,0.0);
			ros::param::param("~gripper/position/y",gripper_pose_.position.y,0.0);
			ros::param::param("~gripper/position/z",gripper_pose_.position.z,0.0);

			// relative to absolute position
			gripper_pose_.position.x += id_.bb.pose.position.x;
			gripper_pose_.position.y += id_.bb.pose.position.y;
			gripper_pose_.position.z += id_.bb.pose.position.z;
			//gripper_pose_.position.z += id_.bb.lwh.z/2.0;

			double tmp;
			ros::param::param("~bb/success_min_dur",tmp,2.0);
			bb_success_min_dur_ = ros::Duration(tmp);

			ros::param::param("~gr/success_min_dur",tmp,2.0);
			gr_success_min_dur_ = ros::Duration(tmp);

			ros::param::param("~bb/success_val",tmp,0.8);
			bb_success_val_ = tmp;

			ros::param::param("~gr/success_val",tmp,0.05);
			gr_success_val_ = tmp;

			bb_success_first_ = ros::Time(0);
			bb_success_last_ = ros::Time(0);
			bb_success_tmp_ = ros::Time(0);

			gr_success_first_ = ros::Time(0);
			gr_success_last_ = ros::Time(0);
			gr_success_tmp_ = ros::Time(0);

			// TODO read this (+ pose) from params....
			/*id_.bb.lwh.x = 0.2; // length
			id_.bb.lwh.y = 0.1; // width
			id_.bb.lwh.z = 0.4; // height*/

			/*id_.bb.pose.position.x = -0.15;
			id_.bb.pose.position.y = 0.6;
			id_.bb.pose.position.z = 0.5;*/

			/*id_.bb.pose.orientation.x = -8.79007958941e-07;
			id_.bb.pose.orientation.y = 3.27846130193e-07;
			id_.bb.pose.orientation.z = 0.429966424682;
			id_.bb.pose.orientation.w = 0.90284486721;*/

			id_.vol = id_.bb.lwh.x * id_.bb.lwh.y * id_.bb.lwh.z;


			id_.marker.header.frame_id = "/map";

			id_.marker.pose.position = id_.bb.pose.position;
			id_.marker.pose.position.z += id_.bb.lwh.z/2.0;

			id_.marker.pose.orientation = id_.bb.pose.orientation;

			id_.marker.color.g = 1.0;
			id_.marker.color.a = 0.6;

			id_.marker.type = visualization_msgs::Marker::CUBE;
			id_.marker.scale.x = id_.bb.lwh.x*2.0;
			id_.marker.scale.y = id_.bb.lwh.y*2.0;
			id_.marker.scale.z = id_.bb.lwh.z;


			gripper_marker_.color.r = 1.0;
			gripper_marker_.color.g = 140.0/255.0;
			gripper_marker_.color.b = 0.0;
			gripper_marker_.color.a = 0.6;
			gripper_marker_.header.frame_id = "/map";
			gripper_marker_.pose = gripper_pose_;
			gripper_marker_.type = visualization_msgs::Marker::SPHERE;
			gripper_marker_.scale.x = 0.05;
			gripper_marker_.scale.y = 0.05;
			gripper_marker_.scale.z = 0.05;
			gripper_marker_.lifetime = ros::Duration(1.5);

			// generate eight points for ideal position
			update_points(id_.bb,id_.points);

			sub_im_feedback_ = nh.subscribe<visualization_msgs::InteractiveMarkerFeedback>("/interaction_primitives/feedback",1,&BBOverlap::im_feedback_cb,this);
			sub_im_scale_ = nh.subscribe<srs_interaction_primitives::ScaleChanged>("/interaction_primitives/unknown_object/update/scale_changed",1,&BBOverlap::im_scale_cb,this);
			sub_gripper_update_ = nh.subscribe<visualization_msgs::InteractiveMarkerUpdate>("/planning_scene_warehouse_viewer_controls/update",1,&BBOverlap::gripper_im_cb,this);

			arm_state_ok_ = false;
			sub_arm_state_ = nh.subscribe<srs_assisted_arm_navigation_msgs::AssistedArmNavigationState>("/but_arm_manip/state",1,&BBOverlap::arm_nav_state_cb,this);


			timer_ = nh.createTimer(ros::Duration(0.05),&BBOverlap::timer_cb,this);

			ROS_INFO("Initialized.");

		}

bool BBOverlap::moveX(SetFloat::Request& req, SetFloat::Response& res) {

	bool update_marker = false;


	if (req.axis=="x") {

		id_.bb.pose.position.x += req.v;
		update_marker = true;

	}

	if (req.axis=="y") {

		id_.bb.pose.position.y += req.v;
		update_marker = true;

	}

	if (req.axis=="z") {

		tf::Quaternion rot = tf::createQuaternionFromRPY(0.0,0.0,req.v);

		tf::Quaternion g; // current IM marker pose
		tf::quaternionMsgToTF(id_.bb.pose.orientation, g);

		g = rot*g;
		tf::quaternionTFToMsg(g,id_.bb.pose.orientation);
		update_marker = true;

	}

	if (update_marker) {

		id_.marker.pose.position = id_.bb.pose.position;
		id_.marker.pose.position.z += id_.bb.lwh.z/2.0;

		id_.marker.pose.orientation = id_.bb.pose.orientation;

		return true;

	}

	ROS_WARN("Use x,y or z as axis!");
	return false;

}


void BBOverlap::arm_nav_state_cb(const srs_assisted_arm_navigation_msgs::AssistedArmNavigationStateConstPtr& msg) {

	ROS_INFO_ONCE("Assisted arm navigation state received.");
	arm_state_ok_ = msg->position_reachable;

}

void BBOverlap::update_points(tbb bb, tpoints &pp) {

	pp.clear();

	tpoint p(0.0, 0.0, 0.0);

	Eigen::Quaternionf q(bb.pose.orientation.w, bb.pose.orientation.x, bb.pose.orientation.y, bb.pose.orientation.z);
	Eigen::Affine3f tr(q);

	Eigen::Quaternionf q1(id_.bb.pose.orientation.w, id_.bb.pose.orientation.x, id_.bb.pose.orientation.y, id_.bb.pose.orientation.z);
	Eigen::Affine3f tr2(q1.inverse());

	// 1
	p(0) = bb.lwh.x;
    p(1) = bb.lwh.y;
    p(2) = -0.5 * bb.lwh.z;
    p = tr*p;
    p = tr2*p;
    p(0) += bb.pose.position.x;
    p(1) += bb.pose.position.y;
    p(2) += bb.pose.position.z;
    p(2) += 0.5 * bb.lwh.z;
    pp.push_back(p);

    // 2
    p(0) = bb.lwh.x;
    p(1) = - bb.lwh.y;
    p(2) = -0.5 * bb.lwh.z;
    p = tr*p;
    p = tr2*p;
    p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += 0.5 * bb.lwh.z;
	pp.push_back(p);

	// 3
	p(0) = - bb.lwh.x;
	p(1) = + bb.lwh.y;
	p(2) = -0.5 * bb.lwh.z;
	p = tr*p;
	p = tr2*p;
	p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += 0.5 * bb.lwh.z;
    pp.push_back(p);

    // 4
	p(0) = - bb.lwh.x;
	p(1) = - bb.lwh.y;
	p(2) = -0.5 * bb.lwh.z;
	p = tr*p;
	p = tr2*p;
	p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += 0.5 * bb.lwh.z;
	pp.push_back(p);

	// 5
	p(0) = bb.lwh.x;
    p(1) = bb.lwh.y;
    p(2) = 0.5*bb.lwh.z;
    p = tr*p;
    p = tr2*p;
    p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += bb.lwh.z*0.5;
    pp.push_back(p);

    // 6
	p(0) = bb.lwh.x;
	p(1) = - bb.lwh.y;
	p(2) = 0.5*bb.lwh.z;
	p = tr*p;
	p = tr2*p;
	p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += bb.lwh.z*0.5;
	pp.push_back(p);

	// 7
	p(0) = - bb.lwh.x;
    p(1) = bb.lwh.y;
    p(2) = 0.5*bb.lwh.z;
    p = tr*p;
    p = tr2*p;
    p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += bb.lwh.z*0.5;
    pp.push_back(p);

    // 8
	p(0) = - bb.lwh.x;
	p(1) = - bb.lwh.y;
	p(2) = 0.5*bb.lwh.z;
	p = tr*p;
	p = tr2*p;
	p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += bb.lwh.z*0.5;
	pp.push_back(p);

}

void BBOverlap::gripper_im_cb(const visualization_msgs::InteractiveMarkerUpdateConstPtr& msg) {

	if (msg->poses.size()==1 && msg->poses[0].name=="MPR 0_end_control") {

		tfl_.waitForTransform("/map",msg->poses[0].header.frame_id,ros::Time(0),ros::Duration(5.0));

		geometry_msgs::PoseStamped tmp;

		tmp.header.stamp = ros::Time(0);
		tmp.header.frame_id = msg->poses[0].header.frame_id;
		tmp.pose = msg->poses[0].pose;

		try {

			tfl_.transformPose("/map",tmp,tmp);

		} catch(tf::TransformException& ex){

		   std::cerr << "Transform error: " << ex.what() << std::endl;

		   ROS_ERROR("Exception on TF transf.: %s",ex.what());

		   return;
		}

		ROS_INFO_ONCE("First gripper pose received.");
		gripper_pose_rec_ = true;
		gripper_pose_curr_ = tmp.pose;

	}

}

void BBOverlap::publish_points(tpoints points, ros::Publisher &pub, std_msgs::ColorRGBA c) {

	if (!publish_debug_markers_) return;

	visualization_msgs::Marker m;

	m.header.frame_id = "/map";
	m.header.stamp = ros::Time::now();
	m.type = visualization_msgs::Marker::SPHERE_LIST;
	m.action = visualization_msgs::Marker::ADD;
	m.color = c;
	m.lifetime = ros::Duration(1.5);

	m.scale.x = 0.05;
	m.scale.y = 0.05;
	m.scale.z = 0.05;

	m.pose.orientation.x = 0.0;
	m.pose.orientation.y = 0.0;
	m.pose.orientation.z = 0.0;
	m.pose.orientation.w = 1.0;

	for (unsigned int i=0; i < points.size(); i++) {

		geometry_msgs::Point p;

		p.x = points[i](0);
		p.y = points[i](1);
		p.z = points[i](2);

		m.points.push_back(p);

	}

	if (pub.getNumSubscribers() > 0) {

		pub.publish(m);
		ROS_INFO_ONCE("Publishing %d spheres.",(int)m.points.size());
	}

}

/*int BBOverlap::getch()
{
    int r;
    unsigned char c;
    if ((r = read(0, &c, sizeof(c))) < 0) {
        return r;
    } else {
        return c;
    }
}*/


BBOverlap::~BBOverlap() {

}


void BBOverlap::im_feedback_cb(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg) {


	if (msg->marker_name == "unknown_object") {

		ROS_INFO_ONCE("IM feedback received");

		boost::mutex::scoped_lock(im_.mutex);

		im_.bb.pose = msg->pose;
		im_.bb.pose.position.z -= im_.bb.lwh.z/2.0;

		im_.pose_rec = true;

	}

}

// computes volume for AABB
double BBOverlap::points_volume(const tpoints &p) {

	double x = fabs((double)p[4](0) - (double)p[3](0));
	double y = fabs((double)p[4](1) - (double)p[3](1));
	double z = fabs((double)p[4](2) - (double)p[3](2));

	return x*y*z;

	//return fabs( (double)p[4](0)*(double)p[4](1)*(double)p[4](2) );

}


void BBOverlap::im_scale_cb(const srs_interaction_primitives::ScaleChangedConstPtr& msg) {

	ROS_INFO_ONCE("Scale received");

	boost::mutex::scoped_lock(im_.mutex);

	im_.bb.lwh = msg->new_scale;
	im_.bb.lwh.x *= 0.5;
	im_.bb.lwh.y *= 0.5;

	im_.scale_rec = true;

}

double BBOverlap::rmin(double val1, double val2) {

	if (val1<val2) return val1;
	else return val2;

}

double BBOverlap::rmax(double val1, double val2) {

	if (val1>val2) return val1;
	else return val2;

}


void BBOverlap::timer_cb(const ros::TimerEvent&) {



	ROS_INFO_ONCE("Timer triggered");

	boost::mutex::scoped_lock(im_.mutex);

	if (id_.pub.getNumSubscribers() > 0) {

		ROS_INFO_ONCE("Publishing BB ideal position.");

		id_.marker.header.stamp = ros::Time::now();
		id_.pub.publish(id_.marker);

	}

	bool suc = false;
	double overlap = 0.0;
	ros::Time now = ros::Time::now();

	// we have received something...
	if (im_.pose_rec && im_.scale_rec) {


		geometry_msgs::Pose pos = im_.bb.pose;

		Eigen::Quaternionf q1(id_.bb.pose.orientation.w, id_.bb.pose.orientation.x, id_.bb.pose.orientation.y, id_.bb.pose.orientation.z);
		Eigen::Affine3f tr2(q1.inverse());

		tpoint p(pos.position.x, pos.position.y, pos.position.z);

		p = tr2*p;

		pos.position.x = p(0);
		pos.position.y = p(1);
		pos.position.z = p(2);

		tbb tmp;

		tmp.pose = pos;
		tmp.lwh = im_.bb.lwh;

		// generate new points
		update_points(tmp,im_.points);


		// take ideal and actual points in /map coord. system
		tpoints id_p = id_.points;
		tpoints im_p = im_.points;

		tpoint p2(id_.bb.pose.position.x, id_.bb.pose.position.y, id_.bb.pose.position.z);

		p2 = tr2*p2;

		// shift it to 0,0
		for (unsigned int i=0; i<im_p.size(); i++) {

			id_p[i](0) -= id_.bb.pose.position.x;
			id_p[i](1) -= id_.bb.pose.position.y;
			id_p[i](2) -= id_.bb.pose.position.z;

			im_p[i](0) -= p2(0);
			im_p[i](1) -= p2(1);
			im_p[i](2) -= p2(2);

		}

		double pdx = id_p[0](0);
		double pdy = id_p[0](1);
		double pdz = id_p[0](2);


		// correction - shift one corner to 0,0
		for (unsigned int i=0; i<im_p.size(); i++) {

			if (id_p[i](0) < pdx) pdx = id_p[i](0);
			if (id_p[i](1) < pdy) pdy = id_p[i](1);
			if (id_p[i](2) < pdz) pdz = id_p[i](2);

		}

		/*tpoint p3(dx,dy,dz);

		p3 = tr2*p3;*/

		for (unsigned int i=0; i<im_p.size(); i++) {

			id_p[i](0) -= pdx;
			id_p[i](1) -= pdy;
			id_p[i](2) -= pdz;

			im_p[i](0) -= pdx;
			im_p[i](1) -= pdy;
			im_p[i](2) -= pdz;

		}

		if (points_volume(id_p) <= 0.0) {

			ROS_ERROR("ID vol negative or zero!");

		}

		//publish_points(tpoints points, ros::Publisher &pub, std_msgs::ColorRGBA c)
		std_msgs::ColorRGBA c;
		c.a = 0.6;
		c.g = 1.0;

		publish_points(id_p, id_.points_pub, c);

		c.g = 0.0;
		c.b = 1.0;
		publish_points(im_p, im_.points_pub, c);

		// axis aligned
		tpoints imaa_p = im_p;

		double min_x = imaa_p[0](0);
		double max_x = imaa_p[0](0);

		double min_y = imaa_p[0](1);
		double max_y = imaa_p[0](1);

		double min_z = imaa_p[0](2);
		double max_z = imaa_p[0](2);

       /* 			x               y               z
		idp(0)=[0.400000,0.200000,0.000000]
		idp(1)=[0.400000,0.000000,0.000000]
		idp(2)=[0.000000,0.200000,0.000000]
		idp(3)=[0.000000,0.000000,0.000000]
		idp(4)=[0.400000,0.200000,0.400000]
		idp(5)=[0.400000,0.000000,0.400000]
		idp(6)=[0.000000,0.200000,0.400000]
		idp(7)=[0.000000,0.000000,0.400000]*/

		for (unsigned int i=0; i<im_p.size(); i++) {

			if (min_x > imaa_p[i](0)) min_x =  imaa_p[i](0);
			if (max_x < imaa_p[i](0)) max_x =  imaa_p[i](0);

			if (min_y > imaa_p[i](1)) min_y =  imaa_p[i](1);
			if (max_y < imaa_p[i](1)) max_y =  imaa_p[i](1);

			if (min_z > imaa_p[i](2)) min_z =  imaa_p[i](2);
			if (max_z < imaa_p[i](2)) max_z =  imaa_p[i](2);

		}

		imaa_p[0] = tpoint(max_x,max_y,min_z);
		imaa_p[1] = tpoint(max_x,min_y,min_z);
		imaa_p[2] = tpoint(min_x,max_y,min_z);
		imaa_p[3] = tpoint(min_x,min_y,min_z);
		imaa_p[4] = tpoint(max_x,max_y,max_z);
		imaa_p[5] = tpoint(max_x,min_y,max_z);
		imaa_p[6] = tpoint(min_x,max_y,max_z);
		imaa_p[7] = tpoint(min_x,min_y,max_z);

		c.g = 0.5;
		c.b = 0.5;
		publish_points(imaa_p, points_proc_pub_, c);


		//tpoints ov_p;

		double ov_p_vol = 0.0;

		double dx = 0.0;
		double dy = 0.0;
		double dz = 0.0;

		// compute "overlap bb"
		if ( (max_z > 0.0 && min_z < id_p[4](2)) && (max_x > 0.0 && min_x <  id_p[4](0)) && (max_y > 0.0 && min_y <  id_p[4](1)) ) {

			dx = rmin((double)id_p[0](0),(double)imaa_p[0](0)) - rmax((double)id_p[2](0),(double)imaa_p[2](0));

			dy = rmin((double)id_p[0](1),(double)imaa_p[0](1)) - rmax((double)id_p[1](1),(double)imaa_p[1](1));

			dz = rmin((double)id_p[4](2),(double)imaa_p[4](2)) - rmax((double)id_p[0](2),(double)imaa_p[0](2));

			// x*y*z
			ov_p_vol = dx*dy*dz;

		};

		double max_v = rmax(points_volume(id_p),points_volume(imaa_p));

		overlap = ov_p_vol / max_v;

		if (overlap > 1.0) overlap = 1.0; // just for a case...


		if (overlap > bb_success_val_) {

			if (bb_success_tmp_ == ros::Time(0)) bb_success_tmp_ = now;

			if ((now - bb_success_tmp_) > ros::Duration(bb_success_min_dur_)) {

				if (bb_success_first_ == ros::Time(0)) bb_success_first_ = now;
				
				suc = true;

				if (bb_suc_ == false) {
					
				  bb_success_last_ = now;
				  bb_suc_ = true;	

				}

			}


		} else {

			bb_success_tmp_ = ros::Time(0);
			bb_suc_ = false;

		}


	} // if we have something

	double gripper_dist = -1.0;

	bool gr_suc = false;

	if (gripper_pose_rec_) {

		double tmp;

		tmp = pow(gripper_pose_.position.x - gripper_pose_curr_.position.x,2.0)
				+ pow(gripper_pose_.position.y - gripper_pose_curr_.position.y,2.0)
				+ pow(gripper_pose_.position.z - gripper_pose_curr_.position.z,2.0);

		gripper_dist = pow(fabs(tmp),0.5);

		if ( (gripper_dist < gr_success_val_) && arm_state_ok_) {

			if (gr_success_tmp_ == ros::Time(0)) gr_success_tmp_ = now;

				if ((now - gr_success_tmp_) > ros::Duration(gr_success_min_dur_)) {

					if (gr_success_first_ == ros::Time(0)) gr_success_first_ = now;
					
					gr_suc = true;
					
					if (gr_suc_ == false) {
					
				  gr_success_last_ = now;
				  gr_suc_ = true;	

				}

				}



		} else {
		
		  
		  gr_success_tmp_ = ros::Time(0);
		  gr_suc_ = false;
		  
		  }

	}

	if ( (ros::Time::now() - last_log_out_) > ros::Duration(2.0) ) {

		//ROS_INFO("ID vol: %f, MAA vol: %f",points_volume(id_p),points_volume(imaa_p));

		//ROS_INFO("dx: %f, dy: %f, dz: %f, vol: %f",dx,dy,dz,ov_p_vol);
		//ROS_INFO("overlap: %f%% (%f / %f)",overlap*100.0,ov_p_vol,max_v);

		/*boost::posix_time::ptime f_s = bb_success_first_.toBoost();
		boost::posix_time::ptime l_s = bb_success_last_.toBoost();

		boost::posix_time::ptime f_gr = gr_success_first_.toBoost();
		boost::posix_time::ptime l_gr = gr_success_last_.toBoost();*/

		if (suc) printf("BB OVERLAP: %03.1f%% (success)\n",overlap*100);
		else printf("BB OVERLAP: %03.1f%% (fail)\n",overlap*100);

		double gdx = gripper_pose_.position.x - gripper_pose_curr_.position.x;
		double gdy = gripper_pose_.position.y - gripper_pose_curr_.position.y;
		double gdz = gripper_pose_.position.z - gripper_pose_curr_.position.z;

		//printf("BB First success: %02d:%02d:%02d\n",f_s.time_of_day().hours(), f_s.time_of_day().minutes(), f_s.time_of_day().seconds());
		//printf("BB Last success: %02d:%02d:%02d\n", l_s.time_of_day().hours(), l_s.time_of_day().minutes(), l_s.time_of_day().seconds());
		printf("BB First success: %04d\n",(int)floor(bb_success_first_.toSec()));
		printf("BB Last success: %04d\n", (int)floor(bb_success_last_.toSec()));

		if (gr_suc) printf("GR Distance: %f [dx: %.2f, dy: %.2f, dz: %.2f] (success)\n",gripper_dist,gdx,gdy,gdz);
		else printf("GR Distance: %f [dx: %.2f, dy: %.2f, dz: %.2f] (fail)\n",gripper_dist,gdx,gdy,gdz);

		/*printf("GR First success: %02d:%02d:%02d\n",f_gr.time_of_day().hours(), f_gr.time_of_day().minutes(), f_gr.time_of_day().seconds());
		printf("GR Last success: %02d:%02d:%02d\n", l_gr.time_of_day().hours(), l_gr.time_of_day().minutes(), l_gr.time_of_day().seconds());*/
		printf("GR First success: %04d\n",(int)floor(gr_success_first_.toSec()));
		printf("GR Last success: %04d\n", (int)floor(gr_success_last_.toSec()));
		printf("\n");

		last_log_out_ = ros::Time::now();


	} // if


	// publish ideal position of gripper
	if (gripper_pub_.getNumSubscribers() != 0) {

		ROS_INFO_ONCE("Publishing gripper ideal position.");

		gripper_marker_.header.stamp = ros::Time::now();
		gripper_pub_.publish(gripper_marker_);


	}


}


int main(int argc, char** argv)
{


      ROS_INFO("Starting");
      ros::init(argc, argv, "bb_overlap");


      BBOverlap *bb = new BBOverlap();

      ros::spin();

      delete bb;

      return 0;

}
