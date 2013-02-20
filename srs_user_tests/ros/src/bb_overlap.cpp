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

			ros::NodeHandle nh;

			id_.pub = nh.advertise<visualization_msgs::Marker>("ideal_bb_pose",1);

			id_.points_pub = nh.advertise<visualization_msgs::Marker>("ideal_bb_points",1);
			im_.points_pub = nh.advertise<visualization_msgs::Marker>("actual_bb_points",1);
			points_proc_pub_ = nh.advertise<visualization_msgs::Marker>("proc_points",1);

			im_.pose_rec = false;
			im_.scale_rec = false;

			// TODO read this (+ pose) from params....
			id_.bb.lwh.x = 0.2; // length
			id_.bb.lwh.y = 0.1; // width
			id_.bb.lwh.z = 0.4; // height

			id_.bb.pose.position.x = 0.0;
			id_.bb.pose.position.y = 0.0;
			id_.bb.pose.position.z = 0.0;

			id_.bb.pose.orientation.x = 0.0;
			id_.bb.pose.orientation.y = 0.707;
			id_.bb.pose.orientation.z = 0.0;
			id_.bb.pose.orientation.w = 0.707;


			id_.vol = id_.bb.lwh.x * id_.bb.lwh.y * id_.bb.lwh.z;

			id_.marker.color.g = 1.0;
			id_.marker.color.a = 0.6;
			id_.marker.header.frame_id = "/map";

			id_.marker.pose.position = id_.bb.pose.position;
			id_.marker.pose.position.z += id_.bb.lwh.z/2.0;

			id_.marker.pose.orientation = id_.bb.pose.orientation;

			id_.marker.type = visualization_msgs::Marker::CUBE;
			id_.marker.scale.x = id_.bb.lwh.x*2.0;
			id_.marker.scale.y = id_.bb.lwh.y*2.0;
			id_.marker.scale.z = id_.bb.lwh.z;

			// generate eight points for ideal position
			update_points(id_.bb,id_.points);

			sub_im_feedback_ = nh.subscribe<visualization_msgs::InteractiveMarkerFeedback>("/interaction_primitives/feedback",1,&BBOverlap::im_feedback_cb,this);
			sub_im_scale_ = nh.subscribe<srs_interaction_primitives::ScaleChanged>("/interaction_primitives/unknown_object/update/scale_changed",1,&BBOverlap::im_scale_cb,this);

			timer_ = nh.createTimer(ros::Duration(0.05),&BBOverlap::timer_cb,this);

			ROS_INFO("Initialized.");

		}

void BBOverlap::update_points(tbb bb, tpoints &pp) {

	pp.clear();

	tpoint p(0.0, 0.0, 0.0);

	Eigen::Quaternionf q(bb.pose.orientation.w, bb.pose.orientation.x, bb.pose.orientation.y, bb.pose.orientation.z);
	Eigen::Affine3f tr(q);

	/*Eigen::Quaternionf q1(id_.bb.pose.orientation.w, id_.bb.pose.orientation.x, id_.bb.pose.orientation.y, id_.bb.pose.orientation.z);
	Eigen::Affine3f tr2(q1.inverse());*/

	// 1
	p(0) = bb.lwh.x;
    p(1) = bb.lwh.y;
    p(2) = -0.5 * bb.lwh.z;
    p = tr*p;
    //p = tr2*p;
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
    //p = tr2*p;
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
	//p = tr2*p;
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
	//p = tr2*p;
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
    //p = tr2*p;
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
	//p = tr2*p;
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
    //p = tr2*p;
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
	//p = tr2*p;
	p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += bb.lwh.z*0.5;
	pp.push_back(p);

}


void BBOverlap::publish_points(tpoints points, ros::Publisher &pub, std_msgs::ColorRGBA c) {

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


void BBOverlap::im_scale_cb(const srs_interaction_primitives::ScaleChangedConstPtr& msg) {

	ROS_INFO_ONCE("Scale received");

	boost::mutex::scoped_lock(im_.mutex);

	im_.bb.lwh = msg->new_scale;
	im_.bb.lwh.x *= 0.5;
	im_.bb.lwh.y *= 0.5;

	im_.scale_rec = true;

}

void BBOverlap::timer_cb(const ros::TimerEvent&) {

	ROS_INFO_ONCE("Timer triggered");

	boost::mutex::scoped_lock(im_.mutex);

	double im_vol = im_.bb.lwh.x * im_.bb.lwh.y * im_.bb.lwh.z;

	// we have received something...
	if (im_.pose_rec && im_.scale_rec) {


		if ( (ros::Time::now() - last_log_out_) > ros::Duration(2.0) ) {

			ROS_INFO("ID vol: %f, IM vol: %f",id_.vol,im_vol);

			last_log_out_ = ros::Time::now();

		}

		if (id_.pub.getNumSubscribers() > 0) {

			id_.marker.header.stamp = ros::Time::now();
			id_.pub.publish(id_.marker);

		}

		// generate new points
		update_points(im_.bb,im_.points);

		//publish_points(tpoints points, ros::Publisher &pub, std_msgs::ColorRGBA c)
		std_msgs::ColorRGBA c;
		c.a = 0.6;
		c.g = 1.0;

		publish_points(id_.points, id_.points_pub, c);

		c.g = 0.0;
		c.b = 1.0;

		publish_points(im_.points, im_.points_pub, c);


		// take ideal and actual points in /map coord. system
		tpoints id_p = id_.points;
		tpoints im_p = im_.points;


		/*
		 *
		 * p(0) = - bb.lwh.x;
	p(1) = - bb.lwh.y;
	p(2) = -0.5 * bb.lwh.z;
	p = tr*p;
	p(0) += bb.pose.position.x;
	p(1) += bb.pose.position.y;
	p(2) += bb.pose.position.z;
	p(2) += 0.5 * bb.lwh.z;
	pp.push_back(p);
		 *
		 */

		Eigen::Quaternionf q1(id_.bb.pose.orientation.w, id_.bb.pose.orientation.x, id_.bb.pose.orientation.y, id_.bb.pose.orientation.z);
		Eigen::Affine3f tr(q1.inverse());

		for (unsigned int i=0; i<im_p.size(); i++) {

			im_p[i] = tr*im_p[i];

		}


		c.b = 0.0;
		c.r = 1.0;

		publish_points(im_p,points_proc_pub_,c);

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
