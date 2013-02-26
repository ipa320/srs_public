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

			im_.pose_rec = false;
			im_.scale_rec = false;

			ros::param::param("~publish_debug_markers",publish_debug_markers_,true);

			if (publish_debug_markers_) {

			  id_.points_pub = nh.advertise<visualization_msgs::Marker>("ideal_bb_points",1);
			  im_.points_pub = nh.advertise<visualization_msgs::Marker>("actual_bb_points",1);
			  points_proc_pub_ = nh.advertise<visualization_msgs::Marker>("proc_points",1);

			}

			ros::param::param("~bb/lwh/x",id_.bb.lwh.x,0.2);
			ros::param::param("~bb/lwh/y",id_.bb.lwh.y,0.1);
			ros::param::param("~bb/lwh/z",id_.bb.lwh.z,0.4);

			ros::param::param("~bb/position/x",id_.bb.pose.position.x,-0.15);
			ros::param::param("~bb/position/y",id_.bb.pose.position.y,0.6);
			ros::param::param("~bb/position/z",id_.bb.pose.position.z,0.5);

			ros::param::param("~bb/orientation/x",id_.bb.pose.orientation.x,-8.79007958941e-07);
			ros::param::param("~bb/orientation/y",id_.bb.pose.orientation.y,3.27846130193e-07);
			ros::param::param("~bb/orientation/z",id_.bb.pose.orientation.z,0.429966424682);
			ros::param::param("~bb/orientation/w",id_.bb.pose.orientation.w,0.90284486721);

			double tmp;
			ros::param::param("~sucess_min_dur",tmp,2.0);
			success_min_dur_ = ros::WallDuration(tmp);

			ros::param::param("~sucess_val",tmp,0.8);
			success_val_ = tmp;

			success_first_ = ros::WallTime(0);
			success_last_ = ros::WallTime(0);
			success_tmp_ = ros::WallTime(0);

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

		id_.marker.header.stamp = ros::Time::now();
		id_.pub.publish(id_.marker);

	}

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

		double overlap = 0.0;
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

		bool suc = false;

		if (overlap > 1.0) overlap = 1.0; // just for a case...

		ros::WallTime now = ros::WallTime::now();

		if (overlap > success_val_) {

			if (success_tmp_ == ros::WallTime(0)) success_tmp_ = now;
			else {

				if ((now - success_tmp_) > ros::WallDuration(success_min_dur_)) {

					if (success_first_ == ros::WallTime(0)) success_first_ = now;
					success_last_ = now;
					suc = true;

				}

			}

		} else {

			success_tmp_ = ros::WallTime(0);

		}

		if ( (ros::Time::now() - last_log_out_) > ros::Duration(2.0) ) {

			//ROS_INFO("ID vol: %f, MAA vol: %f",points_volume(id_p),points_volume(imaa_p));

			//ROS_INFO("dx: %f, dy: %f, dz: %f, vol: %f",dx,dy,dz,ov_p_vol);
			//ROS_INFO("overlap: %f%% (%f / %f)",overlap*100.0,ov_p_vol,max_v);

			boost::posix_time::ptime f_s = success_first_.toBoost();
			boost::posix_time::ptime l_s = success_last_.toBoost();

			if (suc) printf("OVERLAP: %03.1f%% (success)\n",overlap*100);
			else printf("OVERLAP: %03.1f%% (fail)\n",overlap*100);

			printf("First success: %02d:%02d:%02d\n",f_s.time_of_day().hours(), f_s.time_of_day().minutes(), f_s.time_of_day().seconds());
			printf("Last success: %02d:%02d:%02d\n", l_s.time_of_day().hours(), l_s.time_of_day().minutes(), l_s.time_of_day().seconds());
			printf("\n");

			last_log_out_ = ros::Time::now();


		} // if

	} // if we have something


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
