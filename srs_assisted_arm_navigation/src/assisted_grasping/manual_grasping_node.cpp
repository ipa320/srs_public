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

#include <srs_assisted_arm_navigation/assisted_grasping/manual_grasping_node.h>

using namespace srs_assisted_arm_navigation;
using namespace std;

void ManualGrasping::execute(const ManualGraspingGoalConstPtr &goal) {

  ROS_INFO("Manual grasping action triggered");

  trajectory_msgs::JointTrajectory jtr;
  ManualGraspingActionResult res;

  float max_force = 1000;
  float min_force = 50;
  bool all_fingers_touch = false;

  if ((float)(goal->max_force) <= max_force) {

    ROS_INFO("Restricting grasping maximal force to %f",max_force);
    max_force = (float)(goal->max_force);

  }

  ros::Time start_time = ros::Time::now();

  ros::Time max_time = start_time + ros::Duration(30);

  jtr.header.stamp = ros::Time::now();
  jtr.joint_names = joints_;

  std::vector<double> positions;
  std::vector<double> velocities;

  // opened (script server) 0.0, -0.9854, 0.9472, -0.9854, 0.9472, -0.9854, 0.9472
  // closed (s.s.) [0.0, 0.0, 1.0472, 0.0, 1.0472, 0.0, 1.0472] -> asi spatne, controller tam neni schopen dojet
  // closed - spravne je asi same nuly.....
  // /sdh_controller/state -> pr2_controllers_msgs/JointTrajectoryControllerState -> desired, actual, error

  // z /sdh_controller/mean_values číst hodnoty síly... std_msgs/Float32MultiArray

  float inc = 0.0005;

  positions.push_back(0.0);
  positions.push_back(-0.9854);
  positions.push_back(0.9472);
  positions.push_back(-0.9854);
  positions.push_back(0.9472);
  positions.push_back(-0.9854);
  positions.push_back(0.9472);


  /*if (goal->grasp_type=="open") {

    positions.push_back(0.0);
    positions.push_back(-0.9854);
    positions.push_back(0.9472);
    positions.push_back(-0.9854);
    positions.push_back(0.9472);
    positions.push_back(-0.9854);
    positions.push_back(0.9472);

  } else {

    positions.push_back(0.0);
    positions.push_back(0.0);
    positions.push_back(0.0);
    positions.push_back(0.0);
    positions.push_back(0.0);
    positions.push_back(0.0);
    positions.push_back(0.0);

  }*/


  // nejdřív pohyb prvni casti prstu - do dosažení kontaktu, potom koncovými články??? -> asi blbost

  velocities.push_back(0.001);
  velocities.push_back(0.001);
  velocities.push_back(0.001);
  velocities.push_back(0.001);
  velocities.push_back(0.001);
  velocities.push_back(0.001);
  velocities.push_back(0.001);

  /*trajectory_msgs::JointTrajectoryPoint jtp;
  jtp.positions = positions;
  jtp.velocities = velocities;
  jtp.time_from_start = ros::Duration(0);
  jtr.points.push_back(jtp);
  jt_publisher_.publish(jtr);*/

  ros::Rate r(50);

  std_msgs::Float32MultiArray tactile_data;
  pr2_controllers_msgs::JointTrajectoryControllerState sdh_data;

  while(ros::ok()) {

    if (ros::Time::now() > max_time) {

      res.result.grasped = true;
      res.result.time_elapsed = ros::Time::now() - start_time;
      server_->setAborted(res.result,"Timeout");

      ROS_ERROR("Cannot reach desired positions");
      break;

    }

    if (server_->isPreemptRequested()) {

       ROS_INFO("%s: Preempted", action_name_.c_str());

       // TODO: clean things...

       res.result.time_elapsed = ros::Time::now() - start_time;
       server_->setPreempted(res.result,string("preempted"));

       break;

    }

    data_mutex_.lock();
    tactile_data = tactile_data_;
    sdh_data = sdh_data_;
    data_mutex_.unlock();


    //vector<float>::iterator cur_max_force_it = max_element(tactile_data.data.begin(),tactile_data.data.end());

    /*float cur_max_force = -1.0;

    for (uint8_t i=0; i<tactile_data.data.size(); i++) {

      if (cur_max_force < tactile_data.data.at(i)) cur_max_force = tactile_data.data.at(i);

    }

    if (cur_max_force>=max_force) {

      ROS_INFO("Manual grasping action was successful! ;)");
      res.result.grasped = true;
      res.result.time_elapsed = ros::Time::now() - start_time;
      server_->setSucceeded(res.result,"Ok");

      break;

    }*/

    // what's frequency of tactile data? -> cca 18 Hz
    // check if we have new tactile data, otherwise stop! -> risk of damage... maybe

    uint8_t fingers[3] = {0,0,0};

    if ( (ceil(sdh_data.actual.positions[1]*10) == 0.0)
      && (ceil(sdh_data.actual.positions[2]*10) == 0.0) ) fingers[0] = 1;

    if ( (ceil(sdh_data.actual.positions[3]*10) == 0.0)
      && (ceil(sdh_data.actual.positions[4]*10) == 0.0) ) fingers[1] = 1;

    if ( (ceil(sdh_data.actual.positions[5]*10) == 0.0)
      && (ceil(sdh_data.actual.positions[6]*10) == 0.0) ) fingers[2] = 1;


    if ( fingers[0]!=0 || fingers[1]!=0 || fingers[2]!=0) {

      ROS_INFO("Manual grasping failed - one of the fingers reached 0,0 position.");
      res.result.grasped = false;
      res.result.time_elapsed = ros::Time::now() - start_time;
      server_->setAborted(res.result,"grasping failed");
      break;


    }


    if (!all_fingers_touch) {

      all_fingers_touch = true;

      // first finger
      if (tactile_data.data[1]<min_force) {

        positions[1] += inc;
        positions[2] += inc;
        all_fingers_touch = false;

      }

      // second finger
      if (tactile_data.data[3]<min_force) {

        positions[3] += inc;
        positions[4] += inc;
        all_fingers_touch = false;

      }

      // third finger
      if (tactile_data.data[5]<min_force) {

        positions[5] += inc;
        positions[6] += inc;
        all_fingers_touch = false;

      }

      // TODO check if previous command was finished before sending another one

      trajectory_msgs::JointTrajectoryPoint jtp;
      jtp.positions = positions;
      jtp.velocities = velocities;
      jtp.time_from_start = ros::Duration(0);
      jtr.points.push_back(jtp);
      jt_publisher_.publish(jtr);


    } else {


      if ((tactile_data.data[1] < max_force) || (tactile_data.data[3]< max_force) || (tactile_data.data[5]< max_force)) {

        for (uint8_t i=1; i<positions.size(); i++) {

          if (fabs(positions[i])<inc) positions[i] = 0;

          if (positions[i]<(-inc)) positions[i] += inc;
          if (positions[i]>inc) positions[i] -= inc;

        }

        trajectory_msgs::JointTrajectoryPoint jtp;
        jtp.positions = positions;
        jtp.velocities = velocities;
        jtp.time_from_start = ros::Duration(0);
        jtr.points.push_back(jtp);
        jt_publisher_.publish(jtr);

      } else {

        float cur_max_force = -1.0;

        if (tactile_data.data[1] > cur_max_force) cur_max_force = tactile_data.data[1];
        if (tactile_data.data[3] > cur_max_force) cur_max_force = tactile_data.data[1];
        if (tactile_data.data[5] > cur_max_force) cur_max_force = tactile_data.data[1];

        ROS_INFO("Manual grasping succeeded.");
        res.result.grasped = true;
        res.result.force = cur_max_force;
        res.result.time_elapsed = ros::Time::now() - start_time;
        server_->setSucceeded(res.result,"grasping should be fine");
        break;

      }


    }

    r.sleep();

  } // while




}

void ManualGrasping::SdhStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr & msg) {

  data_mutex_.lock();
  sdh_data_.actual = msg->actual;
  sdh_data_.desired = msg->desired;
  sdh_data_.error = msg->error;
  sdh_data_.header = msg->header;
  //sdh_data_.joint_names = msg->joint_names;
  data_mutex_.unlock();

}


void ManualGrasping::TactileDataCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {

  data_mutex_.lock();
  //tactile_data_ = msg;
  tactile_data_.data = msg->data;
  tactile_data_stamp_ = ros::Time::now();
  data_mutex_.unlock();

}

void ManualGrasping::addJoint(std::string joint) {

  joints_.push_back(joint);

}

void ManualGrasping::inited(bool val) {

  inited_ = val;

}

int main(int argc, char** argv)
{


      ROS_INFO("Starting manual grasping node");
      ros::init(argc, argv, "but_manual_grasping_node");

      //ros::NodeHandle n;

      string tmp(ACT_GRASP);

      ManualGrasping mg(tmp);

      mg.addJoint("sdh_knuckle_joint");
      mg.addJoint("sdh_thumb_2_joint");
      mg.addJoint("sdh_thumb_3_joint");
      mg.addJoint("sdh_finger_12_joint");
      mg.addJoint("sdh_finger_13_joint");
      mg.addJoint("sdh_finger_22_joint");
      mg.addJoint("sdh_finger_23_joint");

      mg.inited(true);

      ROS_INFO("Spinning...");

      ros::spin();
      ros::shutdown();

}
