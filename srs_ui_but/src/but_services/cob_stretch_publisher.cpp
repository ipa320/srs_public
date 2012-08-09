/******************************************************************************
 * \file
 *
 * $Id:
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 07/02/2012
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
#include <ros/ros.h>
#include <ros/publisher.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <math.h>
#include <vector>

#include <srs_ui_but/COBStretch.h>
#include <srs_ui_but/topics_list.h>

// Transform listener
tf::TransformListener *tfListener;

// Transformer
tf::Transformer transformer;

// Transformations
tf::StampedTransform linkToBaseTf;

srs_ui_but::COBStretch calculateStretch(std::vector<std::string> links)
{
  ros::Time time_stamp = ros::Time().now();

  float radius = 0;
  float height = 0;

  for (std::vector<std::string>::iterator i = links.begin(); i != links.end(); i++)
  {
    try
    {
      tfListener->waitForTransform(*i, srs_ui_but::DEFAULT_COB_BASE_LINK, time_stamp, ros::Duration(0.2));
      tfListener->lookupTransform(*i, srs_ui_but::DEFAULT_COB_BASE_LINK, time_stamp, linkToBaseTf);

      transformer.setTransform(linkToBaseTf);

      // Transform link to camera
      tf::Stamped<btVector3> p;
      p.setX(0);
      p.setY(0);
      p.setZ(0);
      p.frame_id_ = *i;
      transformer.transformPoint(srs_ui_but::DEFAULT_COB_BASE_LINK, p, p);

      //std::cout << p.getX() << ", " << p.getY() << ", " << p.getZ() << ", " << std::endl;

      float r = pow(p.getX(), 2) + pow(p.getY(), 2);
      if (r > radius)
      {
        radius = r;
        height = p.getZ();
      }
    }

    catch (tf::TransformException& ex)
    {
      ROS_WARN("Transform ERROR:\n %s", ex.what());
    }
  }

  srs_ui_but::COBStretch cob_stretch;
  cob_stretch.radius = sqrt(radius);
  cob_stretch.height = height;
  cob_stretch.time_stamp = time_stamp;

  return cob_stretch;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cob_stretch_publisher");

  ros::NodeHandle nh;

  tfListener = new tf::TransformListener();

  std::vector<std::string> links;
  links.push_back("arm_2_link");
  links.push_back("arm_3_link");
  links.push_back("arm_4_link  ");
  links.push_back("arm_5_link");
  links.push_back("arm_6_link");
  links.push_back("arm_7_link");
  links.push_back("sdh_finger_11_link");
  links.push_back("sdh_finger_12_link");
  links.push_back("sdh_finger_13_link");
  links.push_back("sdh_finger_21_link");
  links.push_back("sdh_finger_22_link");
  links.push_back("sdh_finger_23_link");
  links.push_back("sdh_grasp_link");
  links.push_back("sdh_palm_link");
  links.push_back("sdh_thumb_1_link");
  links.push_back("sdh_thumb_2_link");
  links.push_back("sdh_thumb_2_link");
  links.push_back("sdh_tip_link");
  links.push_back("tray_left_link");
  links.push_back("tray_right_link");

  ros::Publisher pub = nh.advertise<srs_ui_but::COBStretch> (srs_ui_but::COBStretch_TOPIC, 10);

  while (ros::ok())
  {
    srs_ui_but::COBStretch cob_stretch = calculateStretch(links);
    pub.publish(cob_stretch);
  }
  ros::spin();

  return 0;
}
