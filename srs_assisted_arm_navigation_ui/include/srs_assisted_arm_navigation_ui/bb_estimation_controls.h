/******************************************************************************
 * \file
 * \brief RVIZ plugin which provides simplified GUI based on Warehouse Viewer.
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 5/4/2012
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

#pragma once
#ifndef BUT_BB_ESTIMATION_H
#define BUT_BB_ESTIMATION_H

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/msgdlg.h>
#include <wx/sizer.h>
#include <wx/radiobox.h>
#include <wx/checkbox.h>
#include <ros/ros.h>
#include <string.h>


//#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <map>
#include <iostream>
#include <sstream>

#include "srs_assisted_arm_navigation/services_list.h"
#include "srs_assisted_arm_navigation/topics_list.h"

#include "srs_assisted_arm_navigation_msgs/ManualBBEstimationAction.h"


#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/cv.h>
#include <image_transport/image_transport.h>


namespace rviz
{
    class WindowManagerInterface;
}

namespace srs_assisted_arm_navigation_ui {



class CButBBEstimationControls : public wxPanel
{
public:
    /// Constructor
	CButBBEstimationControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi );
    ~CButBBEstimationControls();

    void newData(int event, int x, int y);


protected:

    //typedef actionlib::SimpleActionClient<ManualGraspingAction> grasping_action_client;
    typedef actionlib::SimpleActionServer<srs_assisted_arm_navigation_msgs::ManualBBEstimationAction> bb_est_action_server;

    //! stored window manager interface pointer
    rviz::WindowManagerInterface * m_wmi;

    ros::NodeHandle nh_;

    bb_est_action_server as_;

    image_transport::ImageTransport it_;

    //void ExecuteAction(const ManualBBEstimationGoalConstPtr &goal);
    void actionGoalCallback();
    void actionPreemptCallback();

    void OnOk(wxCommandEvent& event);

    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

    wxButton * m_button_ok_;

    wxWindow *parent_;


    image_transport::Subscriber sub_image_;

    srs_assisted_arm_navigation_msgs::ManualBBEstimationGoalConstPtr goal_;
    srs_assisted_arm_navigation_msgs::ManualBBEstimationFeedback fb_;

    ros::Time max_time_;

    bool action_in_progress_;

    boost::mutex data_mutex_;
    bool some_data_ready_;
    int16_t p1_[2];
    int16_t p2_[2];

    bool is_video_flipped_;

    unsigned int image_width_;
    unsigned int image_height_;


    int butt_down_x_;
    int butt_down_y_;

private:

    DECLARE_EVENT_TABLE();


};

} // namespace

#endif // BUT_GRASPING_H
