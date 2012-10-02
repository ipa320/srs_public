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
 * Date: 31/7/2012
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

#include <srs_assisted_arm_navigation_ui/bb_estimation_controls.h>
#include <rviz/window_manager_interface.h>

using namespace std;
using namespace srs_assisted_arm_navigation_ui;
using namespace srs_assisted_arm_navigation;
using namespace srs_assisted_arm_navigation_msgs;

static const string cv_win = "Image window";

const int ID_BUTTON_OK(101);

CButBBEstimationControls * class_ptr;

/**
 Constructor
 */
CButBBEstimationControls::CButBBEstimationControls(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi )
    : wxPanel( parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title)
    , m_wmi( wmi ),
    as_(nh_,ACT_BB_SELECT,false),
    it_(nh_)
{


	ros::param::param<bool>("~is_video_flipped", is_video_flipped_ , true);

	class_ptr = this;

    parent_ = parent;

    m_button_ok_ = new wxButton(this, ID_BUTTON_OK, wxT("Ok"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    m_button_ok_->Enable(false);

    as_.registerGoalCallback(    boost::bind(&CButBBEstimationControls::actionGoalCallback, this));
    as_.registerPreemptCallback( boost::bind(&CButBBEstimationControls::actionPreemptCallback, this));

    image_width_ = 0;
    image_height_ = 0;

    as_.start();


}




CButBBEstimationControls::~CButBBEstimationControls() {

	delete m_button_ok_;
	class_ptr = NULL;

}

void CButBBEstimationControls::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	ROS_INFO_ONCE("Received image");

	cv_bridge::CvImagePtr cv_ptr;

	try {

		cv_ptr = cv_bridge::toCvCopy(msg,msg->encoding);

	} catch (cv_bridge::Exception& e) {

		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;

	}

	if (image_width_ == 0) {


		image_width_ = msg->width;
		image_height_ = msg->height;

	}

	if (some_data_ready_) {

		cv::Point tl,br;

		// TODO add mutex here???????

		if (p1_[0] < p2_[0]) {

			tl.x = p1_[0];
			tl.y = p1_[1];

			br.x = p2_[0];
			br.y = p2_[1];

		} else {

			tl.x = p2_[0];
			tl.y = p2_[1];

			br.x = p1_[0];
			br.y = p1_[1];

		}

		/* draw a blue box */
		cv::rectangle(cv_ptr->image,                    /* the dest image */
					  tl,        /* top left point */
					  br,       /* bottom right point */
					  cv::Scalar(255, 0, 0, 0), /* the color; blue */
					1, 8, 0);               /* thickness, line type, shift */


	}


	cv::imshow(cv_win.c_str(), cv_ptr->image);
	cv::waitKey(3);

}

void CButBBEstimationControls::newData(int event, int x, int y) {

	if (!as_.isActive()) return;

	switch(event) {

		case CV_EVENT_LBUTTONDOWN: {

			butt_down_x_ = x;
			butt_down_y_ = y;

		   } break;

		   // Mouse UP (end-point of the ROI)
		case CV_EVENT_LBUTTONUP: {

			p1_[0] = butt_down_x_;
		    p1_[1] = butt_down_y_;

		    p2_[0] = x;
		    p2_[1] = y;

		    some_data_ready_ = true;

			// TODO: handle flipped video

			if (!is_video_flipped_) {

				fb_.p1[0] = p1_[0];
				fb_.p1[1] = p1_[1];
				fb_.p2[0] = p2_[0];
				fb_.p2[1] = p2_[1];

			} else {

				fb_.p1[0] = image_width_  - p1_[0];
				fb_.p1[1] = image_height_ - p1_[1];
				fb_.p2[0] = image_width_  - p2_[0];
				fb_.p2[1] = image_height_ - p2_[1];

			}

		    fb_.timestamp = ros::Time::now();

		    as_.publishFeedback(fb_);

		    ROS_INFO("New ROI at [%d,%d], [%d,%d]",p1_[0],p1_[1],p2_[0],p2_[1]);

		    m_button_ok_->Enable(true);

		} break;


		 } // switch



}

void onMouse(int event, int x, int y, int flags, void* param) {

	if (class_ptr!=NULL)
		class_ptr->newData(event,x,y);

}

void CButBBEstimationControls::actionGoalCallback() {

	ROS_INFO("Asking user to select ROI in image.");

	goal_ = as_.acceptNewGoal();

	some_data_ready_ = false;

	wxMessageBox(wxString::FromAscii("Please select ROI of unknown object in image"), wxString::FromAscii("BB estimation"), wxOK, parent_,-1,-1);

	max_time_ = ros::Time::now() + ros::Duration(30);


	cv::namedWindow(cv_win.c_str());

	cv::setMouseCallback(cv_win,onMouse,NULL);

	sub_image_ = it_.subscribe("bb_video_in", 1, &CButBBEstimationControls::imageCallback,this);

	//action_in_progress_ = true;

}



void CButBBEstimationControls::actionPreemptCallback() {

	ROS_INFO("%s: Preempted",ACT_BB_SELECT.c_str());
	as_.setPreempted();

	//action_in_progress_ = false;

	cv::destroyWindow(cv_win.c_str());
	sub_image_.shutdown();

}

void CButBBEstimationControls::OnOk(wxCommandEvent& event) {


	if (!as_.isActive()) return;

	ManualBBEstimationResult result;

	if (!is_video_flipped_) {

		result.p1[0] = p1_[0];
		result.p1[1] = p1_[1];
		result.p2[0] = p2_[0];
		result.p2[1] = p2_[1];

	} else {

		result.p1[0] = image_width_  - p1_[0];
		result.p1[1] = image_height_ - p1_[1];
		result.p2[0] = image_width_  - p2_[0];
		result.p2[1] = image_height_ - p2_[1];

	}

	//action_in_progress_ = false;

	as_.setSucceeded(result, "Go on...");

	cv::destroyWindow(cv_win.c_str());
	sub_image_.shutdown();

	m_button_ok_->Enable(false);

	some_data_ready_ = false;

}




BEGIN_EVENT_TABLE(CButBBEstimationControls, wxPanel)
	EVT_BUTTON(ID_BUTTON_OK, CButBBEstimationControls::OnOk)
END_EVENT_TABLE()

