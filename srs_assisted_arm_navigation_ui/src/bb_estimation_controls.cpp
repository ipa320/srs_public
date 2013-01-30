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

static const string cv_win = "Assisted object detection";

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

	ros::NodeHandle nh;

	ros::param::param<bool>("~is_video_flipped", is_video_flipped_ , true);
	//ros::param::param<bool>("~disable_video", disable_video_ , false);

	class_ptr = this;

    parent_ = parent;

    m_button_ok_ = new wxButton(this, ID_BUTTON_OK, wxT("Positioning completed"),wxDefaultPosition,wxDefaultSize,wxBU_EXACTFIT);

    m_button_ok_->Enable(false);

    //sub_image_ = it_.subscribe("bb_video_in", 1, &CButBBEstimationControls::imageCallback,this);

    as_.registerGoalCallback(    boost::bind(&CButBBEstimationControls::actionGoalCallback, this));
    as_.registerPreemptCallback( boost::bind(&CButBBEstimationControls::actionPreemptCallback, this));

    image_ = NULL;
    image_tmp_ = NULL;

    image_width_ = 0;
    image_height_ = 0;

    butt_down_x_ = -1;
    butt_down_y_ = -1;

    disable_video_  = false;

    data_ready_ = false;
    some_data_ready_ = false;
    action_in_progress_ = false;

    // 20 Hz timer
    /*if (!disable_video_)*/ timer_ = nh.createTimer(ros::Duration(0.04),&CButBBEstimationControls::timerCallback,this);
    //else ROS_INFO("Started in (almost) useless mode (video disabled).");

    as_.start();


}




CButBBEstimationControls::~CButBBEstimationControls() {

	delete m_button_ok_;
	class_ptr = NULL;

}

void CButBBEstimationControls::timerCallback(const ros::TimerEvent& ev) {

	ROS_INFO_ONCE("Timer triggered.");

	if (action_in_progress_ && !disable_video_) {


		image_mutex_.lock();

		if (image_ == NULL) {

			image_mutex_.unlock();
			return;

		}

		if (image_tmp_ != NULL) {

			image_tmp_->release();
			delete image_tmp_;

		}

		//image_tmp_ = new cv::Mat(*image_);

		image_tmp_ = new cv::Mat();

		image_->copyTo(*image_tmp_);

		image_mutex_.unlock();

		std::string text_line1 = "Please select object.";
		std::string text_line2 = "Press left button, drag and then release.";

		int fontFace = cv::FONT_HERSHEY_COMPLEX_SMALL;
		double fontScale = 0.8;
		int thickness = 1;

		int baseline=0;

		cv::Size size_line1 = cv::getTextSize(text_line1, fontFace, fontScale, thickness, &baseline);
		cv::Size size_line2 = cv::getTextSize(text_line2, fontFace, fontScale, thickness, &baseline);

		int bigger_w = 0;

		if (size_line1.width > size_line2.width) bigger_w = size_line1.width;
		else bigger_w = size_line2.width;

		cv::rectangle(*image_tmp_,                    /* the dest image */
								  cv::Point(0,0),        /* top left point */
								  cv::Point(bigger_w + 20,size_line1.height + size_line2.height + 40),       /* bottom right point */
								  cv::Scalar(250, 250, 250, 125),
								  CV_FILLED,
								  CV_AA,
								  0);

		cv::putText(*image_tmp_, "Please select object.", cvPoint(10,20), fontFace, fontScale, cvScalar(0,0,0), thickness, CV_AA);
		cv::putText(*image_tmp_, "Press left button, drag and then release.", cvPoint(10,50), fontFace, fontScale, cvScalar(0,0,0), thickness, CV_AA);



		if (data_ready_ || some_data_ready_) {

			cv::Point tl,br;

			data_mutex_.lock();

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

			data_mutex_.unlock();

			cv::Scalar color;

			if (some_data_ready_) color = cv::Scalar(255, 0, 0, 0); // blue - while selecting
			else color = cv::Scalar(0, 0, 255, 0); // red - when selected

			cv::rectangle(*image_tmp_,                    /* the dest image */
						  tl,        /* top left point */
						  br,       /* bottom right point */
						  color, /* the color; blue */
						1, 8, 0);               /* thickness, line type, shift */


		}

		cv::imshow(cv_win.c_str(), *image_tmp_);
		//cv::waitKey(1);


	}


}

void CButBBEstimationControls::imageCallback(const sensor_msgs::ImageConstPtr& msg) {

	ROS_INFO_ONCE("Received image");

	if (disable_video_) return;

	//cv_bridge::CvImagePtr cv_ptr;


	try {

		/*if (cv_ptr_ == NULL) cv_ptr_ = cv_bridge::toCvCopy(msg,msg->encoding);
		else {

			cv_ptr_.reset();
			cv_ptr_ = cv_bridge::toCvCopy(msg,msg->encoding);

		}*/

		cv_bridge::CvImagePtr cv_ptr;
		cv_ptr = cv_bridge::toCvCopy(msg,msg->encoding);

		image_mutex_.lock();

		if (image_ != NULL) {

			image_->release();
			delete image_;

		}

		image_ = new cv::Mat(cv_ptr->image);

		image_mutex_.unlock();

	} catch (cv_bridge::Exception& e) {

		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;

	}

	if (image_width_ == 0) {


		image_width_ = msg->width;
		image_height_ = msg->height;

	}

}

void CButBBEstimationControls::newData(int event, int x, int y) {

	if (!as_.isActive()) return;

	switch(event) {

		case CV_EVENT_LBUTTONDOWN: {

			butt_down_x_ = x;
			butt_down_y_ = y;

		   } break;


		case CV_EVENT_MOUSEMOVE: {

			if (butt_down_x_ != -1) {

				some_data_ready_ = true;

				data_mutex_.lock();
				p1_[0] = butt_down_x_;
				p1_[1] = butt_down_y_;

				p2_[0] = x;
				p2_[1] = y;
				data_mutex_.unlock();

			}


		} break;

		   // Mouse UP (end-point of the ROI)
		case CV_EVENT_LBUTTONUP: {

			data_mutex_.lock();

			p1_[0] = butt_down_x_;
		    p1_[1] = butt_down_y_;

		    butt_down_x_ = -1;
		    butt_down_y_ = -1;

		    p2_[0] = x;
		    p2_[1] = y;

		    data_mutex_.unlock();

		    some_data_ready_ = false;
		    data_ready_ = true;

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

	disable_video_ = goal_->disable_video;

	if (!disable_video_) {

		sub_image_ = it_.subscribe("bb_video_in", 1, &CButBBEstimationControls::imageCallback,this);

		wxMessageBox(wxString::FromAscii("Please select unknown object in image. You can select object in image several times. After selecting, it will appear as interactive marker in RVIZ and you can tune it. When it will fit real object, please click on \"Positioning finished\" button."), wxString::FromAscii("Assisted object detection"), wxOK, parent_,-1,-1);

		cv::namedWindow(cv_win.c_str());
		cv::setMouseCallback(cv_win,onMouse,NULL);

	} else {

		wxMessageBox(wxString::FromAscii("Please tune position and orientation of interactive marker to fit the object. When finished, please click on \"Positioning finished\" button."), wxString::FromAscii("Assisted object detection"), wxOK, parent_,-1,-1);
		m_button_ok_->Enable(true);

	}

	//max_time_ = ros::Time::now() + ros::Duration(30);





	action_in_progress_ = true;

}



void CButBBEstimationControls::actionPreemptCallback() {

	ROS_INFO("%s: Preempted",ACT_BB_SELECT.c_str());
	as_.setPreempted();

	action_in_progress_ = false;
	disable_video_ = false;

	if (!disable_video_) {

		cv::destroyWindow(cv_win.c_str());
		sub_image_.shutdown();
		some_data_ready_ = false;
		data_ready_ = false;
		image_width_ = 0;
		image_height_ = 0;

	}

}

void CButBBEstimationControls::OnOk(wxCommandEvent& event) {


	if (!as_.isActive()) return;

	ManualBBEstimationResult result;

	if (!disable_video_) {

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

	} else {

		result.p1[0] = 0.0;
		result.p1[1] = 0.0;
		result.p2[0] = 0.0;
		result.p2[1] = 0.0;

	}

	//action_in_progress_ = false;

	as_.setSucceeded(result, "Go on...");

	m_button_ok_->Enable(false);

	if (disable_video_) {

		cv::destroyWindow(cv_win.c_str());
		sub_image_.shutdown();

		some_data_ready_ = false;
		data_ready_ = false;
		action_in_progress_ = false;
		image_width_ = 0;
		image_height_ = 0;

	}

	disable_video_ = false;

}




BEGIN_EVENT_TABLE(CButBBEstimationControls, wxPanel)
	EVT_BUTTON(ID_BUTTON_OK, CButBBEstimationControls::OnOk)
END_EVENT_TABLE()

