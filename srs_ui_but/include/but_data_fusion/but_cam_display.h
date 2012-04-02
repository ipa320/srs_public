/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vladimir Blahoz
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 09/02/2012
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

#ifndef RVIZ_BUT_CAM_DISPLAY_H
#define RVIZ_BUT_CAM_DISPLAY_H

#include "rviz/display.h"
#include "rviz/properties/forwards.h"

#include <OGRE/OgreTexture.h>
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreVector3.h>

#include <srs_ui_but/ButCamMsg.h>
#include <sensor_msgs/Image.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>

#include <ros/time.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

//#include <nav_msgs/OccupancyGrid.h>

namespace Ogre {
class SceneNode;
class ManualObject;
}

namespace rviz {

/**
 * \class MapDisplay
 * \brief Displays a map along the XZ plane (XY in robot space)
 *
 */
class CButCamDisplay: public Display {
public:
	CButCamDisplay(const std::string& name, VisualizationManager* manager);
	virtual ~CButCamDisplay();

	void setImageTopic(const std::string& topic);
	const std::string& getImageTopic() {
		return image_topic_;
	}

	void setMarkerTopic(const std::string& topic);
	const std::string& getMarkerTopic() {
		return marker_topic_;
	}

	float getResolution() {
		return resolution_;
	}
	float getWidth() {
		return width_;
	}
	float getHeight() {
		return height_;
	}
	int getImageWidth() {
		return image_width_;
	}
	int getImageHeight() {
		return image_height_;
	}
	Ogre::Vector3 getPosition() {
		return position_;
	}
	Ogre::Quaternion getOrientation() {
		return orientation_;
	}

	float getAlpha() {
		return alpha_;
	}
	void setAlpha(float alpha);

	bool getDrawUnder() {
		return draw_under_;
	}
	void setDrawUnder(bool write);

	// Overrides from Display
	virtual void targetFrameChanged() {
	}
	virtual void fixedFrameChanged();
	virtual void createProperties();
	virtual void update(float wall_dt, float ros_dt);
	virtual void reset();

protected:
	// overrides from Display
	virtual void onEnable();
	virtual void onDisable();

	/**
	 * \brief Subscribes to the "visualization_marker" topic
	 */
	virtual void subscribe();
	/**
	 * \brief Unsubscribes from the "visualization_marker" topic
	 */
	virtual void unsubscribe();

	/**
	 * \brief Subscribes to the "Image" topic
	 */
	void imSubscribe();
	/**
	 * \brief Unsubscribes from the "Image" topic
	 */
	void imUnsubscribe();
	/**
	 * \brief Starts and stops time synchronization of messages
	 * \note Checks if both topics are subscribed and if so, starts time synchronization, otherwise
	 * unsubscribes callback of time synchronizer
	 */
	void synchronise();

	void incomingMarker(const srs_ui_but::ButCamMsg::ConstPtr& msg);

	void incomingImage(const sensor_msgs::Image::ConstPtr& image);

	void incoming(const srs_ui_but::ButCamMsg::ConstPtr& msg,
			const sensor_msgs::Image::ConstPtr& image);

	void clear();
	void clearMarker();
	void load(const srs_ui_but::ButCamMsg::ConstPtr& msg);
	void loadImage(const sensor_msgs::Image::ConstPtr& image);
	void transformCam();

	//void requestThreadFunc();

	Ogre::SceneNode* scene_node_;
	Ogre::ManualObject* manual_object_;
	Ogre::TexturePtr texture_;
	Ogre::MaterialPtr material_;
	bool marker_loaded_;
	bool image_loaded_;

	std::string marker_topic_;
	std::string image_topic_;

	std::string frame_;
	sensor_msgs::Image::ConstPtr image_;
	srs_ui_but::ButCamMsg::ConstPtr marker_;

	message_filters::Subscriber<srs_ui_but::ButCamMsg> *marker_sub_ptr_;
	bool marker_subscribed_;
	message_filters::Subscriber<sensor_msgs::Image> *image_sub_ptr_;
	bool image_subscribed_;

	ROSTopicStringPropertyWPtr marker_topic_property_;
	ROSTopicStringPropertyWPtr image_topic_property_;

	// synchronization policy - approximate time (exact time has too low hit rate)
	typedef message_filters::sync_policies::ApproximateTime<
			srs_ui_but::ButCamMsg, sensor_msgs::Image> App_sync_policy;

	message_filters::Synchronizer<App_sync_policy> *time_sync_ptr_;

	bool time_synced_;

	float resolution_;
	FloatPropertyWPtr resolution_property_;

	int image_width_;
	IntPropertyWPtr image_width_property_;

	int image_height_;
	IntPropertyWPtr image_height_property_;

	float width_;
	FloatPropertyWPtr width_property_;

	float height_;
	FloatPropertyWPtr height_property_;

	Ogre::Vector3 position_;
	Vector3PropertyWPtr position_property_;

	Ogre::Quaternion orientation_;
	QuaternionPropertyWPtr orientation_property_;

	float alpha_;
	FloatPropertyWPtr alpha_property_;

	bool draw_under_;
	BoolPropertyWPtr draw_under_property_;
};

} // namespace rviz

#endif
