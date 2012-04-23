/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vladimir Blahoz (xblaho02@stud.fit.vutbr.cz)
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

#include "topics_list.h"
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

namespace Ogre {
class SceneNode;
class ManualObject;
}

namespace rviz {

/**
 * @class MapDisplay
 *
 * @brief Displays a map along the XZ plane (XY in robot space)
 *
 */
class CButCamDisplay: public Display {
public:
	/**
	 * Constructor
	 */
	CButCamDisplay(const std::string& name, VisualizationManager* manager);

	/**
	 * Destructor
	 */
	virtual ~CButCamDisplay();

	/**
	 * @brief image_topic property setter
	 *
	 * @param topic new image topic
	 */
	void setImageTopic(const std::string& topic);

	/**
	 * @brief image_topic property getter
	 */
	const std::string& getImageTopic() {
		return image_topic_;
	}

	/**
	 * marker_topic property setter
	 *
	 * @param topic new ButCamMsg topic
	 */
	void setMarkerTopic(const std::string& topic);

	/**
	 * @brief marker_topic property getter
	 */
	const std::string& getMarkerTopic() {
		return marker_topic_;
	}

	/**
	 * @brief width property getter
	 */
	float getWidth() {
		return width_;
	}

	/**
	 * @brief height property getter
	 */
	float getHeight() {
		return height_;
	}

	/**
	 * @brief image_width property getter
	 */
	int getImageWidth() {
		return image_width_;
	}

	/**
	 * @brief image_height property getter
	 */
	int getImageHeight() {
		return image_height_;
	}

	/**
	 * @brief position property getter
	 */
	Ogre::Vector3 getPosition() {
		return position_;
	}

	/**
	 * @brief orientation property getter
	 */
	Ogre::Quaternion getOrientation() {
		return orientation_;
	}

	/**
	 * @brief alpha property getter
	 */
	float getAlpha() {
		return alpha_;
	}

	/**
	 * @brief alpha property setter
	 *
	 * @param alpha new alpha
	 */
	void setAlpha(float alpha);

	/**
	 * @brief distance property getter
	 */
	float getDistance() {
		return distance_;
	}

	/**
	 * @brief distance property setter
	 *
	 * @param distance new ploygon rendering distance
	 */
	void setDistance(float distance);

	/**
	 * @brief draw_under property getter
	 */
	bool getDrawUnder() {
		return draw_under_;
	}

	/**
	 * @brief draw_under property setter
	 *
	 * @param write true for drawing under, false for not
	 */
	void setDrawUnder(bool write);

	/**
	 * @brief Overriden from Display
	 */
	virtual void targetFrameChanged() {
	}

	/**
	 * @brief Overriden from Display
	 */
	virtual void fixedFrameChanged();

	/**
	 * @brief Overriden from Display
	 */
	virtual void createProperties();

	/**
	 * @brief Overriden from Display
	 */
	virtual void update(float wall_dt, float ros_dt);

	/**
	 * @brief Overriden from Display
	 */
	virtual void reset();

protected:
	/**
	 * @brief Overriden from Display
	 */
	virtual void onEnable();

	/**
	 * @brief Overriden from Display
	 */
	virtual void onDisable();

	/**
	 * @brief Subscribes to the "visualization_marker" topic
	 */
	virtual void subscribe();

	/**
	 * @brief Unsubscribes from the "visualization_marker" topic
	 */
	virtual void unsubscribe();

	/**
	 * @brief Subscribes to the "Image" topic
	 */
	void imSubscribe();

	/**
	 * @brief Unsubscribes from the "Image" topic
	 */
	void imUnsubscribe();

	/**
	 * @brief Starts and stops time synchronization of messages
	 * @note Checks if both topics are subscribed and if so, starts time synchronization.
	 */
	void synchronise();

	/**
	 * @brief callback function for ButCamMsg message arrival
	 *
	 * @param msg newly arrived ButCamMsg message
	 */
	void incomingMarker(const srs_ui_but::ButCamMsg::ConstPtr& msg);

	/**
	 * @brief callback function for Image message arrival
	 *
	 * @param image newly arrived camera image message
	 */
	void incomingImage(const sensor_msgs::Image::ConstPtr& image);

	/**
	 * @brief callback function for both ButCamMsg and Image messages synchronized arrival
	 *
	 * @param msg newly arrived ButCamMsg message
	 * @param image newly arrived camera image message
	 */
	void incoming(const srs_ui_but::ButCamMsg::ConstPtr& msg,
			const sensor_msgs::Image::ConstPtr& image);

	/**
	 * @brief Destroy created Ogre objects and textures
	 */
	void clear();

	/**
	 * @brief Destroy only created Ogre object manual_object_
	 */
	void clearMarker();

	/**
	 * @brief Creates and/or updates position of video frame polygon
	 *
	 * @param msg obtained ButCamMsg message
	 */
	void load(const srs_ui_but::ButCamMsg::ConstPtr& msg);

	/**
	 * @brief Creates and/or updates texture of video frame polygon
	 *
	 * @param image obtained camera image message
	 */
	void loadImage(const sensor_msgs::Image::ConstPtr& image);

	/**
	 * @brief Makes sure the polygon position/orientation informations are related to correct frame
	 */
	void transformCam();

	// The scene node
	Ogre::SceneNode* scene_node_;

	// Object representing polygon with camera frame
	Ogre::ManualObject* manual_object_;

	// Texture of polygon with camera frame
	Ogre::TexturePtr texture_;

	// Material of polygon with camera frame
	Ogre::MaterialPtr material_;

	// Flag that geometry of polygon is loaded
	bool marker_loaded_;

	// Flag that texture of polygon is loaded
	bool image_loaded_;

	// Subscribed topic, where ButCamMsg messages arrive and corresponding display property
	std::string marker_topic_;
	ROSTopicStringPropertyWPtr marker_topic_property_;

	// Subscribed topic, where Image messages arrive and corresponding display property
	std::string image_topic_;
	ROSTopicStringPropertyWPtr image_topic_property_;

	// fixed frame
	std::string frame_;

	// last arrived Image message
	sensor_msgs::Image::ConstPtr image_;

	// last arrived ButCamMsg message
	srs_ui_but::ButCamMsg::ConstPtr marker_;

	// subscribers
	message_filters::Subscriber<srs_ui_but::ButCamMsg> *marker_sub_ptr_;
	bool marker_subscribed_;
	message_filters::Subscriber<sensor_msgs::Image> *image_sub_ptr_;
	bool image_subscribed_;

	// synchronization policy - approximate time (exact time has too low hit rate)
	typedef message_filters::sync_policies::ApproximateTime<
			srs_ui_but::ButCamMsg, sensor_msgs::Image> App_sync_policy;

	// messages synchronizer
	message_filters::Synchronizer<App_sync_policy> *time_sync_ptr_;
	bool time_synced_;

	// image width and corresponding display property
	int image_width_;
	IntPropertyWPtr image_width_property_;

	// image height and corresponding display property
	int image_height_;
	IntPropertyWPtr image_height_property_;

	// polygon width and corresponding display property
	float width_;
	FloatPropertyWPtr width_property_;

	// polygon height and corresponding display property
	float height_;
	FloatPropertyWPtr height_property_;

	// rendering polygon distance and corresponding display property
	float distance_;
	FloatPropertyWPtr distance_property_;

	// rendering polygon position and corresponding display property
	Ogre::Vector3 position_;
	Vector3PropertyWPtr position_property_;

	// rendering polygon orientation and corresponding display property
	Ogre::Quaternion orientation_;
	QuaternionPropertyWPtr orientation_property_;

	// polygon transparency and corresponding display property
	float alpha_;
	FloatPropertyWPtr alpha_property_;

	// draw polygon on the background and corresponding display property
	bool draw_under_;
	BoolPropertyWPtr draw_under_property_;
};

} // namespace rviz

#endif
