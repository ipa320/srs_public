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

#include "but_cam_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/frame_manager.h"
#include "rviz/validate_floats.h"

#include <tf/transform_listener.h>

#include <ogre_tools/grid.h>

#include <ros/ros.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreTextureManager.h>

namespace rviz {

/**
 * Constructor
 */
CButCamDisplay::CButCamDisplay(const std::string& name,
		VisualizationManager* manager) :
			// Default values
			Display(name, manager), manual_object_(NULL),
			marker_loaded_(false), image_loaded_(false), marker_sub_ptr_(NULL),
			marker_subscribed_(false), image_sub_ptr_(NULL), image_subscribed_(
					false), time_sync_ptr_(NULL), time_synced_(false),
			image_width_(0), image_height_(0), width_(0),
			height_(0), position_(Ogre::Vector3::ZERO), orientation_(
					Ogre::Quaternion::IDENTITY), distance_(1.0), draw_under_(
					false) {
	ROS_DEBUG("CButCamDisplay: constructor");

	// Create new scene node
	scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

	// Create new material for polygon
	static int count = 0;
	std::stringstream ss;
	ss << "CamObjectMaterial" << count++;
	material_ = Ogre::MaterialManager::getSingleton().create(ss.str(),
			Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	material_->setReceiveShadows(false);
	material_->getTechnique(0)->setLightingEnabled(false);
	material_->setDepthBias(-16.0f, 0.0f);
	material_->setCullingMode(Ogre::CULL_NONE);
	material_->setDepthWriteEnabled(false);

	// Default alpha value
	setAlpha(0.7f);
}

/**
 * Destructor
 */
CButCamDisplay::~CButCamDisplay() {
	// Unsubscribe from all topics
	unsubscribe();
	imUnsubscribe();

	// Destroy all Ogre created resources
	clear();
}

void CButCamDisplay::onEnable() {
	ROS_DEBUG("CButCamDisplay: onEnable");

	// Subscribe to all topics
	subscribe();
	imSubscribe();

	scene_node_->setVisible(true);
}

void CButCamDisplay::onDisable() {
	ROS_DEBUG("CButCamDisplay: onDisable");

	// Unsubscribe from all topics
	unsubscribe();
	imUnsubscribe();

	scene_node_->setVisible(false);
	clear();
}

void CButCamDisplay::synchronise() {
	ROS_DEBUG("CButCamDisplay: synchronise1");

	if (marker_subscribed_ && image_subscribed_) {
		// check if both needed topcs are subscribed to

		if (!time_synced_) {
			// create new synchronizer if needed

			time_sync_ptr_
					= new message_filters::Synchronizer<App_sync_policy>(
							App_sync_policy(10), *marker_sub_ptr_,
							*image_sub_ptr_);
			time_sync_ptr_->registerCallback(boost::bind(
					&CButCamDisplay::incoming, this, _1, _2));
			time_synced_ = true;
		} else {
			// redirect inputs of previous time synchronizer

			time_sync_ptr_->connectInput(*marker_sub_ptr_, *image_sub_ptr_);
		}
	}

}

void CButCamDisplay::subscribe() {
	ROS_DEBUG("CButCamDisplay: subscribe");
	if (!isEnabled()) {
		return;
	}

	// subscribe to marker_topic_ if set
	if (!marker_topic_.empty()) {
		//marker_sub_ = update_nh_.subscribe(marker_topic_, 1, this);
		//marker_sub_.subscribe(update_nh_, marker_topic_, 1);
		marker_sub_ptr_
				= new message_filters::Subscriber<srs_ui_but::ButCamMsg>(
						update_nh_, marker_topic_, 1);
		marker_subscribed_ = true;
	}

	synchronise();
}

void CButCamDisplay::unsubscribe() {
	ROS_DEBUG("CButCamDisplay: unsubscribe");

	// unsubscribe from marker_sub_ptr_ if set
	if (marker_sub_ptr_ != NULL)
		marker_sub_ptr_->unsubscribe();
	marker_subscribed_ = false;
}

void CButCamDisplay::imSubscribe() {
	ROS_DEBUG("CButCamDisplay: imSubscribe");
	if (!isEnabled()) {
		return;
	}

	// subscribe to image_topic_ if set
	if (!image_topic_.empty()) {
		//image_sub_.subscribe(update_nh_, image_topic_, 1);
		//image_sub_ = update_nh_.subscribe(image_topic_, 1, this);
		image_sub_ptr_ = new message_filters::Subscriber<sensor_msgs::Image>(
				update_nh_, image_topic_, 1);
		image_subscribed_ = true;
	}

	synchronise();
}

void CButCamDisplay::imUnsubscribe() {
	ROS_DEBUG("CButCamDisplay: imUnubscribe");

	// unsubscribe from image_sub_ptr_ if set
	if (image_sub_ptr_ != NULL)
		image_sub_ptr_->unsubscribe();
	image_subscribed_ = false;
}

void CButCamDisplay::setDistance(float distance) {

	// check if distance is between 0.0 and 1.0
	if (distance > 1)
		distance_ = 1;
	else if (distance < 0)
		distance_ = 0;
	else
		distance_ = distance;

	// set parameter, view node listens for it to recalculate position
	ros::param::set(BUT_DEPTH_PAR, distance_);
}

void CButCamDisplay::setAlpha(float alpha) {
	ROS_DEBUG("CButCamDisplay: setAlpha");

	// check if alpha is between 0.0 and 1.0
	if (alpha > 1)
		alpha_ = 1;
	else if (alpha < 0)
		alpha_ = 0;
	else
		alpha_ = alpha;

	// get Texture unit
	Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
	Ogre::TextureUnitState* tex_unit = NULL;
	if (pass->getNumTextureUnitStates() > 0) {
		tex_unit = pass->getTextureUnitState(0);
	} else {
		tex_unit = pass->createTextureUnitState();
	}

	// set transparency to texture
	tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
			Ogre::LBS_CURRENT, alpha_);

	// set transparency to material
	if (alpha_ < 0.9998) {
		material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
		material_->setDepthWriteEnabled(false);
	} else {
		material_->setSceneBlending(Ogre::SBT_REPLACE);
		material_->setDepthWriteEnabled(!draw_under_);
	}

	propertyChanged(alpha_property_);
}

void CButCamDisplay::setDrawUnder(bool under) {
	ROS_DEBUG("CButCamDisplay: setDrawUnder");

	// set internal variabla
	draw_under_ = under;

	if (alpha_ >= 0.9998) {
		material_->setDepthWriteEnabled(!draw_under_);
	}

	if (manual_object_) {
		if (draw_under_) {
			// map is in RENDER_QUEUE_4, don't want this display under map
			manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
		} else {
			manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_MAIN);
		}
	}

	propertyChanged(draw_under_property_);
}

void CButCamDisplay::setMarkerTopic(const std::string& topic) {
	ROS_DEBUG("CButCamDisplay: setMarkerTopic");

	// unsubscribe from previous topic
	unsubscribe();

	marker_topic_ = topic;

	// subscribe to new topic
	subscribe();

	// destroy previously created Ogre objects
	clear();

	propertyChanged(marker_topic_property_);
}

void CButCamDisplay::setImageTopic(const std::string& topic) {
	ROS_DEBUG("CButCamDisplay: setImageTopic");

	// unsubscribe from previous topic
	imUnsubscribe();

	image_topic_ = topic;

	// set corresponding parameter, view node listens to for
	// recalculating camera parameters
	ros::param::set(BUT_CAMERA_PAR, image_topic_);

	// subscribe to new topic
	imSubscribe();

	// destroy previously created Ogre objects
	clear();

	propertyChanged(image_topic_property_);
}
void CButCamDisplay::clearMarker() {
	ROS_DEBUG("CButCamDisplay: clearMarker");

	// Destroy Ogre object if created
	if (marker_loaded_) {
		scene_manager_->destroyManualObject(manual_object_);
		manual_object_ = NULL;
		marker_loaded_ = false;
	} else
		setStatus(status_levels::Warn, "Marker", "No marker message received");

}

void CButCamDisplay::clear() {
	ROS_DEBUG("CButCamDisplay: clear()");

	// Destroy Ogre object if created
	if (marker_loaded_) {
		scene_manager_->destroyManualObject(manual_object_);
		manual_object_ = NULL;
		marker_loaded_ = false;
	} else
		setStatus(status_levels::Warn, "Marker", "No marker message received");

	// Destroy Ogre texture if created
	if (image_loaded_) {
		std::string tex_name = texture_->getName();
		texture_.setNull();
		Ogre::TextureManager::getSingleton().unload(tex_name);
		image_loaded_ = false;
	} else
		setStatus(status_levels::Warn, "Image", "No image received");

}

bool validateFloats(const srs_ui_but::ButCamMsg& msg) {
	return validateFloats(msg.pose);
}

void CButCamDisplay::loadImage(const sensor_msgs::Image::ConstPtr& image) {
	ROS_DEBUG("CButCamDisplay: loadImage");
	//	setStatus(status_levels::Ok, "Image", "Image received");

	// set internal variables
	image_width_ = image->width;
	image_height_ = image->height;

	// check image size
	if (image->height * image->width == 0) {
		std::stringstream ss;
		ss << "Image is zero-sized (" << image_height_ << "x" << image_width_
				<< ")";
		setStatus(status_levels::Error, "Image", ss.str());
		return;
	}

	ROS_DEBUG("Received a %d X %d image, encoding: %s, pixels: %d, data size: %d",
			image->height,
			image->width,
			image->encoding.c_str(),
			image_width_ * image_height_,
			image->data.size()
	);

	// check image encoding
	if (image->encoding != "rgb8") {
		std::stringstream ss;
		ss << "Unsupported image encoding (" << image->encoding.c_str()
				<< "), expected 'rgb8'";
		setStatus(status_levels::Error, "Image", ss.str());
		return;
	}

	// Prepair image RGB data array
	unsigned int pixels_size = image_width_ * image_height_ * 3;
	unsigned char* pixels = new unsigned char[pixels_size];
	//memset(pixels, 255, pixels_size);

	// check if image size matches its data size
	if (pixels_size != image->data.size()) {
		std::stringstream ss;
		ss << "Data size doesn't match width*height: width = " << image_width_
				<< ", height = " << image_height_ << ", data size = "
				<< image->data.size();
		setStatus(status_levels::Error, "Image", ss.str());

		return;
	}

	// copy image data to data array
	for (unsigned int pixel_index = 0; pixel_index < pixels_size; pixel_index++) {
		pixels[pixel_index] = image->data[pixel_index];
	}

	// Create new Ogre texture
	Ogre::DataStreamPtr pixel_stream;
	pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
	static int tex_count = 0;
	std::stringstream ss2;
	ss2 << "CamTexture" << (tex_count - 1);
	std::stringstream ss;
	ss.clear();
	ss << "CamTexture" << tex_count++;

	// Try to fill texture with rgb data
	try {
		Ogre::TextureManager::getSingleton().remove(ss2.str());
		texture_ = Ogre::TextureManager::getSingleton().loadRawData(ss.str(),
				Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
				pixel_stream, image_width_, image_height_, Ogre::PF_B8G8R8,
				Ogre::TEX_TYPE_2D, 0);

		setStatus(status_levels::Ok, "Image", "Image OK");

	} catch (Ogre::RenderingAPIException&) {
		// if resolution is too big, downsample the image

		Ogre::Image image;
		pixel_stream->seek(0);
		float width = image_width_;
		float height = image_height_;
		if (image_width_ > image_height_) {
			float aspect = height / width;
			width = 2048;
			height = width * aspect;
		} else {
			float aspect = width / height;
			height = 2048;
			width = height * aspect;
		}

		{
			std::stringstream ss;
			ss
					<< "Map is larger than your graphics card supports.  Downsampled from ["
					<< width_ << "x" << height_ << "] to [" << width << "x"
					<< height << "]";
			setStatus(status_levels::Ok, "Map", ss.str());
		}

		ROS_WARN("Failed to create full-size map texture, likely because your graphics card does not support textures of size > 2048.  Downsampling to [%d x %d]...", (int)width, (int)height);
		//ROS_INFO("Stream size [%d], width [%f], height [%f], w * h [%f]", pixel_stream->size(), width_, height_, width_ * height_);
		image.loadRawData(pixel_stream, image_width_, image_height_,
				Ogre::PF_R8G8B8);
		image.resize(width, height, Ogre::Image::FILTER_NEAREST);
		ss << "Downsampled";
		texture_ = Ogre::TextureManager::getSingleton().loadImage(ss.str(),
				Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, image);
	}

	// delete data array
	delete[] pixels;

	// Get polygon texture unit
	Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
	Ogre::TextureUnitState* tex_unit = NULL;
	if (pass->getNumTextureUnitStates() > 0) {
		tex_unit = pass->getTextureUnitState(0);
	} else {
		tex_unit = pass->createTextureUnitState();
	}

	// Bind polygon texture unit with new texture
	tex_unit->setTextureName(texture_->getName());
	tex_unit->setTextureFiltering(Ogre::TFO_NONE);

	image_loaded_ = true;

	propertyChanged(image_width_property_);
	propertyChanged(image_height_property_);

	causeRender();

}
void CButCamDisplay::load(const srs_ui_but::ButCamMsg::ConstPtr& msg) {
	ROS_DEBUG("CButCamDisplay: load");

	setStatus(status_levels::Ok, "Marker", "Marker OK");

	// chcek floating point values
	if (!validateFloats(*msg)) {
		setStatus(status_levels::Error, "Marker",
				"Message contained invalid floating point values (nans or infs)");
		return;
	}

	//clearMarker();

	// Set internal variables
	width_ = msg->scale.x;//(int)pow(2,ceil(log2(msg->info.width)));
	height_ = msg->scale.y;//(int)pow(2,ceil(log2(msg->info.height)));

	marker_ = msg;
	position_.x = msg->pose.position.x;
	position_.y = msg->pose.position.y;
	position_.z = msg->pose.position.z;
	orientation_.w = msg->pose.orientation.w;
	orientation_.x = msg->pose.orientation.x;
	orientation_.y = msg->pose.orientation.y;
	orientation_.z = msg->pose.orientation.z;
	frame_ = msg->header.frame_id;

	//material->getTechnique(0)->setAmbient( 0.5, 0.5, 0.5 );


	if (!image_loaded_) {
		// set material parameters

		material_->getTechnique(0)->setLightingEnabled(true);

		float r = 1.0;
		float g = 1.0;
		float b = 1.0;
		float a = 1.0;
		material_->getTechnique(0)->setAmbient(r, g, b);

		material_->getTechnique(0)->setDiffuse(r, g, b, a);

		// Create new Ogre object representing polygon with camera image
		static int cam_count = 0;
		std::stringstream ss2;
		ss2 << "CamObject" << cam_count++;
		manual_object_ = scene_manager_->createManualObject(ss2.str());
		scene_node_->attachObject(manual_object_);

		//manual_object_->setMaterialName()

		// Set the polygon geometry (1x1 square at the coordinates origin)
		manual_object_->begin(material_->getName(),
				Ogre::RenderOperation::OT_TRIANGLE_LIST);
		{
			// First triangle
			{
				// Bottom left
				manual_object_->position(0.0f, 0.0f, 0.0f);
				manual_object_->textureCoord(0.0f, 0.0f);
				manual_object_->normal(0.0f, 0.0f, 1.0f);

				// Top right
				manual_object_->position(1.0f, 1.0f, 0.0f);
				manual_object_->textureCoord(1.0f, 1.0f);
				manual_object_->normal(0.0f, 0.0f, 1.0f);

				// Top left
				manual_object_->position(0.0f, 1.0f, 0.0f);
				manual_object_->textureCoord(0.0f, 1.0f);
				manual_object_->normal(0.0f, 0.0f, 1.0f);
			}

			// Second triangle
			{
				// Bottom left
				manual_object_->position(0.0f, 0.0f, 0.0f);
				manual_object_->textureCoord(0.0f, 0.0f);
				manual_object_->normal(0.0f, 0.0f, 1.0f);

				// Bottom right
				manual_object_->position(1.0, 0.0f, 0.0f);
				manual_object_->textureCoord(1.0f, 0.0f);
				manual_object_->normal(0.0f, 0.0f, 1.0f);

				// Top right
				manual_object_->position(1.0, 1.0, 0.0f);
				manual_object_->textureCoord(1.0f, 1.0f);
				manual_object_->normal(0.0f, 0.0f, 1.0f);
			}
		}
		manual_object_->end();
	}
	if (draw_under_) {
		// map is in RENDER_QUEUE_4, don't want this display under map
		manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
	}

	propertyChanged(width_property_);
	propertyChanged(height_property_);
	propertyChanged(position_property_);
	propertyChanged(orientation_property_);

	// transform polygon to its real position and size
	transformCam();

	marker_loaded_ = true;

	causeRender();
}

void CButCamDisplay::transformCam() {
	ROS_DEBUG_NAMED("CButCamDisplay", "transforming cam from frame '%s' to frame '%s'", frame_.c_str(), fixed_frame_.c_str());

	if (!marker_) {
		return;
	}

	// get polygon real geometry from last message
	Ogre::Vector3 position;
	Ogre::Quaternion orientation;
	if (!vis_manager_->getFrameManager()->transform(frame_, ros::Time(),
			marker_->pose, position, orientation)) {
		ROS_DEBUG("Error transforming marker '%s' from frame '%s' to frame '%s'", name_.c_str(), frame_.c_str(), fixed_frame_.c_str());

		std::stringstream ss;
		ss << "No transform from [" << frame_ << "] to [" << fixed_frame_
				<< "]";
		setStatus(status_levels::Error, "Transform", ss.str());
	} else {
		setStatus(status_levels::Ok, "Transform", "Transform OK");
	}

	// transform polygon to its real position and size
	// (transforming scene node, polygon is the only object attached)
	scene_node_->setPosition(position);
	scene_node_->setOrientation(orientation);
	scene_node_->setScale(Ogre::Vector3(marker_->scale.x, marker_->scale.y,
			marker_->scale.z));
}

void CButCamDisplay::update(float wall_dt, float ros_dt) {
	//ROS_DEBUG("CButCamDisplay: update");
}

void CButCamDisplay::createProperties() {
	ROS_DEBUG("CButCamDisplay: createProperties");

	// Subscribed Marker Topic
	// the name "marker_topic" is residue, when polygon was defined by marker message
	marker_topic_property_ = property_manager_->createProperty<
			ROSTopicStringProperty> ("Marker Topic", property_prefix_,
			boost::bind(&CButCamDisplay::getMarkerTopic, this), boost::bind(
					&CButCamDisplay::setMarkerTopic, this, _1),
			parent_category_, this);
	setPropertyHelpText(marker_topic_property_,
			"srs_ui_but::ButCamMsg topic to subscribe to.");
	ROSTopicStringPropertyPtr marker_topic_prop = marker_topic_property_.lock();
	marker_topic_prop->setMessageType(ros::message_traits::datatype<
			srs_ui_but::ButCamMsg>());
	marker_topic_prop->addLegacyName("Service"); // something of a hack, but should provide reasonable backwards compatibility

	// Subscribed Image Topic
	image_topic_property_ = property_manager_->createProperty<
			ROSTopicStringProperty> ("Image Topic", property_prefix_,
			boost::bind(&CButCamDisplay::getImageTopic, this), boost::bind(
					&CButCamDisplay::setImageTopic, this, _1),
			parent_category_, this);
	setPropertyHelpText(image_topic_property_,
			"sensor_msgs::Image topic to subscribe to.");
	ROSTopicStringPropertyPtr image_topic_prop = image_topic_property_.lock();
	image_topic_prop->setMessageType(ros::message_traits::datatype<
			sensor_msgs::Image>());
	image_topic_prop->addLegacyName("Service"); // something of a hack, but should provide reasonable backwards compatibility

	// Polygon transparency
	alpha_property_ = property_manager_->createProperty<FloatProperty> (
			"Alpha", property_prefix_, boost::bind(&CButCamDisplay::getAlpha,
					this), boost::bind(&CButCamDisplay::setAlpha, this, _1),
			parent_category_, this);
	setPropertyHelpText(alpha_property_,
			"Amount of transparency to apply to the marker.");

	// Polygon distance
	distance_property_ = property_manager_->createProperty<FloatProperty> (
			"Video distance", property_prefix_, boost::bind(
					&CButCamDisplay::getDistance, this), boost::bind(
					&CButCamDisplay::setDistance, this, _1), parent_category_,
			this);
	setPropertyHelpText(
			distance_property_,
			"Distance of the video polygon from the robot. Value 1 means the biggest distance possible, 0.5 is the half way to the robot and so forth...");

	// Draw under
	draw_under_property_ = property_manager_->createProperty<BoolProperty> (
			"Draw Behind", property_prefix_, boost::bind(
					&CButCamDisplay::getDrawUnder, this), boost::bind(
					&CButCamDisplay::setDrawUnder, this, _1), parent_category_,
			this);
	setPropertyHelpText(
			draw_under_property_,
			"Rendering option, controls whether or not the marker is always drawn behind everything else.");

	// Polygon width
	width_property_ = property_manager_->createProperty<FloatProperty> (
			"Marker Width", property_prefix_, boost::bind(
					&CButCamDisplay::getWidth, this), FloatProperty::Setter(),
			parent_category_, this);
	setPropertyHelpText(width_property_,
			"Width of the marker, in meters. (not editable)");

	// Polygon height
	height_property_ = property_manager_->createProperty<FloatProperty> (
			"Marker Height", property_prefix_, boost::bind(
					&CButCamDisplay::getHeight, this), FloatProperty::Setter(),
			parent_category_, this);
	setPropertyHelpText(height_property_,
			"Height of the marker, in meters. (not editable)");

	// Image width
	image_width_property_ = property_manager_->createProperty<IntProperty> (
			"Image Width", property_prefix_, boost::bind(
					&CButCamDisplay::getImageWidth, this),
			IntProperty::Setter(), parent_category_, this);
	setPropertyHelpText(image_width_property_,
			"Width of the camera image, in pixels. (not editable)");

	// Image height
	image_height_property_ = property_manager_->createProperty<IntProperty> (
			"Image Height", property_prefix_, boost::bind(
					&CButCamDisplay::getImageHeight, this),
			IntProperty::Setter(), parent_category_, this);
	setPropertyHelpText(image_height_property_,
			"Height of the camera image, in pixels. (not editable)");

	// Polygon position
	position_property_ = property_manager_->createProperty<Vector3Property> (
			"Position", property_prefix_, boost::bind(
					&CButCamDisplay::getPosition, this),
			Vector3Property::Setter(), parent_category_, this);
	setPropertyHelpText(position_property_,
			"Position of the middle of the marker, in meters. (not editable)");

	// Polygon orientation
	orientation_property_ = property_manager_->createProperty<
			QuaternionProperty> ("Orientation", property_prefix_, boost::bind(
			&CButCamDisplay::getOrientation, this),
			QuaternionProperty::Setter(), parent_category_, this);
	setPropertyHelpText(orientation_property_,
			"Orientation of the marker. (not editable)");

}

void CButCamDisplay::fixedFrameChanged() {
	ROS_DEBUG("CButCamDisplay: fixedFrameChanged");
	transformCam();
}

void CButCamDisplay::reset() {
	ROS_DEBUG("CButCamDisplay: reset");
	Display::reset();

	// Destroy all Ogre object created
	clear();

	// Force resubscription so that the marker will be re-sent
	setMarkerTopic(marker_topic_);
	setImageTopic(image_topic_);
}

void CButCamDisplay::incoming(const srs_ui_but::ButCamMsg::ConstPtr& msg,
		const sensor_msgs::Image::ConstPtr& image) {
	ROS_DEBUG("CButCamDisplay: incoming");
	// incoming synchronised geometry and texture

	// update geometry
	load(msg);

	// update texture
	loadImage(image);
}

// function is not being used, CButCamDisplay::incoming is called instead
void CButCamDisplay::incomingMarker(const srs_ui_but::ButCamMsg::ConstPtr& msg) {
	ROS_DEBUG_NAMED("CButCamDisplay", "incoming marker");
	// incoming geometry, update it
	load(msg);
}

// function is not being used, CButCamDisplay::incoming is called instead
void CButCamDisplay::incomingImage(const sensor_msgs::Image::ConstPtr& image) {
	ROS_DEBUG_NAMED("CButCamDisplay", "incoming image");
	// incoming texture, update it
	loadImage(image);
}

} // namespace rviz
