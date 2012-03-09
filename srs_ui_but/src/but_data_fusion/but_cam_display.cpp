/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
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

ButCamDisplay::ButCamDisplay(const std::string& name,
		VisualizationManager* manager) :
	Display(name, manager), manual_object_(NULL), marker_loaded_(false),
			image_loaded_(false), marker_sub_ptr_(NULL), marker_subscribed_(
					false), image_sub_ptr_(NULL), image_subscribed_(false),
			time_sync_ptr_(NULL), time_synced_(false), resolution_(0.0f),
			image_width_(0), image_height_(0), width_(0), height_(0),
			position_(Ogre::Vector3::ZERO), orientation_(
					Ogre::Quaternion::IDENTITY), draw_under_(false) {
	ROS_DEBUG("ButCamDisplay: constructor");

	scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

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

	setAlpha(0.7f);
}

ButCamDisplay::~ButCamDisplay() {
	unsubscribe();
	imUnsubscribe();

	clear();
}

void ButCamDisplay::onEnable() {
	subscribe();
	imSubscribe();

	scene_node_->setVisible(true);
}

void ButCamDisplay::onDisable() {
	unsubscribe();
	imUnsubscribe();

	scene_node_->setVisible(false);
	clear();
}

void ButCamDisplay::synchronise() {
	if (marker_subscribed_ && image_subscribed_) {
		if (!time_synced_) {
			// time-synchronizing both messages with CameraInfo tf transformation
			time_sync_ptr_
					= new message_filters::Synchronizer<App_sync_policy>(
							App_sync_policy(10), *marker_sub_ptr_,
							*image_sub_ptr_);
			//			time_sync_ptr_ = new message_filters::TimeSynchronizer<srs_ui_but::ButCamMsg, sensor_msgs::Image>(*marker_sub_ptr_,*image_sub_ptr_, 10);
			time_sync_ptr_->registerCallback(boost::bind(
					&ButCamDisplay::incoming, this, _1, _2));
			time_synced_ = true;
		} else {
			time_sync_ptr_->connectInput(*marker_sub_ptr_, *image_sub_ptr_);
		}
	}
}

void ButCamDisplay::subscribe() {
	ROS_DEBUG("ButCamDisplay: subscribe");
	if (!isEnabled()) {
		return;
	}

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

void ButCamDisplay::unsubscribe() {
	ROS_DEBUG("ButCamDisplay: unsubscribe");
	if (marker_sub_ptr_ != NULL)
		marker_sub_ptr_->unsubscribe();
	marker_subscribed_ = false;
}

void ButCamDisplay::imSubscribe() {
	ROS_DEBUG("ButCamDisplay: imSubscribe");
	if (!isEnabled()) {
		return;
	}

	if (!image_topic_.empty()) {
		//image_sub_.subscribe(update_nh_, image_topic_, 1);
		//image_sub_ = update_nh_.subscribe(image_topic_, 1, this);
		image_sub_ptr_ = new message_filters::Subscriber<sensor_msgs::Image>(
				update_nh_, image_topic_, 1);
		image_subscribed_ = true;
	}
	synchronise();
}

void ButCamDisplay::imUnsubscribe() {
	ROS_DEBUG("ButCamDisplay: imUnubscribe");
	if (image_sub_ptr_ != NULL)
		image_sub_ptr_->unsubscribe();
	marker_subscribed_ = false;
}

void ButCamDisplay::setAlpha(float alpha) {
	ROS_DEBUG("ButCamDisplay: setAlpha");

	alpha_ = alpha;

	Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
	Ogre::TextureUnitState* tex_unit = NULL;
	if (pass->getNumTextureUnitStates() > 0) {
		tex_unit = pass->getTextureUnitState(0);
	} else {
		tex_unit = pass->createTextureUnitState();
	}

	tex_unit->setAlphaOperation(Ogre::LBX_SOURCE1, Ogre::LBS_MANUAL,
			Ogre::LBS_CURRENT, alpha_);

	if (alpha_ < 0.9998) {
		material_->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
		material_->setDepthWriteEnabled(false);
	} else {
		material_->setSceneBlending(Ogre::SBT_REPLACE);
		material_->setDepthWriteEnabled(!draw_under_);
	}

	propertyChanged(alpha_property_);
}

void ButCamDisplay::setDrawUnder(bool under) {
	ROS_DEBUG("ButCamDisplay: setDrawUnder");

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

void ButCamDisplay::setMarkerTopic(const std::string& topic) {
	ROS_DEBUG("ButCamDisplay: setMarkerTopic");

	unsubscribe();

	marker_topic_ = topic;

	subscribe();

	clear();

	propertyChanged(marker_topic_property_);
}

void ButCamDisplay::setImageTopic(const std::string& topic) {

	imUnsubscribe();

	image_topic_ = topic;

	imSubscribe();

	clear();

	propertyChanged(image_topic_property_);
}
void ButCamDisplay::clearMarker() {
	if (marker_loaded_) {
		scene_manager_->destroyManualObject(manual_object_);
		manual_object_ = NULL;
		marker_loaded_ = false;
	} else
		setStatus(status_levels::Warn, "Marker", "No marker message received");

}

void ButCamDisplay::clear() {
	ROS_DEBUG("ButCamDisplay: clear()");

	if (marker_loaded_) {
		scene_manager_->destroyManualObject(manual_object_);
		manual_object_ = NULL;
		marker_loaded_ = false;
	} else
		setStatus(status_levels::Warn, "Marker", "No marker message received");

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

void ButCamDisplay::loadImage(const sensor_msgs::Image::ConstPtr& image) {
	//	setStatus(status_levels::Ok, "Image", "Image received");

	image_width_ = image->width;
	image_height_ = image->height;

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

	if (image->encoding != "rgb8") {
		std::stringstream ss;
		ss << "Unsupported image encoding (" << image->encoding.c_str()
				<< "), expected 'rgb8'";
		setStatus(status_levels::Error, "Image", ss.str());
		return;
	}

	// Expand it to be RGB data
	unsigned int pixels_size = image_width_ * image_height_ * 3;
	unsigned char* pixels = new unsigned char[pixels_size];
	//memset(pixels, 255, pixels_size);


	//bool image_status_set = false;
	//unsigned int num_pixels_to_copy = pixels_size;
	if (pixels_size != image->data.size()) {
		std::stringstream ss;
		ss << "Data size doesn't match width*height: width = " << image_width_
				<< ", height = " << image_height_ << ", data size = "
				<< image->data.size();
		setStatus(status_levels::Error, "Image", ss.str());

		return;
		/*
		 image_status_set = true;

		 // Keep going, but don't read past the end of the data.
		 if( image->data.size() < pixels_size )
		 {
		 num_pixels_to_copy = image->data.size();
		 }*/
	}

	for (unsigned int pixel_index = 0; pixel_index < pixels_size; pixel_index++) {
		pixels[pixel_index] = image->data[pixel_index];
	}

	Ogre::DataStreamPtr pixel_stream;
	pixel_stream.bind(new Ogre::MemoryDataStream(pixels, pixels_size));
	static int tex_count = 0;
	std::stringstream ss;
	ss << "CamTexture" << tex_count++;
	try {
		texture_ = Ogre::TextureManager::getSingleton().loadRawData(ss.str(),
				Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
				pixel_stream, image_width_, image_height_, Ogre::PF_B8G8R8,
				Ogre::TEX_TYPE_2D, 0);

		setStatus(status_levels::Ok, "Image", "Image OK");
		/*if( !map_status_set )
		 {
		 setStatus(status_levels::Ok, "Map", "Map OK");
		 }*/
	} catch (Ogre::RenderingAPIException&) {
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

	delete[] pixels;

	Ogre::Pass* pass = material_->getTechnique(0)->getPass(0);
	Ogre::TextureUnitState* tex_unit = NULL;
	if (pass->getNumTextureUnitStates() > 0) {
		tex_unit = pass->getTextureUnitState(0);
	} else {
		tex_unit = pass->createTextureUnitState();
	}

	tex_unit->setTextureName(texture_->getName());
	tex_unit->setTextureFiltering(Ogre::TFO_NONE);

	image_loaded_ = true;

	causeRender();

}
void ButCamDisplay::load(const srs_ui_but::ButCamMsg::ConstPtr& msg) {
	ROS_DEBUG("Loading marker");

	setStatus(status_levels::Ok, "Marker", "Marker OK");

	if (!validateFloats(*msg)) {
		setStatus(status_levels::Error, "Marker",
				"Message contained invalid floating point values (nans or infs)");
		return;
	}

	//clearMarker();

	// Pad dimensions to power of 2
	width_ = msg->scale.x;//(int)pow(2,ceil(log2(msg->info.width)));
	height_ = msg->scale.y;//(int)pow(2,ceil(log2(msg->info.height)));

	//printf("Padded dimensions to %d X %d\n", width_, height_);

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
		material_->getTechnique(0)->setLightingEnabled(true);

		float r = 1.0;
		float g = 1.0;
		float b = 1.0;
		float a = 1.0;
		material_->getTechnique(0)->setAmbient(r, g, b);

		material_->getTechnique(0)->setDiffuse(r, g, b, a);

		static int cam_count = 0;
		std::stringstream ss2;
		ss2 << "CamObject" << cam_count++;
		manual_object_ = scene_manager_->createManualObject(ss2.str());
		scene_node_->attachObject(manual_object_);

		//manual_object_->setMaterialName()

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
		manual_object_->setRenderQueueGroup(Ogre::RENDER_QUEUE_3);
	}

	propertyChanged(resolution_property_);
	propertyChanged(width_property_);
	propertyChanged(width_property_);
	propertyChanged(position_property_);
	propertyChanged(orientation_property_);

	transformCam();

	marker_loaded_ = true;

	causeRender();
}

void ButCamDisplay::transformCam() {
	ROS_DEBUG_NAMED("ButCamDisplay", "transforming cam from frame '%s' to frame '%s'", frame_.c_str(), fixed_frame_.c_str());

	if (!marker_) {
		return;
	}

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

	scene_node_->setPosition(position);
	scene_node_->setOrientation(orientation);
	scene_node_->setScale(Ogre::Vector3(marker_->scale.x, marker_->scale.y,
			marker_->scale.z));
}

void ButCamDisplay::update(float wall_dt, float ros_dt) {
	//ROS_DEBUG("ButCamDisplay: update");
	//ROS_DEBUG_NAMED("ButCamDisplay", "update");
}

void ButCamDisplay::createProperties() {
	ROS_DEBUG("ButCamDisplay: createProperties");

	// the name "marker_topic" is residue, when polygon was defined by marker message
	marker_topic_property_ = property_manager_->createProperty<
			ROSTopicStringProperty> ("Marker Topic", property_prefix_,
			boost::bind(&ButCamDisplay::getMarkerTopic, this), boost::bind(
					&ButCamDisplay::setMarkerTopic, this, _1),
			parent_category_, this);
	setPropertyHelpText(marker_topic_property_,
			"srs_ui_but::ButCamMsg topic to subscribe to.");
	ROSTopicStringPropertyPtr marker_topic_prop = marker_topic_property_.lock();
	marker_topic_prop->setMessageType(ros::message_traits::datatype<
			srs_ui_but::ButCamMsg>());
	marker_topic_prop->addLegacyName("Service"); // something of a hack, but should provide reasonable backwards compatibility

	image_topic_property_ = property_manager_->createProperty<
			ROSTopicStringProperty> ("Image Topic", property_prefix_,
			boost::bind(&ButCamDisplay::getImageTopic, this), boost::bind(
					&ButCamDisplay::setImageTopic, this, _1), parent_category_,
			this);
	setPropertyHelpText(image_topic_property_,
			"sensor_msgs::Image topic to subscribe to.");
	ROSTopicStringPropertyPtr image_topic_prop = image_topic_property_.lock();
	image_topic_prop->setMessageType(ros::message_traits::datatype<
			sensor_msgs::Image>());
	image_topic_prop->addLegacyName("Service"); // something of a hack, but should provide reasonable backwards compatibility

	alpha_property_ = property_manager_->createProperty<FloatProperty> (
			"Alpha", property_prefix_, boost::bind(&ButCamDisplay::getAlpha,
					this), boost::bind(&ButCamDisplay::setAlpha, this, _1),
			parent_category_, this);
	setPropertyHelpText(alpha_property_,
			"Amount of transparency to apply to the marker.");
	draw_under_property_ = property_manager_->createProperty<BoolProperty> (
			"Draw Behind", property_prefix_, boost::bind(
					&ButCamDisplay::getDrawUnder, this), boost::bind(
					&ButCamDisplay::setDrawUnder, this, _1), parent_category_,
			this);
	setPropertyHelpText(
			draw_under_property_,
			"Rendering option, controls whether or not the marker is always drawn behind everything else.");

	resolution_property_ = property_manager_->createProperty<FloatProperty> (
			"Resolution", property_prefix_, boost::bind(
					&ButCamDisplay::getResolution, this),
			FloatProperty::Setter(), parent_category_, this);
	setPropertyHelpText(resolution_property_,
			"Resolution of the image. (not editable)");
	width_property_ = property_manager_->createProperty<FloatProperty> (
			"Marker Width", property_prefix_, boost::bind(
					&ButCamDisplay::getWidth, this), FloatProperty::Setter(),
			parent_category_, this);
	setPropertyHelpText(width_property_,
			"Width of the marker, in meters. (not editable)");
	height_property_ = property_manager_->createProperty<FloatProperty> (
			"Marker Height", property_prefix_, boost::bind(
					&ButCamDisplay::getHeight, this), FloatProperty::Setter(),
			parent_category_, this);
	setPropertyHelpText(height_property_,
			"Height of the marker, in meters. (not editable)");

	image_width_property_ = property_manager_->createProperty<IntProperty> (
			"Image Width", property_prefix_, boost::bind(
					&ButCamDisplay::getImageWidth, this),
			IntProperty::Setter(), parent_category_, this);
	setPropertyHelpText(image_width_property_,
			"Width of the camera image, in pixels. (not editable)");
	image_height_property_ = property_manager_->createProperty<IntProperty> (
			"Image Height", property_prefix_, boost::bind(
					&ButCamDisplay::getImageHeight, this),
			IntProperty::Setter(), parent_category_, this);
	setPropertyHelpText(image_height_property_,
			"Height of the camera image, in pixels. (not editable)");

	position_property_ = property_manager_->createProperty<Vector3Property> (
			"Position", property_prefix_, boost::bind(
					&ButCamDisplay::getPosition, this),
			Vector3Property::Setter(), parent_category_, this);
	setPropertyHelpText(position_property_,
			"Position of the middle of the marker, in meters. (not editable)");
	orientation_property_ = property_manager_->createProperty<
			QuaternionProperty> ("Orientation", property_prefix_, boost::bind(
			&ButCamDisplay::getOrientation, this),
			QuaternionProperty::Setter(), parent_category_, this);
	setPropertyHelpText(orientation_property_,
			"Orientation of the marker. (not editable)");
}

void ButCamDisplay::fixedFrameChanged() {
	ROS_DEBUG("ButCamDisplay: fixedFrameChanged");
	transformCam();
}

void ButCamDisplay::reset() {
	ROS_DEBUG("ButCamDisplay: reset");
	Display::reset();

	clear();
	// Force resubscription so that the map will be re-sent
	setMarkerTopic(marker_topic_);
	setImageTopic(image_topic_);
}

void ButCamDisplay::incoming(const srs_ui_but::ButCamMsg::ConstPtr& msg,
		const sensor_msgs::Image::ConstPtr& image) {
	ROS_DEBUG("incoming synchronized");
	load(msg);
	loadImage(image);
}

void ButCamDisplay::incomingMarker(const srs_ui_but::ButCamMsg::ConstPtr& msg) {
	ROS_DEBUG_NAMED("ButCamDisplay", "incoming marker");

	load(msg);
}

void ButCamDisplay::incomingImage(const sensor_msgs::Image::ConstPtr& image) {
	ROS_DEBUG_NAMED("ButCamDisplay", "incoming image");

	loadImage(image);
}

} // namespace rviz
