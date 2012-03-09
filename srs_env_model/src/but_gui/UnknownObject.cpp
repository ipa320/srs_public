/*
 *******************************************************************************
 * $Id: UnknownObject.cpp 252 2012-02-24 10:54:11Z xlokaj03 $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 11.12.2011
 * Description:
 *******************************************************************************
 */

#include "but_gui/UnknownObject.h"

namespace but_gui
{

UnknownObject::UnknownObject(InteractiveMarkerServerPtr server, string frame_id, string name) :
  Primitive(server, frame_id, name, srs_env_model::PrimitiveType::UNKNOWN_OBJECT)
{
  description_ = "";
  color_.r = 0.3;
  color_.g = 0.5;
  color_.b = 0.6;
  color_.a = 1.0;
  show_movement_control_ = show_scale_control_ = show_rotation_control_ = show_measure_control_
      = show_description_control_ = false;
}

void UnknownObject::uboxCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (show_scale_control_)
  {
    updateScaleControls();
    server_->applyChanges();
  }

  defaultCallback(feedback);
}

void UnknownObject::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  MenuHandler::EntryHandle handle = feedback->menu_entry_id;
  MenuHandler::CheckState state;
  string title;
  menu_handler_.getCheckState(handle, state);
  menu_handler_.getTitle(handle, title);

  updatePublisher_->publishMenuClicked(title, state);

  InteractiveMarker o;
  if (server_->get(name_, o))
  {
    object_.pose = o.pose;
    switch (feedback->menu_entry_id)
    {
      case 1:
        /**
         * Uknown object description
         */
        if (state == MenuHandler::CHECKED)
        {
          removeDescriptionControl();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addDescriptionControl();
          menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 2:
        /**
         * Unknown object measure
         */
        if (state == MenuHandler::CHECKED)
        {
          removeMeasureControl();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addMeasureControl();
          menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 4:
        /*
         * Movement controls
         */
        if (state == MenuHandler::CHECKED)
        {
          removeMovementControls();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addMovementControls();
          menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 5:
        /*
         * Rotation controls
         */
        if (state == MenuHandler::CHECKED)
        {
          removeRotationControls();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addRotationControls();
          menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 6:
        /*
         * Scale controls
         */
        if (state == MenuHandler::CHECKED)
        {
          removeScaleControls();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addScaleControls();
          menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
    }
    server_->insert(object_);
  }

  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void UnknownObject::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(menu_handler_.insert("Show description", boost::bind(&UnknownObject::menuCallback,
                                                                                     this, _1)), MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show measure",
                                                     boost::bind(&UnknownObject::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);

    MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Interaction");
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Movement",
                                                     boost::bind(&UnknownObject::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Rotation",
                                                     boost::bind(&UnknownObject::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Scale",
                                                     boost::bind(&UnknownObject::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
  }
}

void UnknownObject::createBox()
{
  box_.type = Marker::MESH_RESOURCE;
  box_.mesh_use_embedded_materials = true;
  box_.scale = scale_;
  box_.mesh_resource = "package://srs_env_model/meshes/unknown_object.dae";
}

void UnknownObject::createUnknownBox()
{
  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = description_;
  object_.pose = pose_;
  object_.scale = but_gui::maxScale(scale_);
}

void UnknownObject::create()
{
  clearObject();

  createUnknownBox();

  control_.name = "box_control";
  createBox();
  control_.markers.push_back(box_);
  control_.interaction_mode = InteractiveMarkerControl::MENU;
  control_.always_visible = true;
  object_.controls.push_back(control_);

  createMenu();
}

void UnknownObject::insert()
{
  create();

  server_->insert(object_, boost::bind(&UnknownObject::uboxCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
