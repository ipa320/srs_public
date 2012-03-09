/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 20.1.2012
 * Description:
 *******************************************************************************
 */

#include <but_gui/Object.h>

namespace but_gui
{

Object::Object(InteractiveMarkerServerPtr server, string frame_id, string name) :
  Primitive(server, frame_id, name, srs_env_model::PrimitiveType::OBJECT)
{
  description_ = "";
  use_material_ = false;
  pregrasp1_.name = "control_grasp_1";
  pregrasp2_.name = "control_grasp_2";
  pregrasp3_.name = "control_grasp_3";
  pregrasp4_.name = "control_grasp_4";
  pregrasp5_.name = "control_grasp_5";
  pregrasp6_.name = "control_grasp_6";
}

void Object::objectCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (show_scale_control_)
  {
    updateScaleControls();
    server_->applyChanges();
  }
  defaultCallback(feedback);
}

void Object::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
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
         * Object description
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
         * Object measure
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
      case 3:
        /**
         * Object grasping positions
         */
        if (state == MenuHandler::CHECKED)
        {
          removePregraspPositions();
          menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
        }
        else
        {
          addPregraspPositions();
          menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
        }
        break;
      case 5:
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
      case 6:
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
      case 7:
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

void Object::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(menu_handler_.insert("Show description", boost::bind(&Object::menuCallback, this, _1)),
                               MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show measure", boost::bind(&Object::menuCallback, this, _1)),
                               MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show grasping positions", boost::bind(&Object::menuCallback, this,
                                                                                          _1)), MenuHandler::UNCHECKED);

    MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Interaction");
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Movement", boost::bind(&Object::menuCallback,
                                                                                            this, _1)),
                               MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Rotation", boost::bind(&Object::menuCallback,
                                                                                            this, _1)),
                               MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Scale", boost::bind(&Object::menuCallback, this,
                                                                                         _1)), MenuHandler::UNCHECKED);
  }
}

void Object::createBox()
{
  mesh_.type = Marker::MESH_RESOURCE;
  mesh_.color = color_;
  mesh_.mesh_use_embedded_materials = use_material_;
  mesh_.scale = scale_;
  mesh_.mesh_resource = resource_;
}

void Object::createObjectBox()
{
  object_.header.frame_id = frame_id_;
  object_.header.stamp = ros::Time::now();
  object_.name = name_;
  object_.description = description_;
  object_.pose = pose_;
  object_.scale = but_gui::maxScale(scale_);
}

void Object::create()
{
  clearObject();

  createObjectBox();

  control_.name = "box_control";
  createBox();
  control_.markers.push_back(mesh_);
  control_.interaction_mode = InteractiveMarkerControl::MENU;
  control_.always_visible = true;
  object_.controls.push_back(control_);

  createMenu();
}

void Object::insert()
{
  create();

  server_->insert(object_, boost::bind(&Object::objectCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
