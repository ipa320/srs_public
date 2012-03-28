/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 24.2.2012
 *******************************************************************************
 */

#include <but_gui/ObjectWithBoundingBox.h>

namespace but_gui
{

ObjectWithBoundingBox::ObjectWithBoundingBox(InteractiveMarkerServerPtr server, string frame_id, string name) :
  BoundingBox(server, frame_id, name)
{
  setPrimitiveType(srs_env_model::PrimitiveType::OBJECT_WITH_BOUNDING_BOX);
  use_material_ = false;
  pregrasp1_.name = "control_grasp_xp";
  pregrasp2_.name = "control_grasp_xm";
  pregrasp3_.name = "control_grasp_yp";
  pregrasp4_.name = "control_grasp_ym";
  pregrasp5_.name = "control_grasp_zp";
  pregrasp6_.name = "control_grasp_zm";
}

void ObjectWithBoundingBox::objectWithBoundingBoxCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
{
  defaultCallback(feedback);
}

void ObjectWithBoundingBox::menuCallback(const InteractiveMarkerFeedbackConstPtr &feedback)
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
    pose_ = o.pose;
    object_.pose = pose_;
  }

  switch (feedback->menu_entry_id)
  {
    case 1:
      /**
       * Object's bounding box visibility
       */
      if (state == MenuHandler::CHECKED)
      {
        //TODO predeleat na remove a add
        showBoundingBoxControl(false);
        menu_handler_.setCheckState(handle, MenuHandler::UNCHECKED);
      }
      else
      {
        showBoundingBoxControl(true);
        menu_handler_.setCheckState(handle, MenuHandler::CHECKED);
      }
      break;
    case 2:
      /**
       * Object's description visibility
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
    case 3:
      /**
       * Object's measure visibility
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
      /**
       * Object's grasping positions visibility
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
    case 6:
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
    case 7:
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
  }

  server_->insert(object_);
  menu_handler_.reApply(*server_);
  server_->applyChanges();
}

void ObjectWithBoundingBox::createMenu()
{
  if (!menu_created_)
  {
    menu_created_ = true;
    menu_handler_.setCheckState(menu_handler_.insert("Show bounding box",
                                                     boost::bind(&ObjectWithBoundingBox::menuCallback, this, _1)),
                                MenuHandler::CHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show description",
                                                     boost::bind(&ObjectWithBoundingBox::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show measure", boost::bind(&ObjectWithBoundingBox::menuCallback,
                                                                                 this, _1)), MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert("Show pre-grasp positions",
                                                     boost::bind(&ObjectWithBoundingBox::menuCallback, this, _1)),
                                MenuHandler::CHECKED);
    addPregraspPositions();

    MenuHandler::EntryHandle sub_menu_handle = menu_handler_.insert("Interaction");
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Movement",
                                                     boost::bind(&ObjectWithBoundingBox::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);
    menu_handler_.setCheckState(menu_handler_.insert(sub_menu_handle, "Rotation",
                                                     boost::bind(&ObjectWithBoundingBox::menuCallback, this, _1)),
                                MenuHandler::UNCHECKED);

  }
}

void ObjectWithBoundingBox::createMesh()
{
  mesh_.color = color_;
  mesh_.scale = Vector3();
  switch (object_resource_)
  {
    case RESOURCE_FILE:
      mesh_.type = Marker::MESH_RESOURCE;
      mesh_.mesh_use_embedded_materials = use_material_;
      mesh_.mesh_resource = resource_;
      break;
    case SHAPE:
      mesh_.type = Marker::TRIANGLE_LIST;
      for (int i = 0; i < shape_.triangles.size(); i++)
        mesh_.points.push_back(shape_.vertices[shape_.triangles[i]]);
      break;
  }
}

void ObjectWithBoundingBox::create()
{
  scale_.x = fabs(bounding_box_min_.x - bounding_box_max_.x) / 2;
  scale_.y = fabs(bounding_box_min_.y - bounding_box_max_.y) / 2;
//  scale_.z = fabs(bounding_box_min_.z - bounding_box_max_.z) * 1.5;
  scale_.z = fabs(bounding_box_min_.z - bounding_box_max_.z) / 2;

  BoundingBox::create();
  createMesh();
  control_.markers.push_back(mesh_);
  object_.controls.clear();
  object_.controls.push_back(control_);

  createMenu();
}

void ObjectWithBoundingBox::insert()
{
  create();
  updateControls();
  server_->insert(object_, boost::bind(&ObjectWithBoundingBox::objectWithBoundingBoxCallback, this, _1));
  menu_handler_.apply(*server_, name_);
}

}
