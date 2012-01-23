/*
 *******************************************************************************
 * $Id: but_gui_objects.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 24.11.2011
 * Description:
 *      This is example!
 *******************************************************************************
 */

#include "but_gui/BoundingBox.h"
#include "but_gui/Billboard.h"
#include "but_gui/Plane.h"
#include "but_gui/UnknownObject.h"
#include "math.h"

using namespace but_gui;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "objects");

  InteractiveMarkerServerPtr server;
  server.reset(new InteractiveMarkerServer("but_gui", "", false));

  ColorRGBA c;
  c.a = 0.2;
  c.r = 1.0;
  c.g = 0.0;
  c.b = 0.0;
  Pose p;
  p.position.x = 5;
  p.position.y = 2;
  p.position.z = 1;
  p.orientation.x = M_PI_4;
  p.orientation.y = 0;
  p.orientation.z = M_PI_4;
  Scale s;
  s.x = 1.0;
  s.y = 1.0;
  s.z = 1.0;

  // Object
  Billboard *chairBillboard = new Billboard(server, "/world", "person");
  chairBillboard->setType(srs_env_model::BillboardType::PERSON);
  chairBillboard->setPose(p);
  chairBillboard->setScale(s);
  chairBillboard->setFrameID("/world");
  chairBillboard->create();
  chairBillboard->insert();
  // Bounding box
  BoundingBox * chairBoundingBox = new BoundingBox(server, "/world", chairBillboard->getName() + "_bbox",
                                                   chairBillboard->getName(), p, s, c, "Person bounding box");
  chairBoundingBox->insert();

  p.position.x = 1;
  p.position.y = 2;
  p.position.z = 2;
  s.x = 0.5;
  s.y = 0.5;
  s.z = 0.5;
  // Object
  Billboard *milkBillboard = new Billboard(server, "/world", "milk", srs_env_model::BillboardType::MILK, p, s);
  Quaternion direction;
  direction.x = 2.0;
  direction.y = 1.0;
  direction.z = 3.0;
  direction.w = 1.0;
  milkBillboard->setDirection(direction);
  milkBillboard->setVelocity(3.4);
  milkBillboard->create();
  milkBillboard->insert();
  // Bounding box

  /* BoundingBox * milkBoundingBox = new BoundingBox(server, "/world", milkBillboard->getName() + "_bbox");
   milkBoundingBox->setAttachedObjectName(milkBillboard->getName());
   milkBoundingBox->setFrameID("/world");
   milkBoundingBox->setPose(p);
   milkBoundingBox->setScale(s);
   milkBoundingBox->setColor(c);
   milkBoundingBox->setDescription("Table bounding box");
   milkBoundingBox->create();
   milkBoundingBox->insert();*/

  UnknownObject * unknowObject = new UnknownObject(server, "/world", "unknown_object");
  unknowObject->setFrameID("/world");
  Pose pp;
  pp.position.x = -3;
  pp.position.y = 5;
  pp.position.z = 7;
  unknowObject->setPose(pp);
  Scale ss;
  ss.x = 4;
  ss.y = 0.7;
  ss.z = 6;
  unknowObject->setScale(ss);
  unknowObject->setDescription("Uknown object");
  unknowObject->create();
  unknowObject->insert();

  Plane *plane = new Plane(server, "/world", "plane1");
  c.a = 1.0;
  p.position.x = 0;
  p.position.y = 0;
  p.position.z = 0;
  s.x = 5;
  s.y = 2;
  plane->setColor(c);
  plane->setFrameID("/world");
  plane->setPose(p);
  plane->setScale(s);
  plane->create();
  plane->insert();
  c.g = 1.0;
  plane->changeColor(c);

  //Object
  s.x = 6;
  s.y = 3.2;
  s.z = 4;
  InteractiveMarker object;
  Marker sphere;
  sphere.type = Marker::SPHERE;
  sphere.color.r = c.g;
  sphere.color.g = c.r;
  sphere.color.b = c.b;
  sphere.color.a = c.a;
  sphere.scale = s;
  object.header.frame_id = "/world";
  object.name = "sphere";
  object.description = "Sphere";
  p.position.x = 10;
  object.pose = p;
  InteractiveMarkerControl control;
  control.name = "sphere_control";
  control.interaction_mode = InteractiveMarkerControl::NONE;
  control.always_visible = true;
  control.markers.push_back(sphere);
  object.controls.push_back(control);
  server->insert(object);
  // Bounding box
  c.r=1;
  BoundingBox * sphereBoundingBox = new BoundingBox(server, "/world", "sphere_bbox", "sphere", p, s, c,
                                                    "Sphere bounding box");
  sphereBoundingBox->insert();

  server->applyChanges();
  ros::spin();
}
