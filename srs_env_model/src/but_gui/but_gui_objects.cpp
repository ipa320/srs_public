/******************************************************************************
 * \file
 *
 * $Id: but_gui_objects.cpp 397 2012-03-29 12:50:30Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 05/12/2011
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

#include "but_gui/BoundingBox.h"
#include "but_gui/Billboard.h"
#include "but_gui/Plane.h"
#include "but_gui/UnknownObject.h"
#include "but_gui/Object.h"
#include "but_gui/ObjectWithBoundingBox.h"
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
  Vector3 s;
  s.x = 1.0;
  s.y = 1.0;
  s.z = 1.0;

  // Object
  Billboard *chairBillboard = new Billboard(server, "/world", "person");
  chairBillboard->setType(srs_env_model::BillboardType::CHAIR);
  chairBillboard->setPose(p);
  chairBillboard->setScale(s);
  chairBillboard->setFrameID("/world");
  chairBillboard->insert();
  // Bounding box
  BoundingBox * chairBoundingBox = new BoundingBox(server, "/world", chairBillboard->getName() + "_bbox");
  chairBoundingBox->setAttachedObjectName(chairBillboard->getName());
  chairBoundingBox->setPose(p);
  chairBoundingBox->setScale(s);
  chairBoundingBox->setColor(c);
  chairBoundingBox->setDescription("Person bounding box");
  chairBoundingBox->insert();

  p.position.x = 1;
  p.position.y = 2;
  p.position.z = 2;
  s.x = 1;
  s.y = 1;
  s.z = 1;
  // Object
  Billboard *milkBillboard = new Billboard(server, "/world", "milk_billboard");
  Quaternion direction;
  direction.x = 2.0;
  direction.y = 1.0;
  direction.z = 3.0;
  direction.w = 1.0;
  milkBillboard->setType(srs_env_model::BillboardType::MILK);
  milkBillboard->setPose(p);
  milkBillboard->setScale(s);
  milkBillboard->setDirection(direction);
  milkBillboard->setVelocity(3.4);
  milkBillboard->setDescription("MLEEEEKO");
  milkBillboard->insert();
  // Bounding box

  /* BoundingBox * milkBoundingBox = new BoundingBox(server, "/world", milkBillboard->getName() + "_bbox");
   milkBoundingBox->setAttachedObjectName(milkBillboard->getName());
   milkBoundingBox->setFrameID("/world");
   milkBoundingBox->setPose(p);
   milkBoundingBox->setScale(s);
   milkBoundingBox->setColor(c);
   milkBoundingBox->setDescription("Table bounding box");
   milkBoundingBox->insert();*/

  UnknownObject * unknowObject = new UnknownObject(server, "/world", "unknown_object");
  unknowObject->setFrameID("/world");
  Pose pp;
  pp.position.x = 0;
  pp.position.y = 0;
  pp.position.z = 0;
  unknowObject->setPose(pp);
  Vector3 ss;
  ss.x = 1;
  ss.y = 1;
  ss.z = 1;
  unknowObject->setScale(ss);
  unknowObject->setDescription("Uknown object");
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
  p.position.x = 20;
  object.pose = p;
  InteractiveMarkerControl control;
  control.name = "sphere_control";
  control.interaction_mode = InteractiveMarkerControl::NONE;
  control.always_visible = true;
  control.markers.push_back(sphere);
  object.controls.push_back(control);
  server->insert(object);
  // Bounding box
  c.r = 1;
  c.g = 0;
  c.b = 0;
  BoundingBox * sphereBoundingBox = new BoundingBox(server, "/world", "sphere_bbox");
  sphereBoundingBox->setAttachedObjectName("sphere");
  sphereBoundingBox->setPose(p);
  sphereBoundingBox->setScale(s);
  sphereBoundingBox->setColor(c);
  sphereBoundingBox->setDescription("Sphere bounding box");
  sphereBoundingBox->insert();

  Object * obj = new Object(server, "/world", "table_object");
  obj->setFrameID("/world");
  Pose ppp;
  ppp.position.x = 6;
  ppp.position.y = 5;
  ppp.position.z = 0;
  obj->setPose(ppp);
  Vector3 sss;
  sss.x = 1;
  sss.y = 1;
  sss.z = 1;
  c.a = 1.0;
  obj->setScale(sss);
  obj->setDescription("Table");
  obj->setColor(c);
  obj->setResource("package://gazebo_worlds/Media/models/table.dae");
  obj->setUseMaterial(false);
  obj->insert();

  ObjectWithBoundingBox * objbb = new ObjectWithBoundingBox(server, "/world", "table_with_bb");
  ppp.position.x = 2;
  ppp.position.y = 2;
  ppp.position.z = 2;
  objbb->setPose(ppp);
  c.a = 1.0;
  Vector3 gp;
  gp.x = 0.7;
  gp.y = 1.2;
  gp.z = 0;
  objbb->setGraspingPosition(GRASP_1, gp);
  gp.x = 0;
  gp.y = 1.2;
  gp.z = 0.9;
  objbb->setGraspingPosition(GRASP_2, gp);
  gp.x = 0.1;
  gp.y = 0.1;
  gp.z = 0.1;
  objbb->setGraspingPosition(GRASP_3, gp);
  Scale sbb;
  sbb.x = 0.2;
  sbb.y = 0.2;
  sbb.z = 0.2;
  Point bbm;
  bbm = Point();
  bbm.x = 1;
  bbm.y = 1;
  bbm.z = 1;
  objbb->setBoundingBoxLWH(bbm);
  objbb->setDescription("Table with Bounding Box");
  objbb->setColor(c);
  objbb->setResource("package://gazebo_worlds/Media/models/table.dae");
  objbb->setUseMaterial(true);

  arm_navigation_msgs::Shape shape;
  Point sp;
  sp.x = 0;
  sp.y = 0;
  sp.z = 0;
  shape.vertices.push_back(sp);
  sp.x = 1;
  shape.vertices.push_back(sp);
  sp.y = 2;
  shape.vertices.push_back(sp);
  shape.triangles.push_back(0);
  shape.triangles.push_back(1);
  shape.triangles.push_back(2);
  objbb->setShape(shape);

  objbb->insert();

  server->applyChanges();
  ros::spin();
}
