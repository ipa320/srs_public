/******************************************************************************
 * \file
 *
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 09/08/2012
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

#include "but_object_manager.h"

namespace srs_ui_but
{
CButObjectManager::CButObjectManager(const std::string & name, rviz::VisualizationManager * manager) :
    Display(name, manager)
{

  rviz::WindowManagerInterface * wi(manager->getWindowManager());

  get_object_client_ = threaded_nh_.serviceClient<srs_interaction_primitives::GetObject>(
      srs_interaction_primitives::GetObject_SRV);

  get_added_objects_service_ = threaded_nh_.advertiseService(GetAddedObjects_SRV, &CButObjectManager::getAddedObjects,
                                                             this);

  if (wi != 0)
  {
    m_objectWindow_ = new CObjectControlPane(wi->getParentWindow(), _T("Object control"), wi);

    if (m_objectWindow_ != 0)
    {
      wi->addPane("Object control", m_objectWindow_);
      wi->showPane(m_objectWindow_);
    }
  }

}

CButObjectManager::~CButObjectManager()
{
  delete m_objectWindow_;
}

void CButObjectManager::createProperties()
{

}

void CButObjectManager::onEnable()
{
  m_objectWindow_->setEnabled(true);
}

void CButObjectManager::onDisable()
{
  m_objectWindow_->setEnabled(false);
}

bool CButObjectManager::getAddedObjects(GetAddedObjects::Request &req, GetAddedObjects::Response &res)
{

  std::vector<std::string> objectsIds = m_objectWindow_->getObjectIds();
  for (unsigned int i = 0; i < objectsIds.size(); i++)
  {
    srs_interaction_primitives::GetObject srv;
    srv.request.name = objectsIds.at(i);
    if (get_object_client_.call(srv))
    {
      cob_object_detection_msgs::Detection detection;
      detection.pose.header.frame_id = srv.response.frame_id;
      detection.pose.pose = srv.response.pose;
      detection.bounding_box_lwh = srv.response.bounding_box_lwh;
      detection.label = objectsIds.at(i);

      res.object_list.detections.push_back(detection);
    }
  }
  return true;
}
}
