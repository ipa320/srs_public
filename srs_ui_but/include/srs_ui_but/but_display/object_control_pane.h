/******************************************************************************
 * \file
 *
 * $Id$
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
#pragma once
#ifndef OBJECT_CONTROL_PANE_H
#define OBJECT_CONTROL_PANE_H

#include <ros/ros.h>
#include <ros/service.h>

#include <rviz/window_manager_interface.h>
#include <rviz/visualization_manager.h>

#include <srs_object_database_msgs/GetObjectId.h>
#include <srs_object_database_msgs/GetMesh.h>
#include <srs_interaction_primitives/services_list.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <wx/wx.h>
#include <wx/menu.h>
#include <wx/panel.h>
#include <wx/dialog.h>
#include <wx/choice.h>
#include <wx/textctrl.h>
#include <wx/stattext.h>
#include <wx/colordlg.h>
#include <wx/clrpicker.h>

#include <boost/shared_ptr.hpp>

#include <string>
#include <vector>

#define DefaultFrame_PARAM "/srs_ui_but/default_frame"

const int ID_ADD_OBJECT_BUTTON(101);
const int ID_OBJECTS_CHOICE(102);
const int ID_STATUS_LABEL(103);
const int ID_DESCRIPTION_LABEL(104);
const int ID_DESCRIPTION_TEXT(105);
const int ID_COLOR_LABEL(106);
const int ID_FRAME_LABEL(107);
const int ID_FRAME_TEXT(108);
const int ID_POS_LABEL(109);
const int ID_POSX_TEXT(110);
const int ID_POSY_TEXT(111);
const int ID_POSZ_TEXT(112);
const int ID_ADDED_OBJECTS_CHOICE(113);
const int ID_REMOVE_OBJECT_BUTTON(114);
const int ID_COLOR_CLRPICKER(115);

namespace rviz
{
class WindowManagerInterface;
}

namespace srs_ui_but
{

class CObjectControlPane : public wxPanel
{
public:
  /**
   * @brief Constructors
   */
  CObjectControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi);

  /*
   * @brief Enables or disables
   */
  void setEnabled(bool enabled)
  {
    enabled_ = enabled;
    if (enabled)
      setStatus("Enabled");
    else
      setStatus("Disabled");
  }

  std::vector<std::string> getObjectIds();

protected:
  /**
   * @brief On add object command event handler
   */
  void OnAddObject(wxCommandEvent& event);

  /**
   * @brief On remove object command event handler
   */
  void OnRemoveObject(wxCommandEvent& event);

  /**
   * @brief Sets status
   */
  void setStatus(std::string text)
  {
    m_statusLabel->SetLabel(wxString::FromUTF8((const char*)text.c_str()));
  }

  void poseClickedCallback(const geometry_msgs::PoseWithCovarianceStamped &update);

  bool enabled_;

  rviz::WindowManagerInterface * m_wmi;

  int object_count_;
  std::string default_frame_;

  wxButton *m_addObjectButton, *m_removeObjectButton;
  wxChoice *m_objectsChoice, *m_addedObjectsChoice;
  wxTextCtrl *m_descriptionText, *m_frameText, *m_posxText, *m_posyText, *m_poszText;
  wxStaticText *m_descriptionLabel, *m_statusLabel, *m_frameLabel, *m_posLabel, *m_colorLabel;
  //wxColourDialog *m_colorDialog;
  wxColourPickerCtrl *m_colorClrpicker;

  // Service clients
  ros::ServiceClient get_models_client_, get_model_mesh_client_, add_object_client_, remove_primitive_client_;

  // Get node handle
  ros::NodeHandle nh_;

  // Containers
  std::map<int, const char*> database_objects_;
  std::vector<int> database_ids_;
  std::vector<std::string> database_descs_;
  std::vector<geometry_msgs::Vector3> database_sizes_;

  // // Topic soubscribers
  ros::Subscriber pose_clicked_;

private:
DECLARE_EVENT_TABLE()

};

} // namespace srs_ui_but

// OBJECT_CONTROL_PANE_H
#endif

