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

#include "object_control_pane.h"

namespace srs_ui_but
{
CObjectControlPane::CObjectControlPane(wxWindow *parent, const wxString& title, rviz::WindowManagerInterface * wmi) :
    wxPanel(parent, wxID_ANY, wxDefaultPosition, wxSize(280, 180), wxTAB_TRAVERSAL, title), m_wmi(wmi), enabled_(true)
{
  if (ros::param::has(DefaultFrame_PARAM))
    ros::param::get(DefaultFrame_PARAM, default_frame_);
  else
    default_frame_ = "/map";

  // Service clients
  // ---------------------------------------------------------------------------
  get_models_client_ = nh_.serviceClient<srs_object_database_msgs::GetObjectId>("get_models");
  get_model_mesh_client_ = nh_.serviceClient<srs_object_database_msgs::GetMesh>("get_model_mesh");
  add_object_client_ = nh_.serviceClient<srs_interaction_primitives::AddObject>(
      srs_interaction_primitives::AddObject_SRV);
  remove_primitive_client_ = nh_.serviceClient<srs_interaction_primitives::RemovePrimitive>(
      srs_interaction_primitives::RemovePrimitive_SRV);

  srs_object_database_msgs::GetObjectId srv;

  // GUI
  //----------------------------------------------------------------------------
  m_frameLabel = new wxStaticText(this, ID_FRAME_LABEL, wxT("Frame:"), wxDefaultPosition, wxDefaultSize, wxALIGN_LEFT,
                                  wxT(""));
  m_frameText = new wxTextCtrl(this, ID_FRAME_TEXT, wxString::FromUTF8(default_frame_.c_str()), wxDefaultPosition,
                               wxDefaultSize, wxTE_LEFT, wxDefaultValidator, wxT(""));

  m_descriptionLabel = new wxStaticText(this, ID_DESCRIPTION_LABEL, wxT("Description:"), wxDefaultPosition,
                                        wxDefaultSize, wxALIGN_LEFT, wxT(""));
  m_descriptionText = new wxTextCtrl(this, ID_DESCRIPTION_TEXT, wxT(""), wxDefaultPosition, wxDefaultSize, wxTE_LEFT,
                                     wxDefaultValidator, wxT(""));

  m_posLabel = new wxStaticText(this, ID_POS_LABEL, wxT("Position (x, y, z):"), wxDefaultPosition, wxDefaultSize,
                                wxALIGN_LEFT, wxT(""));
  m_posxText = new wxTextCtrl(this, ID_POSX_TEXT, wxT("0"), wxDefaultPosition, wxDefaultSize, wxTE_LEFT,
                              wxDefaultValidator, wxT(""));
  m_posyText = new wxTextCtrl(this, ID_POSY_TEXT, wxT("0"), wxDefaultPosition, wxDefaultSize, wxTE_LEFT,
                              wxDefaultValidator, wxT(""));
  m_poszText = new wxTextCtrl(this, ID_POSZ_TEXT, wxT("0"), wxDefaultPosition, wxDefaultSize, wxTE_LEFT,
                              wxDefaultValidator, wxT(""));

  //m_colorDialog = new wxColourDialog(this);
  m_colorLabel = new wxStaticText(this, ID_COLOR_LABEL, wxT("Color:"), wxDefaultPosition, wxDefaultSize, wxALIGN_LEFT,
                                  wxT(""));

  m_colorClrpicker = new wxColourPickerCtrl(this, ID_COLOR_CLRPICKER, *wxGREEN, wxDefaultPosition, wxDefaultSize,
                                            wxCLRP_DEFAULT_STYLE, wxDefaultValidator, wxColourPickerCtrlNameStr);

  m_addObjectButton = new wxButton(this, ID_ADD_OBJECT_BUTTON, wxT("Add object"), wxDefaultPosition, wxDefaultSize,
                                   wxBU_EXACTFIT);
  m_removeObjectButton = new wxButton(this, ID_REMOVE_OBJECT_BUTTON, wxT("Remove object"), wxDefaultPosition,
                                      wxDefaultSize, wxBU_EXACTFIT);

  m_statusLabel = new wxStaticText(this, ID_STATUS_LABEL, wxT(""), wxDefaultPosition, wxDefaultSize, wxALIGN_CENTRE,
                                   wxT(""));

  m_objectsChoice = new wxChoice(this, ID_OBJECTS_CHOICE, wxPoint(0, 0), wxSize(-1, -1));
  m_addedObjectsChoice = new wxChoice(this, ID_ADDED_OBJECTS_CHOICE, wxPoint(0, 0), wxSize(-1, -1));

  ROS_INFO("Service is retrieving data");
  if (get_models_client_.call(srv))
  {
    ROS_INFO("Service invoked");
    for (unsigned int i = 0; i < srv.response.model_ids.size(); i++)
    {
      database_ids_.push_back((int)srv.response.model_ids[i]);
      database_descs_.push_back((std::string)srv.response.model_category[i].c_str());
      geometry_msgs::Vector3 size;
      size.x = srv.response.model_x_size[i];
      size.y = srv.response.model_y_size[i];
      size.z = srv.response.model_z_size[i];
      database_sizes_.push_back(size);

      m_objectsChoice->AppendString(wxString::FromUTF8((const char*)srv.response.model_category[i].c_str()));
    }
    m_objectsChoice->SetSelection(0);
  }
  else
  {
    setStatus("Failed to call service /get_models");
  }

  wxSizer *mainSizer = new wxBoxSizer(wxVERTICAL);
  this->SetSizer(mainSizer);

  wxSizer *sizer = new wxBoxSizer(wxHORIZONTAL);
  wxSizer *col1 = new wxBoxSizer(wxVERTICAL);
  col1->Add(m_frameLabel, ID_FRAME_LABEL, wxALIGN_LEFT);
  col1->Add(m_posLabel, ID_POS_LABEL, wxALIGN_LEFT);
  col1->Add(m_descriptionLabel, ID_DESCRIPTION_LABEL, wxALIGN_LEFT);
  col1->Add(m_colorLabel, ID_COLOR_LABEL, wxALIGN_LEFT);
  col1->Add(m_objectsChoice, ID_OBJECTS_CHOICE, wxALIGN_LEFT);
  col1->Add(m_addedObjectsChoice, ID_OBJECTS_CHOICE, wxALIGN_LEFT);
  sizer->Add(col1, 0, wxALIGN_TOP);

  wxSizer *col2 = new wxBoxSizer(wxVERTICAL);
  col2->Add(m_frameText, ID_FRAME_TEXT, wxALIGN_LEFT);
  wxSizer *posRow = new wxBoxSizer(wxHORIZONTAL);
  posRow->Add(m_posxText, ID_POSX_TEXT, wxALIGN_LEFT);
  posRow->Add(m_posyText, ID_POSY_TEXT, wxALIGN_LEFT);
  posRow->Add(m_poszText, ID_POSZ_TEXT, wxALIGN_LEFT);
  col2->Add(posRow, 0, wxALIGN_LEFT);
  col2->Add(m_descriptionText, ID_DESCRIPTION_TEXT, wxALIGN_LEFT);
  col2->Add(m_colorClrpicker, ID_COLOR_CLRPICKER, wxALIGN_LEFT);
  col2->Add(m_addObjectButton, ID_ADD_OBJECT_BUTTON, wxALIGN_LEFT);
  col2->Add(m_removeObjectButton, ID_REMOVE_OBJECT_BUTTON, wxALIGN_LEFT);
  sizer->Add(col2, 0, wxALIGN_TOP);

  mainSizer->Add(sizer, 0, wxALIGN_LEFT);
  mainSizer->Add(m_statusLabel, ID_STATUS_LABEL, wxALIGN_LEFT);

  mainSizer->SetSizeHints(this);
}

void CObjectControlPane::OnRemoveObject(wxCommandEvent& event)
{
  if (enabled_)
  {
    srs_interaction_primitives::RemovePrimitive remove_primitive_srv;
    remove_primitive_srv.request.name = m_addedObjectsChoice->GetStringSelection().ToUTF8().data();
    if (remove_primitive_client_.call(remove_primitive_srv))
    {
      setStatus("Object removed");
      m_addedObjectsChoice->Delete(m_addedObjectsChoice->GetSelection());
      m_addedObjectsChoice->SetSelection(0);
    }
  }
}

void CObjectControlPane::OnAddObject(wxCommandEvent& event)
{
  if (enabled_)
  {
    srs_object_database_msgs::GetMesh get_mesh_srv;
    srs_interaction_primitives::AddObject add_object_srv;

    std::vector<int> ids;
    int index = m_objectsChoice->GetSelection();
    ids.push_back(database_ids_.at(index));
    get_mesh_srv.request.model_ids = ids;

    if (get_model_mesh_client_.call(get_mesh_srv))
    {
      if (get_mesh_srv.response.msg.size() > 0)
      {
        //if (m_colorDialog->ShowModal() == wxID_CANCEL)
        //return;

        static int object_count = 0;
        std::stringstream object_name;
        object_name << "om_";
        object_name << object_count;
        double posx, posy, posz;
        m_posxText->GetValue().ToDouble(&posx);
        m_posyText->GetValue().ToDouble(&posy);
        m_poszText->GetValue().ToDouble(&posz);

        add_object_srv.request.frame_id = m_frameText->GetValue().ToUTF8().data();
        add_object_srv.request.name = object_name.str();
        add_object_srv.request.description = m_descriptionText->GetValue().ToUTF8().data();
        add_object_srv.request.pose.position.x = posx;
        add_object_srv.request.pose.position.y = posy;
        add_object_srv.request.pose.position.z = posz;
        add_object_srv.request.color.r = (float)(m_colorClrpicker->GetColour().Red()) / 255;
        add_object_srv.request.color.g = (float)(m_colorClrpicker->GetColour().Green()) / 255;
        add_object_srv.request.color.b = (float)(m_colorClrpicker->GetColour().Blue()) / 255;
        add_object_srv.request.color.a = (float)(m_colorClrpicker->GetColour().Alpha()) / 255;
        /*add_object_srv.request.color.r = (float)(m_colorDialog->GetColourData().GetColour().Red()) / 255;
         add_object_srv.request.color.g = (float)(m_colorDialog->GetColourData().GetColour().Green()) / 255;
         add_object_srv.request.color.b = (float)(m_colorDialog->GetColourData().GetColour().Blue()) / 255;
         add_object_srv.request.color.a = (float)(m_colorDialog->GetColourData().GetColour().Alpha()) / 255;*/
        add_object_srv.request.pose_type = 0;
        add_object_srv.request.bounding_box_lwh.x = database_sizes_.at(index).x;
        add_object_srv.request.bounding_box_lwh.y = database_sizes_.at(index).y;
        add_object_srv.request.bounding_box_lwh.z = database_sizes_.at(index).z;
        add_object_srv.request.shape = get_mesh_srv.response.msg[0].mesh;

        if (add_object_client_.call(add_object_srv))
        {
          m_addedObjectsChoice->AppendString(wxString::FromUTF8((const char*)object_name.str().c_str()));
          m_addedObjectsChoice->SetSelection(0);
          std::string text = "Object ";
          text.append(database_descs_.at(index));
          text.append(" added.");
          setStatus(text);
          //object_count_++;
          object_count++;
        }
        else
        {
          setStatus("Cannot add object!");
        }
      }
      else
      {
        setStatus("Mesh not found!");
      }
    }
  }
}

std::vector<std::string> CObjectControlPane::getObjectIds()
{
  std::vector<std::string> objectIds;
  for (int i = 0; i < m_addedObjectsChoice->GetStrings().size(); i++)
  {
    objectIds.push_back(m_addedObjectsChoice->GetStrings().Item(i).ToUTF8().data());
  }
  return objectIds;
}

}
///////////////////////////////////////////////////////////////////////////////
BEGIN_EVENT_TABLE(srs_ui_but::CObjectControlPane, wxPanel) EVT_BUTTON(ID_ADD_OBJECT_BUTTON, srs_ui_but::CObjectControlPane::OnAddObject)
EVT_BUTTON(ID_REMOVE_OBJECT_BUTTON, srs_ui_but::CObjectControlPane::OnRemoveObject)
END_EVENT_TABLE()
