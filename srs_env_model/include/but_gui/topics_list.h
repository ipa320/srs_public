/******************************************************************************
 * \file
 *
 * $Id: topics_list.h 603 2012-04-16 10:50:03Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 05/03/2012
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

#ifndef TOPICS_LIST_H_
#define TOPICS_LIST_H_

#include <srs_env_model/PoseChanged.h>
#include <srs_env_model/ScaleChanged.h>
#include <srs_env_model/MenuClicked.h>
#include <srs_env_model/MovementChanged.h>
#include <srs_env_model/TagChanged.h>
#include <srs_env_model/PrimitiveType.h>

#define BUT_GUI_PREFIX std::string("/but_gui")
#define BUT_GUI_UPDATE_TOPIC(marker_name,topic) BUT_GUI_PREFIX + "/" + std::string(marker_name) + "/update" + std::string(topic)

#define BUT_PoseChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"/pose_changed")
#define BUT_ScaleChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"/scale_changed")
#define BUT_MenuClicked_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"/menu_clicked")
#define BUT_MovementChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"/movement_changed")
#define BUT_TagChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"/tag_changed")

#endif /* TOPICS_LIST_H_ */
