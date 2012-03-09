/*
 *******************************************************************************
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 05.03.2012
 *******************************************************************************
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

#define BUT_PoseChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"pose_changed")
#define BUT_ScaleChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"scale_changed")
#define BUT_MenuClicked_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"menu_clicked")
#define BUT_MovementChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"movement_changed")
#define BUT_TagChanged_TOPIC(im_name) BUT_GUI_UPDATE_TOPIC(im_name,"tag_changed")

#endif /* TOPICS_LIST_H_ */
