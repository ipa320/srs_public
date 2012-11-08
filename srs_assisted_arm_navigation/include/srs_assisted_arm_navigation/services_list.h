/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Zdenek Materna (imaterna@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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
#ifndef BUT_ARM_NAVIGATION_SERVICES_H
#define BUT_ARM_NAVIGATION_SERVICES_H

#include <string>

namespace srs_assisted_arm_navigation
{
    static const std::string PACKAGE_NAME_PREFIX = "/but_arm_manip";

	/**
	 * services
	 */
    static const std::string SRV_START = PACKAGE_NAME_PREFIX + std::string("/arm_nav_start");
    static const std::string SRV_NEW = PACKAGE_NAME_PREFIX + std::string("/arm_nav_new");
    static const std::string SRV_PLAN = PACKAGE_NAME_PREFIX + std::string("/arm_nav_plan");
    static const std::string SRV_PLAY = PACKAGE_NAME_PREFIX + std::string("/arm_nav_play");
    static const std::string SRV_EXECUTE = PACKAGE_NAME_PREFIX + std::string("/arm_nav_execute");
    static const std::string SRV_RESET = PACKAGE_NAME_PREFIX + std::string("/arm_nav_reset");
    static const std::string SRV_SUCCESS = PACKAGE_NAME_PREFIX + std::string("/arm_nav_success");
    static const std::string SRV_FAILED = PACKAGE_NAME_PREFIX + std::string("/arm_nav_failed");
    static const std::string SRV_REFRESH = PACKAGE_NAME_PREFIX + std::string("/arm_nav_refresh");
    static const std::string SRV_COLLOBJ = PACKAGE_NAME_PREFIX + std::string("/arm_nav_coll_obj");
    static const std::string SRV_SET_ATTACHED = PACKAGE_NAME_PREFIX + std::string("/arm_nav_set_attached");
    static const std::string SRV_MOVE_PALM_LINK = PACKAGE_NAME_PREFIX + std::string("/arm_nav_move_palm_link");
    static const std::string SRV_MOVE_PALM_LINK_REL = PACKAGE_NAME_PREFIX + std::string("/arm_nav_move_palm_link_rel");
    static const std::string SRV_SWITCH = PACKAGE_NAME_PREFIX + std::string("/arm_nav_switch_aco");
    static const std::string SRV_REPEAT = PACKAGE_NAME_PREFIX + std::string("/arm_nav_repeat");
    static const std::string SRV_STEP = PACKAGE_NAME_PREFIX + std::string("/arm_nav_step");
    static const std::string SRV_STOP = PACKAGE_NAME_PREFIX + std::string("/arm_nav_stop");

    static const std::string SRV_ALLOW = PACKAGE_NAME_PREFIX + std::string("/grasping_allow");
}

#endif // BUT_ARM_NAVIGATION_SERVICES_H
