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
#ifndef BUT_ARM_NAVIGATION_TOPICS_H
#define BUT_ARM_NAVIGATION_TOPICS_H

#include "services_list.h"

namespace srs_assisted_arm_navigation
{
	/**
	 * topics
	 */
    static const std::string TOP_GRIPPER_RPY = PACKAGE_NAME_PREFIX + std::string("/gripper_rpy");

    /**
     * actions
     */
    static const std::string ACT_ARM_MANIP = PACKAGE_NAME_PREFIX + std::string("/manual_arm_manip_action");
    static const std::string ACT_GRASP = PACKAGE_NAME_PREFIX + std::string("/manual_grasping_action");
}

#endif // BUT_ARM_NAVIGATION_TOPICS_H
