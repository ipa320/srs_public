/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav hulik (ihulik@fit.vutbr.cz)
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
#ifndef BUT_PLANE_DETECTOR_SERVICES_LIST_H
#define BUT_PLANE_DETECTOR_SERVICES_LIST_H

#include <string>

namespace srs_env_model_percp
{
    static const std::string PACKAGE_NAME_PREFIX = "/but_env_percp";


    /**
     * but_plane_detector
     */
    static const std::string PLANE_DETECTOR_PREFIX = "/but_plane_detector";

    /**
     * but_plane_detector - services
     */
	static const std::string DET_SERVICE_CLEAR_PLANES = PLANE_DETECTOR_PREFIX + std::string("/but_env_model/clear_planes");


    /**
     * bb_estimator
     */
    static const std::string BB_ESTIMATOR_PREFIX = "/bb_estimator";

    /**
     * bb_estimator - services performing bounding box estimation
     */
    const std::string EstimateBB_SRV = BB_ESTIMATOR_PREFIX + std::string("/estimate_bb");
    const std::string EstimateBBAlt_SRV = BB_ESTIMATOR_PREFIX + std::string("/estimate_bb_alt");

    /**
     * bb_estimator - services performing 2D ractangle estimation
     */
    const std::string EstimateRect_SRV = BB_ESTIMATOR_PREFIX + std::string("/estimate_rect");
    const std::string EstimateRectAlt_SRV = BB_ESTIMATOR_PREFIX + std::string("/estimate_rect_alt");
}

#endif //BUT_PLANE_DETECTOR_SERVICES_LIST_H

