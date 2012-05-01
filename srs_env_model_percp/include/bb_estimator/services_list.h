/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Hodan (xhodan04@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 02.03.2012 (version 5.0)
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

/*
 * Description:
 * Definition of service names provided by bb_estimator.
 *------------------------------------------------------------------------------
 */

// Services performing bounding box estimation
const std::string BB_ESTIMATOR_Estimate_SRV("/bb_estimator/estimate_bb");
const std::string BB_ESTIMATOR_EstimateAlt_SRV("/bb_estimator/estimate_bb_alt");

// Services performing 2D ractangle estimation
const std::string BB_ESTIMATOR_EstimateRect_SRV("/bb_estimator/estimate_rect");
const std::string BB_ESTIMATOR_EstimateRectAlt_SRV("/bb_estimator/estimate_rect_alt");

