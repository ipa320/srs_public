/******************************************************************************
 * \file
 *
 * $Id: plane_detector_params.h 693 2012-04-20 09:22:39Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 15.06.2012 (version 1.0)
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
#ifndef PLANE_DETECTOR_PARAMS_H
#define PLANE_DETECTOR_PARAMS_H

#include <string>
#include "srs_env_model_percp/topics_list.h"

namespace srs_env_model_percp
{
    /**
      * Global node settings
      */
	const std::string PARAM_NODE_INPUT 						= "planedet_input";
	const std::string PARAM_NODE_INPUT_PCL					= "pcl";
	const std::string PARAM_NODE_INPUT_KINECT				= "kinect";
	const std::string PARAM_NODE_INPUT_DEFAULT 				= PARAM_NODE_INPUT_PCL;

    /**
      * Input frame
      * TODO: make this automatic
      */
    const std::string PARAM_NODE_ORIGINAL_FRAME             = "original_frame";
    const std::string PARAM_NODE_ORIGINAL_FRAME_DEFAULT     = "/head_cam3d_link";

	/**
      * Target frame
      */
	const std::string PARAM_NODE_OUTPUT_FRAME				= "global_frame";
	const std::string PARAM_NODE_OUTPUT_FRAME_DEFAULT		= "/map";


    /**
      * Hough space settings
      */
	const std::string PARAM_HT_MAXDEPTH						= "planedet_ht_maxdepth";
	const double 	  PARAM_HT_MAXDEPTH_DEFAULT				= 100.0;

	const std::string PARAM_HT_MINSHIFT						= "planedet_ht_minshift";
	const double 	  PARAM_HT_MINSHIFT_DEFAULT				= -40.0;

	const std::string PARAM_HT_MAXSHIFT						= "planedet_ht_maxshift";
	const double 	  PARAM_HT_MAXSHIFT_DEFAULT				= 40.0;

	const std::string PARAM_HT_ANGLE_RES					= "planedet_ht_angle_res";
	const double 	  PARAM_HT_ANGLE_RES_DEFAULT			= 512;

	const std::string PARAM_HT_SHIFT_RES					= "planedet_ht_shift_res";
	const double 	  PARAM_HT_SHIFT_RES_DEFAULT			= 4096;

	const std::string PARAM_HT_GAUSS_ANGLE_RES				= "planedet_ht_gauss_angle_res";
	const double 	  PARAM_HT_GAUSS_ANGLE_RES_DEFAULT		= 11;

	const std::string PARAM_HT_GAUSS_SHIFT_RES				= "planedet_ht_gauss_shift_res";
	const double 	  PARAM_HT_GAUSS_SHIFT_RES_DEFAULT		= 11;

	const std::string PARAM_HT_GAUSS_ANGLE_SIGMA			= "planedet_ht_gauss_angle_sigma";
	const double 	  PARAM_HT_GAUSS_ANGLE_SIGMA_DEFAULT	= 0.04;

	const std::string PARAM_HT_GAUSS_SHIFT_SIGMA			= "planedet_ht_gauss_shift_sigma";
	const double 	  PARAM_HT_GAUSS_SHIFT_SIGMA_DEFAULT	= 0.15;


    /**
      * Plane search
      */
	const std::string PARAM_SEARCH_MINIMUM_CURRENT_SPACE 				= "planedet_search_minimum_current_space";
	const double	  PARAM_SEARCH_MINIMUM_CURRENT_SPACE_DEFAULT 		= 1500.0;

	const std::string PARAM_SEARCH_MINIMUM_GLOBAL_SPACE 				= "planedet_search_minimum_global_space";
	const double	  PARAM_SEARCH_MINIMUM_GLOBAL_SPACE_DEFAULT 		= 1.5;

	const std::string PARAM_SEARCH_MAXIMA_SEARCH_NEIGHBORHOOD			= "planedet_search_maxima_search_neighborhood";
	const int		  PARAM_SEARCH_MAXIMA_SEARCH_NEIGHBORHOOD_DEFAULT 	= 1;

	const std::string PARAM_SEARCH_MAXIMA_SEARCH_BLUR					= "planedet_search_maxima_search_blur";
	const int		  PARAM_SEARCH_MAXIMA_SEARCH_BLUR_DEFAULT 			= 0;


    /**
      * Visualisation
      */
	const std::string PARAM_VISUALISATION_DISTANCE						= "planedet_visualisation_distance";
	const double 	  PARAM_VISUALISATION_DISTANCE_DEFAULT				= 0.05;

	const std::string PARAM_VISUALISATION_MIN_COUNT						= "planedet_visualisation_min_count";
	const int	 	  PARAM_VISUALISATION_MIN_COUNT_DEFAULT				= 2000;

	const std::string PARAM_VISUALISATION_PLANE_NORMAL_DEV				= "planedet_visualisation_plane_normal_dev";
	const double 	  PARAM_VISUALISATION_PLANE_NORMAL_DEV_DEFAULT		= 0.15;

	const std::string PARAM_VISUALISATION_PLANE_SHIFT_DEV				= "planedet_visualisation_plane_shift_dev";
	const double 	  PARAM_VISUALISATION_PLANE_SHIFT_DEV_DEFAULT		= 0.45;


	/**
	 * Structure encapsulates all parameters of the plane detector.
	 */
    struct PlaneDetectorSettings
    {
        std::string param_node_input;
        std::string param_output_frame;
        std::string param_original_frame;

        double param_ht_maxdepth;
        double param_ht_minshift;
        double param_ht_maxshift;
        double param_ht_angle_res;
        double param_ht_shift_res;
        double param_ht_gauss_angle_res;
        double param_ht_gauss_shift_res;
        double param_ht_gauss_angle_sigma;
        double param_ht_gauss_shift_sigma;

        double param_search_minimum_current_space;
        double param_search_minimum_global_space;
        int    param_search_maxima_search_neighborhood;
        int    param_search_maxima_search_blur;

        double param_visualisation_distance;
        double param_visualisation_plane_normal_dev;
        double param_visualisation_plane_shift_dev;
        int    param_visualisation_min_count;
    };
}


#endif
