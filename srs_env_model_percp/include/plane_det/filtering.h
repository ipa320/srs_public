/******************************************************************************
 * \file
 *
 * $Id: filtering.h 619 2012-04-16 13:47:28Z ihulik $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 1.0)
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

/**
 * Description:
 *	 Contains necessary classes for depth map filtering
 */

#ifndef FILTERING_H
#define FILTERING_H

// opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc_c.h>

// ros
#include <sensor_msgs/CameraInfo.h>

// but_scenemodel
#include "normals.h"

#define DEFAULT_SIZE 2
#define DEPTH_DEFAULT_THRESH 200
#define NORMAL_DEFAULT_THRESH 0.3
#define NORMAL_COMPUTATION_SIZE 5
#define NORMAL_COMPUTATION_TYPE NormalType::PCL
#define PREDICTOR_DEFAULT_THRESH 0.02
#define PREDICTOR_SOBEL_SIZE 5

#define MIN_DISTANCE 0.1
#define MAX_DISTANCE 3.1
#define RANSACMAX 1000

using namespace sensor_msgs;
using namespace cv;

namespace but_scenemodel
{
	/**
	 * Gradient extraction method enum class
	 */
	class WatershedType
	{
		public:
			/**
			 * Gradient image is extracted from depth data only
			 */
			static const int DepthDiff = 1;

			/**
			 * Gradient image is extracted from normal data only
			 */
			static const int NormalDiff = 2;

			/**
			 * Gradient image is extracted as a combination depth and normal data
			 */
			static const int Combined = 4;

			/**
			 * Gradient image is extracted with plane prediction algorithm
			 */
			static const int PredictorDiff = 8;
	};

	/**
	 * Class providing an interface to depth image segmentation
	 */
	class Regions
	{
		public:
			/**
			 * A class providing an interface to depth image segmenter
			 * @param normals Normals object with precomputed real points and normals - not necessary - only if you do not want to compute it in this module (for speed reasons)
			 * @see Normals()
			 */
			Regions(Normals *normals = NULL);

			/**
			 * Destructor - obviously, it destructs this class
			 */
			~Regions();

			/**
			 * Method which segments a depth image with watershed algorithm. There is several approaches, so please be aware of input parameters
			 * @param src Input CV_16UC depth matrix (raw input from kinect)
			 * @param cam_info Camera info message (ROS)
			 * @param type Type of watershed input - one of WatershedType constants
			 * @see WatershedType
			 * @param seedThreshold Threshold which specifies number of anomal neighbors for selecting this pixel as seed for watershed
			 * @param alpha It is always size of neigborhood which is considered for searching the image in watershed preprocessing. If 0.0, default is selected.
			 * @param beta When simple preprocessing is selected (depth, normal, predictor), it is always a threshold for selecting anomal neighbors (in depth is in milimeters, normal is in radians ). If combined type is selected, it represents a depth difference.
			 * @param gamma It is used only in combined and predictor. In combined, it specifiesd normal difference threshold, in predictor a sobel size.
			 * @see Normals()
			 */
			void watershedRegions(Mat &src, const CameraInfoConstPtr& cam_info, int type = WatershedType::Combined, int seedThreshold = 1, float alpha = 0.0, float beta = 0.0, float gamma = 0.0);

			/**
			 * Method segments a depth image using own tile plane searching RANSAC method.
			 * @param src Input CV_16UC depth matrix (raw input from kinect)
			 * @param cam_info Camera info message (ROS)
			 * @param minimumPlaneDistance Minimal distance threshold from triangle plane in whole tile (in meters)
			 * @param minimumTriDistance Minimal distance threshold from triangle plane in triangle flood fill (in meters)
			 * @param minimumFloodDistance Minimal distance threshold from triangle plane in whole image (in meters)
			 * @param tileSize Tile size
			 * @param smallestTriangleArea Smalest randomly found triangle to be considered for plane computation (in pixels^2)
			 * @param minRegionSize Minimal region size to be considered
			 * @param minCandidates Minimal number of pixels in tile to start a RANSAC
			 */
			void independentTileRegions(Mat &src, const CameraInfoConstPtr& cam_info, float minimumPlaneDistance = 0.01, float minimumTriDistance= 0.02, float minimumFloodDistance = 0.05, int tileSize = 30, int smallestTriangleArea = 100, int minRegionSize = 10, int minCandidates = 10);

			/**
			 * Method computes from m_regionMatrix statistical information (fills m_planes and m_stddeviation matrices)
			 * @param threshold Not used now (TODO)
			 */
			bool computeStatistics(float threshold);

			/**
			 * Method converts an input image into gradient image using expected plane prediction. Pixel value is number of pixels around which are not in difference threshold from expected plane.
			 * @param src Input CV_16UC depth matrix (raw input from kinect)
			 * @param dst Final gradient image
			 * @param differenceThreshold Difference from expected plane threshold to mark a pixel as anomal
			 * @param size Size of neigborhood (number of maximal pixel distance)
			 * @param sobelSize Size of sobel kernel used for gradient image computation
			 */
			static void gradientPlanePredictor(Mat &src, Mat& dst, float differenceThreshold = 0.02, int size = 2, int sobelSize = 5);//, Mat& normals, Mat& coefs, Mat& points);

			/**
			 * Method converts an input image into gradient image using comparison of normals
			 * @param src Input CV_16UC depth matrix (raw input from kinect)
			 * @param dst Final gradient image
			 * @param size Size of neigborhood (number of maximal pixel distance)
			 * @param differenceThreshold Difference from center normal to mark a pixel as anomal
			 */
			static void gradientNormalDifference(Mat &src, Mat& dst, int size, float differenceThreshold);

			/**
			 * Method converts an input image into gradient image using comparison of depth data
			 * @param src Input CV_16UC depth matrix (raw input from kinect)
			 * @param dst Final gradient image
			 * @param size Size of neigborhood (number of maximal pixel distance)
			 * @param differenceThreshold Difference from center depth to mark a pixel as anomal
			 */
			static void gradientDepthDifference(Mat &src, Mat& dst, int size, float differenceThreshold);

			/**
			 * Region matrix (filled after "Regions" methods call)
			 * @see watershedRegions()
			 * @see independentTileRegions()
			 */
			Mat m_regionMatrix;

			/**
			 * Computed planes matrix (Vec4f type, each component represents component of computed region normal)
			 * Filled after computeStatistics method
			 * @see computeStatistics()
			 */
			Mat m_planes;

			/**
			 * Computed standard deviation of region pixels normals from region mean normal
			 * Filled after computeStatistics method
			 * @see computeStatistics()
			 */
			Mat m_stddeviation;

			/**
			 * Instance of Normals object - precomputed real point positions and normals.
			 * Standard is null if not specified otherwise in constructor or in Region method
			 * Freed in destructor!
			 */
			Normals *m_normals;

		private:
			/**
			 * Auxiliary method which flood fills a tile untill min_planeDistance threshold difference is met
			 * @param tile Tile subimage of depth data
			 * @param tileMask Mask subimage
			 * @param pointMask Point mask subimage
			 * @param points Vector of tile points
			 * @param index Index of filling region
			 * @param plane Equation of filled plane
			 * @param ioffset Row offset of tile
			 * @param joffset Column offset of tile
			 * @param min_planeDistance Minimal distance from plane to fill
			 */
			int floodFillTile(Mat &tile, Mat&tileMask, Mat&pointMask, Vec3f *points, int index, Plane<float> &plane, int ioffset, int joffset, float min_planeDistance = 0.02);

			/**
			 * Auxiliary function extracts current region's plane using LSQ
			 * @param plane Extracted plane
			 * @param tile Tile subimage of depth data
			 * @param tileMask Mask subimage
			 * @param ioffset Row offset of tile
			 * @param joffset Column offset of tile
			 * @param index Index of filling region
			 */
			void getLeastSquaresAndBorder(Plane<float> &plane, Mat &tile, Mat&tileMask, int ioffset, int joffset, int index);

			/**
			 * Auxiliary function for filling entire image from mask seed (until threshold from plane is met)
			 * @param plane Plane equation
			 * @param depth Depth image
			 * @param mask Region mask
			 * @param points Vector of seed points
			 * @param i_tile Row tile offset
			 * @param j_tile Column tile offset
			 * @param tileSize Size of tile
			 * @param index Index of filled region
			 * @param min_planeDistance Maximal distance from plane to be filled
			 */
			void fillEverything(Plane<float> &plane, Mat & depth, Mat & mask, Mat & points, int i_tile, int j_tile, int tileSize, int index, float min_planeDistance = 0.05);
	}; // class Regions
}
#endif
