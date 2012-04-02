/******************************************************************************
 * \file
 *
 * $Id: filtering.cpp 397 2012-03-29 12:50:30Z spanel $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 11.01.2012 (version 0.8)
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
 *
 */

#include "plane_det/filtering.h"
#include <cv.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>


namespace but_scenemodel{

//////////////////////////////////////////////////////////////////////
// A class providing an interface to depth image segmenter
// @param normals Normals object with precomputed real points and normals - not necessary - only if you do not want to compute it in this module (for speed reasons)
// @see Normals()
//////////////////////////////////////////////////////////////////////
Regions::Regions(Normals *normals)
{
	m_normals = normals;
}

//////////////////////////////////////////////////////////////////////
// Destructor - obviously, it destructs this class
//////////////////////////////////////////////////////////////////////
Regions::~Regions()
{
	//if (m_normals != NULL)
	//	free(m_normals);
}

//////////////////////////////////////////////////////////////////////
// Method computes from m_regionMatrix statistical information (fills m_planes and m_stddeviation matrices)
// @param threshold Not used now (TODO)
//////////////////////////////////////////////////////////////////////
bool Regions::computeStatistics(float threshold)
{
	using namespace cv;
	if (m_normals == NULL) return false;

	std::vector<Vec4f> normals;
	std::vector<int> sizes;
	Vec4f nullVector = Vec4f(0.0, 0.0, 0.0, 0.0);
	Vec4f auxVector;
	int aux;

	for (int i = 0; i < m_regionMatrix.rows; ++i)
	for (int j = 0; j < m_regionMatrix.cols; ++j)
	{
		auxVector = m_normals->m_planes.at<Vec4f>(i, j);
		if (auxVector != nullVector)
		{
			aux = m_regionMatrix.at<int>(i, j);
			if (aux >= 0)
			{
				if (aux >= normals.size())
				{
					sizes.resize(aux+1, 0);
					normals.resize(aux+1, Vec4f(0.0, 0.0, 0.0, 0.0));
				}
				sizes[aux] += 1;
				normals[aux] += auxVector;
			}

		}
		else
		{
			m_regionMatrix.at<int>(i, j) = -1;
		}
	}

	for (unsigned int i = 0; i < normals.size(); ++i)
	{
		normals[i][0] /= sizes[i];
		normals[i][1] /= sizes[i];
		normals[i][2] /= sizes[i];
		normals[i][3] /= sizes[i];
	}

	std::vector<float> deviations(normals.size());
	std::vector<float> means(normals.size());

	for (int i = 0; i < m_regionMatrix.rows; ++i)
	for (int j = 0; j < m_regionMatrix.cols; ++j)
	{
		aux = m_regionMatrix.at<int>(i, j);
		if (aux >= 0)
		{
			auxVector = m_normals->m_planes.at<Vec4f>(i, j);
			float angle = acos(normals[aux][0]*auxVector[0]+normals[aux][1]*auxVector[1]+normals[aux][2]*auxVector[2]);
			means[aux] += angle*angle;
		}
	}

	for (unsigned int i = 0; i < normals.size(); ++i)
	{
		means[i] /= sizes[i];
		means[i] = sqrt(means[i]);
	}

	m_planes = Mat::zeros(m_regionMatrix.size(), CV_32FC4);
	m_stddeviation = Mat::zeros(m_regionMatrix.size(), CV_32FC1);

	for (int i = 0; i < m_regionMatrix.rows; ++i)
	for (int j = 0; j < m_regionMatrix.cols; ++j)
	{
		if (m_regionMatrix.at<int>(i, j) >= 0)
		{
			m_stddeviation.at<float>(i, j) = means[m_regionMatrix.at<int>(i, j)];
			m_planes.at<Vec4f>(i, j) = normals[m_regionMatrix.at<int>(i, j)];
		}
	}
	return true;
}

//////////////////////////////////////////////////////////////////////
// Method which segments a depth image with watershed algorithm. There is several approaches, so please be aware of input parameters
// @param src Input CV_16UC depth matrix (raw input from kinect)
// @param cam_info Camera info message (ROS)
// @param type Type of watershed input - one of WatershedType constants
// @see WatershedType
// @param seedThreshold Threshold which specifies number of anomal neighbors for selecting this pixel as seed for watershed
// @param alpha It is always size of neigborhood which is considered for searching the image in watershed preprocessing. If 0.0, default is selected.
// @param beta When simple preprocessing is selected (depth, normal, predictor), it is always a threshold for selecting anomal neighbors (in depth is in milimeters, normal is in radians ). If combined type is selected, it represents a depth difference.
// @param gamma It is used only in combined and predictor. In combined, it specifiesd normal difference threshold, in predictor a sobel size.
// @see Normals()
//////////////////////////////////////////////////////////////////////
void Regions::watershedRegions(Mat &src, const CameraInfoConstPtr& cam_info,
						   int type, int seedThreshold,
						   float alpha, float beta, float gamma
						   )
{
	Mat output1;
	Mat output2;

	Mat markers = Mat::zeros(src.size(), CV_32FC1);
	Mat inputWatershed = Mat::zeros(src.size(), CV_8UC3);


	unsigned short val;
	if (type & WatershedType::DepthDiff)
	{
		if (alpha == 0)
			alpha = DEFAULT_SIZE;
		if (beta == 0)
			beta = DEPTH_DEFAULT_THRESH;
		gradientDepthDifference(src, output1, alpha, beta);
		for (int i = 0; i < output1.rows; ++i)
		for (int j = 0; j < output1.cols; ++j)
		{
			val = output1.at<unsigned short>(i, j);
			if (val > 255) val = 255;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[0] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[1] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[2] = (unsigned char)val;
			if (val >= seedThreshold)
				markers.at<float>(i, j) = 1;
		}
	}
	else if (type & WatershedType::NormalDiff)
	{
		if (alpha == 0)
			alpha = DEFAULT_SIZE;
		if (beta == 0)
			beta = NORMAL_DEFAULT_THRESH;
		if (m_normals == NULL)
			m_normals = new Normals(src, cam_info, NORMAL_COMPUTATION_TYPE, NORMAL_COMPUTATION_SIZE);
		gradientNormalDifference(m_normals->m_planes, output1, alpha, beta);
		for (int i = 0; i < output1.rows; ++i)
		for (int j = 0; j < output1.cols; ++j)
		{
			val = output1.at<unsigned short>(i, j);
			if (val > 255) val = 255;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[0] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[1] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[2] = (unsigned char)val;
			if (val >= seedThreshold)
				markers.at<float>(i, j) = 1;
		}
	}
	else if (type & WatershedType::Combined)
	{
		if (alpha == 0)
			alpha = DEFAULT_SIZE;
		if (beta == 0)
			beta = DEPTH_DEFAULT_THRESH;
		if (gamma == 0)
			gamma = NORMAL_DEFAULT_THRESH;
		if (m_normals == NULL)
			m_normals = new Normals(src, cam_info, NORMAL_COMPUTATION_TYPE, NORMAL_COMPUTATION_SIZE);
		gradientNormalDifference(m_normals->m_planes, output1, alpha, gamma);
		gradientDepthDifference(src, output2, alpha, beta);
		for (int i = 0; i < output1.rows; ++i)
		for (int j = 0; j < output1.cols; ++j)
		{
			val = output1.at<unsigned short>(i, j) + output2.at<unsigned short>(i, j);
			if (val > 255) val = 255;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[0] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[1] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[2] = (unsigned char)val;
			if (val >= seedThreshold)
				markers.at<float>(i, j) = 1;
		}
	}
	else if (type & WatershedType::PredictorDiff)
	{
		if (alpha == 0)
			alpha = DEFAULT_SIZE;
		if (beta == 0)
			beta = PREDICTOR_DEFAULT_THRESH;
		if (gamma == 0)
			gamma = PREDICTOR_SOBEL_SIZE;
		gradientPlanePredictor(src, output1, beta, alpha, gamma);
		for (int i = 0; i < output1.rows; ++i)
		for (int j = 0; j < output1.cols; ++j)
		{
			val = output1.at<unsigned short>(i, j);
			if (val > 255) val = 255;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[0] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[1] = (unsigned char)val;
			inputWatershed.at<Vec<unsigned char, 3> >(i, j)[2] = (unsigned char)val;
			if (val >= seedThreshold)
				markers.at<float>(i, j) = 1;
		}
	}

	float aux;
	unsigned int index = 2;
	for (int i = 0; i < markers.rows; ++i)
	for (int j = 0; j < markers.cols; ++j)
	{
		aux = markers.at<float>(i, j);
		if (aux < 1)
		{
			if (floodFill(markers, Point(j, i), Scalar(index)) < 100)
				floodFill(markers, Point(j, i), Scalar(1));
			else
				++index;
		}
	}


	for (int i = 0; i < markers.rows; ++i)
	for (int j = 0; j < markers.cols; ++j)
	{
		if (markers.at<float>(i, j) == 1)
			markers.at<float>(i, j) = 0;
	}

	markers.convertTo(output1, CV_32SC1);
	watershed(inputWatershed, output1);

	output1.convertTo(m_regionMatrix, CV_32SC1);
}

//////////////////////////////////////////////////////////////////////
// Method converts an input image into gradient image using comparison of depth data
// @param src Input CV_16UC depth matrix (raw input from kinect)
// @param dst Final gradient image
// @param size Size of neigborhood (number of maximal pixel distance)
// @param differenceThreshold Difference from center depth to mark a pixel as anomal
//////////////////////////////////////////////////////////////////////
void Regions::gradientDepthDifference(Mat &src, Mat& dst, int size, float differenceThreshold)
{
	Mat input(src.size(), CV_32FC1);
	Mat output = Mat::ones(src.size(), CV_32FC1);

	src.convertTo(input, CV_32F);

	for (int i = size; i < src.rows-size; ++i)
	for (int j = size; j < src.cols-size; ++j)
	{
		int count = 0;
		float center = input.at<float>(i, j);
		for (int x = i-size; x < i+size; ++x)
		for (int y = j-size; y < j+size; ++y)
		{
			if (abs(input.at<float>(x, y) - center) > differenceThreshold)
				count++;
		}
		output.at<float>(i, j) = count;
	}
	output.convertTo(dst, CV_16U);
}

//////////////////////////////////////////////////////////////////////
// Method converts an input image into gradient image using comparison of normals
// @param src Input CV_16UC depth matrix (raw input from kinect)
// @param dst Final gradient image
// @param size Size of neigborhood (number of maximal pixel distance)
// @param differenceThreshold Difference from center normal to mark a pixel as anomal
//////////////////////////////////////////////////////////////////////
void Regions::gradientNormalDifference(Mat &src, Mat& dst, int size, float differenceThreshold)
{
	Mat output = Mat::zeros(src.size(), CV_32FC1);

	float minangle = differenceThreshold;
	float maxangle = M_PI - differenceThreshold;
	Vec4f aux;

	for (int i = size; i < src.rows-size; ++i)
	for (int j = size; j < src.cols-size; ++j)
	{
		Vec4f center = src.at<Vec4f>(i, j);
		int count = 0;

		if (center == Vec4f(0.0, 0.0, 0.0, 0.0))
		{
			output.at<float>(i, j) = 0;
			continue;
		}

		for (int x = i-size; x < i+size; ++x)
		for (int y = j-size; y < j+size; ++y)
		{
			aux = src.at<Vec4f>(x, y);

			float angle = 	acos(aux[0]*center[0]+aux[1]*center[1]+aux[2]*center[2]);
			if (angle > minangle && angle < maxangle)
				count++;
		}
		output.at<float>(i, j) = count;
	}
	output.convertTo(dst, CV_16U);
}

//////////////////////////////////////////////////////////////////////
// Method converts an input image into gradient image using expected plane prediction. Pixel value is number of pixels around which are not in difference threshold from expected plane.
// @param src Input CV_16UC depth matrix (raw input from kinect)
// @param dst Final gradient image
// @param differenceThreshold Difference from expected plane threshold to mark a pixel as anomal
// @param size Size of neigborhood (number of maximal pixel distance)
// @param sobelSize Size of sobel kernel used for gradient image computation
//////////////////////////////////////////////////////////////////////
void Regions::gradientPlanePredictor(Mat &src, Mat& dst, float differenceThreshold, int size, int sobelSize)//, Mat& normals, Mat& coefs, Mat& points)
{
	Mat input(src.size(), CV_32FC1);
	Mat output(src.size(), CV_32FC1);
	Mat output2(src.size(), CV_32FC1);

	//GaussianBlur(normals, normals, cvSize(3,3), 3);
	src.convertTo(input, CV_32F);

	int size2 = size+size+1;
	int sizesq = size2 * size2;

	input.copyTo(output);

	////////////////////////////////////////////////////
	// normalized sobel
	// get dx, dy
	Mat dx(src.size(), CV_32FC1);
	Mat dy(src.size(), CV_32FC1);

	Mat kx(cvSize(sobelSize,sobelSize), CV_32FC1);
	Mat ky(cvSize(sobelSize,sobelSize), CV_32FC1);

	getDerivKernels(kx, ky, 1, 1, sobelSize, true, CV_32F);
	filter2D(output, dx, CV_32F, kx);
	filter2D(output, dy, CV_32F, ky);

	Vec3f centerNormal;
	Vec3f nowNormal;

	// null outputs
	output = Mat::zeros(output.size(), CV_32FC1);
	output2 = Mat::zeros(output.size(), CV_32FC1);

	float xNow, yNow, xCenter, yCenter, dist, theoreticalDepth, realDepth, centerDepth, difference, magVec, d;

	// for each
	Vec3f z(0,0,1);

	for (int i = size; i < src.rows-size; ++i)
		for (int j = size; j < src.cols-size; ++j)
		{
			centerDepth = input.at<float>(i, j);
			if (centerDepth != 0.0)
			{
			// get center
			xCenter = dx.at<float>(i, j);
			yCenter = dy.at<float>(i, j);

			dist = 0.0;


			// compute gradient magnitude
			if (xCenter == 0 && yCenter == 0)
				magVec = 0;
			else
				magVec = sqrt(xCenter*xCenter + yCenter*yCenter);

			int ones = 0;
			int last = 0;
			int changes = 0;

			// neighbourhood
			int x = -size;
			int y = -size;
			int plusX = 1;
			int plusY = 0;
			Mat diffs(cvSize(size+size+1, size+size+1), CV_8UC1);

			float vals[sizesq];
			int index = 0;
			changes = 0;

			// For each in points neighbourhood
			for (int cnt = 0; cnt <= size*8; ++cnt)
			{
				xNow = dx.at<float>(i+x, j+y);
				yNow = dy.at<float>(i+x, j+y);

				// compute how much of points in neighbourhood are in predicted depth
				realDepth = input.at<float>(i+x, j+y);
				vals[index] = acos(z.dot(nowNormal));
				++index;

				d = (xCenter*x + yCenter*y);
				theoreticalDepth = d + centerDepth;
				difference = abs(theoreticalDepth - realDepth) / SHRT_MAX;

				if (cnt != 0)
				{
					// threshold difference - first trait
					if (difference > differenceThreshold)
					{
						ones+=1;
						if (last == 0)
						{
							last = 1;
							changes++;
						}
					}
					else
						if (last == 1)
						{
							last = 0;
							changes++;
						}
				}
				else
				{
					if (difference > differenceThreshold)
						last = 1;
					else
						last = 0;
				}

				dist += abs(realDepth-centerDepth);

				// move next pixel around
				if (x == -size && y == -size)
				{
					plusX = 1;
					plusY = 0;
				}
				if (x == size && y == -size)
				{
					plusX = 0;
					plusY = 1;
				}
				if (x == size && y == size)
				{
					plusX = -1;
					plusY = 0;
				}
				if (x == -size && y == size)
				{
					plusX = 0;
					plusY = -1;
				}
				x += plusX;
				y += plusY;
			}
			dist /= size * 8;
			dist /= SHRT_MAX;

			float ch = ((cos((float)changes * ((2.0*M_PI)/(size*8)))     +1.0)     /2.0) * ones;

			output2.at<float>(i,j) = ch;
			output.at<float>(i, j) = dist;
		}
		}

	// normalize the output
	double min, max;
	Point minLoc, maxLoc;

//	medianBlur(output2, input, 3);
//	medianBlur(output, output2, 5);

	input = output + output2;

	minMaxLoc(input, &min, &max, &minLoc, &maxLoc);
	input.convertTo(dst, CV_16U, SHRT_MAX/(max-min), -min);
}

//////////////////////////////////////////////////////////////////////
// Auxiliary method which flood fills a tile untill min_planeDistance threshold difference is met
// @param tile Tile subimage of depth data
// @param tileMask Mask subimage
// @param pointMask Point mask subimage
// @param points Vector of tile points
// @param index Index of filling region
// @param plane Equation of filled plane
// @param ioffset Row offset of tile
// @param joffset Column offset of tile
// @param min_planeDistance Minimal distance from plane to fill
//////////////////////////////////////////////////////////////////////
int Regions::floodFillTile(Mat &tile, Mat&tileMask, Mat&pointMask, Vec3f *points, int index, Plane<float> &plane, int ioffset, int joffset, float min_planeDistance /*0.02*/)
{
	std::vector<Vec2i> unprocessed;
	Mat processedMask = Mat::zeros(tileMask.size(), CV_8UC1);
	unprocessed.push_back(Vec2i(points[0][0] - ioffset, points[0][1] - joffset));

	// if some of points is non zero (was filled before), bail
	if (tileMask.at<int>(points[0][0] - ioffset, points[0][1] - joffset) != 0 ||
		tileMask.at<int>(points[1][0] - ioffset, points[1][1] - joffset) != 0||
		tileMask.at<int>(points[2][0] - ioffset, points[2][1] - joffset) != 0)
			return 0;

	// mark vertices
	tileMask.at<int>(points[0][0] - ioffset, points[0][1] - joffset) = -1;
	tileMask.at<int>(points[1][0] - ioffset, points[1][1] - joffset) = -1;
	tileMask.at<int>(points[2][0] - ioffset, points[2][1] - joffset) = -1;
	Vec2i current;
	Vec2i newPoint;
	int neighbour;
	int found = 0;
	int size = 0;
	int tileSize = tile.rows;

	// while loop of seed fill
	while (!unprocessed.empty())
	{
		current = unprocessed.back();
		unprocessed.pop_back();

		if (tileMask.at<int>(current[0], current[1]) == -1)
			++found;

		// set current index
		tileMask.at<int>(current[0], current[1]) = index;
		++size;

		// search neighborhood
		for (int i = -1; i <= 1; ++i)
		for (int j = -1; j <= 1; ++j)
		if (i != 0 || j != 0)
		{
			newPoint[0] = current[0] + i;
			newPoint[1] = current[1] + j;
			if (newPoint[0] >= 0 &&
				newPoint[1] >= 0 &&
				newPoint[0] < tileSize &&
				newPoint[1] < tileSize)
			{
				neighbour = tileMask.at<int>(newPoint[0], newPoint[1]);

				if (processedMask.at<unsigned char>(newPoint[0], newPoint[1]) == 0 && (neighbour == 0 || neighbour == -1) && plane.distance(pointMask.at<Vec3f>(newPoint[0], newPoint[1])) < min_planeDistance)
					unprocessed.push_back(newPoint);

				processedMask.at<unsigned char>(newPoint[0], newPoint[1]) = 1;
			}
		}
	}

	// if we touched all three vertices, return size else bail
	if (found < 3)
		return 0;
	else return size;
}

//////////////////////////////////////////////////////////////////////
// Auxiliary function extracts current region's plane using LSQ
// @param plane Extracted plane
// @param tile Tile subimage of depth data
// @param tileMask Mask subimage
// @param ioffset Row offset of tile
// @param joffset Column offset of tile
// @param index Index of filling region
//////////////////////////////////////////////////////////////////////
void Regions::getLeastSquaresAndBorder(Plane<float> &plane, Mat &tile, Mat&tileMask, int ioffset, int joffset, int index)
{
	int tileSize = tile.rows;
	std::vector<Vec3f> points;
	for (int i = 0; i < tileSize; ++i)
		for (int j = 0; j < tileSize; ++j)
		{
			std::cout << tileMask.at<int>(i,j) << " " << index << std::endl;
			if (tileMask.at<int>(i,j) == index)
				points.push_back(Vec3f(i+ioffset, j+joffset, tile.at<float>(i,j)));
		}

	plane = Normals::LeastSquaresPlane(points);
}

//////////////////////////////////////////////////////////////////////
// Auxiliary function for filling entire image from mask seed (until threshold from plane is met)
// @param plane Plane equation
// @param depth Depth image
// @param mask Region mask
// @param points Vector of seed points
// @param i_tile Row tile offset
// @param j_tile Column tile offset
// @param tileSize Size of tile
// @param index Index of filled region
// @param min_planeDistance Maximal distance from plane to be filled
//////////////////////////////////////////////////////////////////////
void Regions::fillEverything(Plane<float> &plane, Mat & depth, Mat & mask, Mat & points, int i_tile, int j_tile, int tileSize, int index, float min_planeDistance /*0.05*/)
{
	int maxi = tileSize+i_tile;
	int maxj = tileSize+j_tile;
	std::vector<Vec2i> unprocessed;
	Mat processedMask = Mat::zeros(depth.size(), CV_8UC1);

	// loop in tile and mark all border pixels as seeds
	for (int i = i_tile; i < maxi; ++i)
		for (int j = j_tile; j < maxj; ++j)
		{
			if (mask.at<int>(i,j) == index)
			{
				if (mask.at<int>(i+1,j+1) != index)
				{
					unprocessed.push_back(Vec2i(i+1, j+1));
					break;
				}
				if (mask.at<int>(i  ,j+1) != index)
				{
					unprocessed.push_back(Vec2i(i, j+1));
					break;
				}
				if (mask.at<int>(i-1,j+1) != index)
				{
					unprocessed.push_back(Vec2i(i-1, j+1));
					break;
				}
				if (mask.at<int>(i+1,j) != index)
				{
					unprocessed.push_back(Vec2i(i+1, j));
					break;
				}
				if (mask.at<int>(i-1,j) != index)
				{
					unprocessed.push_back(Vec2i(i-1, j));
					break;
				}
				if (mask.at<int>(i+1,j-1) != index)
				{
					unprocessed.push_back(Vec2i(i+1, j-1));
					break;
				}
				if (mask.at<int>(i  ,j-1) != index)
				{
					unprocessed.push_back(Vec2i(i, j-1));
					break;
				}
				if (mask.at<int>(i-1,j-1) != index)
				{
					unprocessed.push_back(Vec2i(i-1, j-1));
					break;
				}
			}
		}

	Vec2i current, newPoint;
	Vec3f planenormal;
	planenormal[0] = plane.a;
	planenormal[1] = plane.b;
	planenormal[2] = plane.c;

	int neighbour;

	// seed fill loop
	while (!unprocessed.empty())
	{
		current = unprocessed.back();

		unprocessed.pop_back();

		if (current[0] < 0 || current[1] < 0 || current[0] >= depth.rows || current[1] >= depth.cols)
			continue;


		if (mask.at<int>(current[0], current[1]) != 0)
			continue;

		Vec4f planeVec = m_normals->m_planes.at<Vec4f>(current[0], current[1]);
		Vec3f normal(planeVec[0], planeVec[1], planeVec[2]);
		if (normal[2] < 0)
		{
			normal[0] *= -1;
			normal[1] *= -1;
			normal[2] *= -1;
		}
		// set current index
		mask.at<int>(current[0], current[1]) = index;
		// search neighbourhood
		for (int i = -1; i <= 1; ++i)
		for (int j = -1; j <= 1; ++j)
		if (i != 0 || j != 0)
		{
			newPoint[0] = current[0] + i;
			newPoint[1] = current[1] + j;
			if (newPoint[0] >= 0 && newPoint[1] >= 0 &&	newPoint[0] <  depth.rows && newPoint[1] < depth.cols)
			{
				neighbour = mask.at<int>(newPoint[0], newPoint[1]);
				if (processedMask.at<unsigned char>(newPoint[0], newPoint[1]) == 0 && (neighbour == 0 || neighbour == -1) && plane.distance(points.at<Vec3f>(newPoint[0], newPoint[1])) < min_planeDistance)
					unprocessed.push_back(newPoint);

				processedMask.at<unsigned char>(newPoint[0], newPoint[1]) = 1;
			}
		}
	}
}

//////////////////////////////////////////////////////////////////////
// Method segments a depth image using own tile plane searching RANSAC method.
// @param src Input CV_16UC depth matrix (raw input from kinect)
// @param cam_info Camera info message (ROS)
// @param minimumPlaneDistance Minimal distance threshold from triangle plane in whole tile (in meters)
// @param minimumTriDistance Minimal distance threshold from triangle plane in triangle flood fill (in meters)
// @param minimumFloodDistance Minimal distance threshold from triangle plane in whole image (in meters)
// @param tileSize Tile size
// @param smallestTriangleArea Smalest randomly found triangle to be considered for plane computation (in pixels^2)
// @param minRegionSize Minimal region size to be considered
// @param minCandidates Minimal number of pixels in tile to start a RANSAC
//////////////////////////////////////////////////////////////////////
void Regions::independentTileRegions(Mat &src, const CameraInfoConstPtr& cam_info, float minimumPlaneDistance, float minimumTriDistance, float minimumFloodDistance, int tileSize, int smallestTriangleArea, int minRegionSize, int minCandidates)
{
	Mat input(src.size(), CV_32FC1);
	Mat output(src.size(), CV_32FC1);

	Mat mask = Mat::zeros(src.size(), CV_32SC1);
	src.convertTo(input, CV_32F);


	if (m_normals == NULL)
		m_normals = new Normals(src, cam_info, NORMAL_COMPUTATION_TYPE, NORMAL_COMPUTATION_SIZE);


	int tileCenteri, tileCenterj;
	Vec3f points[3];
	Vec3f pointsInt[3];
	int random;
	Vec3f normal;
	float normalnorm;
	int currentIndex = 1;
	cv::Mat currentWindow, currentWindowMask, currentPoints;
	int maxrow = input.rows - tileSize;
	int maxcol = input.cols - tileSize;
	std::vector<Plane<float> > planes;
	std::vector<int > indices;

	// for each of tiles...
	for (int tilei = 0; tilei < maxrow; tilei+=tileSize)
	for (int tilej = 0; tilej < maxcol; tilej+=tileSize)
	{
		tileCenteri = tilei + tileSize/2;
		tileCenterj = tilej + tileSize/2;
		std::vector<Vec3f> candidates;
		std::vector<Vec3f> candidatesInt;

		// get all points in tile (used in RANSAC)
		for (int i = tilei; i < tilei+tileSize; ++i)
		for (int j = tilej; j < tilej+tileSize; ++j)
			if (mask.at<int>(i,j) == 0)
			{
				candidates.push_back(m_normals->m_points.at<Vec3f>(i,j));
				candidatesInt.push_back(Vec3f(i,j,0));
			}

		int xsize = candidates.size();

		// if there is less than 10 candidate points, skip the tile
		if (xsize < minCandidates)
		continue;

		// GO ransac
		for (unsigned int formax = 0; formax < RANSACMAX; ++formax)
		{
			// get three triangle points
			random = ((float)xsize*rand())/RAND_MAX;
			points[0] = candidates[random];
			pointsInt[0] = candidatesInt[random];
			random = ((float)xsize*rand())/RAND_MAX;
			points[1] = candidates[random];
			pointsInt[1] = candidatesInt[random];
			random = ((float)xsize*rand())/RAND_MAX;
			points[2] = candidates[random];
			pointsInt[2] = candidatesInt[random];


			if (points[0][2] < MIN_DISTANCE ||
				points[1][2] < MIN_DISTANCE ||
				points[2][2] < MIN_DISTANCE ||
				points[0][2] > MAX_DISTANCE ||
				points[1][2] > MAX_DISTANCE ||
				points[2][2] > MAX_DISTANCE ||
				norm((pointsInt[1] - pointsInt[0]).cross(pointsInt[2] - pointsInt[0])) < smallestTriangleArea )
			continue;


			// compute normal of random triangle
			normal = (points[1]-points[0]).cross((points[2]-points[0]));
			normalnorm = norm(normal);
			normal[0] /= normalnorm;
			normal[1] /= normalnorm;
			normal[2] /= normalnorm;
			Plane<float> plane(	normal[0], normal[1], normal[2],
					-(normal[0]*points[0][0] + normal[1]*points[0][1] + normal[2]*points[0][2]));

			// get current tile windows
			currentPoints = m_normals->m_points(Rect(tilej, tilei, tileSize, tileSize));
			currentWindow = input(Rect(tilej, tilei, tileSize, tileSize));
			mask(Rect(tilej, tilei, tileSize, tileSize)).copyTo(currentWindowMask);

			// flood fill tile with current region
			int size = floodFillTile(currentWindow, currentWindowMask, currentPoints, pointsInt, currentIndex, plane, tilei, tilej, minimumTriDistance);

			// if we found sufficient big region
			if (size > minRegionSize)
			{
				// get border and least squares normal
				std::vector<Vec2i> border;
				std::vector<Vec3f> points;
				for (int i = 0; i < tileSize; ++i)
				for (int j = 0; j < tileSize; ++j)
				{
					if (currentWindowMask.at<int>(i,j) == currentIndex)
						points.push_back(currentPoints.at<Vec3f>(i, j));
				}

				// fill the rest of tile
				for (int i = 0; i < tileSize; ++i)
				for (int j = 0; j < tileSize; ++j)
				{
					if (currentWindowMask.at<int>(i,j) == 0 && plane.distance(currentPoints.at<Vec3f>(i, j)) < minimumPlaneDistance)
						currentWindowMask.at<int>(i,j) = currentIndex;
				}

				points.clear();
				std::vector<Vec3f> points2;
				for (int i = 0; i < tileSize; ++i)
				for (int j = 0; j < tileSize; ++j)
				{
					if (currentWindowMask.at<int>(i,j) == currentIndex)
						points2.push_back(currentPoints.at<Vec3f>(i, j));
				}

				plane = Normals::LeastSquaresPlane(points2);
				indices.push_back(currentIndex);
				planes.push_back(plane);

				// save mask into whole image mask
				currentWindow = mask(Rect(tilej, tilei, tileSize, tileSize));
				currentWindowMask.copyTo(currentWindow);

				// floodFillRest()
				fillEverything(plane, input, mask, m_normals->m_points, tilei, tilej, tileSize, currentIndex, minimumFloodDistance);
				++currentIndex;
				// we do not need to search anymore here
				break;
			} // if sufficient big region
		} // RANSAC loop
	} // tile loop

	m_regionMatrix = Mat(src.size(), CV_32SC1);
	mask.convertTo(m_regionMatrix, CV_32SC1);
} // function

} // namespace
