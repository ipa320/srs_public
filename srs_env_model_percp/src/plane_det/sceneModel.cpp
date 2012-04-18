/******************************************************************************
 * \file
 *
 * $Id: sceneModel.cpp 619 2012-04-16 13:47:28Z ihulik $
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

#include "plane_det/sceneModel.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "plane_det/filtering.h"


using namespace pcl;
using namespace cv;

namespace but_scenemodel
{

/////////////////////////////////////////////////////////////////////ineck/////////////////////////////////////////////////////////
SceneModel::SceneModel(	double max_depth,
						double min_shift,
						double max_shift,
						int angle_resolution,
						int shift_resolution,
						int gauss_angle_res,
						int gauss_shift_res,
						double gauss_angle_sigma,
						double gauss_shift_sigma) :	scene_cloud(new PointCloud<PointXYZRGB>),
													space(-M_PI, M_PI, min_shift, max_shift, angle_resolution, shift_resolution),
													cache_space(-M_PI, M_PI, min_shift, max_shift, angle_resolution, shift_resolution),
													gauss(-(gauss_angle_res/2) * space.m_angleStep, (gauss_angle_res/2) * space.m_angleStep, -(gauss_shift_res/2) * space.m_shiftStep, (gauss_shift_res/2) * space.m_shiftStep, gauss_angle_res, gauss_shift_res)
{
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Using hierarchic array
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	// var init

	m_depth = max_depth;

	// init of HS
	std::cout << "Parameter space size: " << space.getSize()*sizeof(double) / 1000000.0 << " MB" << std::endl;
	std::cout << "Parameter shift step: " << space.m_shiftStep << std::endl;
	std::cout << "Parameter angle step: " << space.m_angleStep << std::endl;
	std::cout << "Gauss space size: " << gauss.m_size*sizeof(double) / 1000000.0 << " MB" << std::endl;
	std::cout << "Gauss shift step: " << gauss.m_shiftStep << std::endl;
	std::cout << "Gauss angle step: " << gauss.m_angleStep << std::endl;

	// generate Gauss function in gauss space
	gauss.generateGaussIn(gauss_angle_sigma, gauss_shift_sigma);


}

void SceneModel::recomputePlanes()
{
	planes.clear();

	std::vector<Plane<float> > aux;
	space.findMaxima(aux);

	std::vector<bool> used(aux.size(), false);
	for (unsigned int i = 0; i < aux.size(); ++i)
	if (not used[i])
	{
		Plane<float> final(aux[i].a, aux[i].b, aux[i].c, aux[i].d);
		int count = 1;
		for (unsigned int j = i+1; j < aux.size(); ++j)
		if (not used[j] && aux[i].isSimilar(aux[j], 0.2, 0.5))
		{
			final.a += aux[j].a;
			final.b += aux[j].b;
			final.c += aux[j].c;
			final.d += aux[j].d;
			++count;
		}

		final.a /= count;
		final.b /= count;
		final.c /= count;
		final.d /= count;
		planes.push_back(final);
	}

	std::cout << "Found : " << planes.size() << " planes." << std::endl;
}

void SceneModel::AddNext(Mat &depth, const CameraInfoConstPtr& cam_info, Normals &normals)
{
	int maxi = normals.m_points.rows;
	int maxj = normals.m_points.cols;

	Vec3f point;
	Vec4f plane;
	Regions reg(&normals);
	reg.watershedRegions(depth, cam_info, WatershedType::DepthDiff, 1, 2, 20);
	double min, max;
	minMaxLoc(reg.m_regionMatrix, &min, &max);
	indexFactor = 1.0;

	if (max !=0)
		indexFactor /= (double)max;

	scene_cloud->clear();
	// pass all points and write them into init space
	for (int i = 0; i < maxi; ++i)
		for (int j = 0; j < maxj; ++j)
		{
			point = normals.m_points.at<Vec3f>(i, j);
			plane = normals.m_planes.at<Vec4f>(i, j);

			// skip all which is farer than 3m
			if (point[2] < m_depth)
			{
				// signed angle atan2(b.y,b.x) - atan2(a.y,a.x)
				// angle on XZ plane with X
				float a1, a2;
				ParameterSpace::toAngles(plane[0], plane[1], plane[2], a1, a2);


				PointXYZRGB rgbpoint(255, 255, 255);
				if (reg.m_regionMatrix.at<int>(i, j) > 0)
					rgbpoint.rgb = indexFactor *reg.m_regionMatrix.at<int>(i, j);

				rgbpoint.x = point[0];
				rgbpoint.y = point[1];
				rgbpoint.z = point[2];
				scene_cloud->push_back(rgbpoint);

				int i, j, k;
				cache_space.getIndex(a1, a2, plane[3], i, j, k);
				cache_space.set(i, j, k, cache_space.get(i, j, k) + 1);
			}
		}

		// fire up the iterator on cache space
		ParameterSpaceHierarchyFullIterator it(&cache_space);
		int i, j, k;
		double val;
		// for each point in cache space which is not zero, write a multiplied gauss into the HT
		while (not it.end)
		{
			val = it.getVal();
			if (val > 0.0)
			{
				space.fromIndex(it.currentI, i, j, k);
				space.addVolume(gauss, i, j, k, val);
			}
			++it;
		}

		std::cout << "New parameter space size: " << (double)space.getSize()*sizeof(double) / 1000000.0 << " MB" << std::endl;

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Control visualisaation - uncoment to see HT space
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//		min = 99999999999.0;
		//		max = -99999999999.0;
		//		for (int shift = 0; shift < space.m_shiftSize; shift += 1)
		//		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
		//		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
		//		{
		//			float value = space.get(angle1, angle2, shift);
		//			if (value < min) min = value;
		//			if (value > max) max = value;
		//
		////			if (space(angle1, angle2, shift) > 0.0)
		////			{
		////			PointXYZI pt;
		////			pt.x = (float)angle1;
		////			pt.y = (float)angle2;
		////			pt.z = (float)shift;
		////			pt.intensity = space(angle1, angle2, shift);
		////			current_hough_cloud->push_back(pt);
		////			}
		//		}
		//
		//		for (int shift = 0; shift < space.m_shiftSize; shift += 1)
		//		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
		//		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
		//		{
		//			if (space.get(angle1, angle2, shift) != 0)
		//				space.set(angle1, angle2, shift, 255.0*((space.get(angle1, angle2, shift) - min) / (max - min)));
		//		}
		//
		//		cv::Mat image = cv::Mat::zeros(cvSize(space.m_angleSize, space.m_angleSize), CV_32FC1);
		//		int shiftview = space.m_shiftSize/2;
		//		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
		//		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
		//		{
		//			image.at<float>(angle1, angle2) = space.get(angle1, angle2, shiftview);
		//		}
		//
		//
		//
		//		//create a new window & display the image
		//		cvNamedWindow("Smile", 1);
		//		cvShowImage("Smile", &IplImage(image));
		//		std::cout << "Viewing shift = " << shiftview << std::endl;
		//		//wait for key to close the window
		//		int key = 0;
		//		while(1)
		//		{
		//		    key = cvWaitKey();
		//		    key &= 0x0000ffff;
		//		    std::cout << key << std::endl;
		//		    if(key==27 || key == 0xffff) break;
		//
		//		    switch(key)
		//		    {
		//		        case 'a':
		//		        	if (shiftview < space.m_shiftSize-1)
		//		        	{
		//		        		++shiftview;
		//		        		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
		//		        		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
		//		        		{
		//		        			image.at<float>(angle1, angle2) = space.get(angle1, angle2, shiftview);
		//		        		}
		//		        	}
		//		        	cvShowImage("Smile", &IplImage(image));
		//		        	std::cout << "Viewing shift = " << shiftview << "/" << space.m_shiftSize-1 << std::endl;
		//		            break;
		//		        case 'z':
		//		        	if (shiftview > 0)
		//		        	{
		//		        		--shiftview;
		//		        		for (int angle1 = 0; angle1 < space.m_angleSize; angle1 += 1)
		//		        		for (int angle2 = 0; angle2 < space.m_angleSize; angle2 += 1)
		//		        		{
		//		        			image.at<float>(angle1, angle2) = space.get(angle1, angle2, shiftview);
		//		        		}
		//		        	}
		//		        	cvShowImage("Smile", &IplImage(image));
		//		        	std::cout << "Viewing shift = " << shiftview << "/" << space.m_shiftSize-1 << std::endl;
		//		            break;
		//		    }
		//		}
		//
		//
		//		cvDestroyWindow( "Smile" );
}
}
