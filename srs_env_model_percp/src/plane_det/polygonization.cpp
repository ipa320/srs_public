/******************************************************************************
 * \file
 *
 * $Id: polygonization.cpp 397 2012-03-29 12:50:30Z spanel $
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

#include "plane_det/polygonization.h"

using namespace std;
using namespace sensor_msgs;
using namespace cv;

namespace but_scenemodel
{


void Polygonizer::GetPolyRegions(cv::Mat &regionImage)
{
	// under construction :-)
//	CvMemStorage *mem;
//	CvSeq *contours, *ptr;
//
//	Mat input = Mat(regionImage.size(), CV_32FC1);
//	Mat input8 = Mat(regionImage.size(), CV_8UC1);
//	regionImage.convertTo(input, CV_32FC1);
//	IplImage g_gray = input;
//
//
//	cvThreshold(&g_gray, &g_gray, 1, 255, CV_THRESH_BINARY);
//
//	input = Mat(&g_gray);
//	input.convertTo(input8, CV_8UC1);
//	g_gray = input8;
//
//	mem = cvCreateMemStorage(0);
//	cvFindContours(&g_gray, mem, &contours, sizeof(CvContour), CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cvPoint(0,0));
//
//	cvZero(&g_gray);
////	for (ptr = contours; ptr != NULL; ptr = ptr->h_next) {
////		CvScalar color = CV_RGB( rand()&255, rand()&255, rand()&255 );
////		cvDrawContours(&g_gray, ptr, color, CV_RGB(0,0,0), -1, CV_FILLED, 8, cvPoint(0,0));
////	}
//
//
//    double perimeter1 = cvArcLength((CvSeq*)contours,CV_WHOLE_SEQ,-1);
//	CvSeq* polySeq1 = cvApproxPoly((CvSeq*)contours,sizeof(CvContour),mem,CV_POLY_APPROX_DP,perimeter1*0.002,0);
//	cvDrawContours(&g_gray,	polySeq1, cvScalarAll(255),	cvScalarAll(255), 100);
//
//	input8 = Mat(&g_gray);
//	input8.convertTo(regionImage, CV_32SC1);
//	//regionImage = Mat(&g_gray);


}

}
