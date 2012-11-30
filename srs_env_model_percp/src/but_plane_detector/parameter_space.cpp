/******************************************************************************
 * \file
 *
 * $Id: parameterSpace.cpp 694 2012-04-20 10:24:24Z ihulik $
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
 *	 Class encapsulating Parameter space (i.e. 3D Hough grid)
 *
 *	 Contains methods for construction / maxima search / adding of volumes etc.
 */

#include <srs_env_model_percp/but_plane_detector/parameter_space.h>

using namespace srs_env_model_percp;
using namespace std;
using namespace sensor_msgs;
using namespace cv;
using namespace but_plane_detector;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor - creates and allocates a space (angle X angle X shift)
// @param anglemin Minimal angle in space
// @param anglemax Maximal angle in space
// @param zmin Minimal shift in space
// @param zmax Maximal shift in space
// @param angleRes Angle resolution (step)
// @param shiftRes Shift resolution (step)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpace::ParameterSpace(double anglemin, double anglemax, double zmin, double zmax, double angleRes, double shiftRes)
{
	m_init = false;
	m_angleStep = (anglemax - anglemin) / (angleRes - 1);
	m_shiftStep = (zmax - zmin) / (shiftRes - 1);

	m_angleSize = (anglemax - anglemin) / m_angleStep + 1;
	m_angleSize2 = m_angleSize * m_angleSize;
	m_shiftSize = (zmax - zmin) / m_shiftStep + 1;

	m_shiftmin = zmin;
	m_shiftmax = zmax;
	m_anglemin = anglemin;
	m_anglemax = anglemax;
	m_size = m_angleSize2 * m_shiftSize;
	m_data = new double[m_size];
	memset(m_data, 0, sizeof(double)*m_size);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructor
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpace::~ParameterSpace()
{
	delete m_data;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Conversion from Euclidian representation of normal (x, y, z) to parametrized (a1, a2)
// @param x X vector coordinate
// @param y Y vector coordinate
// @param z Z vector coordinate
// @param a1 First angle
// @param a2 Second angle
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::toAngles(float x, float y, float z, float &a1, float &a2)
{
	a1 = atan2(z, x);
	// align with X on XZ
	x = z * sin(a1) + x * cos(a1);
	a2 = atan2(y, x);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Conversion from parametrized representation of normal (a1, a2) to Euclidian (x, y, z)
// @param x X vector coordinate
// @param y Y vector coordinate
// @param z Z vector coordinate
// @param a1 First angle
// @param a2 Second angle
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::toEuklid(float a1, float a2, float &x, float &y, float &z)
{
	float auxx = cos(-a2);
	y = -sin(-a2);
	x = auxx * cos(-a1);
	z = -auxx * sin(-a1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a value saved at given coordinates (indices)
// @param angle1 First angle coordinate
// @param angle2 Second angle coordinate
// @param z Shift (d param) coordinate
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double &ParameterSpace::operator() (int angle1, int angle2, int z)
{
	assert( angle1 < m_angleSize && angle1 >= 0 && angle2 < m_angleSize && angle2 >= 0 && z < m_shiftSize && z >= 0);
	return m_data[z * m_angleSize2 + angle2 * m_angleSize + angle1];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a value saved at given index
// @param index Given index
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double &ParameterSpace::operator[](int index)
{
	return m_data[index];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpace::getIndex(double angle1, double angle2, double z)
{
	assert( angle1 <= m_anglemax && angle1 >= m_anglemin &&
			angle2 <= m_anglemax && angle2 >= m_anglemin &&
			z >= m_shiftmin && z <= m_shiftmax);
	int a1 = (angle1-m_anglemin) / m_angleStep;
	int a2 = (angle2-m_anglemin) / m_angleStep;
	int zz = (z-m_shiftmin) / m_shiftStep;
	return zz * m_angleSize2 + a2 * m_angleSize + a1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns an index from given values
// @param angle1 First angle value
// @param angle2 Second angle value
// @param z Shift (d param) value
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::getIndex(double angle1, double angle2, double z, int &angle1Index, int &angle2Index, int &shiftIndex)
{
	assert( angle1 <= m_anglemax && angle1 >= m_anglemin &&
			angle2 <= m_anglemax && angle2 >= m_anglemin &&
			z >= m_shiftmin && z <= m_shiftmax);
	angle1Index = (angle1-m_anglemin) / m_angleStep;
	angle2Index = (angle2-m_anglemin) / m_angleStep;
    shiftIndex = (z-m_shiftmin) / m_shiftStep;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adds a second volume to this with offset
// @param second Second volume to be added
// @param angle1 Angle 1 offset
// @param angle2 Angle 2 offset
// @param shift Shift offset
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::addVolume(ParameterSpace &second, int angle1, int angle2, int shift)
{
	double secondAngleHalf = second.m_angleSize / 2;
	double secondShiftHalf = second.m_shiftSize / 2;
	int shiftThis, angle1This, angle2This;
	int shiftS, angle1S, angle2S;

	for (shiftS = 0, shiftThis = shift - secondShiftHalf;
		 shiftS < second.m_shiftSize;
		 ++shiftS, ++shiftThis)
	for (angle2S=0, angle2This = angle2 - secondAngleHalf;
		 angle2S < second.m_angleSize;
		 ++angle2S, ++angle2This)
	for (angle1S=0, angle1This = angle1 - secondAngleHalf;
		 angle1S < second.m_angleSize;
		 ++angle1S, ++angle1This)
	{
		if (angle1This >= 0 && angle1This < m_angleSize &&
			angle2This >= 0 && angle2This < m_angleSize &&
			shiftThis  >= 0 && shiftThis < m_shiftSize)
			this->operator ()(angle1This, angle2This, shiftThis) += second(angle1S, angle2S, shiftS);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Generates a Gauss curve into this space (centered)
// @param angleSigma sigma in angle directions
// @param shiftSigma sigma in shift direction
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::generateGaussIn(double angleSigma, double shiftSigma)
{
	double angleGaussMult = 1.0;// / (angleSigma * sqrt(2 * M_PI));
	double angleGauss2o2 = 2 * angleSigma * angleSigma;
	double shiftGaussMult = 1.0;// / (shiftSigma * sqrt(2 * M_PI));
	double shiftGauss2o2 = 2 * shiftSigma * shiftSigma;

	for (int shift=0; shift < m_shiftSize; ++shift)
	{
		double shiftReal = getShift(shift);
		double shiftGauss = shiftGaussMult * exp(-(shiftReal * shiftReal) / shiftGauss2o2);
		for (int angle2=0; angle2 < m_angleSize; ++angle2)
		{
			double angle2Real = getAngle(angle2);
			double angle2Gauss = angleGaussMult * exp(-(angle2Real * angle2Real) / angleGauss2o2);
			for (int angle1=0; angle1 < m_angleSize; ++angle1)
			{
				double angle1Real = getAngle(angle1);
				double angle1Gauss = angleGaussMult * exp(-(angle1Real * angle1Real) / angleGauss2o2);
				this->operator ()(angle1, angle2, shift) = shiftGauss * angle2Gauss * angle1Gauss;
			}
		}
	}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Finds maximas in this and saves them as planes to given vector
// @param indices Found planes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpace::findMaxima(tPlanes &indices)
{
	int around = 2;
	float a, b, c;
	int maxind = -1;
	float max = -1;

	for (int shift=around; shift < m_shiftSize-around; ++shift)
		{
			for (int angle2=around; angle2 < m_angleSize-around; ++angle2)
			{
				for (int angle1=around; angle1 < m_angleSize-around; ++angle1)
				{
					double val = this->operator ()(angle1, angle2, shift);
					if (val > 1500)
					{
						val += this->operator()(angle1-1, angle2, shift) +
							   this->operator()(angle1+1, angle2, shift) +
							   this->operator()(angle1, angle2-1, shift) +
							   this->operator()(angle1, angle2+1, shift) +
							   this->operator()(angle1, angle2, shift+1) +
							   this->operator()(angle1, angle2, shift-1);

						bool ok = true;
						double aux;
						int xx, yy, zz;
						for (int x = -1; x <= 1; ++x)
						for (int y = -1; y <= 1; ++y)
						for (int z = -1; z <= 1; ++z)
						{
							xx = angle1 + x;
							yy = angle2 + y;
							zz = shift + z;
							aux = this->operator()(xx, yy, zz) +
								  this->operator()(xx-1, yy, zz) +
								  this->operator()(xx+1, yy, zz) +
								  this->operator()(xx, yy-1, zz) +
								  this->operator()(xx, yy+1, zz) +
								  this->operator()(xx, yy, zz+1) +
								  this->operator()(xx, yy, zz-1);
							if (val < aux)
							{
								ok = false;
								break;
							}
						}

						if (ok)
						{
							double aroundx = 0;
							double aroundy = 0;
							double aroundz = 0;
							double arounds = 0;
							for (int x = -1; x <= 1; ++x)
							for (int y = -1; y <= 1; ++y)
							for (int z = -1; z <= 1; ++z)
							{
								xx = angle1 + x;
								yy = angle2 + y;
								zz = shift + z;
								toEuklid(getAngle(xx), getAngle(yy), a, b, c);
								aroundx += a;
								aroundy += b;
								aroundz += c;
								arounds += getShift(zz);
							}
							aroundx /= 7.0;
							aroundy /= 7.0;
							aroundz /= 7.0;
							arounds /= 7.0;
							std::cerr << "Found plane size: " << val << " eq: " << aroundx <<" "<< aroundy <<" "<< aroundz <<" "<< arounds <<" "<< std::endl;
							indices.push_back(Plane<float>(aroundx, aroundy, aroundz, arounds));
							if (val > max)
							{
								max = val;
								maxind = indices.size() - 1;
							}
						}
					}
				}
			}
		}
	return maxind;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Converts index in parameter space into angle value
// @param index Angle axis index
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpace::getAngle(int index)
{
	return (m_angleStep * ((double)index)) + m_anglemin;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Converts index in parameter space into shift value
// @param index Shift axis index
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpace::getShift(int index)
{
	return ((double)index) * m_shiftStep + m_shiftmin;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a value saved at given values
// @param angle1 First angle value
// @param angle2 Second angle value
// @param z Shift (d param) value
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double &ParameterSpace::operator() (double angle1, double angle2, double z)
{
	return m_data[getIndex(angle1, angle2, z)];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a size of this space structure in Bytes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpace::getSize()
{
	return m_angleSize * m_angleSize * m_shiftSize;
}

