/******************************************************************************
 * \file
 *
 * $Id:$
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

#include <srs_env_model_percp/but_plane_detector/parameter_space_hierarchy.h>

using namespace srs_env_model_percp;
using namespace std;
using namespace sensor_msgs;
using namespace cv;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Constructor - creates and allocates a space (angle X angle X shift)
// @param anglemin Minimal angle in space
// @param anglemax Maximal angle in space
// @param zmin Minimal shift in space
// @param zmax Maximal shift in space
// @param angleRes Angle resolution (step)
// @param shiftRes Shift resolution (step)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpaceHierarchy::ParameterSpaceHierarchy(double anglemin, double anglemax, double zmin, double zmax, double angleRes, double shiftRes)
{
	m_init = false;
	m_angleStep = (anglemax - anglemin) / (angleRes - 1);
	m_shiftStep = (zmax - zmin) / (shiftRes - 1);
	m_angleLoStep = m_angleStep * DEFAULT_BIN_SIZE;
	m_shiftLoStep = m_shiftStep * DEFAULT_BIN_SIZE;

	m_angleSize = (anglemax - anglemin) / m_angleStep + 1;
	m_angleSize2 = m_angleSize * m_angleSize;
	m_shiftSize = (zmax - zmin) / m_shiftStep + 1;

	assert(m_angleSize % DEFAULT_BIN_SIZE == 0 && m_shiftSize % DEFAULT_BIN_SIZE == 0);

	m_angleLoSize = m_angleSize / DEFAULT_BIN_SIZE;
	m_angleLoSize2 = m_angleSize2 / (DEFAULT_BIN_SIZE*DEFAULT_BIN_SIZE);
	m_shiftLoSize = m_shiftSize / DEFAULT_BIN_SIZE;

	m_shiftmin = zmin;
	m_shiftmax = zmax;
	m_anglemin = anglemin;
	m_anglemax = anglemax;
	m_size = m_angleSize2 * m_shiftSize;
	m_loSize = m_angleLoSize2 * m_shiftLoSize;

	m_hiSize = DEFAULT_BIN_SIZE*DEFAULT_BIN_SIZE*DEFAULT_BIN_SIZE;
	m_hiSize2 = DEFAULT_BIN_SIZE*DEFAULT_BIN_SIZE;
	m_dataLowRes = (double **)malloc(sizeof(double*)*m_loSize);
	for (int i = 0; i < m_loSize; ++i)
		m_dataLowRes[i] = NULL;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Destructor
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpaceHierarchy::~ParameterSpaceHierarchy()
{
	for (int i = 0; i < m_loSize; ++i)
		if (m_dataLowRes[i] != NULL) free(m_dataLowRes[i]);

	free(m_dataLowRes);
}

void ParameterSpaceHierarchy::clear()
{
	for (int i = 0; i < m_loSize; ++i)
	{
		if (m_dataLowRes[i] != NULL) free(m_dataLowRes[i]);
		m_dataLowRes[i] = NULL;
	}

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Conversion from Euclidian representation of normal (x, y, z) to parametrized (a1, a2)
// @param x X vector coordinate
// @param y Y vector coordinate
// @param z Z vector coordinate
// @param a1 First angle
// @param a2 Second angle
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::toAngles(float x, float y, float z, float &a1, float &a2)
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
void ParameterSpaceHierarchy::toEuklid(float a1, float a2, float &x, float &y, float &z)
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
double ParameterSpaceHierarchy::get(int angle1, int angle2, int z)
{
	IndexStruct index = getIndex(angle1, angle2, z);
	//std::cout << m_loSize << std::endl;
	//std::cout << loIndex << " -> " << z%DEFAULT_BIN_SIZE * m_hiSize2 + angle2%DEFAULT_BIN_SIZE * DEFAULT_BIN_SIZE + angle1%DEFAULT_BIN_SIZE << std::endl;
	//std::cout << std::endl;
	if (m_dataLowRes[index.lowResolutionIndex] == NULL)
		return 0.0;
	else
		return m_dataLowRes[index.lowResolutionIndex][index.highResolutionIndex];

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//* Saves a value at given coordinates (indices)
//* @param angle1 First angle coordinate
//* @param angle2 Second angle coordinate
//* @param z Shift (d param) coordinate
//* @param val Value to be saved
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::set(int angle1, int angle2, int z, double val)
{
	IndexStruct index = getIndex(angle1, angle2, z);
	if (m_dataLowRes[index.lowResolutionIndex] == NULL)
	{
		m_dataLowRes[index.lowResolutionIndex] = (double *)malloc(sizeof(double) *m_hiSize);
		memset(m_dataLowRes[index.lowResolutionIndex], 0, m_hiSize * sizeof(double));
	}
	m_dataLowRes[index.lowResolutionIndex][index.highResolutionIndex] = val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a value saved at given index
// @param index Given index
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::get(int index)
{
	if (m_dataLowRes[index / m_hiSize] == NULL)
		return 0.0;
	else
		return m_dataLowRes[index / m_hiSize][index % m_hiSize];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Saves a value at given index
// @param index Given index
// @param val Value to be saved
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::set(int index, double val)
{
	if (m_dataLowRes[index / m_hiSize] == NULL)
	{
		m_dataLowRes[index / m_hiSize] = (double *)malloc(sizeof(double) *m_hiSize);
		memset(m_dataLowRes[index / m_hiSize], 0, m_hiSize * sizeof(double));
	}
	m_dataLowRes[index / m_hiSize][index % m_hiSize] = val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns an index from given values
// @param angle1 First angle value
// @param angle2 Second angle value
// @param z Shift (d param) value
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IndexStruct ParameterSpaceHierarchy::getIndex(double angle1, double angle2, double z)
{
	IndexStruct index;
	int a1 = (angle1-m_anglemin) / m_angleStep;
	int a2 = (angle2-m_anglemin) / m_angleStep;
	int zz = (z-m_shiftmin) / m_shiftStep;
	index.highResolutionIndex = zz%DEFAULT_BIN_SIZE * m_hiSize2 + a2%DEFAULT_BIN_SIZE * DEFAULT_BIN_SIZE + a1%DEFAULT_BIN_SIZE;
	index.lowResolutionIndex = zz/DEFAULT_BIN_SIZE * m_angleLoSize2 + a2/DEFAULT_BIN_SIZE * m_angleLoSize + a1/DEFAULT_BIN_SIZE;
	index.CompleteIndex = index.lowResolutionIndex * m_hiSize + index.highResolutionIndex;

	return index;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Converts an index into axis coordinates
// @param i Given index
// @param angle1 First angle coordinate
// @param angle2 Second angle coordinate
// @param z Shift (d param) coordinate
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::fromIndex(int i, int& angle1, int& angle2, int& z)
{
	int iLo = i / m_hiSize;
	int iHi = i % m_hiSize;
	int loZ, loA1, loA2;

	z = (iLo / (m_angleLoSize2));
	loZ = iHi / (m_hiSize2);

	angle2 = ((iLo - z * m_angleLoSize2) / m_angleLoSize);
	loA2 = (iHi - loZ * m_hiSize2) / DEFAULT_BIN_SIZE;

	angle1 = (iLo - (z * m_angleLoSize2) - angle2 * m_angleLoSize);
	loA1 =   iHi - (loZ * m_hiSize2) - loA2 * DEFAULT_BIN_SIZE;

	z = z*DEFAULT_BIN_SIZE + loZ;
	angle2 = angle2*DEFAULT_BIN_SIZE + loA2;
	angle1 = angle1*DEFAULT_BIN_SIZE + loA1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns an index from given coordinate indices
// @param angle1 First angle coordinate
// @param angle2 Second angle coordinate
// @param z Shift (d param) coordinate
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IndexStruct ParameterSpaceHierarchy::getIndex(int angle1, int angle2, int z)
{
	IndexStruct index;

	index.highResolutionIndex = z%DEFAULT_BIN_SIZE * m_hiSize2 + angle2%DEFAULT_BIN_SIZE * DEFAULT_BIN_SIZE + angle1%DEFAULT_BIN_SIZE;
	index.lowResolutionIndex = z/DEFAULT_BIN_SIZE * m_angleLoSize2 + angle2/DEFAULT_BIN_SIZE * m_angleLoSize + angle1/DEFAULT_BIN_SIZE;
	index.CompleteIndex = index.lowResolutionIndex * m_hiSize + index.highResolutionIndex;

	return index;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Converts between values and coordinates
// @param angle1 First angle value
// @param angle2 Second angle value
// @param z Shift (d param) value
// @param angle1Index First angle coordinate
// @param angle2Index Second angle coordinate
// @param shiftIndex (d param) coordinate
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::getIndex(double angle1, double angle2, double z, int &angle1Index, int &angle2Index, int &shiftIndex)
{
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
void ParameterSpaceHierarchy::addVolume(ParameterSpace &second, int angle1, int angle2, int shift)
{
	//std::cout << angle1 << " " << angle2 << " " << shift << " " << std::endl;
	double secondAngleHalf = second.m_angleSize / 2;
	double secondShiftHalf = second.m_shiftSize / 2;
	int shiftThis, angle1This, angle2This;
	int shiftS, angle1S, angle2S;

	for (shiftS = 0, shiftThis = shift - secondShiftHalf; shiftS < second.m_shiftSize; ++shiftS, ++shiftThis)
	for (angle2S=0, angle2This = angle2 - secondAngleHalf; angle2S < second.m_angleSize; ++angle2S, ++angle2This)
	for (angle1S=0, angle1This = angle1 - secondAngleHalf; angle1S < second.m_angleSize; ++angle1S, ++angle1This)
	{
		if (angle1This >= 0 && angle1This < m_angleSize &&
			angle2This >= 0 && angle2This < m_angleSize &&
			shiftThis  >= 0 && shiftThis < m_shiftSize)
		{
			set(angle1This, angle2This, shiftThis, get(angle1This, angle2This, shiftThis) + second(angle1S, angle2S, shiftS));
		}

	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Adds a second volume to this with offset and multiplied by given factor
// @param second Second volume to be added
// @param angle1 Angle 1 offset
// @param angle2 Angle 2 offset
// @param shift Shift offset
// @param factor number by which each gauss function will be multiplied
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::addVolume(ParameterSpace &second, int angle1, int angle2, int shift, float factor)
{
	double secondAngleHalf = second.m_angleSize / 2;
	double secondShiftHalf = second.m_shiftSize / 2;
	int shiftThis, angle1This, angle2This;
	int shiftS, angle1S, angle2S;

	for (shiftS = 0, shiftThis = shift - secondShiftHalf; shiftS < second.m_shiftSize; ++shiftS, ++shiftThis)
	for (angle2S=0, angle2This = angle2 - secondAngleHalf; angle2S < second.m_angleSize; ++angle2S, ++angle2This)
	for (angle1S=0, angle1This = angle1 - secondAngleHalf; angle1S < second.m_angleSize; ++angle1S, ++angle1This)
	{
		if (angle1This >= 0 && angle1This < m_angleSize &&
			angle2This >= 0 && angle2This < m_angleSize &&
			shiftThis  >= 0 && shiftThis < m_shiftSize)
		{
			set(angle1This, angle2This, shiftThis, get(angle1This, angle2This, shiftThis) + factor * second(angle1S, angle2S, shiftS));
		}

	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Finds maximas in this and saves them as planes to given vector
// @param indices Found planes
// @returns index of maximal plane
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpaceHierarchy::findMaxima(std::vector<Plane<float> > &indices, double min_value)
{
	float a, b, c;
	int maxind = -1;
	float max = -1;
	int angle1, angle2, shift;

	ParameterSpaceHierarchyFullIterator it(this);

	// pass all non null points
	while (not it.end)
	{
		double val = it.getVal();
		if (val > min_value)
		{
			this->fromIndex(it.currentI, angle1, angle2, shift);
			if (angle1 >= 1 && angle1 < m_angleSize &&
				angle2 >= 1 && angle2 < m_angleSize &&
				shift >= 1 && shift < m_shiftSize)
			{
				val +=  get(angle1-1, angle2, shift) +
						get(angle1+1, angle2, shift) +
						get(angle1, angle2-1, shift) +
						get(angle1, angle2+1, shift) +
						get(angle1, angle2, shift+1) +
						get(angle1, angle2, shift-1);

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
					aux = 	get(xx, yy, zz) +
							get(xx-1, yy, zz) +
							get(xx+1, yy, zz) +
							get(xx, yy-1, zz) +
							get(xx, yy+1, zz) +
							get(xx, yy, zz+1) +
							get(xx, yy, zz-1);
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
						std::cout << "Found plane size: " << val << " eq: " << aroundx <<" "<< aroundy <<" "<< aroundz <<" "<< arounds <<" "<< std::endl;
						indices.push_back(Plane<float>(aroundx, aroundy, aroundz, arounds));
						if (val > max)
						{
							max = val;
							maxind = indices.size() - 1;
						}
					}
			}
		}
		++it;
	}
	return maxind;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Converts index in parameter space into angle value
// @param index Angle axis index
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::getAngle(int index)
{
	return (m_angleStep * ((double)index)) + m_anglemin;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Converts index in parameter space into shift value
// @param index Shift axis index
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::getShift(int index)
{
	return ((double)index) * m_shiftStep + m_shiftmin;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a size of this space structure in Bytes
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpaceHierarchy::getSize()
{
	int size = 0;
	for (int i = 0; i < m_loSize; ++i)
		if (m_dataLowRes[i] != NULL) size += m_hiSize;
	return size;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Returns a value saved at given values
// @param angle1 First angle value
// @param angle2 Second angle value
// @param z Shift (d param) value
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::get(double angle1, double angle2, double z)
{
	IndexStruct index = getIndex(angle1, angle2, z);

	if (m_dataLowRes[index.lowResolutionIndex] == NULL)
			return 0.0;
		else
			return m_dataLowRes[index.lowResolutionIndex][index.highResolutionIndex];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Saves a value at given values
// @param angle1 First angle value
// @param angle2 Second angle value
// @param z Shift (d param) value
// @param val Value to be saved
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::set(double angle1, double angle2, double z, double val)
{
	IndexStruct index = getIndex(angle1, angle2, z);

	if (m_dataLowRes[index.lowResolutionIndex] == NULL)
		{
			m_dataLowRes[index.lowResolutionIndex] = (double *)malloc(sizeof(double) *m_hiSize);
			memset(m_dataLowRes[index.lowResolutionIndex], 0, m_hiSize * sizeof(double));
		}
		m_dataLowRes[index.lowResolutionIndex][index.highResolutionIndex] = val;
}


