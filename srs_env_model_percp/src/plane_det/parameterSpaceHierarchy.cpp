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
 *
 */

#include "plane_det/parameterSpaceHierarchy.h"

using namespace but_scenemodel;
using namespace std;
using namespace sensor_msgs;
using namespace cv;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpaceHierarchy::~ParameterSpaceHierarchy()
{
	for (int i = 0; i < m_loSize; ++i)
		if (m_dataLowRes[i] != NULL) free(m_dataLowRes[i]);

	free(m_dataLowRes);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::toAngles(float x, float y, float z, float &a1, float &a2)
{
	a1 = atan2(z, x);
	// align with X on XZ
	x = z * sin(a1) + x * cos(a1);
	a2 = atan2(y, x);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::toEuklid(float a1, float a2, float &x, float &y, float &z)
{
	float auxx = cos(-a2);
	y = -sin(-a2);
	x = auxx * cos(-a1);
	z = -auxx * sin(-a1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::get(int index)
{
	if (m_dataLowRes[index / m_hiSize] == NULL)
		return 0.0;
	else
		return m_dataLowRes[index / m_hiSize][index % m_hiSize];
}

void ParameterSpaceHierarchy::set(int index, double val)
{
	if (m_dataLowRes[index / m_hiSize] == NULL)
	{
		m_dataLowRes[index / m_hiSize] = (double *)malloc(sizeof(double) *m_hiSize);
		memset(m_dataLowRes[index / m_hiSize], 0, m_hiSize * sizeof(double));
	}
	m_dataLowRes[index / m_hiSize][index % m_hiSize] = val;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
IndexStruct ParameterSpaceHierarchy::getIndex(double angle1, double angle2, double z)
{
	IndexStruct index;
	int a1 = (angle1-m_anglemin) / m_angleStep;
	int a2 = (angle2-m_anglemin) / m_angleStep;
	int zz = (z-m_shiftmin) / m_shiftStep;
	index.highResolutionIndex = zz%DEFAULT_BIN_SIZE * m_hiSize2 + a2%DEFAULT_BIN_SIZE * DEFAULT_BIN_SIZE + a1%DEFAULT_BIN_SIZE;
	index.lowResolutionIndex = zz/DEFAULT_BIN_SIZE * m_angleLoSize2 + a2/DEFAULT_BIN_SIZE * m_angleLoSize + a1/DEFAULT_BIN_SIZE;
	index.CompleteIndex = index.lowResolutionIndex * m_hiSize + index.highResolutionIndex;

	//std::cout << index.highResolutionIndex << " " << index.lowResolutionIndex << " " << index.CompleteIndex << std::endl;
	return index;
}

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

IndexStruct ParameterSpaceHierarchy::getIndex(int angle1, int angle2, int z)
{
	IndexStruct index;
	//std::cout << z << " "<< angle2 << " " << angle1 << std::endl;
	index.highResolutionIndex = z%DEFAULT_BIN_SIZE * m_hiSize2 + angle2%DEFAULT_BIN_SIZE * DEFAULT_BIN_SIZE + angle1%DEFAULT_BIN_SIZE;
	index.lowResolutionIndex = z/DEFAULT_BIN_SIZE * m_angleLoSize2 + angle2/DEFAULT_BIN_SIZE * m_angleLoSize + angle1/DEFAULT_BIN_SIZE;
	index.CompleteIndex = index.lowResolutionIndex * m_hiSize + index.highResolutionIndex;

	//std::cout << z/DEFAULT_BIN_SIZE << " "<< angle2/DEFAULT_BIN_SIZE << " " << angle1/DEFAULT_BIN_SIZE << std::endl;
	//std::cout << index.highResolutionIndex << " " << index.lowResolutionIndex << " " << index.CompleteIndex << std::endl;
	return index;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceHierarchy::getIndex(double angle1, double angle2, double z, int &angle1Index, int &angle2Index, int &shiftIndex)
{
	angle1Index = (angle1-m_anglemin) / m_angleStep;
	angle2Index = (angle2-m_anglemin) / m_angleStep;
    shiftIndex = (z-m_shiftmin) / m_shiftStep;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpaceHierarchy::findMaxima(std::vector<Plane<float> > &indices)
{
	float a, b, c;
	int maxind = -1;
	float max = -1;
	int angle1, angle2, shift;
	/////////////////////////////////////////////////
	ParameterSpaceHierarchyFullIterator it(this);
	while (not it.end)
	{
		double val = it.getVal();
		if (val > 1500)
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



////	for (int shift=around; shift < m_shiftSize-around; ++shift)
////	{
////		for (int angle2=around; angle2 < m_angleSize-around; ++angle2)
////		{
////			for (int angle1=around; angle1 < m_angleSize-around; ++angle1)
////			{
////				double val = this->operator ()(angle1, angle2, shift);
////				if (val > 1500)
////				if (val > this->operator()(angle1-around, angle2, shift) &&
////					val > this->operator()(angle1+around, angle2, shift) &&
////					val > this->operator()(angle1, angle2-around, shift) &&
////					val > this->operator()(angle1, angle2+around, shift) &&
////					val > this->operator()(angle1, angle2, shift+around) &&
////					val > this->operator()(angle1, angle2, shift-around))
////				{
////
////					toEuklid(getAngle(angle1), getAngle(angle2), a, b, c);
////
////
////					indices.push_back(Plane<float>(a, b, c, getShift(shift)));
////					if (val > max)
////					{
////						max = val;
////						maxind = indices.size() - 1;
////					}
////					//std::cout << angle1 << " " << angle2 << " " << shift << std::endl;
////
////				}
////			}
////		}
////	}
//	//std::cout << "=========" << std::endl;
//	for(int i = 0; i < indices.size(); ++i)
//	{
//		int a1, a2, s;
//		toAngles(indices[i].a,indices[i].b, indices[i].c, a, b);
//		getIndex((double)a, (double)b, (double)indices[i].d, a1, a2, s);
//		this->operator()(a1-1, a2, s) = 0;
//		this->operator()(a1+1, a2, s) = 0;
//		this->operator()(a1, a2-1, s) = 0;
//		this->operator()(a1, a2+1, s) = 0;
//		this->operator()(a1, a2, s+1) = 0;
//		this->operator()(a1, a2, s-1) = 0;
//		//std::cout << a1 << " " << a2 << " " << s << std::endl;
//	}
	return maxind;

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::getAngle(int index)
{
	return (m_angleStep * ((double)index)) + m_anglemin;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::getShift(int index)
{
	return ((double)index) * m_shiftStep + m_shiftmin;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpaceHierarchy::getSize()
{
	int size = 0;
	for (int i = 0; i < m_loSize; ++i)
		if (m_dataLowRes[i] != NULL) size += m_hiSize;
	return size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpaceHierarchy::get(double angle1, double angle2, double z)
{
	IndexStruct index = getIndex(angle1, angle2, z);

	if (m_dataLowRes[index.lowResolutionIndex] == NULL)
			return 0.0;
		else
			return m_dataLowRes[index.lowResolutionIndex][index.highResolutionIndex];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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


