/**
 * $Id: parameterSpace.cpp 134 2012-01-12 13:52:36Z spanel $
 *
 * $Id: parameterSpace.cpp 134 2012-01-12 13:52:36Z spanel $
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: dd.mm.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *
 */

#include "plane_det/parameterSpace.h"

using namespace but_scenemodel;
using namespace std;
using namespace sensor_msgs;
using namespace cv;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpace::~ParameterSpace()
{
	delete m_data;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::toAngles(float x, float y, float z, float &a1, float &a2)
{
	a1 = atan2(z, x);
	// align with X on XZ
	x = z * sin(a1) + x * cos(a1);
	a2 = atan2(y, x);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::toEuklid(float a1, float a2, float &x, float &y, float &z)
{
	float auxx = cos(-a2);
	y = -sin(-a2);
	x = auxx * cos(-a1);
	z = -auxx * sin(-a1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double &ParameterSpace::operator() (int angle1, int angle2, int z)
{
	assert( angle1 < m_angleSize && angle1 >= 0 && angle2 < m_angleSize && angle2 >= 0 && z < m_shiftSize && z >= 0);
	return m_data[z * m_angleSize2 + angle2 * m_angleSize + angle1];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double &ParameterSpace::operator[](int index)
{
	return m_data[index];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpace::getIndex(double angle1, double angle2, double z, int &angle1Index, int &angle2Index, int &shiftIndex)
{
	assert( angle1 <= m_anglemax && angle1 >= m_anglemin &&
			angle2 <= m_anglemax && angle2 >= m_anglemin &&
			z >= m_shiftmin && z <= m_shiftmax);
	angle1Index = (angle1-m_anglemin) / m_angleStep;
	angle2Index = (angle2-m_anglemin) / m_angleStep;
    shiftIndex = (z-m_shiftmin) / m_shiftStep;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpace::findMaxima(std::vector<Plane<float> > &indices)
{
	int around = 1;
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
				if (val > this->operator()(angle1-around, angle2, shift) &&
					val > this->operator()(angle1+around, angle2, shift) &&
					val > this->operator()(angle1, angle2-around, shift) &&
					val > this->operator()(angle1, angle2+around, shift) &&
					val > this->operator()(angle1, angle2, shift+around) &&
					val > this->operator()(angle1, angle2, shift-around) &&
					val > 500)
				{

					toEuklid(getAngle(angle1), getAngle(angle2), a, b, c);


					indices.push_back(Plane<float>(a, b, c, getShift(shift)));
					if (val > max)
					{
						max = val;
						maxind = indices.size() - 1;
					}
					//std::cout << angle1 << " " << angle2 << " " << shift << std::endl;

				}
			}
		}
	}
	//std::cout << "=========" << std::endl;
	for(int i = 0; i < indices.size(); ++i)
	{
		int a1, a2, s;
		toAngles(indices[i].a,indices[i].b, indices[i].c, a, b);
		getIndex((double)a, (double)b, (double)indices[i].d, a1, a2, s);
		this->operator()(a1-around, a2, s) = 0;
		this->operator()(a1+around, a2, s) = 0;
		this->operator()(a1, a2-around, s) = 0;
		this->operator()(a1, a2+around, s) = 0;
		this->operator()(a1, a2, s+around) = 0;
		this->operator()(a1, a2, s-around) = 0;
		//std::cout << a1 << " " << a2 << " " << s << std::endl;
	}
	return maxind;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpace::getAngle(int index)
{
	return (m_angleStep * ((double)index)) + m_anglemin;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double ParameterSpace::getShift(int index)
{
	return ((double)index) * m_shiftStep + m_shiftmin;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double &ParameterSpace::operator() (double angle1, double angle2, double z)
{
	return m_data[getIndex(angle1, angle2, z)];
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int ParameterSpace::getSize()
{
	return m_angleSize * m_angleSize * m_shiftSize;
}

