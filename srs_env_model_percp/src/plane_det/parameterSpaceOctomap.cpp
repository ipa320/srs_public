/**
 * $Id: ParameterSpaceOctomap.cpp 152 2012-01-13 12:49:57Z ihulik $
 *
 * $Id: ParameterSpaceOctomap.cpp 152 2012-01-13 12:49:57Z ihulik $
 * Developed by dcgm-robotics@FIT group
 * Author: Rostislav Hulik (ihulik@fit.vutbr.cz)
 * Date: dd.mm.2012 (version 1.0)
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 * Description:
 *
 */

#include "plane_det/parameterSpaceOctomap.h"

using namespace but_scenemodel;
using namespace std;
using namespace sensor_msgs;
using namespace cv;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpaceOctomap::ParameterSpaceOctomap(double resolution) : m_paramSpace(resolution)
{
	m_resolution = resolution;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ParameterSpaceOctomap::~ParameterSpaceOctomap()
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceOctomap::toAngles(float x, float y, float z, float &a1, float &a2)
{
	a1 = atan2(z, x);
	// align with X on XZ
	x = z * sin(a1) + x * cos(a1);
	a2 = atan2(y, x);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceOctomap::toEuklid(float a1, float a2, float &x, float &y, float &z)
{
	float auxx = cos(-a2);
	y = -sin(-a2);
	x = auxx * cos(-a1);
	z = -auxx * sin(-a1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float ParameterSpaceOctomap::get(float angle1, float angle2, float z)
{
	octomap::OcTreeNode *node = m_paramSpace.search(angle1, angle2, z);
	if (node == NULL)
		return 0.0;
	else
		return node->getValue();
}

void ParameterSpaceOctomap::set(float angle1, float angle2, float z, float val)
{
	m_paramSpace.updateNode(octomath::Vector3(angle1, angle2, z), true)->setValue(val);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ParameterSpaceOctomap::addVolume(ParameterSpace &second, float angle1, float angle2, float shift)
{
	assert (second.m_angleStep == m_resolution && second.m_shiftStep == m_resolution);

	angle1 -= second.m_angleStep * (second.m_angleSize / 2);
	angle2 -= second.m_angleStep * (second.m_angleSize / 2);
	shift -= second.m_shiftStep * (second.m_shiftSize / 2);

	for (int shiftS = 0;  shiftS < second.m_shiftSize;  ++shiftS, shift += m_resolution)
	for (int angle2S = 0; angle2S < second.m_angleSize; ++angle2S, angle2 += m_resolution)
	for (int angle1S = 0; angle1S < second.m_angleSize; ++angle1S, angle1 += m_resolution)
	{
		octomap::OcTreeNode *node = m_paramSpace.updateNode(octomath::Vector3(angle1, angle2, shift), true);
		node->setValue(node->getValue() + second(angle1S, angle2S, shiftS));
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int ParameterSpaceOctomap::findMaxima(std::vector<Plane<float> > &indices)
//{
//	int around = 2;
//	float a, b, c;
//	int maxind = -1;
//	float max = -1;
//
////	/////////////////////////////////////////////////
//	for (int shift=around; shift < m_shiftSize-around; ++shift)
//		{
//			for (int angle2=around; angle2 < m_angleSize-around; ++angle2)
//			{
//				for (int angle1=around; angle1 < m_angleSize-around; ++angle1)
//				{
//					double val = this->operator ()(angle1, angle2, shift);
//					if (val > 1500)
//					{
//						val += this->operator()(angle1-1, angle2, shift) +
//							   this->operator()(angle1+1, angle2, shift) +
//							   this->operator()(angle1, angle2-1, shift) +
//							   this->operator()(angle1, angle2+1, shift) +
//							   this->operator()(angle1, angle2, shift+1) +
//							   this->operator()(angle1, angle2, shift-1);
//
//						bool ok = true;
//						double aux;
//						int xx, yy, zz;
//						for (int x = -1; x <= 1; ++x)
//						for (int y = -1; y <= 1; ++y)
//						for (int z = -1; z <= 1; ++z)
//						{
//							xx = angle1 + x;
//							yy = angle2 + y;
//							zz = shift + z;
//							aux = this->operator()(xx, yy, zz) +
//								  this->operator()(xx-1, yy, zz) +
//								  this->operator()(xx+1, yy, zz) +
//								  this->operator()(xx, yy-1, zz) +
//								  this->operator()(xx, yy+1, zz) +
//								  this->operator()(xx, yy, zz+1) +
//								  this->operator()(xx, yy, zz-1);
//							if (val < aux)
//							{
//								ok = false;
//								break;
//							}
//						}
//
//						if (ok)
//						{
//							double aroundx = 0;
//							double aroundy = 0;
//							double aroundz = 0;
//							double arounds = 0;
//							for (int x = -1; x <= 1; ++x)
//							for (int y = -1; y <= 1; ++y)
//							for (int z = -1; z <= 1; ++z)
//							{
//								xx = angle1 + x;
//								yy = angle2 + y;
//								zz = shift + z;
//								toEuklid(getAngle(xx), getAngle(yy), a, b, c);
//								aroundx += a;
//								aroundy += b;
//								aroundz += c;
//								arounds += getShift(zz);
//							}
//							aroundx /= 7.0;
//							aroundy /= 7.0;
//							aroundz /= 7.0;
//							arounds /= 7.0;
//							std::cout << "Found plane size: " << val << " eq: " << aroundx <<" "<< aroundy <<" "<< aroundz <<" "<< arounds <<" "<< std::endl;
//							indices.push_back(Plane<float>(aroundx, aroundy, aroundz, arounds));
//							if (val > max)
//							{
//								max = val;
//								maxind = indices.size() - 1;
//							}
//						}
//					}
//				}
//			}
//		}
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
//	return maxind;
//}
//



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//int ParameterSpaceOctomap::getSize()
//{
//	return m_angleSize * m_angleSize * m_shiftSize;
//}

