/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Jan Gorig (xgorig01@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 12/04/2012
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

#include <objtree/filter.h>

namespace objtree
{

FilterBox::FilterBox(const Box &box) :
    m_box(box)
{
}

bool FilterBox::filter(const Box &dim) const
{
    if(dim.x > m_box.x+m_box.w || dim.x+dim.w < m_box.x
            || dim.y > m_box.y+m_box.h || dim.y+dim.h < m_box.y
            || dim.z > m_box.z+m_box.d || dim.z+dim.d < m_box.z)
    {
        return false;
    }

    return true;
}

FilterPlane::FilterPlane(float posX, float posY, float posZ, float vecX, float vecY, float vecZ)
{
    m_posX = posX;
    m_posY = posY;
    m_posZ = posZ;

    m_vecX = vecX;
    m_vecY = vecY;
    m_vecZ = vecZ;
}

bool FilterPlane::filter(const Box &dim) const
{
    float pointX = dim.x, pointY = dim.y, pointZ = dim.z;

    if(m_vecX >= 0.0f) pointX += dim.w;
    if(m_vecY >= 0.0f) pointY += dim.h;
    if(m_vecZ >= 0.0f) pointZ += dim.d;

    float d = m_vecX*m_posX+m_vecY*m_posY+m_vecZ*m_posZ;

    if(pointX*m_vecX+pointY*m_vecY+pointZ*m_vecZ > d)
    {
        return true;
    }

    return false;
}

bool FilterZero::filter(const Box &dim) const
{
    return true;
}

FilterSphere::FilterSphere(float x, float y, float z, float radius) :
    m_x(x), m_y(y), m_z(z), m_radiusSquare(radius*radius)
{
}

bool FilterSphere::filter(const Box &dim) const
{
    float nearX, nearY, nearZ;

    if(m_x < dim.x) nearX = dim.x;
    else if(m_x > dim.x+dim.w) nearX = dim.x+dim.w;
    else nearX = m_x;

    if(m_y < dim.y) nearY = dim.y;
    else if(m_y > dim.y+dim.h) nearY = dim.y+dim.h;
    else nearY = m_y;

    if(m_z < dim.z) nearZ = dim.z;
    else if(m_z > dim.z+dim.d) nearZ = dim.z+dim.d;
    else nearZ = m_z;

    float xDist = m_x-nearX;
    float yDist = m_y-nearY;
    float zDist = m_z-nearZ;

    if(xDist*xDist+yDist*yDist+zDist*zDist <= m_radiusSquare)
    {
        return true;
    }


    return false;
}

}
