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

#include <cstdio>
#include <cmath>
#include <objtree/plane.h>

namespace objtree
{

Plane::Plane(const Polygon &points) :
    m_points(points)
{
    m_type = PLANE;

    m_pos = points[0];

    Vector vec1(points[0]-points[1]);
    Vector vec2(points[0]-points[2]);

    m_normal.x = vec1.y*vec2.z-vec1.z*vec2.y;
    m_normal.y = vec1.z*vec2.x-vec1.x*vec2.z;
    m_normal.z = vec1.x*vec2.y-vec1.y*vec2.x;

    float vecSize = sqrt(m_normal.x*m_normal.x + m_normal.y*m_normal.y + m_normal.z*m_normal.z);
    m_normal.x /= vecSize;
    m_normal.y /= vecSize;
    m_normal.z /= vecSize;

    m_d = - m_normal.x*m_pos.x - m_normal.y*m_pos.y - m_normal.z*m_pos.z;

    m_boundingMin = m_boundingMax = points[0];

    for(Polygon::const_iterator i = points.begin(); i != points.end(); i++)
    {
        if(i->x < m_boundingMin.x) m_boundingMin.x = i->x;
        if(i->y < m_boundingMin.y) m_boundingMin.y = i->y;
        if(i->z < m_boundingMin.z) m_boundingMin.z = i->z;

        if(i->x > m_boundingMax.x) m_boundingMax.x = i->x;
        if(i->y > m_boundingMax.y) m_boundingMax.y = i->y;
        if(i->z > m_boundingMax.z) m_boundingMax.z = i->z;
    }
}

Plane::Plane(const Point &center, const Vector &normal, const Point &scale)
{
    m_type = PLANE;

    m_pos = center;
    m_normal = normal;

    float vecSize = sqrt(m_normal.x*m_normal.x + m_normal.y*m_normal.y + m_normal.z*m_normal.z);
    m_normal.x /= vecSize;
    m_normal.y /= vecSize;
    m_normal.z /= vecSize;

    m_d = - m_normal.x*m_pos.x - m_normal.y*m_pos.y - m_normal.z*m_pos.z;

    m_boundingMin = center - scale/2.0f;
    m_boundingMax = center + scale/2.0f;

    //TODO: points
}

bool Plane::fitsIntoBox(const Box &box) const
{
    return false;
}

bool Plane::interfereWithBox(const Box &box) const
{
    Point min(box.x, box.y, box.z);
    Point max(min);

    if(m_normal.x >= 0.0f) max.x += box.w;
    else min.x += box.w;
    if(m_normal.y >= 0.0f) max.y += box.h;
    else min.y += box.h;
    if(m_normal.z >= 0.0f) max.z += box.d;
    else min.z += box.d;

    if(min.x*m_normal.x+min.y*m_normal.y+min.z*m_normal.z > -m_d)
        return false;

    if(max.x*m_normal.x+max.y*m_normal.y+max.z*m_normal.z < -m_d)
        return false;

    if(m_boundingMin.x > box.x+box.w || m_boundingMin.y > box.y+box.h || m_boundingMin.z > box.z+box.d)
        return false;

    if(m_boundingMax.x < box.x || m_boundingMax.y < box.y || m_boundingMax.z < box.z)
        return false;

    return true;
}

bool Plane::isSimilar(const Object *object) const
{
    if(object->type() != PLANE) return false;

    Plane *plane = (Plane*)object;

    //Normaly jsou normalizovane - vyuzito v obou vypoctech

    //Uhel je vetsi nez 0.5 = false
    if(fabs(m_normal.x*plane->m_normal.x+m_normal.y*plane->m_normal.y+m_normal.z*plane->m_normal.z) > 0.5f) return false;

    //Vzdalenost bodu od roviny
    const Point &p = plane->m_pos;
    if(fabs(p.x*m_normal.x+p.y*m_normal.y+p.z*m_normal.z+m_d) > 0.5f) return false;

    return true;
}

bool Plane::isPointInside(float x, float y, float z) const
{
    return fabs(x*m_normal.x+y*m_normal.y+z*m_normal.z+m_d) < 0.1f;
}

}
