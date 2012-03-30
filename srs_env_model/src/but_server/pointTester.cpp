/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: dd/mm/2012
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

#include <but_server/pointTester.h>
#include <BulletCollision/Gimpact/btClipPolygon.h>

/**
 * Create point tester
 *
 * @param point Plane point
 * @param normal Plane normal vector
 * @return
 */
srs::CInFrontOfPlane::CInFrontOfPlane( const btVector3 & point, const btVector3 & normal )
{
	setPlane( point, normal );
}

/**
 * Update plane parameters
 *
 * @param point Plane point
 * @param normal Plane normal vector
 */
void srs::CInFrontOfPlane::setPlane(const btVector3 & point, const btVector3 & normal)
{
	// Compute normalized normal vector
	btVector3 n( normal.normalized() );

	// Set coefficients
	m_plane.setValue( n.x(), n.y(), n.z(), -n.dot( point ) );
}

/**
 *
 * @param point Tested point
 * @return
 */
bool srs::CInFrontOfPlane::operator()( const btVector3 & point )
{
	return bt_distance_point_plane( m_plane, point );
}
