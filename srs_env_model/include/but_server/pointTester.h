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

#ifndef POINT_TESTER_H_INCLUDED
#define POINT_TESTER_H_INCLUDED

#include "LinearMath/btVector3.h"


namespace srs
{

/**
 *  This functor test given point if it is in front of plane
 */
class CInFrontOfPlane
{
public:
	//! Constructor - get plane point and normal
	CInFrontOfPlane( const btVector3 & point, const btVector3 & normal );

	//! Update plane parameters
	void setPlane( const btVector3 & point, const btVector3 & normal );

	//! Is point in front of the plane?
	bool operator()( const btVector3 & point );

protected:
	//! Plane
	btVector4 m_plane;

}; // class CInFrontOfPlane

} // namespace srs

// POINT_TESTER_H_INCLUDED
#endif
