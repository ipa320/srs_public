/**
 * $Id$
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: 27.01.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
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
