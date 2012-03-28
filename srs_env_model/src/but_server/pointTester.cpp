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
