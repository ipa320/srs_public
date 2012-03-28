/**
 * $Id$
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: 06.02.2011
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#include <but_server/plugins/IMarkersPlugin.h>
#include <pcl_ros/transforms.h>


srs::CIMarkersPlugin::CIMarkersPlugin(const std::string & name)
: srs::CServerPluginBase(name)
, m_uniqueNameCounter(0)
{
}

srs::CIMarkersPlugin::~CIMarkersPlugin()
{
}

void srs::CIMarkersPlugin::init(ros::NodeHandle & node_handle)
{
	// Initialize interactive markers server
	m_imServer.reset(new InteractiveMarkerServer("BUT_IM_Server", "", false));

	// Connect to the services
	m_removeInteractiveMarkerService = node_handle.serviceClient<srs_env_model::RemovePrimitive> (BUT_RemovePrimitive_SRV);
	m_addInteractivePlaneService = node_handle.serviceClient<srs_env_model::AddPlane> (BUT_AddPlane_SRV);

	m_serviceInsertPlanes = node_handle.advertiseService("insert_plane", &srs::CIMarkersPlugin::insertPlaneCallback, this);

	// Interactive marker server test
		{
			// Creating Plane object with name "plane1"
			but_gui::Plane *plane = new but_gui::Plane(m_imServer, "plane1", "");

			// Color
			std_msgs::ColorRGBA c;
			c.r = 1.0;
			c.g = c.b = 0.0;
			c.a = 1.0;
			plane->setColor(c);

			// Positioning
			geometry_msgs::Pose p;
			p.position.x = p.position.y = p.position.z = 0.0;
			plane->setPose(p);

			// Scaling
			but_gui::Scale s;
			s.x = s.y = s.z = 10.0;
			plane->setScale(s);

			// Creating plane with specified attributes
			/* deprecated
			 * plane->create();
			 */

			// Inserting object into server
			plane->insert();
		}

}


///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Insert or modify plane array
 *
 * @param pa Array of planes
 */
bool srs::CIMarkersPlugin::insertPlaneCallback(srs_env_model::AddPlanes::Request & req,
		srs_env_model::AddPlanes::Response & res) {
	std::cerr << "Inset plane called" << std::endl;
	// Get plane array
	srs_env_model_msgs::PlaneArray & planea(req.plane_array);
	m_planesFrameId = planea.header.frame_id;
	std::vector<srs_env_model_msgs::PlaneDesc> & planes(planea.planes);
	std::vector<srs_env_model_msgs::PlaneDesc>::iterator i;

	for (i = planes.begin(); i != planes.end(); ++i) {
		operatePlane(*i);
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Insert/modify/remove plane
 *
 * @param plane Plane
 */
void srs::CIMarkersPlugin::operatePlane(const srs_env_model_msgs::PlaneDesc & plane) {

	std::string name;

	switch (plane.flags) {
	case srs_env_model_msgs::PlaneDesc::INSERT:
		name = getUniqueName();
		m_dataPlanes[plane.id] = tNamedPlane(name, plane);

		addPlaneSrvCall(plane, name);
		break;

	case srs_env_model_msgs::PlaneDesc::MODIFY:
		m_dataPlanes[plane.id].second = plane;
		name = m_dataPlanes[plane.id].first;

		// call remove plane, add plane
		removePlaneSrvCall(plane, name);
		addPlaneSrvCall(plane, name);
		break;

	case srs_env_model_msgs::PlaneDesc::REMOVE:
		name = m_dataPlanes[plane.id].first;

		m_dataPlanes.erase(plane.id);
		// call remove plane
		removePlaneSrvCall(plane, name);
		break;

	default:
		break;
	}

	std::cerr << "Current planes count: " << m_dataPlanes.size() << std::endl;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Service helper - add plane
 *
 * @param plane Added plane
 */
void srs::CIMarkersPlugin::addPlaneSrvCall(const srs_env_model_msgs::PlaneDesc & plane,
		const std::string & name) {
	// Create service
	srs_env_model::AddPlane addPlaneSrv;

	// Modify service
	addPlaneSrv.request.name = name;
	addPlaneSrv.request.frame_id = m_planesFrameId;
	addPlaneSrv.request.pose = plane.pose;
	addPlaneSrv.request.scale = plane.scale;
	addPlaneSrv.request.color.r = 1.0;
	addPlaneSrv.request.color.g = 0.0;
	addPlaneSrv.request.color.b = 0.0;
	addPlaneSrv.request.color.a = 0.8;

	m_addInteractivePlaneService.call(addPlaneSrv);

	std::cerr << "Adding plane: " << name << ", frame: " << m_planesFrameId
			<< ", pose: " << plane.pose << "scale: " << plane.scale
			<< std::endl;
}

///////////////////////////////////////////////////////////////////////////////

/**
 * @brief Service helper - remove plane
 *
 * @param plane Added plane
 */
void srs::CIMarkersPlugin::removePlaneSrvCall(const srs_env_model_msgs::PlaneDesc & plane,
		const std::string & name) {
	// Create service
	srs_env_model::RemovePrimitive removeObjectSrv;

	// Modify service
	removeObjectSrv.request.name = name;

	m_removeInteractiveMarkerService.call(removeObjectSrv);

	std::cerr << "Removing plane: " << name;
}

///////////////////////////////////////////////////////////////////////////////

/**
 *  @brief Get unique string (used as interactive marker name)
 */
std::string srs::CIMarkersPlugin::getUniqueName() {
	std::stringstream ss;
	ss << "imn" << ++m_uniqueNameCounter;
	return ss.str();
}
