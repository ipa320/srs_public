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

#ifndef IMarkersPlugin_H_included
#define IMarkersPlugin_H_included

#include <but_server/ServerTools.h>
#include <but_gui/Plane.h>
#include <but_gui/services_list.h>
#include <srs_env_model/RemovePrimitive.h>
#include <srs_env_model/AddPlanes.h>
#include <srs_env_model/AddPlane.h>
#include <srs_env_model_msgs/PlaneDesc.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

namespace srs
{

class CIMarkersPlugin : public CServerPluginBase
{
public:
	typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;

public:
	/// Constructor
	CIMarkersPlugin(const std::string & name);

	/// Destructor
	virtual ~CIMarkersPlugin();

	//! Initialize plugin - called in server constructor
	virtual void init(ros::NodeHandle & node_handle);


protected:
    /**
     * @brief Insert or modify plane array
     *
     * @param pa Array of planes
     */
    bool insertPlaneCallback( srs_env_model::AddPlanes::Request & req, srs_env_model::AddPlanes::Response & res );

    /**
     * @brief Insert/modify/remove plane
     *
     * @param plane Plane
     */
    void operatePlane( const srs_env_model_msgs::PlaneDesc & plane );

    /**
     * @brief Service helper - add plane
     *
     * @param plane Added plane
     */
    void addPlaneSrvCall( const srs_env_model_msgs::PlaneDesc & plane, const std::string & name );

    /**
     * @brief Service helper - remove plane
     *
     * @param plane Added plane
     */
    void removePlaneSrvCall( const srs_env_model_msgs::PlaneDesc & plane, const std::string & name );

    /**
     *  @brief Get unique string (used as interactive marker name)
     */
    std::string getUniqueName();


protected:
	/// Insert some planes service
    ros::ServiceServer m_serviceInsertPlanes;

    /// Remove object from the interactive markers server pointer
    ros::ServiceClient m_removeInteractiveMarkerService;

    /// Add plane interactive marker service
    ros::ServiceClient m_addInteractivePlaneService;

    //! Used frame id (input data will be transformed to it)
    std::string m_IMarkersFrameId;

    /// Interactive markers server pointer
    but_gui::InteractiveMarkerServerPtr m_imServer;

    // DETECTED ENTITIES
    /// Plane
    typedef std::pair< std::string, srs_env_model_msgs::PlaneDesc > tNamedPlane;
    typedef std::map< int, tNamedPlane > tPlanesMap;
    tPlanesMap m_dataPlanes;

    // Planes frame id
    std::string m_planesFrameId;

    /// Unique name counter
    long int m_uniqueNameCounter;


}; // class CIMarkersPlugin


} // namespace srs



 // namespace srs


// IMarkersPlugin_H_included
#endif

