/**
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Date: 05.03.2012
 *
 * License: BUT OPEN SOURCE LICENSE
 *
 */

#ifndef SERVICES_LIST_H_
#define SERVICES_LIST_H_

#include <srs_ui_but/GetClosestPoint.h>

#define BUT_SERVICES_PREFIX std::string("/but_services")
#define BUT_SERVICES_SERVICE_TOPIC(topic) BUT_SERVICES_PREFIX + std::string(topic)

/*
 * Get closest point service topic
 */
#define BUT_GetClosestPoint_SRV BUT_SERVICES_SERVICE_TOPIC("/get_closest_point")

#endif /* SERVICES_LIST_H_ */
