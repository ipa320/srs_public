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
 * Date: 25/1/2012
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

//! Static member initialization
template <typename PointSource, typename PointTarget, typename Scalar>
const std::string srs_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::m_mode_names[4] = {"NONE", "ICP", "ICPNL", "SCA" };

//! Set used mode
template <typename PointSource, typename PointTarget, typename Scalar>
void srs_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::setMode( EPclRegistrationMode mode )
{
	m_mode = mode;

	switch( m_mode )
	{
	case PCL_REGISTRATION_MODE_NONE:
		m_registrationPtr = 0;
	case PCL_REGISTRATION_MODE_ICP:
		m_registrationPtr = & m_algIcp;
	case PCL_REGISTRATION_MODE_ICPNL:
		m_registrationPtr = & m_algIcpNl;
	case PCL_REGISTRATION_MODE_SCA:
		m_registrationPtr = & m_algSCA;
	default:
		m_registrationPtr = 0;
	}
}

//! Process data
//! @param source Source point cloud - this should be aligned to the target cloud
//! @param target Target point cloud - to this cloud should be source cloud aligned
//! @param output Output point cloud
template <typename PointSource, typename PointTarget, typename Scalar>
bool srs_env_model::CPclRegistration<PointSource, PointTarget, Scalar>::process( PointSourcePtr & source, PointTargetPtr & target, PointSourcePtr & output )
{
	if( m_registrationPtr == 0 )
		return false;

	m_registrationPtr->setInputCloud(source);
	m_registrationPtr->setInputTarget(target);
	m_registrationPtr->align( *output );

	return m_registrationPtr->hasConverged();
}
