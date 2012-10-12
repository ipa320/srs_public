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

#pragma once
#ifndef pcl_registration_module_H_included
#define pcl_registration_module_H_included

#include <pcl/registration/icp.h>
#

namespace srs_env_model
{

template <typename PointSource, typename PointTarget, typename Scalar = float>
class CPclRegistration
{
public:
	enum EMode
	{
		MODE_NONE,
		MODE_ICP
	};

	//! Set used mode
	void setMode( EMode mode ){ m_mode = mode; }

	//! Process data
	bool process( const PointSource & src, const PointTarget & target, PointSource & dst )
	{
		switch( m_mode )
		{
		case MODE_NONE:
			return false;

		case MODE_ICP:
			m_algIcp.setInputCloud( src );
			m_algIcp.setInputTarget( target );
			m_algIcp.align( dst );

			return m_algIcp.hasConverged();
		}
	}

protected:
	//! ICP algorithm
	pcl::IterativeClosestPoint< PointSource, PointTarget, Scalar > m_algIcp;
};

} // namespace srs_env_model

#endif //  pcl_registration_module_H_included

