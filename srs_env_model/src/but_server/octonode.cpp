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

#include <but_server/octonode.h>

/**
 * Constructor
 */
octomap::EModelTreeNode::EModelTreeNode()
: OcTreeNode()
, m_r( 255 )
, m_g( 5 )
, m_b( 255 )
, m_a( 255 )
{

}

/**
 * Destructor
 */
octomap::EModelTreeNode::~EModelTreeNode()
{

}

/**
 * Create child
 */
bool octomap::EModelTreeNode::createChild( unsigned int i )
{
	if (itsChildren == NULL) {
		allocChildren();
	}
	itsChildren[i] = new EModelTreeNode();
	return true;
}

/**
 * Creates a new (empty) OcTree of a given resolution
 * @param _resolution
 */
octomap::EMOcTree::EMOcTree(double _resolution)
: OccupancyOcTreeBase<octomap::EModelTreeNode> (_resolution)
  {
	itsRoot = new EModelTreeNode();
	tree_size++;
  }

/**
 * Reads an OcTree from a binary file
 * @param _filename
 *
 */
octomap::EMOcTree::EMOcTree(std::string _filename)
: OccupancyOcTreeBase<octomap::EModelTreeNode> (0.1)  { // resolution will be set according to tree file
    itsRoot = new EModelTreeNode();
    tree_size++;

    readBinary(_filename);
  }
