/******************************************************************************
 * \file
 *
 * $Id: octonode.h 577 2012-04-12 12:38:50Z stancl $
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

#ifndef OCTONODE_H
#define OCTONODE_H

#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTreeStamped.h>

namespace octomap
{

/**
 * Nodes to be used in a environment model server.
 *
 */
class EModelTreeNode : public OcTreeNodeStamped {

public:
	/**
	 * Constructor
	 */
	EModelTreeNode();

	/**
	 * Destructor
	 */
	~EModelTreeNode();

	bool createChild(unsigned int i);

	// overloaded, so that the return type is correct:
	inline EModelTreeNode* getChild(unsigned int i) {
		return static_cast<EModelTreeNode*> (OcTreeDataNode<float>::getChild(i));
	}
	inline const EModelTreeNode* getChild(unsigned int i) const {
		return static_cast<const EModelTreeNode*> (OcTreeDataNode<float>::getChild(i));
	}

	//! Get color components
	unsigned char r() const { return m_r; }
	unsigned char g() const { return m_g; }
	unsigned char b() const { return m_b; }
	unsigned char a() const { return m_a; }

	//! Get color components - reference version
	unsigned char & r() { return m_r; }
	unsigned char & g() { return m_g; }
	unsigned char & b() { return m_b; }
	unsigned char & a() { return m_a; }

	//! Set color components
	void setColor( unsigned char r, unsigned char g, unsigned char b, unsigned char a = 255 )
	{ m_r = r; m_g = g; m_b = b; m_a = a; }

protected:
	//! Color data
	unsigned char m_r, m_g, m_b, m_a;

}; // class EModelTreeNode

/**
 * octomap main map data structure, stores 3D occupancy grid map in an OcTree.
 * Basic functionality is implemented in OcTreeBase.
 *
 */
class EMOcTree : public OccupancyOcTreeBase <EModelTreeNode> {

public:
	//! Used point type
	typedef point3d tPoint;

	//! Used node type
	typedef EModelTreeNode tNode;

public:

	/**
	 * Creates a new (empty) OcTree of a given resolution
	 * @param _resolution
	 */
	EMOcTree(double _resolution);

	/**
	 * Reads an OcTree from a binary file
	 * @param _filename
	 *
	 */
	EMOcTree(std::string _filename);

	/**
	 * Destructor
	 */
	virtual ~EMOcTree(){};

	/// virtual constructor: creates a new object of same type
	/// (Covariant return type requires an up-to-date compiler)
	EMOcTree* create() const {return new EMOcTree(resolution); }

	/**
	 *  Get tree type as a string.
	 */
	std::string getTreeType() const {return "EMOcTree";}

	    //! \return timestamp of last update
	    unsigned int getLastUpdateTime();

	    void degradeOutdatedNodes(unsigned int time_thres);

	    virtual void updateNodeLogOdds(EModelTreeNode* node, const float& update) const;
	    void integrateMissNoTime(EModelTreeNode* node) const;


protected:
	/**
	 * Static member object which ensures that this EMOcTree prototype
	 * ends up in the classIDMapping only once
	 */
	class StaticMemberInitializer{
	public:
		StaticMemberInitializer() {
			EMOcTree* tree = new EMOcTree(0.1);
			AbstractOcTree::registerTreeType(tree);
		}
	};

	/// to ensure static initialization (only once)
	static StaticMemberInitializer ocTreeMemberInit;
}; // class EMOcTree

}; // namespace octomap

#endif // OCTONODE_H

