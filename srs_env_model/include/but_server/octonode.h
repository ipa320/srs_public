/**
 * $Id: octonode.h 236 2012-02-20 21:10:26Z stancl $
 *
 * Developed by dcgm-robotics@FIT group
 * Author: Vit Stancl (stancl@fit.vutbr.cz)
 * Date: dd.mm.2011
 *
 */

#ifndef OCTONODE_H
#define OCTONODE_H

#include <octomap_ros/OctomapROS.h>

namespace octomap
{

/**
 * Nodes to be used in a environment model server.
 *
 */
class EModelTreeNode : public OcTreeNode {

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

	/**
	 * Traversing function - applies testing function on the node and if succeeds,
	 * uses apply function and recurses to the child nodes (if any).
	 *
	 */
	template< class fncTraverseDown, class fncApply >
	void traverseTree( fncTraverseDown testFunction, fncApply applyFunction, unsigned int max_depth )
	{
		// Does root exist?
		assert(itsRoot);

		// A tree with only the root node is an empty tree (by definition)
		if(tree_size <= 1)
		{
			std::cerr << "tree size <= 1 " << std::endl;
				return;
		}

		// Start recursion
		if( max_depth != 0 )
		{
//			std::cerr << "Traverse only to the depth: " << max_depth << std::endl;
			traverseTreeRec< fncTraverseDown, fncApply >( testFunction, applyFunction, 0, max_depth, tree_center, itsRoot );
		}
		else
		{
//			std::cerr << "No depth tests... " << std::endl;
			traverseTreeRecNDT< fncTraverseDown, fncApply >( testFunction, applyFunction, 0, max_depth, tree_center, itsRoot );
		}

	}

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

	/**
	 * Traversing function - recursive part.
	 *
	 */
	template< class fncTraverseDown, class fncApply >
	void traverseTreeRec( 	fncTraverseDown testFunction, fncApply applyFunction,
							unsigned int depth, unsigned int max_depth,
							const point3d& parent_center, tNode * node );

	/**
	 * Traversing function - recursive part, no depth testing.
	 *
	 */
	template< class fncTraverseDown, class fncApply >
	void traverseTreeRecNDT( 	fncTraverseDown testFunction, fncApply applyFunction,
			unsigned int depth, unsigned int max_depth,
			const point3d& parent_center, tNode * node );

	/// to ensure static initialization (only once)
	static StaticMemberInitializer ocTreeMemberInit;
}; // class EMOcTree

/**
 * Recursive node testing traversal function definition.
 */
template< class fncTraverseDown, class fncApply >
void EMOcTree::traverseTreeRec( fncTraverseDown testFunction,
								 fncApply applyFunction,
								 unsigned int depth,
								 unsigned int max_depth,
								 const point3d& parent_center,
								 tNode * node )
{
	std::cerr << "Starting traversal..." << std::endl;
	// Test recursion depth
	if ((depth <= max_depth) && (node != NULL) )
	{
//		std::cerr << "Step1" << std::endl;

		// Compute this voxel size
		double voxelSize = resolution * pow(2., double(tree_depth - depth));

		// Test node...
//		std::cerr << "Calling tester. Voxel size: " << voxelSize << std::endl;

		if( ! testFunction( node, parent_center, voxelSize ) )
		{
//			std::cerr << "Discard this node..." << std::endl;

			return;
		}

//		std::cerr << "Step2" << std::endl;

		// Call apply function
		applyFunction( this, node, parent_center, voxelSize );

		// Try to recurse to the children
		if (node->hasChildren() && (depth != max_depth)) {

			// Compute center offset
			double center_offset = tree_center(0) / pow( 2., (double) depth+1);
			point3d search_center;
			std::cerr << "Step3" << std::endl;

			for (unsigned int i=0; i<8; i++) {
				if (node->childExists(i)) {

//					std::cerr << "Step4" << std::endl;

					// Compute child center
					computeChildCenter(i, center_offset, parent_center, search_center);

					// cast needed: (nodes need to ensure it's the right pointer)
					tNode * childNode = static_cast<tNode*>(node->getChild(i));
					traverseTreeRec( testFunction, applyFunction, depth + 1, max_depth, search_center, childNode );

				} // GetChild
			}
		}
	}
}

/**
 * Recursive node testing traversal function definition.
 */
template< class fncTraverseDown, class fncApply >
void EMOcTree::traverseTreeRecNDT( fncTraverseDown testFunction,
								 fncApply applyFunction,
								 unsigned int depth,
								 unsigned int max_depth,
								 const point3d& parent_center,
								 tNode * node )
{
//	std::cerr << "Starting NDT traversal..." << std::endl;
	// Test recursion depth
	if ( (node != NULL) )
	{
//		std::cerr << "Step1" << std::endl;

		// Compute this voxel size
		double voxelSize = resolution * pow(2., double(tree_depth - depth));

		// Test node...
//		std::cerr << "Calling tester. Voxel size: " << voxelSize << std::endl;

		if( ! testFunction( node, parent_center, voxelSize ) )
		{
			std::cerr << "Discard this node..." << std::endl;

			return;
		}

//		std::cerr << "Step2" << std::endl;

		// Call apply function
		applyFunction( this, node, parent_center, voxelSize );

		// Try to recurse to the children
		if (node->hasChildren() ) {

			// Compute center offset
			double center_offset = tree_center(0) / pow( 2., (double) depth+1);
			point3d search_center;
//			std::cerr << "Step3" << std::endl;

			for (unsigned int i=0; i<8; i++) {
				if (node->childExists(i)) {

//					std::cerr << "Step4" << std::endl;

					// Compute child center
					computeChildCenter(i, center_offset, parent_center, search_center);

					// cast needed: (nodes need to ensure it's the right pointer)
					tNode * childNode = static_cast<tNode*>(node->getChild(i));
					traverseTreeRecNDT( testFunction, applyFunction, depth + 1, max_depth, search_center, childNode );

				} // GetChild
			}
		}
	}
}

}; // namespace octomap

#endif // OCTONODE_H

