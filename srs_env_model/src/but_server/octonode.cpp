#include <but_server/octonode.h>

/**
 * Constructor
 */
octomap::EModelTreeNode::EModelTreeNode()
: OcTreeNode()
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
