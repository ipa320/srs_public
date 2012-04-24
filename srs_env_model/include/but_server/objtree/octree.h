/******************************************************************************
 * \file
 *
 * $Id:$
 *
 * Copyright (C) Brno University of Technology (BUT)
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Jan Gorig (xgorig01@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 12/04/2012
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

#ifndef OCTREE_H
#define OCTREE_H

#include <map>
#include <list>
#include <set>
#include <objtree/box.h>

namespace objtree
{

class Node;
class Object;
class Filter;

class Octree
{
public:
    static const unsigned int MAX_DEPTH = 4;
    static const Box ROOT_NODE;

private:
    Node *m_root;
    std::map<unsigned int, Object*> m_objects;
    unsigned int m_maxId;

    static void saveObjects(const Node *node, std::set<Object*> &objectList);

public:
    Octree();
    ~Octree();

    void clear();

    unsigned int insert(Object* object);
    unsigned int insertOnFit(Object* object);
    unsigned int insertOnInterfere(Object* object, Node *node, Box box, unsigned int depth = 0);

    bool isPositionFree(float x, float y, float z);

    Node* root() const;
    unsigned int maxId() const;
    unsigned int count() const;

    const Object* object(unsigned int id) const;
    //const Object* object(const std::string &name) const;

    bool removeObject(unsigned int id);
    //bool removeObject(const std::string &name);

    void nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter);
    void nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter, Box dim, Node *node);
    void objects(std::set<Object*> &objectList, const Filter *filter);
};

}

#endif // OCTREE_H
