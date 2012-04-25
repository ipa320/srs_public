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

#include <cstdio>
#include <objtree/octree.h>
#include <objtree/filter.h>
#include <objtree/node.h>
#include <objtree/bbox.h>

namespace objtree
{

const Box Octree::ROOT_NODE = Box(0.0f, 0.0f, 0.0f, 16.0f, 16.0f, 16.0f);

Octree::Octree()
{
    m_root = new Node;
    m_maxId = 0;
}

Octree::~Octree()
{
    delete m_root;
}

void Octree::clear()
{
    delete m_root;
    m_objects.clear();

    m_root = new Node;
    m_maxId = 0;
}

unsigned int Octree::insertOnFit(Object* object)
{
    Node *node = m_root;

    Box box(ROOT_NODE);
    Box childBox;

    bool fits = true;
    unsigned char i;

    while(fits)
    {
        for(i = 0; !(fits = object->fitsIntoBox(Node::getChildBox(i, childBox, box))) && i < Node::CHILDREN; i++);

        if(fits)
        {
            node = node->child(i, true);
            box = childBox;
        }
    }

    node->add(object);
    m_objects[m_maxId] = object;
    object->setId(m_maxId++);

    return object->id();
}

unsigned int Octree::insertOnInterfere(Object* object, Node *node, Box box, unsigned int depth)
{
    Box childBox;

    for(unsigned char i = 0; i < Node::CHILDREN; i++)
    {
        if(object->interfereWithBox(Node::getChildBox(i, childBox, box)))
        {
            if(depth < MAX_DEPTH)
            {
                insertOnInterfere(object, node->child(i, true), childBox, depth+1);
            }
            else
            {
                //Hledani a mazani podobneho objektu
                //Zamyslet se nad odstranovanim vice podobnych objektu
                //Nebude to ale asi nutne
                //TODO: Zoptimalizovat a dat na jine lepsi misto
                Object *similar = NULL;

                Node *child = node->child(i, true);

                for(std::list<Object*>::const_iterator j = child->objects().begin(); j != child->objects().end() && !similar; j++)
                {
                    if(*j != object && (*j)->isSimilar(object))
                    {
                        similar = *j;
                    }
                }

                for(unsigned int n = 0; n < Node::NEIGHBORS && !similar; n++)
                {
                    Node *neighbor = child->neighbor(n);
                    if(neighbor == NULL)
                    {
                        continue;
                    }

                    for(std::list<Object*>::const_iterator j = neighbor->objects().begin(); j != neighbor->objects().end() && !similar; j++)
                    {
                        if(*j != object && (*j)->isSimilar(object))
                        {
                            similar = *j;
                        }
                    }
                }

                child->add(object);

                if(similar != NULL)
                {
                    if(!object->hasId())
                    {
                        m_objects[similar->id()] = object;
                        object->setId(similar->id());
                    }
                    //Nalezen jiny podobny objekt, pak ho musime smazat ze seznamu objektu
                    else
                    {
                        m_objects.erase(similar->id());
                    }

                    delete similar;
                }
                else
                {
                    if(!object->hasId())
                    {
                        m_objects[m_maxId] = object;
                        object->setId(m_maxId++);
                    }
                }
            }
        }
    }

    return object->id();
}

unsigned int Octree::insert(Object* object)
{
    return insertOnInterfere(object, m_root, ROOT_NODE);
}

bool Octree::isPositionFree(float x, float y, float z)
{
    Node *node = m_root;

    Box box(ROOT_NODE);

    for(;;)
    {
        unsigned char id = 0;

        if(x >= box.x+box.w/2.0f) id += 1;
        if(y >= box.y+box.h/2.0f) id += 2;
        if(z >= box.z+box.d/2.0f) id += 4;

        node = node->child(id);

        if(node == NULL)
        {
            return true;
        }

        for(std::list<Object*>::const_iterator i = node->objects().begin(); i != node->objects().end(); i++)
        {
            if((*i)->isPointInside(x, y, z))
            {
                return false;
            }
        }

        Node::getChildBox(id, box, box);
    }
}

inline void Octree::saveObjects(const Node *node, std::set<Object*> &objectList)
{
    for(std::list<Object*>::const_iterator i = node->objects().begin(); i != node->objects().end(); i++)
    {
        objectList.insert(*i);
    }
}

void Octree::nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter)
{
    nodes(nodesList, objectList, filter, ROOT_NODE, m_root);
}

void Octree::nodes(std::list<Box> &nodesList, std::set<Object*> &objectList, const Filter *filter, Box dim, Node *node)
{
    if(!filter->filter(dim)) return;

    saveObjects(node, objectList);
    nodesList.push_back(dim);

    for(unsigned int i = 0; i < Node::CHILDREN; i++)
    {
        Node *child = node->child(i);

        if(child)
        {
            Box newDim;
            Node::getChildBox(i, newDim, dim);
            nodes(nodesList, objectList, filter, newDim, child);
        }
    }
}

void Octree::objects(std::set<Object*> &objectList, const Filter *filter)
{
    std::list<Box> nodesList;
    nodes(nodesList, objectList, filter, ROOT_NODE, m_root);
}

const Object* Octree::object(unsigned int id) const
{
    std::map<unsigned int, Object*>::const_iterator i = m_objects.find(id);

    if(i != m_objects.end()) return i->second;
    else return NULL;
}

bool Octree::removeObject(unsigned int id)
{
    std::map<unsigned int, Object*>::iterator i = m_objects.find(id);

    if(i != m_objects.end())
    {
        delete i->second;
        m_objects.erase(i);

        return true;
    }
    else
    {
        return false;
    }
}

Node* Octree::root() const
{
    return m_root;
}

unsigned int Octree::maxId() const
{
    return m_maxId;
}

unsigned int Octree::count() const
{
    return m_objects.size();
}

}
