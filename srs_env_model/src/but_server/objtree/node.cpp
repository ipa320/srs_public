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
#include <objtree/node.h>

namespace objtree
{

Node::Node(unsigned char place, Node *parent)
{
    m_place = place;
    m_parent = parent;

    if(m_parent != NULL)
    {
        for(unsigned char neighbor = 0; neighbor < NEIGHBORS; neighbor++)
        {
            m_neighbors[neighbor] = computeNeighbor(neighbor);

            if(m_neighbors[neighbor] != NULL)
            {
                m_neighbors[neighbor]->m_neighbors[reverseNeighborId(neighbor)] = this;
            }
        }
    }
    else
    {
        //Root node hasn't got any neighbors
        for(unsigned char neighbor = 0; neighbor < NEIGHBORS; neighbor++)
        {
            m_neighbors[neighbor] = NULL;
        }
    }

    for(unsigned char i = 0; i < CHILDREN; i++)
    {
        m_children[i] = NULL;
    }
}

Node::~Node()
{
    //Nullify all neighbor pointers to this node
    for(unsigned char neighbor = 0; neighbor < NEIGHBORS; neighbor++)
    {
        if(m_neighbors[neighbor] != NULL)
        {
            m_neighbors[neighbor]->m_neighbors[reverseNeighborId(neighbor)] = NULL;
        }
    }

    //Delete all children
    for(unsigned char place = 0; place < CHILDREN; place++)
    {
        if(m_children[place]) delete m_children[place];
    }

    //Delete all objects
    for(std::list<Object*>::iterator i = m_objects.begin(); i != m_objects.end(); i++)
    {
        (*i)->removeNode(this);
        delete *i;
    }
}

Node* Node::parent()
{
    return m_parent;
}

Node* Node::child(unsigned char place, bool createNew)
{
    if(!m_children[place] && createNew)
    {
        m_children[place] = new Node(place, this);
    }

    return m_children[place];
}

Node* Node::neighbor(unsigned char dir)
{
    return m_neighbors[dir];
}

const std::list<Object*>& Node::objects() const
{
    return m_objects;
}

void Node::add(Object* object)
{
    m_objects.push_back(object);
    object->newNode(this);
}

/*Box Node::box() const
{
    if(m_parent == NULL)
    {
        return ROOT_NODE;
    }
    else
    {
        Box myBox;
        getChildBox(m_place, myBox, m_parent->box());
        return myBox;
    }
}*/

Box& Node::getChildBox(unsigned char place, Box &childBox, const Box &parentBox)
{
    childBox = parentBox;

    childBox.w/=2.0f;
    childBox.h/=2.0f;
    childBox.d/=2.0f;

    switch(place)
    {
        case 0:
            break;
        case 1:
            childBox.x+=childBox.w;
            break;
        case 2:
            childBox.y+=childBox.h;
            break;
        case 3:
            childBox.x+=childBox.w;
            childBox.y+=childBox.h;
            break;
        case 4:
            childBox.z+=childBox.d;
            break;
        case 5:
            childBox.x+=childBox.w;
            childBox.z+=childBox.d;
            break;
        case 6:
            childBox.y+=childBox.h;
            childBox.z+=childBox.d;
            break;
        case 7:
            childBox.x+=childBox.w;
            childBox.y+=childBox.h;
            childBox.z+=childBox.d;
            break;
    }

    return childBox;
}

Node* Node::parentNeighborChild(unsigned char parentNeighbor, unsigned char child)
{
    if(m_parent->m_neighbors[parentNeighbor] != NULL)
    {
        return m_parent->m_neighbors[parentNeighbor]->m_children[child];
    }

    return NULL;
}

/*
 * Neighbors ids (from top view)
 * Top part:  Middle part:  Bottom part:
 *  6  7  8     14 15 16      23 24 25
 *  3  4  5     12    13      20 21 22
 *  0  1  2      9 10 11      17 18 19
 *
 * Node children ids (from top view)
 * Top part:  Bottom part:
 *   2  3         6  7
 *   0  1         4  5
 *
 * Temporary ids (from top view)
 * 12 13 14 15  28 29 30 31  44 45 46 47  60 61 62 63
 *  8  9 10 11  24 25 26 27  40 41 42 43  56 57 58 59
 *  4  5  6  7  20 21 22 23  36 37 38 39  52 53 54 55
 *  0  1  2  3  16 17 18 19  32 33 34 35  48 49 50 51
 */
Node* Node::computeNeighbor(unsigned char dir)
{
    unsigned char id = 0;

    switch(m_place)
    {
    case 0: id = 21; break;
    case 1: id = 22; break;
    case 2: id = 25; break;
    case 3: id = 26; break;
    case 4: id = 37; break;
    case 5: id = 38; break;
    case 6: id = 41; break;
    case 7: id = 42; break;
    }

    if(dir <=  8) id -= 16;
    if(dir >= 17) id += 16;

    if(dir == 0 || dir == 3 || dir == 6 || dir ==  9 || dir == 12 || dir == 14 || dir == 17 || dir == 20 || dir == 23) id--;
    if(dir == 2 || dir == 5 || dir == 8 || dir == 11 || dir == 13 || dir == 16 || dir == 19 || dir == 22 || dir == 25) id++;

    if(dir == 0 || dir == 1 || dir == 2 || dir ==  9 || dir == 10 || dir == 11 || dir == 17 || dir == 18 || dir == 19) id-=4;
    if(dir == 6 || dir == 7 || dir == 8 || dir == 14 || dir == 15 || dir == 16 || dir == 23 || dir == 24 || dir == 25) id+=4;

    switch(id)
    {
    case  0: return parentNeighborChild(0, 7);
    case  1: return parentNeighborChild(1, 6);
    case  2: return parentNeighborChild(1, 7);
    case  3: return parentNeighborChild(2, 6);
    case  4: return parentNeighborChild(3, 5);
    case  5: return parentNeighborChild(4, 4);
    case  6: return parentNeighborChild(4, 5);
    case  7: return parentNeighborChild(5, 4);
    case  8: return parentNeighborChild(3, 7);
    case  9: return parentNeighborChild(4, 6);
    case 10: return parentNeighborChild(4, 7);
    case 11: return parentNeighborChild(5, 6);
    case 12: return parentNeighborChild(6, 5);
    case 13: return parentNeighborChild(7, 4);
    case 14: return parentNeighborChild(7, 5);
    case 15: return parentNeighborChild(8, 4);

    case 16: return parentNeighborChild(9, 3);
    case 17: return parentNeighborChild(10, 2);
    case 18: return parentNeighborChild(10, 3);
    case 19: return parentNeighborChild(11, 2);
    case 20: return parentNeighborChild(12, 1);
    case 21: return m_parent->m_children[0];
    case 22: return m_parent->m_children[1];
    case 23: return parentNeighborChild(13, 0);
    case 24: return parentNeighborChild(12, 3);
    case 25: return m_parent->m_children[2];
    case 26: return m_parent->m_children[3];
    case 27: return parentNeighborChild(13, 2);
    case 28: return parentNeighborChild(14, 1);
    case 29: return parentNeighborChild(15, 0);
    case 30: return parentNeighborChild(15, 1);
    case 31: return parentNeighborChild(16, 0);

    case 32: return parentNeighborChild(9, 7);
    case 33: return parentNeighborChild(10, 6);
    case 34: return parentNeighborChild(10, 7);
    case 35: return parentNeighborChild(11, 6);
    case 36: return parentNeighborChild(12, 5);
    case 37: return m_parent->m_children[4];
    case 38: return m_parent->m_children[5];
    case 39: return parentNeighborChild(13, 4);
    case 40: return parentNeighborChild(12, 7);
    case 41: return m_parent->m_children[6];
    case 42: return m_parent->m_children[7];
    case 43: return parentNeighborChild(13, 6);
    case 44: return parentNeighborChild(14, 5);
    case 45: return parentNeighborChild(15, 4);
    case 46: return parentNeighborChild(15, 5);
    case 47: return parentNeighborChild(16, 4);

    case 48: return parentNeighborChild(17, 3);
    case 49: return parentNeighborChild(18, 2);
    case 50: return parentNeighborChild(18, 3);
    case 51: return parentNeighborChild(19, 2);
    case 52: return parentNeighborChild(20, 1);
    case 53: return parentNeighborChild(21, 0);
    case 54: return parentNeighborChild(21, 1);
    case 55: return parentNeighborChild(22, 0);
    case 56: return parentNeighborChild(20, 3);
    case 57: return parentNeighborChild(21, 2);
    case 58: return parentNeighborChild(21, 3);
    case 59: return parentNeighborChild(22, 2);
    case 60: return parentNeighborChild(23, 1);
    case 61: return parentNeighborChild(24, 0);
    case 62: return parentNeighborChild(24, 1);
    case 63: return parentNeighborChild(25, 0);
    }

    return NULL;
}

unsigned char Node::reverseNeighborId(unsigned char dir)
{
    return 25-dir;
}

void Node::removeObject(Object *object)
{
    m_objects.remove(object);

    if(m_objects.size() == 0)
    {
        m_parent->m_children[m_place] = NULL;
        m_parent->deleteIfEmpty();
        delete this;
    }
}

void Node::deleteIfEmpty()
{
    //We don't want to delete root node
    if(m_parent == NULL)
    {
        return;
    }

    for(unsigned char child = 0; child < CHILDREN; child++)
    {
        if(m_children[child] != NULL)
        {
            return;
        }
    }

    if(m_objects.size() != 0)
    {
        return;
    }

    m_parent->m_children[m_place] = NULL;
    m_parent->deleteIfEmpty();
    delete this;
}

}
