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

#include <objtree/node.h>
#include <objtree/object.h>

namespace objtree
{

Object::Object()
{
    m_id = -1;
}

Object::~Object()
{
    for(std::list<Node*>::iterator i = m_inNodes.begin(); i != m_inNodes.end(); i++)
    {
        (*i)->removeObject(this);
    }
}

Object::Type Object::type() const
{
    return m_type;
}

void Object::setId(unsigned int id)
{
    m_id = id;
}

bool Object::hasId() const
{
    return m_id != (unsigned int)-1;
}

unsigned int Object::id() const
{
    return m_id;
}

void Object::newNode(Node *node)
{
    m_inNodes.push_back(node);
}

void Object::removeNode(Node *node)
{
    m_inNodes.remove(node);
}

}
