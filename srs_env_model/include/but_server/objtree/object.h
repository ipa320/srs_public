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

#ifndef OBJECT_H
#define OBJECT_H

#include <list>
#include <objtree/box.h>

namespace objtree
{

class Node;

class Object
{
public:
    enum Type
    {
        BOUNDING_BOX = 1,
        PLANE = 2
    };

private:
    std::list<Node*> m_inNodes;
    unsigned int m_id;

protected:
    Type m_type;

public:
    Object();
    ~Object();

    virtual bool fitsIntoBox(const Box &box) const = 0;
    virtual bool interfereWithBox(const Box &box) const = 0;
    virtual bool isSimilar(const Object *object) const = 0;
    virtual bool isPointInside(float x, float y, float z) const = 0;

    void setId(unsigned int id);
    unsigned int id() const;
    bool hasId() const;
    Type type() const;

    void newNode(Node *node);
    void removeNode(Node *node);
};

}

#endif // OBJECT_H
