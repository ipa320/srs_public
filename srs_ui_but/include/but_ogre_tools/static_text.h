/******************************************************************************
 * \file
 *
 * $Id: static_text.h 555 2012-04-11 14:32:26Z xlokaj03 $
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 28/03/2012
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

#include <ogre_tools/movable_text.h>

#ifndef STATIC_TEXT_H_
#define STATIC_TEXT_H_

namespace ogre_tools
{
/**
 * @brief This class overrides MovableText class in order to show non-movable
 * static text in the scene.
 *
 * @author Tomas Lokaj
 */
class StaticText : public MovableText
{
public:
  /**
   * @brief Constructor.
   * @param caption text
   * @param fontName name of the font family
   * @param charHeight height of the characters
   * @param color color of the text
   */
  StaticText(const Ogre::String &caption, const Ogre::String &fontName = "Arial", Ogre::Real charHeight = 1.0,
             const Ogre::ColourValue &color = Ogre::ColourValue::White) :
    MovableText(caption, fontName, charHeight, color)
  {
  }

protected:
  /**
   * @brief Overriden method which causes static behaviour.
   */
  void getWorldTransforms(Ogre::Matrix4 *xform) const
  {
    if (this->isVisible())
    {
      Ogre::Matrix3 rot3x3 = Ogre::Matrix3::IDENTITY;

      // store rotation in a matrix
      mParentNode->_getDerivedOrientation().ToRotationMatrix(rot3x3);

      // parent node position
      Ogre::Vector3 ppos = mParentNode->_getDerivedPosition() + Ogre::Vector3::UNIT_Y * mGlobalTranslation;
      ppos += rot3x3 * mLocalTranslation;

      *xform = rot3x3;
      xform->setTrans(ppos);
    }
  }

};

}

#endif /* STATIC_TEXT_H_ */
