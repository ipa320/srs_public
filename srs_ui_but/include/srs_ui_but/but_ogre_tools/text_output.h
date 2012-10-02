/******************************************************************************
 * \file
 *
 * $Id:
 *
 * Copyright (C) Brno University of Technology
 *
 * This file is part of software developed by dcgm-robotics@FIT group.
 *
 * Author: Tomas Lokaj (xlokaj03@stud.fit.vutbr.cz)
 * Supervised by: Michal Spanel (spanel@fit.vutbr.cz)
 * Date: 26/10/2012
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

#pragma once
#include <Ogre.h>

#ifndef TEXT_OUTPUT_H_
#define TEXT_OUTPUT_H_

namespace ogre_tools
{
/**
 * @brief This class shows output strings in the Ogre scene window.
 */
class TextOutput
{

public:
  /**
   * @brief Constructor
   *
   * @param id unique string ID
   * @param pos_x x position of the text box
   * @param pos_y y position of the text box
   * @param width width of the text box
   * @param height height of the text box
   */
  TextOutput(const std::string& id, Ogre::Real pos_x, Ogre::Real pos_y, Ogre::Real width, Ogre::Real height) :
      id_(id)
  {
    overlayManager_ = Ogre::OverlayManager::getSingletonPtr();

    panel_ = static_cast<Ogre::OverlayContainer*>(overlayManager_->createOverlayElement("Panel", "text_container"));
    panel_->setDimensions(1, 1);
    panel_->setPosition(0, 0);

    overlay_ = overlayManager_->create("text_overlay");
    overlay_->add2D(panel_);
    overlay_->show();

    textBox_ = overlayManager_->createOverlayElement("TextArea", id_);
    textBox_->setPosition(pos_x, pos_y);
    textBox_->setDimensions(width, height);
    textBox_->setWidth(width);
    textBox_->setHeight(height);
    textBox_->setMetricsMode(Ogre::GMM_PIXELS);
    textBox_->setParameter("font_name", "Arial");

    panel_->addChild(textBox_);
  }

  /**
   * @brief Destructor
   */
  ~TextOutput();

  /**
   * Sets text
   * @param text is text
   */
  void setText(const std::string& Text)
  {
    textBox_->setCaption(Text);
  }

  /**
   * Sets color of the text
   * @param color is color of the text
   */
  void setColor(const Ogre::ColourValue& color)
  {
    textBox_->setColour(color);
  }

  /**
   * Sets size of the font
   * @param size is size of the ofnt
   */
  void setFontSize(const int& size)
  {
    std::stringstream char_height;
    char_height << size;
    textBox_->setParameter("char_height", char_height.str());
  }

private:
  Ogre::OverlayElement* textBox_;
  Ogre::OverlayManager* overlayManager_;
  Ogre::Overlay* overlay_;
  Ogre::OverlayContainer* panel_;
  std::string id_;
};
}

#endif /* TEXT_OUTPUT_H_ */
