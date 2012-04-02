/****************************************************************
 *
 * Copyright (c) 2011, 2012
 *
 * School of Engineering, Cardiff University, UK
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: srs EU FP7 (www.srs-project.eu)
 * ROS stack name: srs
 * ROS package name: srs_knowledge
 * Description: 
 *								
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * @author Ze Ji, email: jiz1@cf.ac.uk
 *
 * Date of creation: Oct 2011:
 * ToDo: 
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *	 * Redistributions of source code must retain the above copyright
 *	   notice, this list of conditions and the following disclaimer.
 *	 * Redistributions in binary form must reproduce the above copyright
 *	   notice, this list of conditions and the following disclaimer in the
 *	   documentation and/or other materials provided with the distribution.
 *	 * Neither the name of the school of Engineering, Cardiff University nor 
 *         the names of its contributors may be used to endorse or promote products 
 *         derived from this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public 
 * License LGPL along with this program. 
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

package org.srs.srs_knowledge.task;

import java.io.*;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;

/**
 * A container of CUAction and additional information
 */
public class ActionTuple
{
    public ActionTuple()
    {
	//ca = new CUAction();
    }

    public void setCUAction(CUAction ca)
    {
	this.ca = ca;
    }

    public void setActionName(String actionName)
    {
	this.actionName = actionName;
    }

    public void setActionLevel(int actionLevel)
    {
	this.actionLevel = actionLevel;
    }
    
    public void setActionId(int actionId)
    {
	this.actionId = actionId;
    }
    
    public void setParentId(int parentId)
    {
	this.parentId = parentId;
    }
    
    public void setCondition(boolean condition)
    {
	this.condition = condition;
    }

    public CUAction getCUAction()
    {
	return this.ca;
    }

    public String getActionName()
    {
	return actionName;
    }

    public int getActionLevel()
    {
	return actionLevel;
    }
    
    public int getActionId()
    {
	return actionId;
    }
    
    public int getParentId()
    {
	return this.parentId;
    }

    public boolean getCondition()
    {
	return condition;
    }


    private CUAction ca;

    private String actionName = "";
    private int actionId;
    private int parentId;
    private int actionLevel;
    private boolean condition; 

}