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
import java.util.StringTokenizer;
import java.util.ArrayList;

import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;

import org.srs.srs_knowledge.task.Task;
import org.srs.srs_knowledge.knowledge_engine.*;


/**
 * An ActionUnit is a container of GenericAction. 
 * Unit does not have to be containing only one action. e.g. an action of detection an object on a table can contain a few steps, move to pos1, detect, move to pos2, detect, move to pos3, detect, etc. 
 */
public class MoveActionUnit extends HighLevelActionUnit {

    public MoveActionUnit(Pose2D position) {
	GenericAction ga = new GenericAction();
	ga.jsonActionInfo = SRSJSONParser.encodeMoveAction("move", position.x, position.y, position.theta);
	actionUnits.add(ga);

	// this actionunit is always set with sufficient parameters
	ifBasePoseSet = true;
	ifParametersSet = true;

	int size = actionUnits.size(); 
	nextActionMapIfFail = new int[size];
	nextActionMapIfSuccess = new int[size];
	
	for(int i = 0; i < size; i++) {
	    nextActionMapIfFail[i] =  COMPLETED_FAIL;
	    nextActionMapIfSuccess[i] = COMPLETED_SUCCESS;  
	}

    }

    public String getActionType() {
	actionType = "Move";
	return actionType;
    }

    @Override
    public boolean setParameters(String action, String para, String reservedParam) {
	if(action.equals("move")) {
	    setBasePose(para);
	}
	return ifParametersSet;
    }

    private void setBasePose(String jsonPose) {

	GenericAction nga = new GenericAction();
	nga.jsonActionInfo = jsonPose;
	actionUnits.set(0, nga);
	ifBasePoseSet = true;
	ifParametersSet = ifBasePoseSet;
    }

    public boolean ifParametersSet() {
	return ifParametersSet;
    }

    private boolean ifBasePoseSet = false;
}