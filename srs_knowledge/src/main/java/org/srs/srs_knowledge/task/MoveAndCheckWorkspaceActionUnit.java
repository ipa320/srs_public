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
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
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

import org.srs.srs_knowledge.task.*;
import org.srs.srs_knowledge.knowledge_engine.*;
import ros.pkg.srs_symbolic_grounding.srv.*;
import ros.pkg.srs_symbolic_grounding.msg.*;

/**
 * An ActionUnit is a container of GenericAction. 
 * Unit does not have to be containing only one action. e.g. an action of detection an object on a table can contain a few steps, move to pos1, detect, move to pos2, detect, move to pos3, detect, etc. 
 */
public class MoveAndCheckWorkspaceActionUnit extends HighLevelActionUnit {

    public MoveAndCheckWorkspaceActionUnit(ArrayList<Pose2D> positions, String objectClassName, SRSFurnitureGeometry targetSurface) {
	for(Pose2D position:positions) {
	    GenericAction ga = new GenericAction();
	    ga.actionInfo.add("move");
	    ga.actionInfo.add(Double.toString(position.x));
	    ga.actionInfo.add(Double.toString(position.y));
	    ga.actionInfo.add(Double.toString(position.theta));

	    actionUnits.add(ga);

	    GenericAction detAct = new GenericAction();
	    detAct.actionInfo.add("check");
	    detAct.actionInfo.add(objectClassName);
	    detAct.actionInfo.add(Double.toString(position.x));
	    detAct.actionInfo.add(Double.toString(position.y));
	    detAct.actionInfo.add(Double.toString(position.theta));

	    detAct.actionInfo.add(Double.toString(targetSurface.pose.position.x));
	    detAct.actionInfo.add(Double.toString(targetSurface.pose.position.y));
	    detAct.actionInfo.add(Double.toString(targetSurface.pose.position.z));
	    detAct.actionInfo.add(Double.toString(targetSurface.pose.orientation.x));
	    detAct.actionInfo.add(Double.toString(targetSurface.pose.orientation.y));
	    detAct.actionInfo.add(Double.toString(targetSurface.pose.orientation.z));
	    detAct.actionInfo.add(Double.toString(targetSurface.pose.orientation.w));
	    detAct.actionInfo.add(Double.toString(targetSurface.l));
	    detAct.actionInfo.add(Double.toString(targetSurface.w));
	    detAct.actionInfo.add(Double.toString(targetSurface.h));

	    actionUnits.add(detAct);
	}

	// this actionunit is always set with sufficient parameters
	ifParametersSet = true;

	int size = actionUnits.size(); 
	nextActionMapIfFail = new int[size];
	nextActionMapIfSuccess = new int[size];

	for(int i = 0; i < size; i++) {
	    if(actionUnits.get(i).actionInfo.get(0).equals("move")) {
		nextActionMapIfSuccess[i] = i + 1;
		nextActionMapIfFail[i] = i + 2;
		System.out.println("move ----   " + i);
	    }
	    else if(actionUnits.get(i).actionInfo.get(0).equals("check")) {
		nextActionMapIfSuccess[i] = COMPLETED_SUCCESS;    // 
		nextActionMapIfFail[i] = i + 1;
		System.out.println("check ----   " + i);
	    }
	    if(nextActionMapIfFail[i] >= size) {
		// out of bound, means this is the last step in this action unit. so -1 means there is no further solution to the current task within this actionunit
		nextActionMapIfFail[i] = COMPLETED_FAIL;  
		System.out.println("overflow ----   " + i);
	    }	    
	}
    }

    public String getActionType() {
	actionType = "MoveAndCheckWorkspace";
	return actionType;
    }
    // a not very safe, but flexible way to assign parameters, using arraylist<string> 
    // set robot move target and object pose etc.
    public boolean setParameters(ArrayList<String> para) {
	boolean res = ifParametersSet;
	return res;
    }

    public boolean ifParametersSet() {
	return ifParametersSet;
    }

}