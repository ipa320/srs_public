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
import ros.pkg.geometry_msgs.msg.Pose;
import org.srs.srs_knowledge.task.Task;
import org.srs.srs_knowledge.knowledge_engine.*;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.json.simple.parser.ParseException;
import org.json.simple.parser.JSONParser;

/**
 * An ActionUnit is a container of GenericAction. 
 * Unit does not have to be containing only one action. e.g. an action of detection an object on a table can contain a few steps, move to pos1, detect, move to pos2, detect, move to pos3, detect, etc. 
 */
public class MoveAndGraspActionUnit extends HighLevelActionUnit {

    public MoveAndGraspActionUnit(Pose2D position, String objectClassName, int houseHoldId, String graspConfig) {
	init(position, objectClassName, houseHoldId, graspConfig, "");
    }

    public MoveAndGraspActionUnit(Pose2D position, String objectClassName, int houseHoldId, String graspConfig, String workspace) {
	init(position, objectClassName, houseHoldId, graspConfig, workspace);
    }

    private void init(Pose2D position, String objectClassName, int houseHoldId, String graspConfig, String workspace) {
	    GenericAction ga = new GenericAction();
	    //ga.actionInfo.add("move");
	    if(position != null) {
		ga.jsonActionInfo = SRSJSONParser.encodeMoveAction("move", position.x, position.y, position.theta);
		ifBasePoseSet = true;
	    }
	    else {
		ga.jsonActionInfo = SRSJSONParser.encodeMoveAction("move", -1000, -1000, -1000);
		ifBasePoseSet = false;
	    }

	    actionUnits.add(ga);

	    GenericAction graspAct = new GenericAction();
	    //graspAct.actionInfo.add("grasp");
	    objectClassName = (objectClassName == null) ? "" : objectClassName;
	    ifObjectInfoSet = (objectClassName.trim().equals("")) ? false : true;

	    graspConfig = (graspConfig == null) ? "" : graspConfig;
	    ifObjectInfoSet = true && ((graspConfig.trim().equals("")) ? false : true);
	    graspAct.jsonActionInfo = SRSJSONParser.encodeGraspAction("grasp", houseHoldId, objectClassName, workspace); 

	    actionUnits.add(graspAct);

	    //ifObjectPoseSet = true;
	    //ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	    // object not considered here
	    ifParametersSet = ifBasePoseSet;

	    int size = actionUnits.size(); 
	    nextActionMapIfFail = new int[size];
	    nextActionMapIfSuccess = new int[size];
	    
	    for(int i = 0; i < size; i++) {
		JSONObject tempAct = SRSJSONParser.decodeJsonActionInfo(actionUnits.get(i).jsonActionInfo);
		if(tempAct.get("action").equals("move")) {
		    nextActionMapIfSuccess[i] = i + 1;
		    nextActionMapIfFail[i] = i + 2;
		}
		else if(tempAct.get("action").equals("grasp")) {
		    nextActionMapIfSuccess[i] = COMPLETED_SUCCESS;    // 
		    nextActionMapIfFail[i] = i + 1;
		}
		if(nextActionMapIfFail[i] >= size) {
		    // out of bound, means this is the last step in this action unit. so -1 means there is no further solution to the current task within this actionunit
		    nextActionMapIfFail[i] = COMPLETED_FAIL;  
		}	    
	}
    }
    
    @Override
    public String getActionType() {
	actionType = "MoveAndGrasp";
	return actionType;
    }

    @Override
    public int getNextCUActionIndex(boolean statusLastStep) {
	if(currentActionInd == -1) {
	    return 0;
	}

	if ( currentActionInd >= 0 && currentActionInd < actionUnits.size() ) {

	    if(statusLastStep) {
		
		System.out.println("NEXT ACTION IND (if Successful): " + nextActionMapIfSuccess[currentActionInd]);
		return nextActionMapIfSuccess[currentActionInd];
	    }
	    else {
		System.out.println("NEXT ACTION IND (if Failed): " + nextActionMapIfFail[currentActionInd]);
		return nextActionMapIfFail[currentActionInd];
	    }
	}
	else {
	    return INVALID_INDEX;
	}
    }

    @Override
    public boolean setParameters(String action, String para, String reservedParam) {
	if(action.equals("move")) {
	    setBasePose(para);
	}
	else if(action.equals("grasp")) {
	    setGraspInfo(para);
	}
	return ifParametersSet;
    }

    private void setBasePose(String jsonPose) {

	GenericAction nga = new GenericAction();
	nga.jsonActionInfo = jsonPose;
	actionUnits.set(0, nga);
	ifBasePoseSet = true;
	ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	
    }

    private void setGraspInfo(String jsonInfo) {

	GenericAction nga = new GenericAction();
	nga.jsonActionInfo = jsonInfo;
	actionUnits.set(1, nga);
	ifObjectInfoSet = true;
	ifParametersSet = ifBasePoseSet && ifObjectInfoSet;

    }

    private boolean setObjectPose(ArrayList<String> objPose) {
	return false;
    }

    @Override
    public boolean ifParametersSet() {
	ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	return ifParametersSet;
    }

    private boolean ifBasePoseSet = false;
    private boolean ifObjectInfoSet = false;
}