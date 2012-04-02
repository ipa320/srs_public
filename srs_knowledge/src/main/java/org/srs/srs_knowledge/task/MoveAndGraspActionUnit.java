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
import ros.pkg.geometry_msgs.msg.Pose;
import org.srs.srs_knowledge.task.Task;
import org.srs.srs_knowledge.knowledge_engine.*;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;

/**
 * An ActionUnit is a container of GenericAction. 
 * Unit does not have to be containing only one action. e.g. an action of detection an object on a table can contain a few steps, move to pos1, detect, move to pos2, detect, move to pos3, detect, etc. 
 */
public class MoveAndGraspActionUnit extends HighLevelActionUnit {

    public MoveAndGraspActionUnit(Pose2D position, String objectClassName, int houseHoldId, String graspConfig) {
	    GenericAction ga = new GenericAction();
	    ga.actionInfo.add("move");
	    if(position != null) {
		ga.actionInfo.add(Double.toString(position.x));
		ga.actionInfo.add(Double.toString(position.y));
		ga.actionInfo.add(Double.toString(position.theta));
		ifBasePoseSet = true;
	    }
	    else {
		ga.actionInfo.add("");
		ga.actionInfo.add("");
		ga.actionInfo.add("");
		ifBasePoseSet = false;
	    }

	    actionUnits.add(ga);

	    GenericAction graspAct = new GenericAction();
	    graspAct.actionInfo.add("grasp");

	    if (objectClassName != null || !objectClassName.equals("")) {
		graspAct.actionInfo.add(Integer.toString(houseHoldId));
		graspAct.actionInfo.add(objectClassName);
		ifObjectInfoSet = true;
	    }
	    else {
		graspAct.actionInfo.add(Integer.toString(houseHoldId));
		graspAct.actionInfo.add("");
		ifObjectInfoSet = false;
	    }

	    if (graspConfig != null || !graspConfig.equals("")) {
		// side, top etc
		graspAct.actionInfo.add(graspConfig);
		ifObjectInfoSet = true && ifObjectInfoSet;
	    }
	    else {
		graspAct.actionInfo.add("");
		ifObjectInfoSet = false;
	    }

	    actionUnits.add(graspAct);

	    //ifObjectPoseSet = true;
	    //ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	    // object not considered here
	    ifParametersSet = ifBasePoseSet;

	    int size = actionUnits.size(); 
	    nextActionMapIfFail = new int[size];
	    nextActionMapIfSuccess = new int[size];
	    
	    for(int i = 0; i < size; i++) {
		if(actionUnits.get(i).actionInfo.get(0).equals("move")) {
		    nextActionMapIfSuccess[i] = i + 1;
		    nextActionMapIfFail[i] = i + 2;
		}
		else if(actionUnits.get(i).actionInfo.get(0).equals("grasp")) {
		    nextActionMapIfSuccess[i] = COMPLETED_SUCCESS;    // 
		    nextActionMapIfFail[i] = i + 1;
		}
		if(nextActionMapIfFail[i] >= size) {
		    // out of bound, means this is the last step in this action unit. so -1 means there is no further solution to the current task within this actionunit
		    nextActionMapIfFail[i] = COMPLETED_FAIL;  
		}	    
	}


    }
    
    public String getActionType() {
	actionType = "MoveAndGrasp";
	return actionType;
    }

    public int getNextCUActionIndex(boolean statusLastStep) {
	System.out.println(" ==> DEBUG 0");
	if(currentActionInd == -1) {
	    System.out.println(" ==> DEBUG 1");
	    return 0;
	}
	System.out.println(" ==> DEBUG 2");

	if ( currentActionInd >= 0 && currentActionInd < actionUnits.size() ) {
	    System.out.println(" ==> DEBUG 3");

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

    public CUAction getCUActionAt(int ind) {
	currentActionInd = ind;
	CUAction ca = new CUAction(); 

	if(ind == COMPLETED_FAIL) {
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("finish_fail");
	    
	    ca.generic = genericAction;
	    ca.actionType = "generic";	    
	    ca.status = -1;
	    
	    return ca;
	}
	else if (ind == COMPLETED_SUCCESS){
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("finish_success");
	    
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    ca.status = 1;
	    return ca;
	}
	else if (ind == INVALID_INDEX) {
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("no_action");
	    
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    ca.status = -1;
	    return ca;
	}

	GenericAction genericAction = actionUnits.get(ind);
	ca.generic = genericAction;
	ca.actionType = "generic";
	return ca;
    }
    
    // a not very safe, but flexible way to assign parameters, using arraylist<string> 
    // set robot move target and object pose etc.
    public boolean setParameters(ArrayList<String> para) {
	//boolean res = ifParametersSet;
	try {
	    setBasePose(para);
	    //setGraspInfo(para);
	    ifParametersSet = true;
	}
	catch(IllegalArgumentException e) {
	    System.out.println(e.getMessage());
	    return false;
	}
	return ifParametersSet;
    }

    private void setBasePose(ArrayList<String> pose) throws IllegalArgumentException {

	GenericAction ga = actionUnits.get(0);

	if( ga.actionInfo.get(0).equals("move") && pose.get(0).equals("move") && pose.size() == ga.actionInfo.size()) {
	    //actionUnits.get(0).clear();
	    
	    GenericAction nga = new GenericAction();
	    nga.actionInfo = pose;
	    actionUnits.set(0, nga);
	    ifBasePoseSet = true;
	    ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	}
	else {
	    System.out.println(ga.actionInfo.get(0));
	    System.out.println(pose.get(0).equals("move"));
	    System.out.println(pose.size());
	    System.out.println(ga.actionInfo.size());
	    throw new IllegalArgumentException("Wrong format exception -- when setting Base Pose with arrayList");
	}
    }

    // objInfo should be in format as defined in constructor
    private void setGraspInfo(ArrayList<String> objInfo) {
	GenericAction ga = actionUnits.get(1);

	if( ga.actionInfo.get(0).equals("grasp") && objInfo.get(0).equals("grasp") && objInfo.size() == ga.actionInfo.size()) {
	    //actionUnits.get(0).clear();
	    GenericAction nga = new GenericAction();
	    nga.actionInfo = objInfo;
	    //actionUnits.set(1, objInfo);
	    /*
	    GenericAction graspAct = new GenericAction();
	    graspAct.actionInfo.add("grasp");
	    graspAct.actionInfo.add(Integer.toString(houseHoldId));
	    graspAct.actionInfo.add(objectClassName);
	    */
	    
	    actionUnits.set(1, nga);
	    ifObjectInfoSet = true;
	    ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	}
	else {
	    throw new IllegalArgumentException("Wrong format exception -- when setting Object Info with arrayList");
	}
    }

    private boolean setObjectPose(ArrayList<String> objPose) {
	return false;
    }

    public boolean ifParametersSet() {
	ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	return ifParametersSet;
    }

    private boolean ifBasePoseSet = false;
    private boolean ifObjectInfoSet = false;
    //private boolean ifObjectIDSet = false;
    //private boolean ifObjectNameSet = false;
}