/*
 * Licensed under the LGPL License.
 * 
 * Author: Ze Ji, Cardiff University. JiZ1@cf.ac.uk
 * The EU FP7 SRS project. 
 * http://www.srs-project.eu
 */

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
	ga.actionInfo.add("move");
	ga.actionInfo.add(Double.toString(position.x));
	ga.actionInfo.add(Double.toString(position.y));
	ga.actionInfo.add(Double.toString(position.theta));

	actionUnits.add(ga);

	// this actionunit is always set with sufficient parameters
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