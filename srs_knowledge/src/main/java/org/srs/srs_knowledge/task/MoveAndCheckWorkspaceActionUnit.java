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