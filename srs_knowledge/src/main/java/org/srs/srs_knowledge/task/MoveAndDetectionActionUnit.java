package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList;

import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;

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
public abstract class MoveAndDetectionActionUnit extends HighLevelActionUnit {

    public MoveAndDetectionActionUnit(ArrayList<Pose2D> positions, String objectClassName, int houseHoldId) {
	for(Pose2D position:positions) {
	    GenericAction ga = new GenericAction();
	    ga.actionInfo.add("move");
	    ga.actionInfo.add(Double.toString(position.x));
	    ga.actionInfo.add(Double.toString(position.y));
	    ga.actionInfo.add(Double.toString(position.theta));

	    actionUnits.add(ga);

	    GenericAction detAct = new GenericAction();
	    detAct.actionInfo.add("detect");
	    detAct.actionInfo.add(Integer.toString(houseHoldId));
	    detAct.actionInfo.add("objectClassName");

	    actionUnits.add(detAct);
	}

	int size = actionUnits.size(); 
	nextActionMapIfFail = new int[size];

	for(int i = 0; i < size; i++) {
	    if(actionUnits.get(i).actionInfo.get(0).equals("move")) {
		nextActionMapIfFail[i] = i + 2;
	    }
	    else if(actionUnits.get(i).actionInfo.get(0).equals("detect")) {
		nextActionMapIfFail[i] = i + 1;
	    }
	    if(nextActionMapIfFail[i] >= size) {
		// out of bound, means this is the last step in this action unit. so -1 means there is no further solution to the current task within this actionunit
		nextActionMapIfFail[i] = -1;  
	    }	    
	}
    }

    public String getActionType() {
	actionType = "MoveAndDetection";
	return actionType;
    }

    /*
    private String actionType;

    private ArrayList<GenericAction> actionUnits = new ArrayList<GenericAction>();
    private int[] nextActionMapIfFail;
    */
}