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

    public MoveAndGraspActionUnit(Pose2D position, String objectClassName, int houseHoldId, Pose objectPose) {
	    GenericAction ga = new GenericAction();
	    ga.actionInfo.add("move");
	    ga.actionInfo.add(Double.toString(position.x));
	    ga.actionInfo.add(Double.toString(position.y));
	    ga.actionInfo.add(Double.toString(position.theta));

	    actionUnits.add(ga);

	    GenericAction graspAct = new GenericAction();
	    graspAct.actionInfo.add("grasp");
	    graspAct.actionInfo.add(Integer.toString(houseHoldId));
	    graspAct.actionInfo.add("objectClassName");
	    graspAct.actionInfo.add(Double.toString(objectPose.position.x));
	    graspAct.actionInfo.add(Double.toString(objectPose.position.y));
	    graspAct.actionInfo.add(Double.toString(objectPose.position.z));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.x));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.y));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.z));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.w));

	    actionUnits.add(graspAct);


	    int size = actionUnits.size(); 
	    nextActionMapIfFail = new int[size];
	    
	    for(int i = 0; i < size; i++) {
		nextActionMapIfFail[i] = -1;  
	    }
    }
    
    public String getActionType() {
	actionType = "MoveAndGrasp";
	return actionType;
    }

    /*
    private String actionType;

    private ArrayList<GenericAction> actionUnits = new ArrayList<GenericAction>();
    private int[] nextActionMapIfFail;
    */
}