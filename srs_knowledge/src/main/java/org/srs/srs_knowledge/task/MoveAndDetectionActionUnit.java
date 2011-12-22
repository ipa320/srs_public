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
public class MoveAndDetectionActionUnit extends HighLevelActionUnit {

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
	nextActionMapIfSuccess = new int[size];

	for(int i = 0; i < size; i++) {
	    if(actionUnits.get(i).actionInfo.get(0).equals("move")) {
		nextActionMapIfSuccess[i] = i + 1;
		nextActionMapIfFail[i] = i + 2;
	    }
	    else if(actionUnits.get(i).actionInfo.get(0).equals("detect")) {
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
	actionType = "MoveAndDetection";
	return actionType;
    }

    private int getNextCUActionIndex() {
	return 0;
    }

    public CUAction getNextCUAction() {
	int ind = getNextCUActionIndex();
	CUAction ca = new CUAction(); 
	MoveAction ma = new MoveAction();
	PerceptionAction pa = new PerceptionAction();
	GraspAction ga = new GraspAction();
	
	if(ind == COMPLETED_FAIL) {
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("finish_fail");
	    
	    ca.ma = ma;
	    ca.pa = pa;
	    ca.ga = ga;
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    int[] af =  {0, 1, 1};
	    ca.actionFlags = af;
	    
	    //act.setActionName("finish_fail");
	    
	    ca.status = -1;
	    
	    return ca;
	}
	else if (ind == COMPLETED_SUCCESS){
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("finish_success");
	    
	    ca.ma = ma;
	    ca.pa = pa;
	    ca.ga = ga;
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    int[] af =  {0, 1, 1};
	    ca.actionFlags = af;
	    
	    //act.setActionName("finish_fail");
	    
	    ca.status = 1;
	    
	    return ca;
	}
	else if (ind > 0 && ind < actionUnits.size()){
	    GenericAction genericAction = actionUnits.get(ind);
	    	    
	    ca.ma = ma;
	    ca.pa = pa;
	    ca.ga = ga;
	    ca.generic = genericAction;

	    ca.actionType = "generic";
	    return ca;
	}
	else {
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("no_action");
	    
	    ca.ma = ma;
	    ca.pa = pa;
	    ca.ga = ga;
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    int[] af =  {1, 1, 1};
	    ca.actionFlags = af;
	    
	    return ca;
	}
    }


    /*
    private String actionType;

    private ArrayList<GenericAction> actionUnits = new ArrayList<GenericAction>();
    private int[] nextActionMapIfFail;
    */
}