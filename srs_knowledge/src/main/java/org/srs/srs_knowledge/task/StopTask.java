/*
 * Copyright 2011,2012.
 *
 * Licensed under the LGPL License.
 * 
 * Author: Ze Ji, Cardiff University. JiZ1@cf.ac.uk
 * The EU FP7 SRS project. 
 * http://www.srs-project.eu
 */

package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import org.srs.srs_knowledge.knowledge_engine.*;
import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import ros.*;
import ros.communication.*;

public class StopTask extends Task {

    public StopTask() {
	// empty constructor.
	this.initTask();
    }

    private void initTask() {
	acts = new ArrayList<ActionTuple>();
	
	System.out.println("TASK.JAVA: Created CurrentTask " + "stop ");
	constructTask();
    }
	
    protected boolean constructTask() {
	return createSimpleMoveTaskNew();
    }
    
    private boolean createSimpleMoveTaskNew() {
	// boolean addNewActionTuple(ActionTuple act)
	ActionTuple act = new ActionTuple();
	
	CUAction ca = new CUAction();
	GenericAction genericAction = new GenericAction();
	/*	
	genericAction.actionInfo.add("stop");
	ca.generic = genericAction;
	ca.actionType = "generic";
	
	act.setCUAction(ca);
	act.setActionId(1);
	addNewActionTuple(act);
	*/
	// add finish action __ success
	
	//act = new ActionTuple();
	
	//ca = new CUAction();
	//genericAction = new GenericAction();
	genericAction.actionInfo.add("finish_success");
	
	ca.generic = genericAction;
	ca.actionType = "generic";
	
	act.setActionName("finish_success");
	ca.status = 1;
	
	act.setCUAction(ca);
	act.setActionId(1);
	//act.setParentId(1);
	//act.setCondition(true);
	addNewActionTuple(act);
	
	// add finish action __ fail
	/*	
	act = new ActionTuple();
	
	ca = new CUAction(); 
	genericAction = new GenericAction();
	genericAction.actionInfo.add("finish_fail");
	
	ca.generic = genericAction;
	ca.actionType = "generic";
	
	act.setActionName("finish_fail");
	
	ca.status = -1;
	act.setCUAction(ca);
	act.setActionId(3);
	act.setParentId(1);
	act.setCondition(false);
	addNewActionTuple(act);
	*/
	System.out.println("number of actions: " + acts.size());
	return true;
    }


    public boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery) {
	return false;
    }

}
