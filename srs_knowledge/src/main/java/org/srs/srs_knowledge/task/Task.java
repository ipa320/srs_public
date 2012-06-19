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

public abstract class Task {
    public enum TaskType {
	GET_OBJECT, MOVETO_LOCATION, SEARCH_OBJECT, SCAN_AROUND, STOP_TASK, UNSPECIFIED
	    };
    
    public Task() {
	// empty constructor.
	acts = new ArrayList<ActionTuple>();     // to be deprecated and replaced with allSubSeqs
	
	setTaskType(TaskType.UNSPECIFIED);
	currentAction = null;
	//ontoDB = new OntologyDB();
    }

    public abstract boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery);

    protected abstract boolean constructTask();

    public void setTaskId(int id) {
	this.taskId = id;
    }
    
    public int getTaskId() {
	return taskId;
    }
    
    public void setTaskTarget(String target) {
	this.targetContent = target;
    }
    
    public String getTaskTarget() {
	return this.targetContent;
    }
    
    public void setTaskType(TaskType type) {
	this.taskType = type;
    }

    //public abstract CUAction getNextCUAction(boolean stateLastAction, String jsonFeedback);
 
    public CUAction getNextCUAction(boolean stateLastAction, ArrayList<String> feedback) {
	//CUAction ca = new CUAction();
	if (currentAction == null) {
	    for (int i = 0; i < acts.size(); i++) {
		if (acts.get(i).getActionId() == 1) {
		    currentAction = acts.get(i);
		    currentActionLoc = i;
		    System.out.println(currentAction.getActionName());
		    
		    if (currentAction != null) 
			return currentAction.getCUAction();
		    else 
			return null;
		}
	    }
	} else {
	    // ActionTuple at;
	    // int parentId = at.getParentId();
	    for (int i = 0; i < acts.size(); i++) {
		// if(acts.get(currentActionLoc).getId() ==
		// acts.get(i).getParentId() && stateLastAction ==
		// acts.get(i).getCondition()){
		if (currentAction.getActionId() == acts.get(i).getParentId()
		    && stateLastAction == acts.get(i).getCondition()) {
		    currentAction = acts.get(i);
		    currentActionLoc = i;
		    System.out.println(i);
		    System.out.println(currentAction.getActionName());
		    if (currentAction != null) 
			return currentAction.getCUAction();
		    else 
			return null;
		}
	    }
	    System.out.println("no action found");
	}	
	
	return null;
    }
    
    public boolean addNewActionTuple(ActionTuple act) {
	return acts.add(act);
    }
    
    public boolean loadPredefinedSequence(String filename) throws IOException,
								  Exception {
	System.out.println("LOAD " + filename);
	File file = null;
	FileReader freader = null;
	LineNumberReader in = null;
	
	try {
	    file = new File(filename);
	    freader = new FileReader(file);
	    in = new LineNumberReader(freader);
	    String line = "";
	    
	    while ((line = in.readLine().trim()) != null) {
		System.out.println("Line: " + in.getLineNumber() + ": " +
				   line);
		if (line.equals("")) {
		    continue;
		} else if (line.substring(0, 1).equals("#")) {
		    continue;
		}
		ActionTuple act = newParseAction(line);
		if (act != null)
		    this.acts.add(act);
	    }
	} finally {
	    freader.close();
	    in.close();
	}
	System.out.println("Number of actions is: " + acts.size());
	return true;
    }
    
    private static ActionTuple newParseAction(String actionDesc) throws Exception {
	ActionTuple act;
	act = new ActionTuple();
	String[] actions;
	CUAction ca = new CUAction();
	actions = actionDesc.split(";");
	if (actions.length != 9) {
	    throw new Exception("Wrong format");
	}
	String actionName = actions[0];
	int actionLevel = Integer.parseInt(actions[1]);
	int actionId = Integer.parseInt(actions[2]);
	
	int parentId = Integer.parseInt(actions[3]);
	String cond = actions[4];
	boolean condition = true;
	if (cond.equals("fail"))
	    condition = false;
	else if (cond.equals("success"))
			condition = true;
	else {
	    // condition = true;
	    System.out.println("Wrong format.");
	    throw new Exception("Wrong format");
	}
	
	String moveAction = actions[5];
	String perceptionAction = actions[6];
	String graspAction = actions[7];
	
	GenericAction ma = newParseMoveAction(moveAction);
	GenericAction pa = newParsePerceptionAction(perceptionAction);
	GenericAction ga = newParseGraspAction(perceptionAction, graspAction);
	
	String actionFlags = actions[8];
	int[] actFlags = parseActionFlags(actionFlags);
	if(actFlags[0] == 0 && actFlags[1] == 1 && actFlags[2] == 1) {
	    ca.generic = ma;
	}
	else if(actFlags[0] == 1 && actFlags[1] == 0 && actFlags[2] == 0) {
	    ca.generic = ga;
	}
	
	ca.actionType = "generic";
	ca.status = 0;
	
	act.setCUAction(ca);
	act.setActionName(actionName);
	act.setActionLevel(actionLevel);
	act.setActionId(actionId);
	act.setParentId(parentId);
	act.setCondition(condition);
	
	if (act.getActionName().indexOf("finish") != -1 && act.getCondition()) {
	    ca.status = 1;
	}
	if (act.getActionName().indexOf("finish") != -1 && !act.getCondition()) {
	    ca.status = -1;
	}
	
	return act;
    }
    
    private static GenericAction newParseMoveAction(String moveAction)
	throws Exception {
	GenericAction geAct = new GenericAction(); 

	String[] parameters = moveAction.split(" ");
	
	geAct.actionInfo.add("move");
	geAct.actionInfo.add(parameters[0]);
	geAct.actionInfo.add(parameters[1]);
	geAct.actionInfo.add(parameters[2]);
	geAct.actionInfo.add(parameters[3]);   //ifWaitForObjectTaken.equals("true") or "false"

	return geAct;
    }
    
    protected static int[] parseActionFlags(String actionFlags) throws Exception {
	int[] _actionFlags = new int[3];
	
	String[] parameters = actionFlags.split(" ");
	_actionFlags[0] = Integer.parseInt(parameters[0].trim());
	_actionFlags[1] = Integer.parseInt(parameters[1].trim());
	_actionFlags[2] = Integer.parseInt(parameters[2].trim());
	
	// System.out.println(actionFlags);
	return _actionFlags;
    }
    
    private static GenericAction newParsePerceptionAction(String perceptionAction) throws Exception {
	String[] parameters = perceptionAction.split(" ");
	GenericAction geAct = new GenericAction(); 
	geAct.actionInfo.add("detect");
	geAct.actionInfo.add(parameters[1]);     // id
	geAct.actionInfo.add(parameters[0]);     // name (not available in this case... )
	return geAct;
    }

    private static GenericAction newParseGraspAction(String perceptionAction, String graspAction) throws Exception {
	
	String[] parameters = graspAction.split(" ");
	if(!parameters[0].equals("true")) {
	    return null;
	}
	
	parameters = perceptionAction.split(" ");
	GenericAction geAct = new GenericAction(); 
	geAct.actionInfo.add("grasp");
	geAct.actionInfo.add(parameters[1]);     // id
	geAct.actionInfo.add(parameters[0]);     // name (not available in this case... )
	geAct.actionInfo.add("side");
	
	return geAct;
    }

    public boolean isEmpty() {
	try {
	    if(allSubSeqs.size() == 0 && acts.size() == 0) {
		return true;
	    }
	}
	catch(NullPointerException e) {
	    System.out.println(e.getMessage() + "\n" + e.toString());
	    return true;
	}
	return false;
    }

    protected TaskType taskType;
    protected String targetContent;
    protected int taskId;
    protected ArrayList<ActionTuple> acts;
    protected int currentActionId = 1;
    protected ActionTuple currentAction;
    protected int currentActionLoc = 0;
    //   protected NodeHandle nodeHandle;
    protected ArrayList<HighLevelActionSequence> allSubSeqs = new ArrayList<HighLevelActionSequence>();
}
