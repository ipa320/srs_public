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

    public CUAction getNextCUActionNew(boolean stateLastAction, String jsonFeedback) {
	
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
