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
import org.srs.srs_knowledge.knowledge_engine.*;
import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;
import org.srs.srs_knowledge.task.*;

import ros.pkg.srs_symbolic_grounding.srv.*;
import ros.pkg.srs_symbolic_grounding.msg.*;
import ros.pkg.srs_msgs.msg.SRSSpatialInfo;

import ros.*;
import ros.communication.*;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.json.simple.parser.ParseException;
import org.json.simple.parser.JSONParser;

public class CheckWorkspaceTask extends org.srs.srs_knowledge.task.Task
{
    public CheckWorkspaceTask(String targetContent)
    {	
	this.initTask(targetContent);
    }
    private void initTask(String targetContent) {
	acts = new ArrayList<ActionTuple>();
	
	setTaskTarget(targetContent);
	System.out.println("TASK.JAVA: Created CurrentTask " + "get "
			   + targetContent);
	constructTask();
    }   

    protected boolean constructTask() {
	return createCheckWorkspaceTask();
    }
    
    private boolean createCheckWorkspaceTask() {
	System.out.println("Create New GET OBJECT Task --- ");

	try {
	    workspaces = OntoQueryUtil.getWorkspaceByName(this.targetContent, OntoQueryUtil.ObjectNameSpace, OntoQueryUtil.GlobalNameSpace);
	}
	catch(Exception e) {
	    System.out.println(e.getMessage() + "\n" + e.toString());
	    return false;
	}
	
	for(Individual u : workspaces) {
	    try{
		System.out.println("Created HLActionSeq ");
		HighLevelActionSequence subSeq = createSubSequenceForSingleWorkspace(u);
		allSubSeqs.add(subSeq);
	    }
	    catch(RosException e) {
		System.out.println("ROSEXCEPTION -- when calling symbolic grounding for scanning positions.  \n" + e.getMessage() + "\n" + e.toString());
		
	    }
	    catch(Exception e) {
		System.out.println(e.getMessage());
		System.out.println(e.toString());
	    }
	}	
	
	if(allSubSeqs.size() > 0) {
	    currentSubAction = 0;
	    return true;
	}
	else {
	    currentSubAction = -1; // no sub_highlevel_action in list
	    return false;
	}
    }
    
    private HighLevelActionSequence createSubSequenceForSingleWorkspace(Individual workspace) throws RosException, Exception {
	HighLevelActionSequence actionList = new HighLevelActionSequence();
	
	// create MoveAndDetectionActionUnit
	//SRSFurnitureGeometry spatialInfo = new SRSFurnitureGeometry();
	SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
	com.hp.hpl.jena.rdf.model.Statement stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "xCoord",  workspace);
	
	spatialInfo.pose.position.x = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "yCoord",  workspace);
	spatialInfo.pose.position.y = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "zCoord",  workspace);
	spatialInfo.pose.position.z = stm.getFloat();
		
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "widthOfObject",  workspace);
	spatialInfo.w = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "heightOfObject",  workspace);
	spatialInfo.h = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "lengthOfObject",  workspace);
	spatialInfo.l = stm.getFloat();
	
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qu",  workspace);
	spatialInfo.pose.orientation.w = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qx",  workspace);
	spatialInfo.pose.orientation.x = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qy",  workspace);
	spatialInfo.pose.orientation.y = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qz",  workspace);
	spatialInfo.pose.orientation.z = stm.getFloat();
	
	// call symbolic grounding service for target pose
	ArrayList<Pose2D> posList;
	try {
	    posList = calculateScanPositions(spatialInfo);
	    System.out.println(posList.size());
	}
	catch(RosException e) {
	    System.out.println(e.toString()); 
	    System.out.println(e.getMessage());
	    throw e;
	}

	// TODO:
	MoveAndCheckWorkspaceActionUnit mdAction = new MoveAndCheckWorkspaceActionUnit(posList, targetContent, spatialInfo);
	
	// create FinishActionUnit
	FinishActionUnit fau = new FinishActionUnit(true);

	actionList.appendHighLevelAction(mdAction);
	actionList.appendHighLevelAction(fau);
		
	System.out.println("ActionList size is " + actionList.getSizeOfHighLevelActionList());
	return actionList;
    }
    
    @Override
    public CUAction getNextCUActionNew(boolean stateLastAction, String jsonFeedback) {
     
	System.out.println("===> Get Next CUACTION -- from CheckWorkspaceTask.java");
	CUAction ca = new CUAction();
	if(allSubSeqs.size() == 0 ) {
	    System.out.println("Sequence size is zero");
	    return null;   // ??? 
	}
	if(currentSubAction >= 0 && currentSubAction < allSubSeqs.size()) {
	    // get the current SubActionSequence item
	    System.out.println("Sequence size is " + allSubSeqs.size());
	    HighLevelActionSequence subActSeq = allSubSeqs.get(currentSubAction);
	    
	    HighLevelActionUnit highAct = subActSeq.getCurrentHighLevelActionUnit();
	    // decide if the current SubActionSequence is finished or stuck somewhere? 
	    // if successfully finished, then finished
	    // if stuck (fail), move to the next subActionSequence
	    if(highAct != null) {

		// TODO:
		//updateDBObjectPose();
				
		int ni = highAct.getNextCUActionIndex(stateLastAction);
		System.out.println(" ========>>>>   " + ni);
		switch(ni) {
		case HighLevelActionUnit.COMPLETED_SUCCESS:
		    System.out.println(".COMPLETED_SUCCESS");
		    lastStepActUnit = highAct;

		    CUAction retact = null;		    	    
		    try {
			retact = handleSuccessMessage(new ActionFeedback(jsonFeedback));
		    }
		    catch(ParseException pe) {
			System.out.println(pe.toString());
			return null;
		    }
 
		    return retact; 
		case HighLevelActionUnit.COMPLETED_FAIL: 
		    lastStepActUnit = null;
		    System.out.println(".COMPLETED_FAIL");
		    return handleFailedMessage();
		case HighLevelActionUnit.INVALID_INDEX:
		    lastStepActUnit = null;
		    System.out.println("INVALID_INDEX");
		    return handleFailedMessage();
		default: 
		    System.out.println(highAct.getActionType());
		    if(!highAct.ifParametersSet()) {
			System.out.println("Parameters not set");
			lastStepActUnit = null;
			return handleFailedMessage();
		    }
		    ca = highAct.getCUActionAt(ni);
		    // since it is going to use String list to represent action info. So cation type is always assumed to be generic, hence the first item in the list actionInfo should contain the action type information...
		    // WARNING: No error checking here
		    //lastActionType = ca.generic.actionInfo.get(0);
		    lastActionType = (String)(SRSJSONParser.decodeJsonActionInfo(ca.generic.jsonActionInfo).get("action"));
		    return ca;
		} 
	    }
	    else {
		return null;
	    }	    
	    // or if still pending CUAction is available, return CUAction
	}
	else if (currentSubAction == -1) {
	}
	return ca;
    }

    private CUAction handleFailedMessage() {
	currentSubAction++;
	
	System.out.println("HANDLE FAILED MESSAGE.... CURRENTSUBACTION IS AT:  " + currentSubAction);
	
	if(currentSubAction >= allSubSeqs.size()) {
	    return null;
	}
	HighLevelActionSequence nextHLActSeq = allSubSeqs.get(currentSubAction);
		
	//	if(nextHLActSeq.hasNextHighLevelActionUnit()) {
	HighLevelActionUnit nextHighActUnit = nextHLActSeq.getCurrentHighLevelActionUnit();
	if(nextHighActUnit != null) {
	    int tempI = nextHighActUnit.getNextCUActionIndex(true); //// it does not matter if true or false, as this is to retrieve the first actionunit 
	    // TODO: COULD BE DONE RECURSIVELY. BUT TOO COMPLEX UNNECESSARY AND DIFFICULT TO DEBUG. 
	    // SO STUPID CODE HERE
	    
	    if(tempI == HighLevelActionUnit.COMPLETED_SUCCESS || tempI == HighLevelActionUnit.COMPLETED_FAIL || tempI == HighLevelActionUnit.INVALID_INDEX) {
		CUAction ca = new CUAction();
		ca.status = -1;
		return ca;
	    }
	    else {
		System.out.println("GET NEXT CU ACTION AT:  " + tempI);
		CUAction ca = nextHighActUnit.getCUActionAt(tempI);
		if(ca == null) {
		    System.out.println("CUACTION IS NULL.......");
		}
		return ca;
	    }
	}
	
	return null;
    }
    
    private CUAction handleSuccessMessage(ActionFeedback fb) {
	// TODO: 
	  
	HighLevelActionSequence currentHLActSeq = allSubSeqs.get(currentSubAction);
	
	if(currentHLActSeq.hasNextHighLevelActionUnit()) {
	    HighLevelActionUnit nextHighActUnit = currentHLActSeq.getNextHighLevelActionUnit();
	    // set feedback? 
	    if(nextHighActUnit != null) {
		int tempI = nextHighActUnit.getNextCUActionIndex(true); //// it does not matter if true or false, as this is to retrieve the first actionunit 
		// TODO: COULD BE DONE RECURSIVELY. BUT TOO COMPLEX UNNECESSARY AND DIFFICULT TO DEBUG. 
		// SO STUPID CODE HERE
		
		if(tempI == HighLevelActionUnit.COMPLETED_SUCCESS) {
		    CUAction ca = new CUAction();
		    ca.status = 1;
		    return ca;
		}
		else if(tempI == HighLevelActionUnit.COMPLETED_FAIL || tempI == HighLevelActionUnit.INVALID_INDEX) {
		    CUAction ca = new CUAction();
		    ca.status = -1;
		    return ca;
		}		
		else {
		    return nextHighActUnit.getCUActionAt(tempI);
		}
	    }
	}
	
	return null;
    }
    
    private boolean updateDBObjectPose() {
	
	return true;
    }
    
    private ArrayList<Pose2D> calculateScanPositions(SRSSpatialInfo furnitureInfo) throws RosException {
	ArrayList<Pose2D> posList = new ArrayList<Pose2D>();
	ServiceClient<SymbolGroundingExploreBasePose.Request, SymbolGroundingExploreBasePose.Response, SymbolGroundingExploreBasePose> sc =
	    KnowledgeEngine.nodeHandle.serviceClient("symbol_grounding_explore_base_pose" , new SymbolGroundingExploreBasePose(), false);
	
	SymbolGroundingExploreBasePose.Request rq = new SymbolGroundingExploreBasePose.Request();
	rq.parent_obj_geometry = furnitureInfo;

	SymbolGroundingExploreBasePose.Response res = sc.call(rq);
	posList = res.explore_base_pose_list;
	sc.shutdown();
	return posList;
    }

    private SRSSpatialInfo getFurnitureGeometryOf(Individual workspace) {
	SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
	com.hp.hpl.jena.rdf.model.Statement stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "xCoord",  workspace);
	spatialInfo.pose.position.x = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "yCoord",  workspace);
	spatialInfo.pose.position.y = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "zCoord",  workspace);
	spatialInfo.pose.position.z = stm.getFloat();
	
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "widthOfObject",  workspace);
	spatialInfo.w = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "heightOfObject",  workspace);
	spatialInfo.h = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "lengthOfObject",  workspace);
	spatialInfo.l = stm.getFloat();
	
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qu",  workspace);
	spatialInfo.pose.orientation.w = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qx",  workspace);
	spatialInfo.pose.orientation.x = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qy",  workspace);
	spatialInfo.pose.orientation.y = stm.getFloat();
	stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qz",  workspace);
	spatialInfo.pose.orientation.z = stm.getFloat();
	return spatialInfo;
    }

    @Override
    public boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery) {
	return false;
    }

    @Override
    public boolean isEmpty() {
	try {
	    if(allSubSeqs.size() == 0) {
		return true;
	    }
	}
	catch(NullPointerException e) {
	    System.out.println(e.getMessage() + "\n" + e.toString());
	    return true;
	}
	return false;
    }

    private ArrayList<Individual> workspaces = new ArrayList<Individual>();
    private int currentSubAction;
    private Pose recentDetectedObject;    // required by MoveAndGraspActionUnit
    private String lastActionType;        // used to handle feedback from last action executed
    private String userPose;
    private HighLevelActionUnit lastStepActUnit;
}
