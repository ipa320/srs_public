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
//import org.srs.srs_knowledge.utils.*;
import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;
import org.srs.srs_knowledge.task.*;

import ros.pkg.srs_symbolic_grounding.srv.*;
import ros.pkg.srs_symbolic_grounding.msg.*;

import ros.*;
import ros.communication.*;

import org.srs.srs_knowledge.utils.*;

public class GetObjectTask extends org.srs.srs_knowledge.task.Task
{
    public GetObjectTask(String taskType, String targetContent) 
    {	
	// this.init(taskType, targetContent, userPose);
	this.initTask(targetContent);
    }

    @Override
    protected boolean constructTask() {
	return createGetObjectTask();
    }
    
    private boolean createGetObjectTask() {
	/*
	// query for tables
	// move to tables (near -- use grounding)
	// detect milk
	// grap milk
	// back to user
	*/
	System.out.println("Create New GET OBJECT Task --- ");

	try {
	    //ArrayList<String> workspaces = OntoQueryUtil.getWorkspaceNamesOfObject(this.targetContent, this.ontoQueryUtil.getObjectNameSpace(), this.ontoQueryUtil.getGlobalNameSpace(), this.ontoDB);
	    //	    workspaces = OntoQueryUtil.getWorkspaceOfObject(this.targetContent, OntoQueryUtilthis.ontoQueryUtil.getObjectNameSpace(), this.ontoQueryUtil.getGlobalNameSpace(), KnowledgeEngine.ontoDB);
	    workspaces = OntoQueryUtil.getWorkspaceOfObject(this.targetContent, OntoQueryUtil.ObjectNameSpace, OntoQueryUtil.GlobalNameSpace, KnowledgeEngine.ontoDB);
	}
	catch(Exception e) {
	    System.out.println(e.getMessage() + "\n" + e.toString());
	    return false;
	}
	
	for(Individual u : workspaces) {
	    System.out.println(u.getLocalName());
	    try{
		System.out.println("Created HLActionSeq ");
		HighLevelActionSequence subSeq = createSubSequenceForSingleWorkspace(u);
		allSubSeqs.add(subSeq);
	    }
	    catch(RosException e) {
		System.out.println("ROSEXCEPTION -- when calling symbolic grounding for scanning positions.  \t" + e.getMessage() + "\t" + e.toString());
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
	SRSFurnitureGeometry spatialInfo = new SRSFurnitureGeometry();
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
	MoveAndDetectionActionUnit mdAction = new MoveAndDetectionActionUnit(posList, targetContent, 9);
	
	// create MoveAndGraspActionUnit
	MoveAndGraspActionUnit mgAction = new MoveAndGraspActionUnit(null, targetContent, 9, "side");

	// create PutOnTrayActionUnit
	PutOnTrayActionUnit trayAction = new PutOnTrayActionUnit("side");

	// FoldArmActionUnit
	//FoldArmActionUnit foldArmAction = new FoldArmActionUnit();

	//create BackToUserActionUnit
	
	//Pose2D posUser = OntoQueryUtil.parsePose2D(userPose);
	//MoveActionUnit mau = new MoveActionUnit(posUser);
	
	// create FinishActionUnit
	FinishActionUnit fau = new FinishActionUnit(true);
	//MoveActionUnit mau1 = new MoveActionUnit(OntoQueryUtil.parsePose2D("home"));
	//actionList.appendHighLevelAction(mau1);

	actionList.appendHighLevelAction(mdAction);
	actionList.appendHighLevelAction(mgAction);
	actionList.appendHighLevelAction(trayAction);
	//actionList.appendHighLevelAction(foldArmAction);
	//actionList.appendHighLevelAction(mau);
	actionList.appendHighLevelAction(fau);
		
	System.out.println("ActionList size is " + actionList.getSizeOfHighLevelActionList());
	return actionList;
    }
    
    @Override
    public CUAction getNextCUAction(boolean stateLastAction, ArrayList<String> feedback) {
     
	System.out.println("===> Get Next CUACTION -- from GetObjectTask.java");
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
		updateDBObjectPose();
				
		int ni = highAct.getNextCUActionIndex(stateLastAction); 
		System.out.println("=========>>>>  " + ni);
		switch(ni) {
		case HighLevelActionUnit.COMPLETED_SUCCESS:
		    System.out.println("COMPLETED_SUCCESS");
		    lastStepActUnit = highAct;
		    	    
		    //HighLevelActionUnit curActUnit = subActSeq.getCurrentHighLevelActionUnit();
		    // Pose2D calculateGraspPosFromFB(ActionFeedback fb) {
		    // private HighLevelActionUnit setParametersGraspPos(Pose2D pos, HighLevelActionUnit mgActUnit)
		    //curActUnit.setParameters(basePos);
		    CUAction retact = handleSuccessMessage(new ActionFeedback(feedback)); 
		    return retact; 
		case HighLevelActionUnit.COMPLETED_FAIL:
		    // The whole task finished (failure). 
		    lastStepActUnit = null;
		    System.out.println("COMPLETED_FAIL");
		    return handleFailedMessage();
		case HighLevelActionUnit.INVALID_INDEX:
		    // The whole task finished failure. Should move to a HighLevelActionUnit in subActSeq of finsihing
		    lastStepActUnit = null;
		    System.out.println("INVALID_INDEX");
		    //currentSubAction++;
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
		    lastActionType = ca.generic.actionInfo.get(0);
		    
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

	// 
	updateTargetOfFailedAct();

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

    private void updateTargetOfSucceededAct(ActionFeedback fb) {
	HighLevelActionSequence currentHLActSeq = allSubSeqs.get(currentSubAction);
	HighLevelActionUnit currentActUnit = currentHLActSeq.getCurrentHighLevelActionUnit();
	if(currentActUnit.getActionType().equals("MoveAndDetection")) {
	    this.recentDetectedObject = ActionFeedback.toPose(fb);
	    BoundingBoxDim bbDim = InformationRetrieval.retrieveBoundingBoxInfo(OntoQueryUtil.GlobalNameSpace + this.targetContent);
	    ros.pkg.srs_knowledge.msg.SRSSpatialInfo spaObj = new ros.pkg.srs_knowledge.msg.SRSSpatialInfo();
	    spaObj.l = bbDim.l;
	    spaObj.h = bbDim.h;
	    spaObj.w = bbDim.w;

	    spaObj.pose = this.recentDetectedObject;

	    // update the knowledge (post-processing)
	    
	    // if there exists one same object on the same workspace, update it --- simple solution
	    //String ws = SpatialCalculator.workspaceHolding(spaObj);
	    
	    String neighbour = SpatialCalculator.nearestObject(spaObj.pose, OntoQueryUtil.GlobalNameSpace + this.targetContent);
	    if(!neighbour.trim().equals("")) {
		System.out.println("Found neighbour of " + neighbour);
		// update its pose
		try{
		    OntoQueryUtil.updatePoseOfObject(spaObj.pose, OntoQueryUtil.GlobalNameSpace, neighbour.trim());
		    OntoQueryUtil.computeOnSpatialRelation();
		}
		catch(Exception e) {
		    System.out.println(e.getMessage());
		}
	    }
	    
	    // if there does not exist such an object, then insert a new one
	    // bounding box can be obtained from HHDB
	    // TODO
	    
	}
	else if(currentActUnit.getActionType().equals("MoveAndGrasp")) {
	    // look for the object at the pose 
	    
	    // update its relationship with the Robot, and remove its pose information
	    
	}
	else if(currentActUnit.getActionType().equals("PutOnTray")) {
	    // look for the object at the pose 
	    
	    // update its relationship with the Robot, and remove its pose information
	    
	}
    }

    private void updateTargetOfFailedAct() {
	HighLevelActionSequence currentHLActSeq = allSubSeqs.get(currentSubAction);
	HighLevelActionUnit currentActUnit = currentHLActSeq.getCurrentHighLevelActionUnit();
	if(currentActUnit.getActionType().equals("MoveAndDetection")) {
	    //this.recentDetectedObject = ActionFeedback.toPose(fb);	 
	    	    
	    // update the knowledge (post-processing)
	    
	    // if there exists one same object on the same workspace, remove it --- simple solution
	    
	    // if there are more objects of the smae type, remove one of them (does not matter which one, as they are identical (not distinguiable).. better to use the closest one )
	    
	    // if there does not exist such an object, then do nothing

	}
	else if(currentActUnit.getActionType().equals("MoveAndGrasp")) {
	    // do nothing
	    
	}
    }
    
    private CUAction handleSuccessMessage(ActionFeedback fb) {
	// TODO: 
	updateTargetOfSucceededAct(fb);
    
	HighLevelActionSequence currentHLActSeq = allSubSeqs.get(currentSubAction);

	if(currentHLActSeq.hasNextHighLevelActionUnit()) {
	    HighLevelActionUnit nextHighActUnit = currentHLActSeq.getNextHighLevelActionUnit();
	    // set feedback? 
	    if(nextHighActUnit.getActionType().equals("MoveAndGrasp") && !nextHighActUnit.ifParametersSet()) {

		Pose2D posBase = calculateGraspPosFromFB(fb);
		if(posBase == null) {
		    return handleFailedMessage();
		}

		ArrayList<String> basePos = constructArrayFromPose2D(posBase);
		if(!nextHighActUnit.setParameters(basePos)) {
		    //currentSubAction++;
		    return handleFailedMessage();		
		}
	    }

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
    
    private Pose2D calculateGraspPosFromFB(ActionFeedback fb) {
	//calculateGraspPosition(SRSFurnitureGeometry furnitureInfo, Pose targetPose)
	// call symbol grounding to get parameters for the MoveAndGrasp action
	try {
	    SRSFurnitureGeometry furGeo = getFurnitureGeometryOf(workspaces.get(currentSubAction));
	    // TODO: recentDetectedObject should be updated accordingly when the MoveAndDetection action finished successfully
	    recentDetectedObject = ActionFeedback.toPose(fb);
	    if(recentDetectedObject == null) {
		return null;
	    }

	    Pose2D pos = calculateGraspPosition(furGeo, recentDetectedObject);
	    return pos;
	}
	catch (Exception e) {
	    System.out.println(e.getMessage() + " ++ " + e.toString());
	    return null;
	}
	
    }
    
    private ArrayList<String> constructArrayFromPose2D(Pose2D pos) {
	try {
	    ArrayList<String> l = new ArrayList<String>();
	    l.add("move");
	    l.add(Double.toString(pos.x));
	    l.add(Double.toString(pos.y));
	    l.add(Double.toString(pos.theta));
	    return l;
	}
	catch(Exception e) {
	    System.out.println(e.toString());
	    return null;
	}
    }
     
    /**
     * @param feedback: array in the order of: action-type-"detect", x, y, z, x, y, z, w, "object class name"-e.g. "MilkBox" (length 9) 
     */
    private Pose convertGenericFeedbackToPose(ArrayList<String> feedback) {
	Pose pos = new Pose();
	// check if feedback is for the last action issued
	
	if(!feedback.get(0).equals(lastActionType)) {
	    throw new IllegalArgumentException("Incompatible type");
	}
	
	pos.position.x = Integer.valueOf(feedback.get(2));
	pos.position.y = Integer.valueOf(feedback.get(3));
	pos.position.z = Integer.valueOf(feedback.get(4));
	pos.orientation.x = Integer.valueOf(feedback.get(5));	    
	pos.orientation.y = Integer.valueOf(feedback.get(6));	    
	pos.orientation.z = Integer.valueOf(feedback.get(7));	    
	pos.orientation.w = Integer.valueOf(feedback.get(8));
	return pos;
    }

    
    private ArrayList<Pose2D> calculateScanPositions(SRSFurnitureGeometry furnitureInfo) throws RosException {
	ArrayList<Pose2D> posList = new ArrayList<Pose2D>();
	ServiceClient<SymbolGroundingScanBasePose.Request, SymbolGroundingScanBasePose.Response, SymbolGroundingScanBasePose> sc =
	    KnowledgeEngine.nodeHandle.serviceClient("symbol_grounding_scan_base_pose" , new SymbolGroundingScanBasePose(), false);
	
	SymbolGroundingScanBasePose.Request rq = new SymbolGroundingScanBasePose.Request();
	rq.parent_obj_geometry = furnitureInfo;

	SymbolGroundingScanBasePose.Response res = sc.call(rq);
	posList = res.scan_base_pose_list;
	sc.shutdown();
	return posList;
    }

    private Pose2D calculateGraspPosition(SRSFurnitureGeometry furnitureInfo, Pose targetPose) throws RosException {
	Pose2D pos = new Pose2D();
	
	ServiceClient<SymbolGroundingGraspBasePoseExperimental.Request, SymbolGroundingGraspBasePoseExperimental.Response, SymbolGroundingGraspBasePoseExperimental> sc = KnowledgeEngine.nodeHandle.serviceClient("symbol_grounding_grasp_base_pose_experimental" , new SymbolGroundingGraspBasePoseExperimental(), false);
	
	SymbolGroundingGraspBasePoseExperimental.Request rq = new SymbolGroundingGraspBasePoseExperimental.Request();
	rq.parent_obj_geometry = furnitureInfo;
	rq.target_obj_pose = targetPose;

	SymbolGroundingGraspBasePoseExperimental.Response res = sc.call(rq);
	boolean obstacleCheck = res.obstacle_check;
	double reach = res.reach;
	pos = res.grasp_base_pose;

	sc.shutdown();
	if(obstacleCheck) {
	    return null;
	}
	return pos;
    }

    private SRSFurnitureGeometry getFurnitureGeometryOf(Individual workspace) {
	SRSFurnitureGeometry spatialInfo = new SRSFurnitureGeometry();
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

    private void initTask(String targetContent) {
	acts = new ArrayList<ActionTuple>();
	
	setTaskTarget(targetContent);
	System.out.println("TASK.JAVA: Created CurrentTask " + "get "
			   + targetContent);
	constructTask();
    }   

    public boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery) {
	return false;
    }

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
    //private String userPose;
    private HighLevelActionUnit lastStepActUnit;
}
