package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
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

import ros.*;
import ros.communication.*;

public class GetObjectTask extends org.srs.srs_knowledge.task.Task
{
    public GetObjectTask(String taskType, String targetContent, Pose2D userPose, OntologyDB onto, OntoQueryUtil ontoQueryUtil, NodeHandle n) 
    {
	if (onto != null) {
	    System.out.println("SET ONTOLOGY DB");
	    this.ontoDB = onto;
	} 
	else {
	    System.out.println("NULL ---- SET ONTOLOGY DB");
	    this.ontoDB = new OntologyDB();
	}

	if (ontoQueryUtil != null) {
	    this.ontoQueryUtil = ontoQueryUtil;
	} 
	else {
	    System.out.println("NULL ---- SET ONTOLOGY DB");
	    this.ontoQueryUtil = new OntoQueryUtil("","");
	}
	
	this.nodeHandle = n;
	// this.init(taskType, targetContent, userPose);
	this.initTask(targetContent, userPose);
    }

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
	    workspaces = OntoQueryUtil.getWorkspaceOfObject(this.targetContent, this.ontoQueryUtil.getObjectNameSpace(), this.ontoQueryUtil.getGlobalNameSpace(), this.ontoDB);

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
		int ni = highAct.getNextCUActionIndex(stateLastAction); 
		System.out.println("=========>>>>  " + ni);
		switch(ni) {
		case HighLevelActionUnit.COMPLETED_SUCCESS:
		    //CUAction ca = new CUAction();
		    System.out.println(".COMPLETED_SUCCESS");
		    
		    //ca.status = 1;
		    return handleSuccessMessage();
		    
		case HighLevelActionUnit.COMPLETED_FAIL:
		    // The whole task finished (failure). Should move to a HighLevelActionUnit in subActSeq of finsihing
		    System.out.println(".COMPLETED_FAIL");
		    //currentSubAction++;
		    return handleFailedMessage();
		case HighLevelActionUnit.INVALID_INDEX:
		    // The whole task finished failure. Should move to a HighLevelActionUnit in subActSeq of finsihing
		    System.out.println("INVALID_INDEX");
		    //currentSubAction++;
		    return handleFailedMessage();
		default: 

		    System.out.println(highAct.getActionType());
		    if(highAct.ifParametersSet()) {
			System.out.println("OK. Parameters set");
		    }

		    if(highAct.getActionType().equals("MoveAndGrasp") && highAct.ifParametersSet()) {
			System.out.println("OK  default");
			// call symbol grounding to get parameters for the MoveAndGrasp action
			try {
			    SRSFurnitureGeometry furGeo = getFurnitureGeometryOf(workspaces.get(currentSubAction));
			    // TODO: recentDetectedObject should be updated accordingly when the MoveAndDetection action finished successfully
			    Pose2D pos = calculateGraspPosition(furGeo, recentDetectedObject);
			    System.out.println("OK  default");
			    
			    ArrayList<String> posInArray = new ArrayList<String>();
			    posInArray.add("move");
			    posInArray.add(Double.toString(pos.x));
			    posInArray.add(Double.toString(pos.y));
			    posInArray.add(Double.toString(pos.theta));
			    if(highAct.setParameters(posInArray)) {
				System.out.println("MoveAndGrasp action move action parameters are set...  " + posInArray.toString());
			    }
			    else {
				System.out.println("MoveAndGrasp action move action parameters are not set...  " + posInArray.toString());
				currentSubAction++;
				return handleFailedMessage();		
			    }
			} 
			catch(RosException e) {
			    System.out.println(e.toString());
			}
			catch(Exception e) {
			    System.out.println(e.toString());
			}
			//private Pose2D calculateGraspPosition(SRSFurnitureGeometry furnitureInfo, Pose targetPose) throws RosException {
		    }

		    
		    ca = highAct.getNextCUAction(ni);
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
    
    /**
     * @param feedback: array in the order of: action-type-"detect", x, y, z, x, y, z, w, "object class name"-e.g. "MilkBox" (length 9) 
     */
    private Pose convertGenericFeedbackToPose(ArrayList<String> feedback) {
	Pose pos = new Pose();
	// check if feedback is for the last action issued
	
	if(!feedback.get(0).equals(lastActionType)) {
	    throw new IllegalArgumentException("Incompatible type");
	}
	
	pos.position.x = Integer.valueOf(feedback.get(1));
	pos.position.y = Integer.valueOf(feedback.get(2));
	pos.position.z = Integer.valueOf(feedback.get(3));
	pos.orientation.x = Integer.valueOf(feedback.get(4));	    
	pos.orientation.y = Integer.valueOf(feedback.get(5));	    
	pos.orientation.z = Integer.valueOf(feedback.get(6));	    
	pos.orientation.w = Integer.valueOf(feedback.get(7));
	return pos;
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
		CUAction ca = nextHighActUnit.getNextCUAction(tempI);
		if(ca == null) {
		    System.out.println("CUACTION IS NULL.......");
		}
		return ca;
	    }
	}
	
	return null;
    }
    
    private CUAction handleSuccessMessage() {
	// TODO: 
	HighLevelActionSequence currentHLActSeq = allSubSeqs.get(currentSubAction);
	
	if(currentHLActSeq.hasNextHighLevelActionUnit()) {
	    HighLevelActionUnit nextHighActUnit = currentHLActSeq.getNextHighLevelActionUnit();
	    
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
		    return nextHighActUnit.getNextCUAction(tempI);
		}
	    }
	}
	return null;
    }
    
    private HighLevelActionSequence createSubSequenceForSingleWorkspace(Individual workspace) throws RosException, Exception {
	HighLevelActionSequence actionList = new HighLevelActionSequence();
	
	// create MoveAndDetectionActionUnit
	SRSFurnitureGeometry spatialInfo = new SRSFurnitureGeometry();
	com.hp.hpl.jena.rdf.model.Statement stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "xCoord",  workspace);
	
	spatialInfo.pose.position.x = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "yCoord",  workspace);
	spatialInfo.pose.position.y = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "zCoord",  workspace);
	spatialInfo.pose.position.z = stm.getFloat();
	
	
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "widthOfObject",  workspace);
	spatialInfo.w = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "heightOfObject",  workspace);
	spatialInfo.h = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "lengthOfObject",  workspace);
	spatialInfo.l = stm.getFloat();
	
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qu",  workspace);
	spatialInfo.pose.orientation.w = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qx",  workspace);
	spatialInfo.pose.orientation.x = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qy",  workspace);
	spatialInfo.pose.orientation.y = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qz",  workspace);
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
	MoveAndDetectionActionUnit mdAction = new MoveAndDetectionActionUnit(posList, "MilkBox", 1);
	
	// create MoveAndGraspActionUnit
	MoveAndGraspActionUnit mgAction = new MoveAndGraspActionUnit(null, "MilkBox", 1, "side");

	// create PutOnTrayActionUnit
	PutOnTrayActionUnit trayAction = new PutOnTrayActionUnit("side");

	// FoldArmActionUnit
	FoldArmActionUnit foldArmAction = new FoldArmActionUnit();

	// create BackToUserActionUnit
	

	actionList.appendHighLevelAction(mdAction);
	actionList.appendHighLevelAction(mgAction);
	actionList.appendHighLevelAction(trayAction);
	actionList.appendHighLevelAction(foldArmAction);
		
	System.out.println("ActionList size is " + actionList.getSizeOfHighLevelActionList());
	return actionList;
    }
    

    private ArrayList<Pose2D> calculateScanPositions(SRSFurnitureGeometry furnitureInfo) throws RosException {
	ArrayList<Pose2D> posList = new ArrayList<Pose2D>();
	ServiceClient<SymbolGroundingScanBasePose.Request, SymbolGroundingScanBasePose.Response, SymbolGroundingScanBasePose> sc =
	    nodeHandle.serviceClient("symbol_grounding_scan_base_pose" , new SymbolGroundingScanBasePose(), false);
	
	SymbolGroundingScanBasePose.Request rq = new SymbolGroundingScanBasePose.Request();
	rq.parent_obj_geometry = furnitureInfo;

	SymbolGroundingScanBasePose.Response res = sc.call(rq);
	posList = res.scan_base_pose_list;
	sc.shutdown();
	return posList;
    }

    private Pose2D calculateGraspPosition(SRSFurnitureGeometry furnitureInfo, Pose targetPose) throws RosException {
	Pose2D pos = new Pose2D();

	ServiceClient<SymbolGroundingGraspBasePoseExperimental.Request, SymbolGroundingGraspBasePoseExperimental.Response, SymbolGroundingGraspBasePoseExperimental> sc =
	    nodeHandle.serviceClient("symbol_grounding_grasp_base_pose" , new SymbolGroundingGraspBasePoseExperimental(), false);
	
	SymbolGroundingGraspBasePoseExperimental.Request rq = new SymbolGroundingGraspBasePoseExperimental.Request();
	rq.parent_obj_geometry = furnitureInfo;
	rq.target_obj_pose = targetPose;

	SymbolGroundingGraspBasePoseExperimental.Response res = sc.call(rq);
	boolean obstacleCheck = res.obstacle_check;
	double reach = res.reach;
	pos = res.grasp_base_pose;

	sc.shutdown();
	return pos;
    }

    private SRSFurnitureGeometry getFurnitureGeometryOf(Individual workspace) {
	SRSFurnitureGeometry spatialInfo = new SRSFurnitureGeometry();
	com.hp.hpl.jena.rdf.model.Statement stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "xCoord",  workspace);
	spatialInfo.pose.position.x = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "yCoord",  workspace);
	spatialInfo.pose.position.y = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "zCoord",  workspace);
	spatialInfo.pose.position.z = stm.getFloat();
			 
	
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "widthOfObject",  workspace);
	spatialInfo.w = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "heightOfObject",  workspace);
	spatialInfo.h = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "lengthOfObject",  workspace);
	spatialInfo.l = stm.getFloat();
	
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qu",  workspace);
	spatialInfo.pose.orientation.w = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qx",  workspace);
	spatialInfo.pose.orientation.x = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qy",  workspace);
	spatialInfo.pose.orientation.y = stm.getFloat();
	stm = ontoDB.getPropertyOf(ontoQueryUtil.getGlobalNameSpace(), "qz",  workspace);
	spatialInfo.pose.orientation.z = stm.getFloat();
	return spatialInfo;
    }

    private void initTask(String targetContent, Pose2D userPose) {
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
}
