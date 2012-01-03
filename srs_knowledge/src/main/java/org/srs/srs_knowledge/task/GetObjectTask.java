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
import com.hp.hpl.jena.ontology.Individual;
import org.srs.srs_knowledge.task.*;

import ros.pkg.srs_symbolic_grounding.srv.*;
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
    
    /*
    public GetObjectTask(String taskType, String targetContent, Pose2D userPose) 
    {
	this.ontoDB = new OntologyDB();
	
	this.initTask(targetContent, userPose);
	setTaskType(TaskType.GET_OBJECT);
    }
    */

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
	ArrayList<Individual> workspaces;
	try {
	    /*
	    System.out.println(this.targetContent);
	    System.out.println(this.ontoQueryUtil.getObjectNameSpace());
	    System.out.println(this.ontoQueryUtil.getGlobalNameSpace());
	    */
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
    

 public CUAction getNextCUAction(boolean stateLastAction) {
     
     System.out.println("===> Get Next CUACTION -- from GetObjectTask.java");
     CUAction ca = new CUAction();
     if(allSubSeqs.size() == 0 ) {
	 
     }
     if(currentSubAction >= 0 && currentSubAction < allSubSeqs.size()) {
	 // get the current SubActionSequence item
	 HighLevelActionSequence subActSeq = allSubSeqs.get(currentSubAction);

	 // decide if the current SubActionSequence is finished or stuck somewhere? 

	 // if successfully finished, then finished

	 // if stuck (fail), move to the next subActionSequence

	 if(subActSeq.hasNextHighLevelActionUnit()) {
	     
	     HighLevelActionUnit highAct = subActSeq.getNextHighLevelActionUnit();
	     if(highAct != null) {
		 int ni = highAct.getNextCUActionIndex(stateLastAction); 
		 switch(ni) {
		 case HighLevelActionUnit.COMPLETED_SUCCESS:
		     break;
		 default: 
		     return highAct.getNextCUAction(ni);
		 
		     //break;
		 } 

	     }
	     else {
		 return null;
	     }

	 }
	 else {
	 }

	 // or if still pending CUAction is available, return CUAction

     }
     else if (currentSubAction == -1) {
     }

     return ca;
 }

    private HighLevelActionSequence createSubSequenceForSingleWorkspace(Individual workspace) throws RosException, Exception {
	HighLevelActionSequence actionList = new HighLevelActionSequence();

	// create MoveAndDetectionActionUnit
	SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
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
	}
	catch(RosException e) {
	    throw e;
	}

	// TODO:
	MoveAndDetectionActionUnit mdAction = new MoveAndDetectionActionUnit(posList, "MilkBox", 1);
	
	// create MoveAndGraspActionUnit
	MoveAndGraspActionUnit mgAction = new MoveAndGraspActionUnit(null, "MilkBox", 1, null);

	// create PutOnTrayActionUnit
	


	// create BackToUserActionUnit

	actionList.appendHighLevelAction(mdAction);
	actionList.appendHighLevelAction(mgAction);
	
	return actionList;
    }

    private ArrayList<Pose2D> calculateScanPositions(SRSSpatialInfo furnitureInfo) throws RosException {
	ArrayList<Pose2D> posList = new ArrayList<Pose2D>();
	ServiceClient<SymbolGroundingScanBasePose.Request, SymbolGroundingScanBasePose.Response, SymbolGroundingScanBasePose> sc =
	    nodeHandle.serviceClient("symbol_grounding_scan_base_pose" , new SymbolGroundingScanBasePose(), false);
	
	SymbolGroundingScanBasePose.Request rq = new SymbolGroundingScanBasePose.Request();
	SymbolGroundingScanBasePose.Response res = sc.call(rq);
	sc.shutdown();
	return posList;
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

    private int currentSubAction;
}
