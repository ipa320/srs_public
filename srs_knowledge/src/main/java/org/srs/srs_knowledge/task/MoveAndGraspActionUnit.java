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

    public MoveAndGraspActionUnit(Pose2D position, String objectClassName, int houseHoldId, String graspConfig) {
	    GenericAction ga = new GenericAction();
	    ga.actionInfo.add("move");
	    if(position != null) {
		ga.actionInfo.add(Double.toString(position.x));
		ga.actionInfo.add(Double.toString(position.y));
		ga.actionInfo.add(Double.toString(position.theta));
		ifBasePoseSet = true;
	    }
	    else {
		ga.actionInfo.add("");
		ga.actionInfo.add("");
		ga.actionInfo.add("");
		ifBasePoseSet = false;
	    }

	    actionUnits.add(ga);

	    GenericAction graspAct = new GenericAction();
	    graspAct.actionInfo.add("grasp");

	    if (objectClassName != null || objectClassName.equals("")) {
		graspAct.actionInfo.add(objectClassName);
		graspAct.actionInfo.add(Integer.toString(houseHoldId));
		ifObjectInfoSet = true;
	    }
	    else {
		graspAct.actionInfo.add("");
		graspAct.actionInfo.add(Integer.toString(houseHoldId));
		ifObjectInfoSet = false;
	    }

	    if (graspConfig != null || graspConfig.equals("")) {
		// side, top etc
		graspAct.actionInfo.add(graspConfig);
		ifObjectInfoSet = true && ifObjectInfoSet;
	    }
	    else {
		graspAct.actionInfo.add("");
		ifObjectInfoSet = false;
	    }



	    /*
	    graspAct.actionInfo.add(Double.toString(objectPose.position.x));
	    graspAct.actionInfo.add(Double.toString(objectPose.position.y));
	    graspAct.actionInfo.add(Double.toString(objectPose.position.z));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.x));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.y));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.z));
	    graspAct.actionInfo.add(Double.toString(objectPose.orientation.w));
	    */

	    actionUnits.add(graspAct);

	    //ifObjectPoseSet = true;
	    ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	    
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

    public int getNextCUActionIndex(boolean statusLastStep) {
	return 0;
    }

    public CUAction getNextCUAction(int ind) {
	return null;
    }
    
    // a not very safe, but flexible way to assign parameters, using arraylist<string> 
    // set robot move target and object pose etc.
    public boolean setParameters(ArrayList<String> para) {
	boolean res = ifParametersSet;
	try {
	    setBasePose(para);
	    setGraspInfo(para);
	    ifParametersSet = true;
	}
	catch(IllegalArgumentException e) {
	    System.out.println(e.getMessage());
	    return false;
	}
	return res;
    }

    private void setBasePose(ArrayList<String> pose) throws IllegalArgumentException {

	GenericAction ga = actionUnits.get(0);

	if( ga.actionInfo.get(0).equals("move") && pose.get(0).equals("move") && pose.size() == ga.actionInfo.size()) {
	    //actionUnits.get(0).clear();
	    
	    GenericAction nga = new GenericAction();
	    nga.actionInfo = pose;
	    /*
	    ga.actionInfo.set(0, "move");
	    ga.actionInfo.set(1, pose.get(1));
	    ga.actionInfo.set(2, pose.get(2));
	    ga.actionInfo.set(3, pose.get(3));
	    */
	    actionUnits.set(0, nga);
	    ifBasePoseSet = true;
	    ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	}
	else {
	    throw new IllegalArgumentException("Wrong format exception -- when setting Base Pose with arrayList");
	}
    }

    // objInfo should be in format as defined in constructor
    private void setGraspInfo(ArrayList<String> objInfo) {
	GenericAction ga = actionUnits.get(1);

	if( ga.actionInfo.get(0).equals("grasp") && objInfo.get(0).equals("grasp") && objInfo.size() == ga.actionInfo.size()) {
	    //actionUnits.get(0).clear();
	    GenericAction nga = new GenericAction();
	    nga.actionInfo = objInfo;
	    //actionUnits.set(1, objInfo);
	    /*
	    GenericAction graspAct = new GenericAction();
	    graspAct.actionInfo.add("grasp");
	    graspAct.actionInfo.add(Integer.toString(houseHoldId));
	    graspAct.actionInfo.add(objectClassName);
	    */
	    
	    actionUnits.set(1, nga);
	    ifObjectInfoSet = true;
	    ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	}
	else {
	    throw new IllegalArgumentException("Wrong format exception -- when setting Object Info with arrayList");
	}
    }

    private boolean setObjectPose(ArrayList<String> objPose) {
	return false;
    }

    public boolean ifParametersSet() {
	ifParametersSet = ifBasePoseSet && ifObjectInfoSet;
	return ifParametersSet;
    }

    /*
    private String actionType;

    private ArrayList<GenericAction> actionUnits = new ArrayList<GenericAction>();
    private int[] nextActionMapIfFail;
    */
    //private boolean ifObjectPoseSet = false;
    private boolean ifBasePoseSet = false;
    private boolean ifObjectInfoSet = false;
    //private boolean ifObjectIDSet = false;
    //private boolean ifObjectNameSet = false;
}