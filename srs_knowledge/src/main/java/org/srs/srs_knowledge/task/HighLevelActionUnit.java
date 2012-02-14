package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import org.srs.srs_knowledge.knowledge_engine.*;
import org.srs.srs_knowledge.task.Task;
import java.util.HashMap;

/**
 * An ActionUnit is a container of GenericAction. 
 * Unit does not have to be containing only one action. e.g. an action of detection an object on a table can contain a few steps, move to pos1, detect, move to pos2, detect, move to pos3, detect, etc. 
 * But generally, it may just contain one genericAction, e.g. pub object on tray, move to a position
 */

public abstract class HighLevelActionUnit {    
    public abstract String getActionType();
    public static final int COMPLETED_SUCCESS = -10;
    public static final int COMPLETED_FAIL = -11;
    public static final int INVALID_INDEX = -100;

    public int getNumOfActions() {
	return actionUnits.size();
    }

    public int getCurrentCUActionIndex() {
	
	return currentActionInd;
    }

    public int getNextCUActionIndex(boolean statusLastStep) {
	
	if(currentActionInd == -1) {
	    return 0;
	}
	//currentActionInd++;

	if ( currentActionInd >= 0 && currentActionInd < actionUnits.size() ) {
	    if(statusLastStep) {
		return nextActionMapIfSuccess[currentActionInd];
	    }
	    else {
		return nextActionMapIfFail[currentActionInd];
	    }
	}
	else {
	    return INVALID_INDEX;
	}
    }

    public CUAction getCUActionAt(int ind) {
	
	currentActionInd = ind;
	if (ind < 0) {
	    System.out.println("Invalid index " + ind);
	    return null;
	}
	
	CUAction ca = new CUAction(); 

	if(ind == COMPLETED_FAIL) {
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("finish_fail");
	    
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    ca.status = -1;
	    
	    return ca;
	}
	else if (ind == COMPLETED_SUCCESS){
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("finish_success");
	    
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    ca.status = 1;
	    
	    return ca;
	}
	else if (ind == INVALID_INDEX) {
	    GenericAction genericAction = new GenericAction();
	    genericAction.actionInfo.add("no_action");
	    
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    ca.status = -1;
	    return ca;
	}

	GenericAction genericAction = actionUnits.get(ind);
	
	ca.generic = genericAction;
	
	ca.actionType = "generic";
	return ca;

    }

    public boolean addFeedback(String key, ActionFeedback fb) {
	if(fb == null) {
	    return false;
	}
	feedbacks.put(key, fb);
	return true;
    }

    public ActionFeedback getFeedback(String key) {
	return feedbacks.get(key);
    }

    // a not very safe, but flexible way to assign parameters, using arraylist<string> 
    public abstract boolean setParameters(ArrayList<String> para);
    public abstract boolean ifParametersSet();

    protected String actionType = "";
    protected ArrayList<GenericAction> actionUnits = new ArrayList<GenericAction>();
    protected int[] nextActionMapIfFail;
    protected int[] nextActionMapIfSuccess;
    protected HashMap<String, ActionFeedback> feedbacks;

    protected int currentActionInd = -1;
    protected boolean ifParametersSet;
}