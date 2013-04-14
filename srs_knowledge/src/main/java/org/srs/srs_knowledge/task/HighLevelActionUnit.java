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
	    //genericAction.actionInfo.add("finish_fail");
	    
	    genericAction.jsonActionInfo = SRSJSONParser.encodeCustomAction("finish_fail", null);
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    ca.status = -1;
	    
	    return ca;
	}
	else if (ind == COMPLETED_SUCCESS){
	    GenericAction genericAction = new GenericAction();
	    //genericAction.actionInfo.add("finish_success");
	    genericAction.jsonActionInfo = SRSJSONParser.encodeCustomAction("finish_success", null);
	    
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    ca.status = 1;
	    
	    return ca;
	}
	else if (ind == INVALID_INDEX) {
	    GenericAction genericAction = new GenericAction();
	    //genericAction.actionInfo.add("no_action");
	    genericAction.jsonActionInfo = SRSJSONParser.encodeCustomAction("no_action", null);
		    
	    ca.generic = genericAction;
	    ca.actionType = "generic";
	    
	    ca.status = -1;
	    return ca;
	}

	GenericAction genericAction = new GenericAction();
	try {
	    genericAction = actionUnits.get(ind);
	}
	catch(IndexOutOfBoundsException e) {
	    return null;
	}

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
    //public abstract boolean setParameters(ArrayList<String> para);
    public abstract boolean ifParametersSet();

    public boolean setParameters(String action, String para, String reservedParam) {
	return this.ifParametersSet;
    }

    protected String actionType = "";
    protected ArrayList<GenericAction> actionUnits = new ArrayList<GenericAction>();
    protected int[] nextActionMapIfFail;
    protected int[] nextActionMapIfSuccess;
    protected HashMap<String, ActionFeedback> feedbacks;

    protected int currentActionInd = -1;
    protected boolean ifParametersSet;
}