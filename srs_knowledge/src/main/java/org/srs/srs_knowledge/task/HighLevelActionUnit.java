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
import org.srs.srs_knowledge.task.Task;

/**
 * An ActionUnit is a container of GenericAction. 
 * Unit does not have to be containing only one action. e.g. an action of detection an object on a table can contain a few steps, move to pos1, detect, move to pos2, detect, move to pos3, detect, etc. 
 * But generally, it may just contain one genericAction, e.g. pub object on tray, move to a position
 */

public abstract class HighLevelActionUnit {    
    public abstract String getActionType();
   
    public int getNumOfActions() {
	return actionUnits.size();
    }

    public void addNewGenericAction(GenericAction gact) {
	actionUnits.add(gact);
    }

    protected String actionType = "";
    protected ArrayList<GenericAction> actionUnits = new ArrayList<GenericAction>();
    protected int[] nextActionMapIfFail;
}