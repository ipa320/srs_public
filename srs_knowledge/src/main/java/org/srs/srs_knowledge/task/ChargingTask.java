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

import org.srs.srs_knowledge.task.Task;

public class ChargingTask extends org.srs.srs_knowledge.task.Task
{
    public ChargingTask(OntologyDB onto) 
    {
	if (onto != null) {
	    System.out.println("SET ONTOLOGY DB");
	    this.ontoDB = onto;
	}
	else {
	    System.out.println("NULL ---- SET ONTOLOGY DB");
	    this.ontoDB = new OntologyDB();
	}
	
	this.initTask();
	setTaskType(TaskType.MOVETO_LOCATION);
    }
    
    private void initTask() {
	acts = new ArrayList<ActionTuple>();
	
	//setTaskTarget(targetContent);
	System.out.println("TASK.JAVA: Created CurrentTask " + "Charging ");
	constructTask();
    }
    
    protected boolean constructTask() {
	return createSimpleChargingTask();
    }
    
    private boolean createSimpleChargingTask() {
	// boolean addNewActionTuple(ActionTuple act)
	ActionTuple act = new ActionTuple();
	
	CUAction ca = new CUAction();
	GenericAction genericAction = new GenericAction();
	ca.actionType = "generic";
	
	double x = 0;
	double y = 0;
	double theta = 0;
	
	targetContent = "ChargingStation0";
	setTaskTarget(targetContent);
	String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
	    + "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
	    + "PREFIX ipa-kitchen-map: <http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#>\n";
	String queryString = "SELECT ?x ?y ?theta WHERE { "
	    + "ipa-kitchen-map:" + targetContent
	    + " srs:xCoordinate ?x . " + "ipa-kitchen-map:"
	    + targetContent + " srs:yCoordinate ?y . "
	    + "ipa-kitchen-map:" + targetContent
	    + " srs:orientationTheta ?theta .}";
	
	if (this.ontoDB == null) {
	    System.out.println("Ontology Database is NULL");
	    return false;
	}
	
	try {
	    ArrayList<QuerySolution> rset = ontoDB.executeQueryRaw(prefix
								   + queryString);
	    if (rset.size() == 0) {
		System.out.println("ERROR: No move target found from database");
		return false;
	    } else if (rset.size() == 1) {
		System.out
		    .println("INFO: OK info retrieved from DB... ");
		QuerySolution qs = rset.get(0);
		x = qs.getLiteral("x").getFloat();
		y = qs.getLiteral("y").getFloat();
		theta = qs.getLiteral("theta").getFloat();
		System.out.println("x is " + x + ". y is  " + y
				   + ". theta is " + theta);
	    } else {
		System.out.println("WARNING: Multiple options... ");
		QuerySolution qs = rset.get(0);
		x = qs.getLiteral("x").getFloat();
		y = qs.getLiteral("y").getFloat();
		theta = qs.getLiteral("theta").getFloat();
		System.out.println("x is " + x + ". y is  " + y
				   + ". theta is " + theta);
	    }
	} catch (Exception e) {
	    System.out.println("Exception -->  " + e.getMessage());
	    return false;
	}
	// return false;
	
	genericAction.actionInfo.add("charging");
	genericAction.actionInfo.add(Double.toString(x));
	genericAction.actionInfo.add(Double.toString(y));
	genericAction.actionInfo.add(Double.toString(theta));
	
	ca.generic = genericAction;
	
	act.setCUAction(ca);
	act.setActionId(1);
	addNewActionTuple(act);
	
	// add finish action __ success
	
	act = new ActionTuple();
	
	ca = new CUAction();
	genericAction = new GenericAction();
	ca.generic = genericAction;
	
	act.setActionName("finish_success");
	
	ca.status = 1;
	
	act.setCUAction(ca);
	act.setActionId(2);
	act.setParentId(1);
	act.setCondition(true);
	addNewActionTuple(act);
	
	// add finish action __ fail
	
	act = new ActionTuple();
	
	ca = new CUAction();
	genericAction = new GenericAction();
	
	ca.generic = genericAction;
	
	act.setActionName("finish_fail");
	
	ca.status = -1;
	
	act.setCUAction(ca);
	act.setActionId(3);
	act.setParentId(1);
	act.setCondition(false);
	addNewActionTuple(act);
	
	System.out.println("number of actions: " + acts.size());
	return true;
    }
    
    public boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery) {
	return false;
    }
}