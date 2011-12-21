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

public class GetObjectTask extends org.srs.srs_knowledge.task.Task
{
    public GetObjectTask(String taskType, String targetContent, Pose2D userPose, OntologyDB onto, OntoQueryUtil ontoQueryUtil) 
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
	try {
	    /*
	    System.out.println(this.targetContent);
	    System.out.println(this.ontoQueryUtil.getObjectNameSpace());
	    System.out.println(this.ontoQueryUtil.getGlobalNameSpace());
	    */
	    //ArrayList<String> workspaces = OntoQueryUtil.getWorkspaceNamesOfObject(this.targetContent, this.ontoQueryUtil.getObjectNameSpace(), this.ontoQueryUtil.getGlobalNameSpace(), this.ontoDB);
	    ArrayList<Individual> workspaces = OntoQueryUtil.getWorkspaceOfObject(this.targetContent, this.ontoQueryUtil.getObjectNameSpace(), this.ontoQueryUtil.getGlobalNameSpace(), this.ontoDB);
	    
	    for(Individual u : workspaces) {
		System.out.println(u.getLocalName());
		createSubSequenceForSingleWorkspace(u);

	    }



	}
	catch(Exception e) {
	    System.out.println(e.getMessage() + "\n" + e.toString());
	    return false;
	}

	return true;
    }
    
    private SubActionSequence createSubSequenceForSingleWorkspace(Individual workspace) {
	SubActionSequence actionList = new SubActionSequence();

	

	return actionList;
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
}