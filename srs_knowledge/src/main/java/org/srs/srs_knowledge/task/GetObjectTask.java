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

public class GetObjectTask extends org.srs.srs_knowledge.task.Task
{
    public GetObjectTask(String taskType, String targetContent, Pose2D userPose, OntologyDB onto) 
    {
	if (onto != null) {
	    System.out.println("SET ONTOLOGY DB");
	    this.ontoDB = onto;
	} 
	else {
	    System.out.println("NULL ---- SET ONTOLOGY DB");
	    this.ontoDB = new OntologyDB();
	}
	
	// this.init(taskType, targetContent, userPose);
	this.initTask(targetContent, userPose);
    }
    
    public GetObjectTask(String taskType, String targetContent, Pose2D userPose) 
    {
	this.ontoDB = new OntologyDB();
	
	this.initTask(targetContent, userPose);
	setTaskType(TaskType.GET_OBJECT);
    }
    
    protected boolean constructTask() {
	return createGetObjectTask();
    }
    
    private boolean createGetObjectTask() {
	return true;
    }
    
    private void initTask(String targetContent, Pose2D userPose) {
	acts = new ArrayList<ActionTuple>();
	
	setTaskTarget(targetContent);
	System.out.println("TASK.JAVA: Created CurrentTask " + "get "
			   + targetContent);
	constructTask();
    }   
}