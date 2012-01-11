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
import ros.*;
import ros.communication.*;

public class TestTask extends Task {

    public TestTask() {
	// empty constructor.
	acts = new ArrayList<ActionTuple>();     // to be deprecated and replaced with allSubSeqs
	
	setTaskType(TaskType.UNSPECIFIED);
	currentAction = null;
    }

    public boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery) {
	return false;
    }

    protected boolean constructTask() {
    	return true;
    }
}
