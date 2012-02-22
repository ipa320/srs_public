package org.srs.srs_knowledge.knowledge_engine;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.vocabulary.*;
import com.hp.hpl.jena.util.FileManager;

import com.hp.hpl.jena.query.Query;
import com.hp.hpl.jena.query.QueryFactory;
import com.hp.hpl.jena.query.ResultSetFormatter;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;
import java.io.*;
import java.util.ArrayList; 
import java.util.Iterator;
import ros.*;
import ros.communication.*;
import ros.pkg.srs_knowledge.srv.AskForActionSequence;  // deprecated
import ros.pkg.srs_knowledge.srv.GenerateSequence;
import ros.pkg.srs_knowledge.srv.QuerySparQL;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.srs_knowledge.msg.SRSSpatialInfo;

import ros.pkg.srs_knowledge.srv.PlanNextAction;
import ros.pkg.srs_knowledge.srv.TaskRequest;
import ros.pkg.srs_knowledge.srv.GetObjectsOnMap;
import ros.pkg.srs_knowledge.srv.GetWorkspaceOnMap;
import com.hp.hpl.jena.rdf.model.Statement;
import org.srs.srs_knowledge.task.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import ros.pkg.geometry_msgs.msg.Pose;

import java.util.Properties;

import java.io.IOException;
import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

/**
 * This class is used for more application specific ontology queries. 
 * OntologyDB is more generic, and only provides basic functions and reusable functions. 
 */
public class OntoQueryUtil 
{
    public static ArrayList<Individual> getWorkspaceOfObject(String objectClassName, String objectNameSpace, String globalNameSpace, OntologyDB onto) { 
	// TODO: 
	ArrayList<String> workspaceList = getWorkspaceNamesOfObject(objectClassName, objectNameSpace, globalNameSpace);
	ArrayList<Individual> workspaceIndList = new ArrayList<Individual>();
	for(String s : workspaceList) {
	    workspaceIndList.add(onto.getModel().getIndividual(objectNameSpace + s));
	}
	return workspaceIndList;
    }

    /**
     * e.g. get workspace of milk box
     */
    public static ArrayList<String> getWorkspaceNamesOfObject(String objectClassName, String objectNameSpace, String globalNameSpace) { 
	ArrayList<String> workspaceList = new ArrayList<String>();

	String className = globalNameSpace + objectClassName; 

	// first, retrieve instance(s) of objectClassName
	Iterator<Individual> instancesOfObject = KnowledgeEngine.ontoDB.getInstancesOfClass(className);

	// second, retrieve instance(s) of workspace for this particular objectClassName
	com.hp.hpl.jena.rdf.model.Statement stm;

	for( ; instancesOfObject.hasNext(); ) {
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(globalNameSpace, "spatiallyRelated", instancesOfObject.next());
	    workspaceList.add(stm.getObject().asResource().getLocalName());
	}
	
	// list possible workspace(s), e.g. tables
	ArrayList<String> otherWorkspaces = tempGetFurnituresLinkedToObject(objectClassName);
	//workspaceList.addAll(otherWorkspaces);

	Iterator<Individual> otherInstances;
	for(int i = 0; i < otherWorkspaces.size(); i++) {
	    otherInstances = KnowledgeEngine.ontoDB.getInstancesOfClass(globalNameSpace + otherWorkspaces.get(i));
	    workspaceList = OntoQueryUtil.addUniqueInstances(workspaceList, otherInstances);
	}

	return workspaceList;
    }

    public static ArrayList<String> addUniqueInstances(ArrayList<String> original, Iterator<Individual> newList) {
	while(newList.hasNext()) {
	    Individual tempIndividual = newList.next();
	    String temp = tempIndividual.getLocalName();
	    if (!original.contains(temp)) {
		original.add(temp);
	    }
	}
	
	return original; 
    }

    public static ArrayList<String> tempGetFurnituresLinkedToObject(String objectClassName) {
	// TODO: Only temporarily provided for object related furnitures or workspaces . should be obtained from ontology instead

	// temporarily create the data
	Map<String, ArrayList<String>> mpWorkspaces = new HashMap<String, ArrayList<String>>();
	ArrayList<String> mb = new ArrayList<String>();
	mb.add("Table-PieceOfFurniture");
	mb.add("Dishwasher");
        mpWorkspaces.put("Milkbox", mb);
        mpWorkspaces.put("Salt", mb);
        mpWorkspaces.put("Bottle", mb);
        mpWorkspaces.put("Pringles", mb);
	return mpWorkspaces.get(objectClassName);
    }	

    public static Pose2D parsePose2D(String targetContent) {
	Pose2D pos = new Pose2D();

	double x = 1;
	double y = 1;
	double theta = 0;
	
	if (targetContent.charAt(0) == '['
	    && targetContent.charAt(targetContent.length() - 1) == ']') {
	    StringTokenizer st = new StringTokenizer(targetContent, " [],");
	    if (st.countTokens() == 3) {
		try {
		    x = Double.parseDouble(st.nextToken());
		    y = Double.parseDouble(st.nextToken());
		    theta = Double.parseDouble(st.nextToken());
		    System.out.println(x + "  " + y + " " + theta);
		} catch (Exception e) {
		    System.out.println(e.getMessage());
		    return null;
		}
	    }
	} else {
	    // Ontology queries
	    String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
		+ "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
		+ "PREFIX ipa-kitchen-map: <http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#>\n";
	    String queryString = "SELECT ?x ?y ?theta WHERE { "
		+ "ipa-kitchen-map:" + targetContent
		+ " srs:xCoordinate ?x . " + "ipa-kitchen-map:"
		+ targetContent + " srs:yCoordinate ?y . "
		+ "ipa-kitchen-map:" + targetContent
		+ " srs:orientationTheta ?theta .}";
	    //System.out.println(prefix + queryString + "\n");
	    
	    if (KnowledgeEngine.ontoDB == null) {
		System.out.println("Ontology Database is NULL");
		return null;
	    }
	    
	    try {
		ArrayList<QuerySolution> rset = KnowledgeEngine.ontoDB.executeQueryRaw(prefix
										       + queryString);
		if (rset.size() == 0) {
		    System.out.println("ERROR: No move target found from database");
		    return null;
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
		return null;
	    }
	}
	
	pos.x = x; 
	pos.y = y;
	pos.theta = theta;
	return pos;
    }

    public static ArrayList<Individual> getWorkspaceByName(String objectClassName, String objectNameSpace, String globalNameSpace) { 
	// TODO: 
	String className = globalNameSpace + objectClassName; 
	System.out.println(className + "  ---");
	// first, retrieve instance(s) of objectClassName
	Iterator<Individual> workspaceIndList = KnowledgeEngine.ontoDB.getInstancesOfClass(className);
	ArrayList<Individual> wList = new ArrayList<Individual>();
	for( ; workspaceIndList.hasNext(); ) {
	    wList.add(workspaceIndList.next());
	}
	
	return wList;
    }

    public static boolean updatePoseOfObject(Pose pos, String objectNSURI, String objectName) throws NonExistenceEntryException {
    	System.out.println("Update the pose of an object or furniture in the semantic map");
	model.enterCriticalSection(Lock.READ);

	Individual ind = model.getIndividual(objectNSURI + objectName);
	if(ind == null) {
	    model.leaveCriticalSection();
	    throw new NonExistenceEntryException(instanceName);
	}
	
	// set property
	Property pro = model.getProperty(objectNSURI, "xCoord");
	// ind.setPropertyValue(pro, ); 
	
	model.leaveCriticalSection();
    	return true;
    }
    
    public static bool updateDimensionOfObject(double l, double w, double h) throws NonExistenceEntryException {
    	System.out.println("Update the dimension of an object or furniture in the semantic map");
	model.enterCriticalSection(Lock.READ);


	Individual ind = model.getIndividual(objectNSURI + objectName);
	if(ind == null) {
	    model.leaveCriticalSection();
	    throw new NonExistenceEntryException(instanceName);
	}
	

	model.leaveCriticalSection();
	return true;
    }

    /*
    public OntoQueryUtil(String objectNameSpace, String globalNameSpace) {
	this.objectNameSpace = objectNameSpace;
	this.globalNameSpace = globalNameSpace;
	System.out.println("Create OntoQueryUtil: this.objectNameSpace is : " + this.objectNameSpace + "  this.globalNameSpace is  : " + this.globalNameSpace );
    }


    // return ipa-kitchen in the srs project. 
    public String getObjectNameSpace() {
	return this.objectNameSpace;
    }
    // return srs namespace
    public String getGlobalNameSpace() {
	return this.globalNameSpace;
    }

    private String objectNameSpace;
    private String globalNameSpace; 
    */
    public static String ObjectNameSpace = "";
    public static String GlobalNameSpace = "";
}

