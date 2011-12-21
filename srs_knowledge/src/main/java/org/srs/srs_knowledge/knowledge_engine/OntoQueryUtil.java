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
	ArrayList<String> workspaceList = getWorkspaceNamesOfObject(objectClassName, objectNameSpace, globalNameSpace, onto);
	ArrayList<Individual> workspaceIndList = new ArrayList<Individual>();

	for(String s : workspaceList) {
	    workspaceIndList.add(onto.getModel().getIndividual(objectNameSpace + s));
	}

	return workspaceIndList;
    }

    /**
     * e.g. get workspace of milk box
     */
    public static ArrayList<String> getWorkspaceNamesOfObject(String objectClassName, String objectNameSpace, String globalNameSpace, OntologyDB onto) { 
	ArrayList<String> workspaceList = new ArrayList<String>();

	String className = globalNameSpace + objectClassName; 

	// first, retrieve instance(s) of objectClassName
	Iterator<Individual> instancesOfObject = onto.getInstancesOfClass(className);

	// second, retrieve instance(s) of workspace for this particular objectClassName
	com.hp.hpl.jena.rdf.model.Statement stm;

	for( ; instancesOfObject.hasNext(); ) {
	    stm = onto.getPropertyOf(globalNameSpace, "spatiallyRelated", instancesOfObject.next());
	    workspaceList.add(stm.getObject().asResource().getLocalName());
	}
	
	// list possible workspace(s), e.g. tables
	ArrayList<String> otherWorkspaces = tempGetFurnituresLinkedToObject(objectClassName);
	//workspaceList.addAll(otherWorkspaces);

	Iterator<Individual> otherInstances;
	for(int i = 0; i < otherWorkspaces.size(); i++) {
	    otherInstances = onto.getInstancesOfClass(globalNameSpace + otherWorkspaces.get(i));
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
        mpWorkspaces.put("MilkBox", mb);

	return mpWorkspaces.get(objectClassName);
    }	

    public OntoQueryUtil(String objectNameSpace, String globalNameSpace) {
	this.objectNameSpace = objectNameSpace;
	this.globalNameSpace = globalNameSpace;
	System.out.println("Create OntoQueryUtil: this.objectNameSpace is : " + this.objectNameSpace + "  this.globalNameSpace is  : " + this.globalNameSpace );
    }

    public String getObjectNameSpace() {
	return this.objectNameSpace;
    }

    public String getGlobalNameSpace() {
	return this.globalNameSpace;
    }

    private String objectNameSpace;
    private String globalNameSpace; 
}

