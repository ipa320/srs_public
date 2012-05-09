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

package org.srs.srs_knowledge.knowledge_engine;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.vocabulary.*;
import com.hp.hpl.jena.util.FileManager;
import com.hp.hpl.jena.ontology.OntClass;
import com.hp.hpl.jena.ontology.SomeValuesFromRestriction;
import com.hp.hpl.jena.ontology.Restriction;
import com.hp.hpl.jena.query.Query;
import com.hp.hpl.jena.query.QueryFactory;
import com.hp.hpl.jena.query.ResultSetFormatter;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;
import com.hp.hpl.jena.ontology.OntProperty;
import java.io.*;
import java.util.ArrayList; 
import java.util.ConcurrentModificationException;
import java.util.NoSuchElementException;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Collections;
import java.util.Comparator;
import ros.*;
import ros.communication.*;
import ros.pkg.srs_knowledge.srv.QuerySparQL;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.srs_knowledge.msg.SRSSpatialInfo;
import com.hp.hpl.jena.shared.Lock;
import ros.pkg.srs_knowledge.srv.PlanNextAction;
import ros.pkg.srs_knowledge.srv.TaskRequest;
import ros.pkg.srs_knowledge.srv.GetObjectsOnMap;
import ros.pkg.srs_knowledge.srv.GetWorkspaceOnMap;
import com.hp.hpl.jena.rdf.model.Statement;
import org.srs.srs_knowledge.task.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.srs_knowledge.srv.GetObjectsOnMap;
import ros.pkg.srs_knowledge.srv.GetWorkspaceOnMap;

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

import org.srs.srs_knowledge.utils.*;

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

    public static ArrayList<String> getWorkspaceNamesOfObject(String objectClassName) {
	return OntoQueryUtil.getWorkspaceNamesOfObject(objectClassName, OntoQueryUtil.ObjectNameSpace, OntoQueryUtil.GlobalNameSpace);
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
	//ArrayList<String> otherWorkspaces = tempGetFurnituresLinkedToObject(objectClassName);
	ArrayList<String> otherWorkspaces = getFurnituresLinkedToObject(objectClassName);
	
	Iterator<Individual> otherInstances;
	//ArrayList<String> otherWSInd = new ArrayList<String>();
	for(int i = 0; i < otherWorkspaces.size(); i++) {
	    otherInstances = KnowledgeEngine.ontoDB.getInstancesOfClass(globalNameSpace + otherWorkspaces.get(i));
	    workspaceList = OntoQueryUtil.addUniqueInstances(workspaceList, otherInstances);
	    //otherWSInd = OntoQueryUtil.addUniqueInstances(otherWSInd, otherInstances);
	}

	/*
	// To sort by distance... TODO: robot current location is unknown... to implement in future
	Collections.sort(otherWSInd, new Comparator<String>() {
	public int compare(String ind1, String ind2) {
	
	// TODO;;;
	Pose p1 = OntoQueryUtil.getPoseOfObject(ind1);
	Pose p2 = OntoQueryUtil.getPoseOfObject(ind2);
	return 0;
	}
	}
	);
	workspaceList.addAll(otherWSInd);
	*/
	
	return workspaceList;
    }

    public static Pose getPoseOfObject(String objectURI) {
	
	Pose p = new Pose();
	
	com.hp.hpl.jena.rdf.model.Statement stm;
	try{
	    Individual temp = KnowledgeEngine.ontoDB.getIndividual(objectURI);
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "xCoord", temp);
	    p.position.x = getFloatOfStatement(stm);
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "yCoord", temp);
	    p.position.y = getFloatOfStatement(stm);
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "zCoord", temp);
	    p.position.z = getFloatOfStatement(stm);
	    
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qu", temp);
	    p.orientation.w = getFloatOfStatement(stm);
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qx", temp);
	    p.orientation.x = getFloatOfStatement(stm);
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qy", temp);
	    p.orientation.y = getFloatOfStatement(stm);
	    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qz", temp);
	    p.orientation.z = getFloatOfStatement(stm);
	}
	catch(NonExistenceEntryException e) {
	    return null;
	}
	catch(Exception e) {
	    //System.out.println("CAUGHT exception: " + e.getMessage()+ ".. added invalid values");
	    
	    p.position.x = -1000;
	    p.position.y = -1000;
	    p.position.z = -1000;
	    
	    p.orientation.w = -1000;
	    p.orientation.x = -1000;
	    p.orientation.y = -1000;
	    p.orientation.z = -1000;
	}
	
	return p;
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
	mpWorkspaces.put("Medicine", mb);

	ArrayList<String> mbook = new ArrayList<String>();
	mbook.add("Table-PieceOfFurniture");
	mbook.add("Dishwasher");
	mbook.add("BookShelf");
	mpWorkspaces.put("Book", mbook);
	mpWorkspaces.put("BookCopy", mbook);

	return mpWorkspaces.get(objectClassName);
    }	

    private static HashSet<String> getSuperAndSubClassesOf(String classURI, String rootClassURI, boolean includeSelf) {
	HashSet<String> ret = new HashSet<String>();
	String nsDef = 
	    "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
	    + " PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
	    + " PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>\n";
	String queryString = " SELECT ?sup  WHERE {\n" 
	    + "<" + classURI + "> rdfs:subClassOf ?sup . \n"
	    + " ?sup rdfs:subClassOf <" + rootClassURI + "> .}\n";
	ArrayList<QuerySolution> qr = KnowledgeEngine.ontoDB.executeQueryRaw(nsDef + queryString);
	for(int i = 0; i < qr.size(); i++) {
	    String tmp = qr.get(i).getResource("sup").getURI();
	    //if(!tmp.equals(rootClassURI)) {
	    ret.add(tmp);
	    //}
	}
	if(includeSelf) {
	    ret.add(classURI);
	}
	
	return ret;
    }

    //return local names
    public static ArrayList<String> getFurnituresLinkedToObject(String objectClassName) {
	ArrayList<String> ret = new ArrayList<String>();
        OntClass objClass = KnowledgeEngine.ontoDB.model.getOntClass(OntoQueryUtil.GlobalNameSpace  + objectClassName );
	/*	
		for(Iterator<OntProperty> pros = objClass.listDeclaredProperties(true); pros.hasNext(); ) {
		System.out.println(pros.next().getLocalName());
		}
	*/
	HashSet<String> supClasses = OntoQueryUtil.getSuperAndSubClassesOf(OntoQueryUtil.GlobalNameSpace + objectClassName, OntoQueryUtil.GlobalNameSpace + "GraspableObject", true);
	Iterator<String> itSup = supClasses.iterator();
	while(itSup.hasNext()) {
	    objClass = KnowledgeEngine.ontoDB.model.getOntClass(itSup.next());
	    for (Iterator<OntClass> supers = objClass.listSuperClasses(false); supers.hasNext(); ) {
		OntClass sup = supers.next();
		if (sup.isRestriction()) {
		    if (sup.asRestriction().isSomeValuesFromRestriction()) {
			//displayRestriction( "some", sup.getOnProperty(), sup.asSomeValuesFromRestriction().getSomeValuesFrom() );
			//}
			//displayRestriction( sup.asRestriction() );
			OntProperty pro = sup.asRestriction().asSomeValuesFromRestriction().getOnProperty();
			//System.out.println(" Found Pro --- " + pro.asResource().toString());
			
			if(pro.getURI().equals(OntoQueryUtil.GlobalNameSpace + "storedAtPlace")) {
			    Resource objRes = sup.asRestriction().asSomeValuesFromRestriction().getSomeValuesFrom();
			    ret.add(objRes.getLocalName());
			    //System.out.println();
			}
		    }
		    // displayType( supers.next() );
		}
	    }
	    
	}
	
	
	return ret;
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
	    /*
	    String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
		+ "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
		+ "PREFIX ipa-kitchen-map: <http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#>\n";
	    String queryString = "SELECT ?x ?y ?theta WHERE { "
		+ "ipa-kitchen-map:" + targetContent
		+ " srs:xCoordinate ?x . " + "ipa-kitchen-map:"
		+ targetContent + " srs:yCoordinate ?y . "
		+ "ipa-kitchen-map:" + targetContent
		+ " srs:orientationTheta ?theta .}";
	    */
	    String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
		+ "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
		+ "PREFIX map: <" + OntoQueryUtil.ObjectNameSpace + ">\n";
	    
	    String queryString = "SELECT ?x ?y ?theta WHERE { "
		+ "map:" + targetContent
		+ " srs:xCoordinate ?x . " 
		+ "map:" + targetContent + " srs:yCoordinate ?y . "
		+ "map:" + targetContent + " srs:orientationTheta ?theta .}";
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
	
	// first, retrieve instance(s) of objectClassName
	Iterator<Individual> workspaceIndList = KnowledgeEngine.ontoDB.getInstancesOfClass(className);
	ArrayList<Individual> wList = new ArrayList<Individual>();
	for( ; workspaceIndList.hasNext(); ) {
	    wList.add(workspaceIndList.next());
	}
	
	return wList;
    }


    public static boolean testUpdateObjectProperty(String proNSURI, String objectNSURI, String objectName) throws NonExistenceEntryException
    {
	try {
	    Individual ind = KnowledgeEngine.ontoDB.getIndividual(objectNSURI + objectName);	    
	    // set property
	    Property pro = KnowledgeEngine.ontoDB.getProperty(proNSURI + "xCoord");
	    // ind.setPropertyValue(pro, ); 
	    com.hp.hpl.jena.rdf.model.Statement stm = ind.getProperty(pro);
	    // KnowledgeEngine.ontoDB.removeStatement(stm);

	    
	    Literal x = KnowledgeEngine.ontoDB.model.createTypedLiteral(10.0f);
	    ind.setPropertyValue(pro, x);
	    
	    //stm.changeLiteralObject(10.0f);
	    
	    //Statement stm1 = KnowledgeEngine.ontoDB.model.createStatement(ind, pro, x);
	    //KnowledgeEngine.ontoDB.model.add(stm1);
	    //model.leaveCriticalSection();
	}
	catch(NonExistenceEntryException e) {
	    throw e;
	}
	return true;
    }

    public static boolean updatePoseOfObject(Pose pos, String propertyNSURI, String objectURI) throws NonExistenceEntryException {
    	System.out.println("Update the pose of an object or furniture in the semantic map");
	try {
	    Individual ind = KnowledgeEngine.ontoDB.getIndividual(objectURI);	    
	    // set property
	    Property pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "xCoord");
	    //	    com.hp.hpl.jena.rdf.model.Statement stm = ind.getProperty(pro);
	    //Literal x = KnowledgeEngine.ontoDB.model.createTypedLiteral(10.0f);
	    Literal x = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.position.x));
       	    ind.setPropertyValue(pro, x);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "yCoord");
	    //stm = ind.getProperty(pro);
	    Literal y = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.position.y));
       	    ind.setPropertyValue(pro, y);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "zCoord");
	    //stm = ind.getProperty(pro);
	    Literal z = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.position.z));
       	    ind.setPropertyValue(pro, z);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qx");
	    //stm = ind.getProperty(pro);
	    Literal qx = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.x));
       	    ind.setPropertyValue(pro, qx);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qy");
	    //stm = ind.getProperty(pro);
	    Literal qy = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.y));
       	    ind.setPropertyValue(pro, qy);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qz");
	    //stm = ind.getProperty(pro);
	    Literal qz = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.z));
       	    ind.setPropertyValue(pro, qz);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qu");
	    //stm = ind.getProperty(pro);
	    Literal qw = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.w));
       	    ind.setPropertyValue(pro, qw);

	    // Update symbolic spatial relation
	    OntoQueryUtil.computeOnSpatialRelation();
	}
	catch(NonExistenceEntryException e) {
	    throw e;
	}
    
	return true;
    }

    public static boolean updatePoseOfObject(Pose pos, String propertyNSURI, String objectNSURI, String objectName) throws NonExistenceEntryException {
	return updatePoseOfObject(pos, propertyNSURI, objectNSURI + objectName);
	/*
    	System.out.println("Update the pose of an object or furniture in the semantic map");
	try {
	    Individual ind = KnowledgeEngine.ontoDB.getIndividual(objectNSURI + objectName);	    
	    // set property
	    Property pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "xCoord");
	    com.hp.hpl.jena.rdf.model.Statement stm = ind.getProperty(pro);
	    //Literal x = KnowledgeEngine.ontoDB.model.createTypedLiteral(10.0f);
	    Literal x = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.position.x));
       	    ind.setPropertyValue(pro, x);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "yCoord");
	    stm = ind.getProperty(pro);
	    Literal y = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.position.y));
       	    ind.setPropertyValue(pro, y);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "zCoord");
	    stm = ind.getProperty(pro);
	    Literal z = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.position.z));
       	    ind.setPropertyValue(pro, z);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qx");
	    stm = ind.getProperty(pro);
	    Literal qx = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.x));
       	    ind.setPropertyValue(pro, qx);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qy");
	    stm = ind.getProperty(pro);
	    Literal qy = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.y));
       	    ind.setPropertyValue(pro, qy);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qz");
	    stm = ind.getProperty(pro);
	    Literal qz = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.z));
       	    ind.setPropertyValue(pro, qz);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "qu");
	    stm = ind.getProperty(pro);
	    Literal qw = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.orientation.w));
       	    ind.setPropertyValue(pro, qw);

	}
	catch(NonExistenceEntryException e) {
	    throw e;
	}
    
	return true;
	*/
    }
    
    public static boolean updateDimensionOfObject(float l, float w, float h, String propertyNSURI, String objectNSURI, String objectName) throws NonExistenceEntryException {
    	System.out.println("Update the dimension of an object or furniture in the semantic map");
	try{	
	    Individual ind = KnowledgeEngine.ontoDB.getIndividual(objectNSURI + objectName);	    
	    // set property
	    Property pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "lengthOfObject");
	    com.hp.hpl.jena.rdf.model.Statement stm = ind.getProperty(pro);
	    
	    Literal ll = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(l));
       	    ind.setPropertyValue(pro, ll);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "widthOfObject");
	    stm = ind.getProperty(pro);
	    Literal lw = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(w));
       	    ind.setPropertyValue(pro, lw);

	    pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "heightOfObject");
	    stm = ind.getProperty(pro);
	    Literal lh = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(h));
       	    ind.setPropertyValue(pro, lh);
	}
	catch(NonExistenceEntryException e) {
	    throw e;
	}

	return true;
    }
    
    public static boolean updateHHIdOfObject(int id, String propertyNSURI, String objectNSURI, String objectName) throws NonExistenceEntryException {
    	System.out.println("Update the household object ID of an object or furniture in the semantic map");
	try{	
	    Individual ind = KnowledgeEngine.ontoDB.getIndividual(objectNSURI + objectName);	    
	    // set property
	    Property pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "houseHoldObjectID");
	    com.hp.hpl.jena.rdf.model.Statement stm = ind.getProperty(pro);
	    Literal litId = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Integer(id));
       	    ind.setPropertyValue(pro, litId);
	}
	catch(NonExistenceEntryException e) {
	    throw e;
	}
	return true;
    }

    public static boolean computeOnSpatialRelation() {
	// get a list of objects and workspaces
	GetObjectsOnMap.Response resOBJ = getObjectsOnMap(OntoQueryUtil.MapName, true);
	GetWorkspaceOnMap.Response resWS = getWorkspaceOnMap(OntoQueryUtil.MapName, true);
	// pair-wise comparison --- if condition met, then update the knowledge base
	//System.out.println("RESULT ------->>>>>>  SIZE OF OBJECTS: " + resOBJ.objects.size());
	//for (String obj : resOBJ.objects) {
	//    System.out.println(obj);
	//}

	//System.out.println("RESULT ------>>>>>> SIZE OF WORKSPACES:  " + resWS.objects.size());
	//for (String ws : resWS.objects) {
	//   System.out.println(ws);
	//}

	for (int i = 0; i < resOBJ.objectsInfo.size(); i++) {
	    for (int j = 0; j < resWS.objectsInfo.size(); j++) {
		if(SpatialCalculator.ifOnObject(resOBJ.objectsInfo.get(i), resWS.objectsInfo.get(j), -1)) {
		    //System.out.println("FOUND ONE PAIR MATCH THE ON RELATIONSHIP");
		    //TODO update rdf graph model
		    try {
			Individual ind1 = KnowledgeEngine.ontoDB.getIndividual(OntoQueryUtil.ObjectNameSpace + resOBJ.objects.get(i));
			Individual ind2 = KnowledgeEngine.ontoDB.getIndividual(OntoQueryUtil.ObjectNameSpace + resWS.objects.get(j));
			//Property proExist = KnowledgeEngine.ontoDB.getProperty(OntoQueryUtil.GlobalNameSpace + "spatiallyRelated");
			//ind1.removeProperty(proExist, ind2);
			OntoQueryUtil.removeAllSubPropertiesOf(OntoQueryUtil.ObjectNameSpace + resOBJ.objects.get(i), OntoQueryUtil.GlobalNameSpace + "spatiallyRelated");

			if (OntoQueryUtil.updateOnSpatialRelation(ind1, ind2) ) {
			    // System.out.println("Added Property: Object " + resOBJ.objects.get(i) + " is aboveOf Object " + resWS.objects.get(j));
			}
			else {
			    //System.out.println("CANNOT add Property: Object " + resOBJ.objects.get(i) + " is aboveOf Object " + resWS.objects.get(j));
			}
			    
		    }
		    catch (NonExistenceEntryException e) {
			System.out.println("Individual not found in ontology:  ---  " + e.getMessage());
		    }
		    
		}
		else {
		    //System.out.println("NO MATCH.... between : " + resOBJ.objects.get(i) + "  and  " + resWS.objects.get(j));
		    
		}
	    }
	}

	// TODO: Better to use SPARQL CONSTRUCT + RULES ... TO REPLACE

	return true;
    }

    private static boolean updateOnSpatialRelation(Individual obj1, Individual obj2) {
	try {
	    Property pro = KnowledgeEngine.ontoDB.getProperty(OntoQueryUtil.GlobalNameSpace + "aboveOf");
	    //com.hp.hpl.jena.rdf.model.Statement stm = obj1.getProperty(pro);
	    obj1.setPropertyValue(pro, obj2);
	} 
	catch(NonExistenceEntryException e) {
	    System.out.println("NonExistenceEntryException --> " + e.getMessage());
	    return false;
	}
	/*
	Individual ind = KnowledgeEngine.ontoDB.getIndividual(objectNSURI + objectName);	    
	// set property
	Property pro = KnowledgeEngine.ontoDB.getProperty(propertyNSURI + "xCoord");
	com.hp.hpl.jena.rdf.model.Statement stm = ind.getProperty(pro);
	//Literal x = KnowledgeEngine.ontoDB.model.createTypedLiteral(10.0f);
	Literal x = KnowledgeEngine.ontoDB.model.createTypedLiteral(new Float(pos.position.x));
	ind.setPropertyValue(pro, x);
	*/
	return true;
    }

    public static GetWorkspaceOnMap.Response getWorkspaceOnMap(String map, boolean ifGeometryInfo) {
    	GetWorkspaceOnMap.Response re = new GetWorkspaceOnMap.Response();
	String className = GlobalNameSpace;
	String mapNS = ObjectNameSpace;
		
	if(map != null) {
	    if(KnowledgeEngine.ontoDB.getNamespaceByPrefix(map) != null) {
		mapNS = KnowledgeEngine.ontoDB.getNamespaceByPrefix(map);
	    }
	}
	
	className = className + "FurniturePiece";
	//System.out.println(className);
	try{
	    Iterator<Individual> instances = KnowledgeEngine.ontoDB.getInstancesOfClass(className);
	    if(instances == null) {
		return re;
	    }

	    if(instances.hasNext()) {
		while (instances.hasNext()) { 
		    Individual temp = (Individual)instances.next();
		    //System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		    if(temp.getNameSpace().equals(ObjectNameSpace)) {
			re.objects.add(temp.getLocalName());
			re.classesOfObjects.add(temp.getRDFType(true).getLocalName());

			if(ifGeometryInfo == true) { 
			    SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
			    
			    com.hp.hpl.jena.rdf.model.Statement stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "xCoord", temp);
			    spatialInfo.pose.position.x = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "yCoord", temp);
			    spatialInfo.pose.position.y = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "zCoord", temp);
			    spatialInfo.pose.position.z = getFloatOfStatement(stm);
			    
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "widthOfObject", temp);
			    spatialInfo.w = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "heightOfObject", temp);
			    spatialInfo.h = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "lengthOfObject", temp);
			    spatialInfo.l = getFloatOfStatement(stm);			    

			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qu", temp);
			    spatialInfo.pose.orientation.w = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qx", temp);
			    spatialInfo.pose.orientation.x = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qy", temp);
			    spatialInfo.pose.orientation.y = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qz", temp);
			    spatialInfo.pose.orientation.z = getFloatOfStatement(stm);

			    re.objectsInfo.add(spatialInfo);

			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "houseHoldObjectID", temp);
			    re.houseHoldId.add(Integer.toString(getIntOfStatement(stm)));
			}
		    }
		}       
	    }
	    else
		System.out.println("<EMPTY>");
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}

	return re;
    }

    public static GetObjectsOnMap.Response getObjectsOnMapOfType(String objectTypeURI, boolean ifGeometryInfo) {
	GetObjectsOnMap.Response re = new GetObjectsOnMap.Response();
	//System.out.println(objectTypeURI + " --- ");
	Iterator<Individual> instances = KnowledgeEngine.ontoDB.getInstancesOfClass(objectTypeURI);
	if(instances == null) {
	    return re;
	}
	com.hp.hpl.jena.rdf.model.Statement stm;
	if(instances.hasNext()) {
	    while (instances.hasNext()) { 
		Individual temp = (Individual)instances.next();
		//System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		if(temp.getNameSpace().equals(ObjectNameSpace)) {
		    re.objects.add(temp.getLocalName());
		    re.classesOfObjects.add(temp.getRDFType(true).getLocalName());
		    try{
			stm = KnowledgeEngine.ontoDB.getPropertyOf(GlobalNameSpace, "spatiallyRelated", temp);
			//stm = KnowledgeEngine.ontoDB.getPropertyOf(GlobalNameSpace, "aboveOf", temp);
			re.spatialRelation.add(stm.getPredicate().getLocalName());
			re.spatialRelatedObject.add(stm.getObject().asResource().getLocalName());
		    }
		    catch(Exception e) {
			//System.out.println("CAUGHT exception: " + e.toString());
			re.spatialRelation.add("NA");
			re.spatialRelatedObject.add("NA");
		    }
		    try{
			stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "houseHoldObjectID", temp);			
			re.houseHoldId.add(Integer.toString(getIntOfStatement(stm)));
		    }
		    catch(Exception e) {
			//System.out.println("CAUGHT exception: " + e.toString());
			re.houseHoldId.add("NA");
		    }
		    if(ifGeometryInfo == true) { 
			SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
			try{
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "xCoord", temp);
			    spatialInfo.pose.position.x = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "yCoord", temp);
			    spatialInfo.pose.position.y = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "zCoord", temp);
			    spatialInfo.pose.position.z = getFloatOfStatement(stm);
			    
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "widthOfObject", temp);
			    spatialInfo.w = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "heightOfObject", temp);
			    spatialInfo.h = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "lengthOfObject", temp);
			    spatialInfo.l = getFloatOfStatement(stm);
			    
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qu", temp);
			    spatialInfo.pose.orientation.w = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qx", temp);
			    spatialInfo.pose.orientation.x = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qy", temp);
			    spatialInfo.pose.orientation.y = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qz", temp);
			    spatialInfo.pose.orientation.z = getFloatOfStatement(stm);
			}
			catch(Exception e) {
			    //System.out.println("CAUGHT exception: " + e.getMessage()+ ".. added invalid values");
			    
			    spatialInfo.pose.position.x = -1000;
			    spatialInfo.pose.position.y = -1000;
			    spatialInfo.pose.position.z = -1000;
			    
			    spatialInfo.w = -1000;
			    spatialInfo.h = -1000;
			    spatialInfo.l = -1000;

			    spatialInfo.pose.orientation.w = -1000;
			    spatialInfo.pose.orientation.x = -1000;
			    spatialInfo.pose.orientation.y = -1000;
			    spatialInfo.pose.orientation.z = -1000;
			}

			re.objectsInfo.add(spatialInfo);
		    }
		    
		}
	    }
	}
	else {
	    System.out.println("<EMPTY>");
	}
	
	return re;
    }

    public static GetObjectsOnMap.Response getObjectsOnMap(String map, boolean ifGeometryInfo) {
	//GetObjectsOnMap.Response re = new GetObjectsOnMap.Response();

	String className = GlobalNameSpace;
	String mapNS = ObjectNameSpace;
		
	if(map != null) {
	    if(KnowledgeEngine.ontoDB.getNamespaceByPrefix(map) != null) {
		mapNS = KnowledgeEngine.ontoDB.getNamespaceByPrefix(map);
	    }
	}
	
	//	className = className + "FoodVessel";
	className = className + "GraspableObject";

	return OntoQueryUtil.getObjectsOnMapOfType(className, ifGeometryInfo);
	/*

	System.out.println(className + " --- ");
	Iterator<Individual> instances = KnowledgeEngine.ontoDB.getInstancesOfClass(className);
	if(instances == null) {
	    return re;
	}
	com.hp.hpl.jena.rdf.model.Statement stm;
	if(instances.hasNext()) {
	    while (instances.hasNext()) { 
		Individual temp = (Individual)instances.next();
		System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		if(temp.getNameSpace().equals(ObjectNameSpace)) {
		    re.objects.add(temp.getLocalName());
		    re.classesOfObjects.add(temp.getRDFType(true).getLocalName());
		    try{
			
			stm = KnowledgeEngine.ontoDB.getPropertyOf(GlobalNameSpace, "spatiallyRelated", temp);			
			re.spatialRelation.add(stm.getPredicate().getLocalName());
			re.spatialRelatedObject.add(stm.getObject().asResource().getLocalName());
		    }
		    catch(Exception e) {
			System.out.println("CAUGHT exception: " + e.toString());
			re.spatialRelation.add("NA");
			re.spatialRelatedObject.add("NA");
		    }
		    try{
			stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "houseHoldObjectID", temp);			
			re.houseHoldId.add(Integer.toString(getIntOfStatement(stm)));
		    }
		    catch(Exception e) {
			System.out.println("CAUGHT exception: " + e.toString());
			re.houseHoldId.add("NA");
		    }
		    if(ifGeometryInfo == true) { 
			SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
			try{
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "xCoord", temp);
			    spatialInfo.pose.position.x = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "yCoord", temp);
			    spatialInfo.pose.position.y = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "zCoord", temp);
			    spatialInfo.pose.position.z = getFloatOfStatement(stm);
			    
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "widthOfObject", temp);
			    spatialInfo.w = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "heightOfObject", temp);
			    spatialInfo.h = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "lengthOfObject", temp);
			    spatialInfo.l = getFloatOfStatement(stm);
			    
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qu", temp);
			    spatialInfo.pose.orientation.w = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qx", temp);
			    spatialInfo.pose.orientation.x = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qy", temp);
			    spatialInfo.pose.orientation.y = getFloatOfStatement(stm);
			    stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "qz", temp);
			    spatialInfo.pose.orientation.z = getFloatOfStatement(stm);
			}
			catch(Exception e) {
			    System.out.println("CAUGHT exception: " + e.getMessage()+ ".. added invalid values");
			    
			    spatialInfo.pose.position.x = -1000;
			    spatialInfo.pose.position.y = -1000;
			    spatialInfo.pose.position.z = -1000;
			    
			    spatialInfo.w = -1000;
			    spatialInfo.h = -1000;
			    spatialInfo.l = -1000;

			    spatialInfo.pose.orientation.w = -1000;
			    spatialInfo.pose.orientation.x = -1000;
			    spatialInfo.pose.orientation.y = -1000;
			    spatialInfo.pose.orientation.z = -1000;
			}

			re.objectsInfo.add(spatialInfo);
		    }
		    
		}
	    }
	}
	else {
	    System.out.println("<EMPTY>");
	}
	
	return re;
	*/
    }

    public static void removeAllSubPropertiesOf(String subjectURI, String propertyURI) {
	try{
	Individual targetInd = KnowledgeEngine.ontoDB.getIndividual(subjectURI);
	OntProperty proExist = KnowledgeEngine.ontoDB.getOntProperty(propertyURI);

	com.hp.hpl.jena.util.iterator.ExtendedIterator<OntProperty> subPros = (com.hp.hpl.jena.util.iterator.ExtendedIterator<OntProperty>)proExist.listSubProperties();

	while(subPros.hasNext()) {
	    com.hp.hpl.jena.rdf.model.Statement stm = targetInd.getProperty(subPros.next());
	    if(stm != null) {
		KnowledgeEngine.ontoDB.removeStatement(stm);
		targetInd.removeAll(proExist);
	    }
	}
	}
	catch(Exception e) {
	    //System.out.println(e.toString() + "  " + e.getMessage());
	    return;
	}
    }

    public static void testRemoveProperty() {
	try {
	    String targetObj = OntoQueryUtil.ObjectNameSpace + "MilkBox0";
	    System.out.println("TARGET OBJECT IS ::: " + targetObj);
	    if(!targetObj.trim().equals("")) {
		Pose tmpPose = new Pose();
		tmpPose.position.x = -1000;
		tmpPose.position.y = -1000;
		tmpPose.position.z = -1000;
		tmpPose.orientation.x = -1000;
		tmpPose.orientation.y = -1000;
		tmpPose.orientation.z = -1000;
		tmpPose.orientation.w = -1000;

		// update its pose
		try{
		    OntoQueryUtil.updatePoseOfObject(tmpPose, OntoQueryUtil.GlobalNameSpace, targetObj.trim());
		    OntoQueryUtil.computeOnSpatialRelation();
		}
		catch(Exception e) {
		    System.out.println(e.getMessage() + "   " + e.toString());
		}
	    }
	    
	    Individual targetInd = KnowledgeEngine.ontoDB.getIndividual(targetObj);

	    OntoQueryUtil.removeAllSubPropertiesOf(targetObj, OntoQueryUtil.GlobalNameSpace + "spatiallyRelated");

	    String gripper = OntoQueryUtil.ObjectNameSpace + "SRSCOBGripper";
	    //System.out.println("<<<<<  " + gripper + "  >>>>>");
	    
	    Individual gripInd = KnowledgeEngine.ontoDB.getIndividual(gripper);

	    /*
	    OntProperty proExist = KnowledgeEngine.ontoDB.getOntProperty(OntoQueryUtil.GlobalNameSpace + "spatiallyRelated");
	    com.hp.hpl.jena.util.iterator.ExtendedIterator<OntProperty> subPros = (com.hp.hpl.jena.util.iterator.ExtendedIterator<OntProperty>)proExist.listSubProperties();

	    while(subPros.hasNext()) {
		com.hp.hpl.jena.rdf.model.Statement stm = targetInd.getProperty(subPros.next());
		if(stm != null) {
		    KnowledgeEngine.ontoDB.removeStatement(stm);
		    targetInd.removeAll(proExist);
		}
	    }
	    */

	    Property pro = KnowledgeEngine.ontoDB.getProperty(OntoQueryUtil.GlobalNameSpace + "grippedBy");
	    
	    targetInd.setPropertyValue(pro, gripInd);
	    
	}
	catch (Exception e) {
	    // System.out.println(" ==================   " + e.getMessage() + "   " + e.toString());
	}
	
    }

    public static float getFloatOfStatement(Statement stm) 
    {
	float t = -1000;
	try { 
	    t = stm.getFloat();
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
	return t;
    }

    public static int getIntOfStatement(Statement stm)
    {
	int t = -1000;
	try { 
	    t = stm.getInt();
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
	return t;
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
    public static String MapName = "";
    public static String RobotName = "cob3-3";
}

