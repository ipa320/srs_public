package org.srs.srs_knowledge.knowledge_engine;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.vocabulary.*;
import com.hp.hpl.jena.util.FileManager;

import com.hp.hpl.jena.query.Query;
import com.hp.hpl.jena.query.QueryFactory;
import com.hp.hpl.jena.query.ResultSetFormatter;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.sparql.engine.ResultSetStream;
import com.hp.hpl.jena.rdf.model.Property;
import org.mindswap.pellet.jena.PelletReasonerFactory;
import com.hp.hpl.jena.ontology.OntClass;
import com.hp.hpl.jena.ontology.OntModel;
import com.hp.hpl.jena.rdf.model.Statement;
import com.hp.hpl.jena.ontology.Individual;
import com.hp.hpl.jena.shared.Lock;
import com.hp.hpl.jena.ontology.OntResource;
import java.io.*;
import java.util.ArrayList;
import java.util.Iterator;

import org.srs.srs_knowledge.knowledge_engine.*;

public class OntologyDB
{
    public OntologyDB()
    {
	// create an empty model
	this.model = ModelFactory.createOntologyModel(PelletReasonerFactory.THE_SPEC);
	this.reasoning();
    }

    public OntologyDB(String filename)
    {
	try {
	    //String modelFileName = filename;
	    this.reloadOWLFile(filename);
	}
	catch(IllegalArgumentException e) {
	    System.out.println("Caught Exception : " + e.getMessage());
	}
	this.reasoning();
    }

    public OntologyDB(ArrayList<String> filenames)
    {
	// create an empty model
	this.model = ModelFactory.createOntologyModel(PelletReasonerFactory.THE_SPEC);

	//this.model = ModelFactory.createDefaultModel();
	try {
	    for(String filename : filenames) {
		//String modelFileName = filename;
		this.importOntology(filename);
	    }
	}
	catch(IllegalArgumentException e) {
	    System.out.println("Caught Exception : " + e.getMessage());
	}
	this.reasoning();
    }

    public void importOntology(String filename) 
    {
	System.out.println("Load OWL File: " + filename);
	// use the FileManager to find the input file
	InputStream in = FileManager.get().open(filename);
	if (in == null) {
	    throw new IllegalArgumentException("File: " + filename + " not found");
	}
	
	// read the RDF/XML file
	model.read(in, null);
    }

    public void reasoning()
    {
	return;
    }

    public String executeQuery(String queryString)
    {
	//System.out.println(queryString);
	Query query = QueryFactory.create(queryString);

	QueryExecution qe = QueryExecutionFactory.create(query, model);
	ResultSet results = qe.execSelect();

	ByteArrayOutputStream ostream = new ByteArrayOutputStream();
	ResultSetFormatter.out(ostream, results, query);
	//ResultSetFormatter.out(System.out, results, query);
	String r = "";
	try{
	    r = new String(ostream.toByteArray(), "UTF-8");
	    //System.out.println(r);
	}
	catch(Exception e){
	    System.out.println(e.getMessage());
	}
	qe.close();
	return r;
    }
    
    public ArrayList<QuerySolution> executeQueryRaw(String queryString)
    {
	//System.out.println(queryString);
	Query query = QueryFactory.create(queryString);
	
	QueryExecution qe = QueryExecutionFactory.create(query, model);
	ResultSet results = qe.execSelect();
	/*
	ByteArrayOutputStream ostream = new ByteArrayOutputStream();
	ResultSetFormatter.out(ostream, results, query);
	//ResultSetFormatter.out(System.out, results, query);
	String r = "";
	try{
	    r = new String(ostream.toByteArray(), "UTF-8");
	    System.out.println(r);
	}
	catch(Exception e){
	    System.out.println(e.getMessage());
	}
	*/
	ArrayList<QuerySolution> resList = new ArrayList<QuerySolution>();
	if(results.hasNext()) {
	    
	    QuerySolution qs = results.next();
	    resList.add(qs);
	    //double x = qs.getLiteral("x").getFloat();
	    //Literal y = qs.getLiteral("y");
	    //Literal theta = qs.getLiteral("theta");
	}

	qe.close();
	return resList; //results;
    }

    public void reloadOWLFile(String file)
    {
	// create an empty model
	//this.model = ModelFactory.createDefaultModel();
	this.model = ModelFactory.createOntologyModel(PelletReasonerFactory.THE_SPEC);

	System.out.println("Load OWL File: " + file);
	// use the FileManager to find the input file
	InputStream in = FileManager.get().open(file);
	if (in == null) {
	    throw new IllegalArgumentException("File: " + file + " not found");
	}
	
	// read the RDF/XML file
	model.read(in, null);
    }
    
    public void printModel()
    { 
	model.write(System.out);

	/*
        StmtIterator iter = model.listStatements();
	
        // print out the predicate, subject and object of each statement
        while (iter.hasNext()) {
            Statement stmt      = iter.nextStatement();         // get next statement
            Resource  subject   = stmt.getSubject();   // get the subject
            Property  predicate = stmt.getPredicate(); // get the predicate
            RDFNode   object    = stmt.getObject();    // get the object
	    
            System.out.print(subject.toString());
            System.out.print(" " + predicate.toString() + " ");
            if (object instanceof Resource) {
                System.out.print(object.toString());
            } else {
                // object is a literal
                System.out.print(" \"" + object.toString() + "\"");
            }
            System.out.println(" .");
	}
	*/
    }

    public Iterator getInstancesOfClass(String className) 
    {
	// get the instances of a class
	OntClass onto = model.getOntClass( className );
	
	if(onto == null) {
	    System.out.println("ONT CLASS IS NULL");
	    return (new ArrayList()).iterator();
	}
	
	Iterator instances = onto.listInstances();
	return instances;
    }

    public String getNamespaceByPrefix(String namespacePrefix)
    {
	//http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#
	return model.getNsPrefixURI(namespacePrefix);
    }

    /**
     * @param proNameSpace property namespace
     * @param proLocalName property name
     * @param ind individual object
     * @return statement containing the property info 
     */
    public com.hp.hpl.jena.rdf.model.Statement getPropertyOf(String proNameSpace, String proLocalName, Individual ind ) 
    {
	com.hp.hpl.jena.rdf.model.Property property = model.getProperty(proNameSpace, proLocalName);
	com.hp.hpl.jena.rdf.model.Statement stm = ind.getProperty(property);
	return stm;
    }

    public OntModel getModel() {
	return model;
    }
   
    public void insertInstance(String classURI, String className, String instanceURI, String instanceName) throws DuplicatedEntryException, UnknownClassException
    {
	Resource rs = model.getResource(classURI + className);
	if(rs == null) {
	    throw new UnknownClassException(className);
	}
	
	//OntClass onto = model.getOntClass(classURI + className);
		
	Individual ind = model.getIndividual(instanceURI + instanceName);
	if(ind != null) {
	    throw new  DuplicatedEntryException(instanceName);
	}
	ind = model.createIndividual(instanceURI + instanceName, rs);	
	ind.setOntClass(rs);
	
    }

    public void deleteInstance(String instanceURI, String instanceName) throws NonExistenceEntryException, UnknownException
    {
	model.enterCriticalSection(Lock.READ);
	Individual ind = model.getIndividual(instanceURI + instanceName);
	if(ind == null) {
	    throw new  NonExistenceEntryException(instanceName);
	}
	
	//ind = model.createIndividual(instanceURI + instanceName, rs);	
	// delete it. ... 	
	try {
	    for(StmtIterator si = ind.listProperties(); si.hasNext(); ) {
		Statement stmt = (Statement)si.next();
		model.remove(stmt);
	    }
	}
	catch(Exception e) {
	    System.out.println(e.toString() + "  ---  " + e.getMessage());
	    throw new UnknownException(e.getMessage());
	}
	finally {
	    model.leaveCriticalSection();
	}
    }
    
    //private String modelFileName;    
    //private Model model;
    private OntModel model;
}
