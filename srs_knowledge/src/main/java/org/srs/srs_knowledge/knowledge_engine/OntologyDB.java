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

import java.io.*;

public class OntologyDB
{
    public OntologyDB(String filename)
    {
	this.modelFileName = filename;
	this.reloadOWLFile(filename);
    }

    public String executeQuery(String queryString)
    {
	/*
	String queryString = "PREFIX house: <http://www.semanticweb.org/ontologies/house.owl#> " + 
	    "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#> " + 
	    "SELECT ?room " + 
	    "WHERE { " +
	    "?room rdf:type house:Table . " +  
	    "}";
	*/
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
	}
	catch(Exception e){
	    System.out.println(e.getMessage());
	}
	qe.close();
	return r;
    }

    public Boolean reloadOWLFile(String file)
    {
	System.out.println("Load OWL File: " + file);
	// create an empty model
	this.model = ModelFactory.createDefaultModel();
	
	// use the FileManager to find the input file
	InputStream in = FileManager.get().open( file );
	if (in == null) {
	    throw new IllegalArgumentException("File: " + file + " not found");
	}
	
	// read the RDF/XML file
	model.read(in, null);
	
	return true;
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

    private String modelFileName;    
    private Model model;
}