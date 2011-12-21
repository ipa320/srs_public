package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList;

import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;

import org.srs.srs_knowledge.task.*;
import org.srs.srs_knowledge.knowledge_engine.*;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;

public class SubActionSequence {
    public SubActionSequence() {
    }

    //public void 

    public void appendSingleAction(HighLevelActionUnit actUnit) {
	highLevelActionList.add(actUnit);
    }
    
    public int getSizeOfHighLevelActionList() {
	return highLevelActionList.size();
    }

    protected int indOfCurrent = 0;
    
    protected ArrayList<HighLevelActionUnit> highLevelActionList = new ArrayList<HighLevelActionUnit>();
}
