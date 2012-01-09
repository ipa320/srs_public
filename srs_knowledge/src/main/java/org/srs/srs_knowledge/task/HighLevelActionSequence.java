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

public class HighLevelActionSequence {
    public HighLevelActionSequence() {
	indOfCurrent = 0;
    }

    //public void 

    public void appendHighLevelAction(HighLevelActionUnit actUnit) {
	highLevelActionList.add(actUnit);
    }
    
    public int getSizeOfHighLevelActionList() {
	return highLevelActionList.size();
    }
    
    public boolean hasNextHighLevelActionUnit() {
	if(indOfCurrent + 1 < getSizeOfHighLevelActionList() && indOfCurrent + 1 >= 0) { 
	    return true;
	}
	else {
	    return false;
	}
    }

    public HighLevelActionUnit getNextHighLevelActionUnit() {
	HighLevelActionUnit ha;
	try{ 
	    ha = highLevelActionList.get(indOfCurrent + 1);
	    indOfCurrent = indOfCurrent + 1;
	}
	catch(NullPointerException ne) {
	    System.out.println(ne.getMessage());
	    return null;
	}
	return ha;
    }

    public HighLevelActionUnit getCurrentHighLevelActionUnit() {
	//HighLevelActionUnit ha;
	try{ 
	    return highLevelActionList.get(indOfCurrent);
	}
	catch(Exception ne) {
	    System.out.println(ne.getMessage());
	    return null;
	}
	//return ha;
    }

    public boolean shiftToNextActionUnit() {
	HighLevelActionUnit ha;
	if((indOfCurrent + 1 )< highLevelActionList.size()) {
	    indOfCurrent++;
	    return true;
	}  
	return false;
    }

    protected int indOfCurrent;
    
    protected ArrayList<HighLevelActionUnit> highLevelActionList = new ArrayList<HighLevelActionUnit>();
}
