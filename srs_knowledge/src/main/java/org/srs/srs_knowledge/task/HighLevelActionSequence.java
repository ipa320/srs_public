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
 *	 * Neither the name of the Fraunhofer Institute for Manufacturing 
 *	   Engineering and Automation (IPA) nor the names of its
 *	   contributors may be used to endorse or promote products derived from
 *	   this software without specific prior written permission.
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
	System.out.println("++++++++ >>   " + indOfCurrent + "   SIZE: " + highLevelActionList.size());
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
