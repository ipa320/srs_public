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
import org.srs.srs_knowledge.knowledge_engine.*;
import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;

import org.srs.srs_knowledge.task.Task;

public class MoveTask extends org.srs.srs_knowledge.task.Task
{
	public MoveTask(String targetContent, Pose2D userPose) 
	{
	    this.initTask(targetContent, userPose);
	    setTaskType(TaskType.MOVETO_LOCATION);
	}
    
    private void initTask(String targetContent, Pose2D userPose) {
		acts = new ArrayList<ActionTuple>();
		
		setTaskTarget(targetContent);
		System.out.println("TASK.JAVA: Created CurrentTask " + "move "
				+ targetContent);
		constructTask();
	}
	
	protected boolean constructTask() {
		return createSimpleMoveTaskNew();
	}

	private boolean createSimpleMoveTaskNew() {
		// boolean addNewActionTuple(ActionTuple act)
		ActionTuple act = new ActionTuple();

		CUAction ca = new CUAction();
		GenericAction genericAction = new GenericAction();

		double x = 1;
		double y = 1;
		double theta = 0;

		if (this.targetContent.charAt(0) == '['
				&& this.targetContent.charAt(targetContent.length() - 1) == ']') {
			StringTokenizer st = new StringTokenizer(targetContent, " [],");
			if (st.countTokens() == 3) {
				try {
					x = Double.parseDouble(st.nextToken());
					y = Double.parseDouble(st.nextToken());
					theta = Double.parseDouble(st.nextToken());
					System.out.println(x + "  " + y + " " + theta);
				} catch (Exception e) {
					System.out.println(e.getMessage());
					return false;
				}
			}
		} else {
			System.out.println("======MOVE COMMAND FORMAT=======");
			// Ontology queries
			String mapNameSpace = OntoQueryUtil.ObjectNameSpace;
			String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
					+ "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
					+ "PREFIX mapNamespacePrefix: <" + mapNameSpace + ">\n";
			String queryString = "SELECT ?x ?y ?theta WHERE { "
					+ "<" + mapNameSpace + targetContent + ">"
					+ " srs:xCoordinate ?x . " + "<" + mapNameSpace + targetContent + ">" + " srs:yCoordinate ?y . "
					+ "<" + mapNameSpace + targetContent + ">"
					+ " srs:orientationTheta ?theta .}";
			//System.out.println(prefix + queryString + "\n");

			if (KnowledgeEngine.ontoDB == null) {
				System.out.println("Ontology Database is NULL");
				return false;
			}

			try {
			    ArrayList<QuerySolution> rset = KnowledgeEngine.ontoDB.executeQueryRaw(prefix
										   + queryString);
			    if (rset.size() == 0) {
				System.out.println("ERROR: No move target found from database");
				return false;
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
			    return false;
			}
		}

		genericAction.actionInfo.add("move");
		genericAction.actionInfo.add(Double.toString(x));
		genericAction.actionInfo.add(Double.toString(y));
		genericAction.actionInfo.add(Double.toString(theta));

		ca.generic = genericAction;
		ca.actionType = "generic";

		act.setCUAction(ca);
		act.setActionId(1);
		addNewActionTuple(act);

		// add finish action __ success

		act = new ActionTuple();

		ca = new CUAction();
		genericAction = new GenericAction();
		genericAction.actionInfo.add("finish_success");
		
		ca.generic = genericAction;
		ca.actionType = "generic";

		act.setActionName("finish_success");
		ca.status = 1;

		act.setCUAction(ca);
		act.setActionId(2);
		act.setParentId(1);
		act.setCondition(true);
		addNewActionTuple(act);

		// add finish action __ fail

		act = new ActionTuple();

		ca = new CUAction();
		genericAction = new GenericAction();
		genericAction.actionInfo.add("finish_fail");
		
		ca.generic = genericAction;
		ca.actionType = "generic";

		act.setActionName("finish_fail");

		ca.status = -1;
		act.setCUAction(ca);
		act.setActionId(3);
		act.setParentId(1);
		act.setCondition(false);
		addNewActionTuple(act);

		System.out.println("number of actions: " + acts.size());
		return true;
	}


    public boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery) {
	return false;
    }
}