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

import com.hp.hpl.jena.query.Query;
import com.hp.hpl.jena.query.QueryFactory;
import com.hp.hpl.jena.query.ResultSetFormatter;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import com.hp.hpl.jena.ontology.Individual;
import ros.*;
import ros.communication.*;
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
import ros.pkg.std_msgs.msg.String;
import java.util.Properties;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList; 
import java.util.Iterator;


public class RosUtil {
    public static Pose getRobotCurrentPose() {
	//Ros ros = Ros.getInstance();
	//ros.init("testKKKK");
	Pose pos = new Pose();
	//NodeHandle nn = ros.createNodeHandle();
	// /robot_pose_ekf/odom_combined
	try {
	Subscriber.QueueingCallback<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped> callback =
	    new Subscriber.QueueingCallback<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped>();
	Subscriber<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped> sub =
	    KnowledgeEngine.nodeHandle.subscribe("/robot_pose_ekf/odom_combined", new ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped(), callback, 10);
	
	KnowledgeEngine.nodeHandle.spinOnce();
	while (!callback.isEmpty()) {
	    pos = callback.pop().pose.pose;
	    System.out.println(pos);
	}
	KnowledgeEngine.nodeHandle.shutdown();
	sub.shutdown();
	}
	catch(RosException e) {
	    System.out.println(e.toString());
	    return null;
	}
	catch(InterruptedException e)  {
	    System.out.println(e.toString());
	    return null;
	}
	return pos;
    }
    
    public static void testPub(java.lang.String c) throws RosException,Exception{

    	Publisher<ros.pkg.std_msgs.msg.String> pub =
    	       KnowledgeEngine.nodeHandle.advertise("/pub", new ros.pkg.std_msgs.msg.String(), 100);

    	  ros.pkg.std_msgs.msg.String m = new ros.pkg.std_msgs.msg.String();
    	  m.data = c;
	  for (int i = 0; i < 1000; i++) 
	      {
		  pub.publish(m);
		  Thread.sleep(1000);
	      }
    	  pub.shutdown();
	 
    }
    public static java.lang.String testSub() throws RosException,Exception {
	Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String> callback =
	    new Subscriber.QueueingCallback<ros.pkg.std_msgs.msg.String>();
	Subscriber<ros.pkg.std_msgs.msg.String> sub =
	    KnowledgeEngine.nodeHandle.subscribe("/topic_name", new ros.pkg.std_msgs.msg.String(), callback, 10);
	
	KnowledgeEngine.nodeHandle.spinOnce();
	java.lang.String ret = "";
	while (!callback.isEmpty()) {
	    ret = callback.pop().data;
	    System.out.println(ret);
	}
	sub.shutdown();
	return ret;
    }

}