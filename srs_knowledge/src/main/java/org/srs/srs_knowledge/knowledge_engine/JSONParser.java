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

import java.io.*;
import java.util.ArrayList; 
import java.util.NoSuchElementException;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Map;

import java.io.StringWriter;
import java.math.BigInteger;

import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.srs_knowledge.msg.SRSSpatialInfo;
import ros.pkg.srs_knowledge.srv.PlanNextAction;
import ros.pkg.srs_knowledge.srv.TaskRequest;
import ros.pkg.srs_knowledge.srv.GetObjectsOnMap;
import ros.pkg.srs_knowledge.srv.GetWorkspaceOnMap;
import org.srs.srs_knowledge.task.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import ros.pkg.geometry_msgs.msg.Pose;

import org.srs.srs_knowledge.utils.*;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;

public class JSONParser
{

    /**
     * decode a single task ... 
     * the original task request sent from UIs should be first decoded by DM, and extract a single task request for here
     */
    public static Task parseJSONToTask(String encodedTask) {
	Task t = null;
	// {"tasks":[{"time_schedule":1263798000000,"task":"move","destination":{"pose2d_string":"[0 1 3.14]"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}
	System.out.println("=======decode======= " + encodedTask);
        
	Object obj = JSONValue.parse(encodedTask);

	JSONObject task = (JSONObject)obj;

	String taskName = (String)task.get("task");
	if(taskName.equals("move")) {
	    t = parseJSONToMoveTask(task);
	}
	else if(taskName.equals("get")) {
	    t = parseJSONToGetTask(task);
	}
	else if(taskName.equals("fetch")) {
	    t = parseJSONToFetchTask(task);
	}
	else if(taskName.equals("search")) {
	    t = parseJSONToMoveTask(task);
	}
	else if(taskName.equals("deliver")) {
	    t = parseJSONToMoveTask(task);
	}
	else if(taskName.equals("stop")) {
	    t = parseJSONToMoveTask(task);
	}
	else {
	    System.out.println("Invalid task type -- error when decoding json task format");
	    return t;
	}

	return t;
    }
    
    public static MoveTask parseJSONToMoveTask(JSONObject task) {
	MoveTask mt = null;
	
	java.lang.Number time = (java.lang.Number)task.get("time_schedule");
	JSONObject destination = (JSONObject)task.get("destination");	
	String dest = "";
	if(destination.containsKey("predefined_pose")) {
	    dest = (String)destination.get("predefined_pose");
	}
	else if(destination.containsKey("pose2d_string")) {
	    dest = (String)destination.get("pose2d_string");
	}
	else if(destination.containsKey("pose2d")) {
	    JSONObject pose = (JSONObject)destination.get("pose2d");
	    double x = ((Number)(pose.get("x"))).doubleValue();
	    double y = ((Number)(pose.get("y"))).doubleValue();
	    double theta = ((Number)(pose.get("theta"))).doubleValue();
	    dest = "[" + x + " " + y + " " + theta + "]";
	}
	else {
	    return null;
	}

	mt = new MoveTask(dest);
	return mt;
    }

    public static GetObjectTask parseJSONToGetTask(JSONObject task) {
	GetObjectTask got = null;
	// {"task":"get","object":{"object_type":"Book"}}
	
	JSONObject obj = (JSONObject)task.get("object");
	String objectType = (String)(obj.get("object_type"));
	
	// This should be passed by dm, not read from param server... to minimize the dependence on other packages
	String graspType = (String)(obj.get("grasping_type"));
	GetObjectTask.GraspType graspingType = GetObjectTask.GraspType.MOVE_AND_GRASP;
	if(graspType.equals("Simple")) {
	    graspingType = GetObjectTask.GraspType.MOVE_AND_GRASP;
	}
	else if(graspType.equals("Planned")) {
	    graspingType = GetObjectTask.GraspType.JUST_GRASP;
	}
      
	got = new GetObjectTask(objectType, graspingType);
	return got;
    }

    public static FetchObjectTask parseJSONToFetchTask(JSONObject task) {
	FetchObjectTask fot = null;
	// {"task":"get","object":{"object_type":"Book"}}
	JSONObject obj = (JSONObject)task.get("object");
	String objectType = (String)(obj.get("object_type"));
	//String graspType = (String)(obj.get("grasping_type"));
	JSONObject deliverDest = (JSONObject)task.get("deliver_destination");
	String dest = "";
	if(deliverDest.containsKey("predefined_pose")) {
	    dest = (String)deliverDest.get("predefined_pose");
	}
	else if(deliverDest.containsKey("pose2d_string")) {
	    dest = (String)deliverDest.get("pose2d_string");
	}
	else if(deliverDest.containsKey("pose2d")) {
	    JSONObject pose = (JSONObject)deliverDest.get("pose2d");
	    double x = ((Number)(pose.get("x"))).doubleValue();
	    double y = ((Number)(pose.get("y"))).doubleValue();
	    double theta = ((Number)(pose.get("theta"))).doubleValue();
	    dest = "[" + x + " " + y + " " + theta + "]";
	}
	else {
	    return null;
	}

	// This should be passed by dm, not read from param server... to minimize the dependence on other packages
	String graspType = (String)(obj.get("grasping_type"));
	GetObjectTask.GraspType graspingType = GetObjectTask.GraspType.MOVE_AND_GRASP;
	if(graspType.equals("Simple")) {
	    graspingType = GetObjectTask.GraspType.MOVE_AND_GRASP;
	}
	else if(graspType.equals("Planned")) {
	    graspingType = GetObjectTask.GraspType.JUST_GRASP;
	}

	fot = new FetchObjectTask(objectType, dest, graspingType);
	return fot;
    }
}