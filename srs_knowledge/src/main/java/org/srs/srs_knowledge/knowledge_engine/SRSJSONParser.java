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
import ros.pkg.srs_msgs.msg.SRSSpatialInfo;
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
import org.json.simple.parser.ParseException;
import org.json.simple.parser.JSONParser;

public class SRSJSONParser
{

    /**
     * decode a single task ... 
     * the original task request sent from UIs should be first decoded by DM, and extract a single task request for here
     */
    public static Task parseJSONToTask(String encodedTask) {
	Task t = null;
	// {"tasks":[{"time_schedule":1263798000000,"task":"move","destination":{"pose2d_string":"[0 1 3.14]"}}],"initializer":{"device_type":"ui_loc","device_id":"ui_loc_0001"}}
	System.out.println("=> decode json: " + encodedTask);

	/*
	Object obj = JSONValue.parse(encodedTask);
	if (obj == null) {
	    System.out.println("Parsing Json error");
	    return null;
	}
	*/
	JSONParser parser = new JSONParser();
	JSONObject task = new JSONObject();
	try {
	    Object obj = parser.parse(encodedTask);
	    task = (JSONObject)obj;	    
	}
	catch(ParseException pe) {
	    System.out.println("Parsing Json error at position: " + pe.getPosition());
	    System.out.println(pe);
	    return null;
	}
	
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
	    t = parseJSONToSearchTask(task);
	}
	else if(taskName.equals("deliver")) {
	    t = parseJSONToGetTask(task);
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
	try {
	JSONObject obj = (JSONObject)task.get("object");
	String objectType = (String)(obj.get("object_type"));
	
	// This should be passed by dm, not read from param server... to minimize the dependence on other packages
	String graspType = (String)(task.get("grasping_type"));
	ConfigInfo.GraspType graspingType = ConfigInfo.GraspType.MOVE_AND_GRASP;
	if(graspType.equals("Simple")) {
	    graspingType = ConfigInfo.GraspType.MOVE_AND_GRASP;
	}
	else if(graspType.equals("Planned")) {
	    graspingType = ConfigInfo.GraspType.JUST_GRASP;
	}
	System.out.println("---->>> " + graspType);
      
	got = new GetObjectTask(objectType, graspingType);
	}
	catch(Exception e) {
	    System.out.println(e.toString());
	}
	
	return got;
    }

    public static FetchObjectTask parseJSONToFetchTask(JSONObject task) {
	FetchObjectTask fot = null;
	// {"task":"get","object":{"object_type":"Book"}}
	try {
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
	    String graspType = (String)task.get("grasping_type");
	    System.out.println("graspType -->  " + graspType);

	    ConfigInfo.GraspType graspingType = ConfigInfo.GraspType.MOVE_AND_GRASP;
	    if(graspType.equals("Simple")) {
		graspingType = ConfigInfo.GraspType.MOVE_AND_GRASP;
	    }
	    else if(graspType.equals("Planned")) {
		graspingType = ConfigInfo.GraspType.JUST_GRASP;
	    }

	    System.out.println("graspingType " + graspingType);
	    fot = new FetchObjectTask(objectType, dest, graspingType);
	}
	catch(Exception e) {
	    System.out.println(e.toString());
	}
	
	return fot;
    }

    public static SearchObjectTask parseJSONToSearchTask(JSONObject task) {
	SearchObjectTask sot = null;

	try {
	    JSONObject obj = (JSONObject)task.get("object");
	    String objectType = (String)(obj.get("object_type"));
	    
	    sot = new SearchObjectTask(objectType);
	}
	catch(Exception e) {
	    System.out.println(e.toString());
	}	
	return sot;
    }

    public static StopTask parseJSONToStopTask(JSONObject task) {
	StopTask st = new StopTask();
	return st;
    }

    public static String encodeMoveAction(String action, double x, double y, double theta) {
	JSONObject moveAct = new JSONObject();
	moveAct.put("action", action);

	JSONObject dest = new JSONObject();
	dest.put("x", x);
	dest.put("y", y);
	dest.put("theta", theta);

	JSONObject pos = new JSONObject();
	pos.put("pose2d", dest);

	moveAct.put("destination", pos);
	try {
	    StringWriter out = new StringWriter();
	    moveAct.writeJSONString(out);
	    String jsonText = out.toString();
	    return jsonText;
	}
	catch(IOException ie) {
	    System.out.println("IO Exception when writing JSON STRING");
	    System.out.println(ie.getMessage());
	    return "";
	}
    }

    public static String encodeCustomAction(String action, Map<String, Object> content) {
	JSONObject act = new JSONObject();
	act.put("action", action);
	try {
	    StringWriter out = new StringWriter();
	    act.writeJSONString(out);
	    String jsonText = out.toString();
	    return jsonText;
	}
	catch(IOException ie) {
	    System.out.println("IO Exception when writing JSON STRING");
	    System.out.println(ie.getMessage());
	    return "";
	}
    }

    public static String encodePutOnTrayAction(String action, String moveConfig) {
	JSONObject act = new JSONObject();
	act.put("action", action);
	act.put("move_config", moveConfig);
	try {
	    StringWriter out = new StringWriter();
	    act.writeJSONString(out);
	    String jsonText = out.toString();
	    return jsonText;
	}
	catch(IOException ie) {
	    System.out.println("IO Exception when writing JSON STRING");
	    System.out.println(ie.getMessage());
	    return "";
	}
    }

    public static String encodeDetectAction(String action, int hhId, String objectType, String workspace) {
	JSONObject obj = new JSONObject();
	obj.put("object_type", objectType);
	obj.put("object_id", hhId);
	obj.put("workspace", workspace);

	JSONObject detAct = new JSONObject();
	detAct.put("object", obj);
	detAct.put("action", action);

	try {
	    StringWriter out = new StringWriter();
	    detAct.writeJSONString(out);
	    String jsonText = out.toString();
	    return jsonText;
	}
	catch(IOException ie) {
	    System.out.println("IO Exception when writing JSON STRING");
	    System.out.println(ie.getMessage());
	    return "";
	}
    }
    
    public static String encodeGraspAction(String action, int hhId, String objectType, String workspace) {
	JSONObject obj = new JSONObject();
	obj.put("object_type", objectType);
	obj.put("object_id", hhId);
	if(workspace != null) {
	    obj.put("workspace", workspace);
	}
	else { 
	    obj.put("workspace", "");
	}

	JSONObject detAct = new JSONObject();
	detAct.put("object", obj);
	detAct.put("action", action);

	try {
	    StringWriter out = new StringWriter();
	    detAct.writeJSONString(out);
	    String jsonText = out.toString();
	    return jsonText;
	}
	catch(IOException ie) {
	    System.out.println("IO Exception when writing JSON STRING");
	    System.out.println(ie.getMessage());
	    return "";
	}
    }

    public static String encodeCheckObjectAction(String action, String objectType, double x, double y, double z, double orix, double oriy, double oriz, double oriw, double l, double w, double h) {
	JSONObject obj = new JSONObject();
	obj.put("object_type", objectType);

	JSONObject pose = new JSONObject();
	JSONObject position = new JSONObject();
	position.put("x", x);
	position.put("y", y);
	position.put("x", z);
	pose.put("position", position);
       
	JSONObject orientation = new JSONObject();
	orientation.put("x", orix);
	orientation.put("y", oriy);
	orientation.put("z", oriz);
	orientation.put("w", oriw);
	pose.put("orientation", orientation);
	
	JSONObject dim = new JSONObject();
	dim.put("l", l);
	dim.put("w", w);
	dim.put("h", h);
	
	obj.put("pose", pose);
	obj.put("dimension", dim);

	JSONObject checkAct = new JSONObject();
	checkAct.put("object", obj);
	checkAct.put("action", action);

	try {
	    StringWriter out = new StringWriter();
	    checkAct.writeJSONString(out);
	    String jsonText = out.toString();
	    return jsonText;
	}
	catch(IOException ie) {
	    System.out.println("IO Exception when writing JSON STRING");
	    System.out.println(ie.getMessage());
	    return "";
	}
    }

    public static String encodeObjectCategoryInfo(HashSet<String> items) {
	JSONArray cats = new JSONArray();
	Iterator<String> it = items.iterator();
	while (it.hasNext()) {
	    cats.add(it.next());
	}

	JSONObject cat = new JSONObject();
	cat.put("object_categories", cats);

	try {
	    StringWriter out = new StringWriter();
	    cat.writeJSONString(out);
	    String jsonText = out.toString();
	    return jsonText;
	}
	catch(IOException ie) {
	    System.out.println("IO Exception when writing JSON STRING");
	    System.out.println(ie.getMessage());
	    return "";
	}
    }


    public static JSONObject decodeJsonActionInfo(String jsonActionInfo) {
	JSONParser parser = new JSONParser();
	JSONObject actionInfo = new JSONObject();
	try {
	    Object obj = parser.parse(jsonActionInfo);
	    actionInfo = (JSONObject)obj;	    
	}
	catch(ParseException pe) {
	    System.out.println("Parsing Json error at position: " + pe.getPosition());
	    System.out.println(pe);
	    return null;
	}
	
	return actionInfo;
    }

    public static String encodeObjectProperties(Map<String, String> pro)
    {
	return JSONObject.toJSONString(pro);
    }

}