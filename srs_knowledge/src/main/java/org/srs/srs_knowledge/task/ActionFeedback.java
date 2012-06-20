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

package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import ros.pkg.geometry_msgs.msg.Pose;
import org.srs.srs_knowledge.knowledge_engine.*;
import org.srs.srs_knowledge.task.Task;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.JSONValue;
import org.json.simple.parser.ParseException;
import org.json.simple.parser.JSONParser;

public class ActionFeedback extends ArrayList<String> {
    /**
     * deprecated
     */
    public ActionFeedback(ArrayList<String> genericFeedback) {
	super(genericFeedback);
	System.out.println("Constructor: extents ArrayList of size: " + this.size());
    }

    public ActionFeedback(String jsonGenericFeedback) throws ParseException {
	this.jsonFeedback = jsonGenericFeedback;
	JSONParser parser = new JSONParser();
	Object obj = parser.parse(this.jsonFeedback);
	this.jsonFBObject = (JSONObject)obj;

	System.out.println("Constructor: using json feedback.");
    }

    public Pose getDetectedObjectPose() {
	Pose pos = new Pose();

	try {
	    JSONObject fb = (JSONObject)this.jsonFBObject.get("feedback");
	    String actionName = (String)fb.get("action");
	    if(!actionName.equals("detect")) {
		System.out.println("Action type is not 'detect'... Format error");
		return null;	    
	    }
	    JSONObject detObj = (JSONObject)fb.get("object");
	    
	    String objectName = (String)detObj.get("object_type");
	    System.out.println("Detected object: " + objectName);
	    JSONObject jsonPos = (JSONObject)fb.get("pose");
	    pos.position.x = ((Number)(jsonPos.get("x"))).doubleValue();
	    pos.position.y = ((Number)(jsonPos.get("y"))).doubleValue();
	    pos.position.z = ((Number)(jsonPos.get("z"))).doubleValue();
	    pos.orientation.x = ((Number)(jsonPos.get("rotx"))).doubleValue();
	    pos.orientation.y = ((Number)(jsonPos.get("roty"))).doubleValue();
	    pos.orientation.z = ((Number)(jsonPos.get("rotz"))).doubleValue();
	    pos.orientation.w = ((Number)(jsonPos.get("rotw"))).doubleValue();	    
	}
	catch(Exception e) {
	    System.out.println(e.toString());
	    return null;
	}

	return pos;
    }

    /**
     * deprecated, kept only for legacy code compatible
     */
    public static Pose toPose(ActionFeedback fb) {
	if(fb.size() < 9) {
	    System.out.println("List array length smaller than 9.    " + fb.size());
	    System.out.println(fb);
	    return null;
	}
	if(!fb.get(0).equals("detect")) {
	    System.out.println("First item is not detect... Format error");
	    return null;
	}
	try {
	    Pose pos = new Pose(); 
	    
	    System.out.println("detect: Object " + fb.get(1));
	    pos.position.x = Double.valueOf(fb.get(2));
	    pos.position.y = Double.valueOf(fb.get(3));
	    pos.position.z = Double.valueOf(fb.get(4));
	    pos.orientation.x = Double.valueOf(fb.get(5));
	    pos.orientation.y = Double.valueOf(fb.get(6));
	    pos.orientation.z = Double.valueOf(fb.get(7));
	    pos.orientation.w = Double.valueOf(fb.get(8));
	    return pos;
	}
	catch(Exception e) {
	    
	    System.out.println(e.toString());
	    return null;

	}
    }
    private String jsonFeedback = "";
    private JSONObject jsonFBObject = new JSONObject();
}