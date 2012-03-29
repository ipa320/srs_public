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

public class ActionFeedback extends ArrayList<String> {
    public ActionFeedback(ArrayList<String> genericFeedback) {
	super(genericFeedback);
	System.out.println("Constructor: extents ArrayList of size: " + this.size());
    }
    
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

}