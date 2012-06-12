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
import java.util.ConcurrentModificationException;
import java.util.NoSuchElementException;
import java.util.HashSet;
import java.util.Iterator;
import java.util.Collections;
import java.util.Comparator;
import ros.*;
import ros.communication.*;
import ros.pkg.srs_knowledge.srv.QuerySparQL;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.srs_knowledge.msg.SRSSpatialInfo;
import com.hp.hpl.jena.shared.Lock;
import ros.pkg.srs_knowledge.srv.PlanNextAction;
import ros.pkg.srs_knowledge.srv.TaskRequest;
import ros.pkg.srs_knowledge.srv.GetObjectsOnMap;
import ros.pkg.srs_knowledge.srv.GetWorkspaceOnMap;
import com.hp.hpl.jena.rdf.model.Statement;
import org.srs.srs_knowledge.task.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import ros.pkg.geometry_msgs.msg.Pose;
import ros.pkg.srs_knowledge.srv.GetObjectsOnMap;
import ros.pkg.srs_knowledge.srv.GetWorkspaceOnMap;

import java.util.Properties;

import java.io.IOException;
import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Set;

import org.srs.srs_knowledge.utils.*;

import java.io.StringWriter;
import java.math.BigInteger;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;

public class JSONParser
{
    public Task parseJSONToTask(String encodedTask) {
	Task t = null;
	return t;
    }
}