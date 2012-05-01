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
 * @author Ze Ji, email: jiz1(at)cf.ac.uk
 *
 * Date of creation: April 2012
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
 *	 * Neither the name of the school of engineering, Cardiff University
 *         nor the names of its contributors may be used to endorse or promote 
 *         products derived from this software without specific prior written 
 *         permission.
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

package org.srs.srs_knowledge.utils;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList;
import java.util.Iterator;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import org.srs.srs_knowledge.knowledge_engine.*;

import ros.*;
import ros.communication.*;

import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.vocabulary.*;
import com.hp.hpl.jena.util.FileManager;

import com.hp.hpl.jena.rdf.model.Property;
import com.hp.hpl.jena.ontology.OntClass;
import com.hp.hpl.jena.ontology.OntModel;
import com.hp.hpl.jena.rdf.model.Statement;
import com.hp.hpl.jena.ontology.Individual;
import com.hp.hpl.jena.shared.Lock;
import com.hp.hpl.jena.ontology.OntResource;

import tfjava.*;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;

import math.geom2d.*;
import math.geom2d.line.LineSegment2D;
import math.geom2d.polygon.Polygon2DUtils;
import math.geom2d.polygon.Polygon2D;
import math.geom2d.polygon.SimplePolygon2D;

import math.geom2d.Point2D;

import ros.pkg.srs_object_database_msgs.srv.GetObjectId;
//import ros.pkg.srs_object_database_msgs.msg.*;

public class InformationRetrieval 
{
    /**
     * @param objectClassURI  the object name that the bounding box info is needed. Should be the full URI (not only the local name) 
     */
    public static BoundingBoxDim retrieveBoundingBoxInfo(String objectClassURI) {
	// global variables (ROS, Ontology handler etc)
	// if there does not exist such an object, then insert a new one
	// bounding box can be obtained from HHDB
	BoundingBoxDim d = new BoundingBoxDim();
	// look for existing or history data
	
	Iterator<Individual> instances = KnowledgeEngine.ontoDB.getInstancesOfClass(objectClassURI);
	if(instances != null) {
	    if(instances.hasNext()) {
		com.hp.hpl.jena.rdf.model.Statement stm;
		Individual temp = instances.next();
		stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "widthOfObject", temp);
		d.w = getFloatOfStatement(stm, -1000);
		stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "heightOfObject", temp);
		d.h = getFloatOfStatement(stm, -1000);
		stm = KnowledgeEngine.ontoDB.getPropertyOf(OntoQueryUtil.GlobalNameSpace, "lengthOfObject", temp);
		d.l = getFloatOfStatement(stm, -1000);
		if(d.w != -1000 && d.h != -1000 && d.l != -1000) {
		    return d;
		}
	    }
	}
	
	// if no, check hhdb by calling ros service 

	ServiceClient<GetObjectId.Request, GetObjectId.Response, GetObjectId> sc = KnowledgeEngine.nodeHandle.serviceClient("get_models" , new GetObjectId(), false);
	
	GetObjectId.Request rq = new GetObjectId.Request();	
	
	rq.type = "item";
	String[] subStrs = objectClassURI.trim().split("#");
	String objectClass = objectClassURI;
	if(subStrs.length == 2) {
	    objectClass = subStrs[1];
	}
	rq.item = objectClass;
	ArrayList<String> modelDesc = new ArrayList<String>();
	ArrayList<String> modelCategory = new ArrayList<String>();
	int id = -1000;
	
	try {
	    GetObjectId.Response res = sc.call(rq);
	    int[] ids = res.model_ids;
	    modelDesc = res.model_desc;
	    modelCategory = res.model_category;
	    for (int i = 0; i < modelDesc.size(); i++) {
		if(modelDesc.get(i).equals(objectClass)) {
		    // objectClass in Knowledgebase is model_desc in HHDB, not model_category... 
		    id = ids[i];
		    
		    d.l = res.model_x_size[i];
		    d.w = res.model_y_size[i];
		    d.h = res.model_z_size[i];
		    if(d.l != 0 && d.h != 0 && d.w != 0) {
			sc.shutdown();
			return d;
		    }
		}
	    }
	}
	catch(RosException e) {
	    System.out.println(e.getMessage());
	    sc.shutdown();
	    return new BoundingBoxDim();
	}	
	sc.shutdown();

	return new BoundingBoxDim();
    }

    public static float getFloatOfStatement(com.hp.hpl.jena.rdf.model.Statement stm, float defaultIfInvalid) {
	float t = defaultIfInvalid;
	try { 
	    t = stm.getFloat();
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
	return t;
    }

    private static int getIntOfStatement(Statement stm, int defaultIfInvalid)
    {
	int t = defaultIfInvalid;
	try { 
	    t = stm.getInt();
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
	return t;
    }
}