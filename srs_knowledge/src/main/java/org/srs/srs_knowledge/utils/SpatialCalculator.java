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
 * Date of creation: Apr 2012
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
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.srs_knowledge.srv.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import org.srs.srs_knowledge.knowledge_engine.*;
import ros.pkg.srs_msgs.msg.SRSSpatialInfo;

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
import math.geom3d.Point3D;

public class SpatialCalculator 
{
    public SpatialCalculator() {
    }

    public static void testTF() 
    {
	System.out.println(" +++++++++++++++++++HELLO TEST TF+++++++++++++++++++");

	Vector3d orig = new Vector3d(0,0,0);
	Quat4d q4d = new Quat4d(0.00000000e+00, 0.00000000e+00, 9.99999683e-01, 7.96326711e-04);
	ros.pkg.geometry_msgs.msg.Point point = new ros.pkg.geometry_msgs.msg.Point();
	point.x = 1;
	point.y = -1;
	point.z = 1;
	ros.pkg.geometry_msgs.msg.Point np = SpatialCalculator.transformPoint(orig, q4d, point);
	System.out.println("NEW POINT: " + np.x + "  " + np.y + "  " + np.z);

	System.out.println(" +++++++++++++++++++++++++++++++++++++++++++++++++++ ");
	
	SRSSpatialInfo o1 = new SRSSpatialInfo();
	o1.l = 5;
	o1.w = 5;
	o1.h = 5;

	o1.pose.position.x = 3;
	o1.pose.position.y = 3;
	o1.pose.position.z = 10;
	o1.pose.orientation.w = 1;

	SRSSpatialInfo o2 = new SRSSpatialInfo();
	o2.l = 10;
	o2.w = 10;
	o2.h = 10;
	o2.pose.position.z = 0;
	o2.pose.orientation.w = 1;
	if(SpatialCalculator.ifOnObject(o1, o2, 1)) {
	    System.out.println(" YES. O1 is on O2");
	}
	else {
	    System.out.println(" NO. O1 is NOT on O2");
	}

	System.out.println(" +++++++++++++++++++++++++++++++++++++++++++++++++++ ");
	BoundingBoxDim bbd = InformationRetrieval.retrieveBoundingBoxInfo(OntoQueryUtil.GlobalNameSpace + "Milkbox");
	System.out.println("Retrieved DIm: " + bbd.l + "  " + bbd.w + "  " + bbd.h);
	System.out.println(" +++++++++++++++++++++++++++++++++++++++++++++++++++ ");
    }


    public static ros.pkg.geometry_msgs.msg.Point transformPoint(Vector3d translation, Quat4d rotation, ros.pkg.geometry_msgs.msg.Point point) {
	
	tfjava.StampedTransform transform = new tfjava.StampedTransform(translation, rotation, null, "", "");
	Point3d p = new Point3d(point.x, point.y, point.z);
	Point3d newp = new Point3d();
	transform.transformPoint(p, newp);
	
	ros.pkg.geometry_msgs.msg.Point ret = new ros.pkg.geometry_msgs.msg.Point();
	ret.x = newp.x;
	ret.y = newp.y;
	ret.z = newp.z;
	return ret;
    }

    public static Point3d transformPoint(StampedTransform transform, Point3d point) {
	Point3d ret = new Point3d();
	transform.transformPoint(point, ret);
	return ret;
    }

    public static boolean ifOverlapping(Polygon2D p1, Polygon2D p2) {
	Polygon2D overlap = Polygon2DUtils.intersection(p1, p2);
	return overlap.getArea() > 0;
    }
    
    /**
     * @param poseCfg 0: pose at bottom of object. 1: pose at center of object
     */
    public static boolean ifOnObject(SRSSpatialInfo obj1, SRSSpatialInfo obj2, int poseCfg) {
	// error checking ..

	ArrayList<ros.pkg.geometry_msgs.msg.Point> corners1 = getBoundingBoxTopCorners(obj1);
	ArrayList<ros.pkg.geometry_msgs.msg.Point> corners2 = getBoundingBoxTopCorners(obj2);
	if(ifOverlapping(createPolygon(corners1), createPolygon(corners2))) {
	    //System.out.println("YES.... OVerlapping..");
	}
	else{
	    //System.out.println("NOOO.... OVerlapping..");
	}
	switch(poseCfg) {
	case 0:
	    return ifOverlapping(createPolygon(corners1), createPolygon(corners2)) && Math.abs(obj1.pose.position.z - obj2.h + obj2.pose.position.z) <= obj1.h/2;
	case 1:
	    return ifOverlapping(createPolygon(corners1), createPolygon(corners2)) && Math.abs(obj1.pose.position.z - obj1.h/2 - obj2.h + obj2.pose.position.z - obj2.h/2) <= obj1.h/2;
	default:
	    return ifOverlapping(createPolygon(corners1), createPolygon(corners2)) && obj2.pose.position.z <= obj1.pose.position.z;
	}
    }
    
    private static Polygon2D createPolygon(ArrayList<ros.pkg.geometry_msgs.msg.Point> vertex) {
	SimplePolygon2D p = new SimplePolygon2D();
	for (ros.pkg.geometry_msgs.msg.Point v : vertex) {
	    Point2D p2d = new Point2D(v.x, v.y);
	    p.addVertex(p2d);
	}
	
	return p;
    }

    private static ArrayList<ros.pkg.geometry_msgs.msg.Point> getBoundingBoxTopCorners(SRSSpatialInfo obj) {
	
	ArrayList<ros.pkg.geometry_msgs.msg.Point> corners = new ArrayList<ros.pkg.geometry_msgs.msg.Point>();
	Vector3d orig = new Vector3d(obj.pose.position.x, obj.pose.position.y, obj.pose.position.z);
	Quat4d q4d = new Quat4d(obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.x, obj.pose.orientation.w);
	ros.pkg.geometry_msgs.msg.Point point = new ros.pkg.geometry_msgs.msg.Point();
	point.x = -obj.l/2;
	point.y = -obj.w/2;
	point.z = obj.h;
	ros.pkg.geometry_msgs.msg.Point np = SpatialCalculator.transformPoint(orig, q4d, point);
	corners.add(np);

	point.x = obj.l/2;
	point.y = -obj.w/2;
	point.z = obj.h;
	np = SpatialCalculator.transformPoint(orig, q4d, point);
	corners.add(np);

	point.x = obj.l/2;
	point.y = obj.w/2;
	point.z = obj.h;
	np = SpatialCalculator.transformPoint(orig, q4d, point);
	corners.add(np);

	point.x = -obj.l/2;
	point.y = obj.w/2;
	point.z = obj.h;
	np = SpatialCalculator.transformPoint(orig, q4d, point);
	corners.add(np);

	return corners;
    }

    // simply use awt Polygon.contains() 
    //private static boolean pointInPolygon(double x, double y, Point2D[] ps) {
	
    //	return false;
    // }

    public static String nearestObject(ros.pkg.geometry_msgs.msg.Pose pose, String objectTypeURI) {
	//GetObjectsOnMap.Response resObj = OntoQueryUtil.getObjectsOnMap(OntoQueryUtil.MapName, true);
	
	//Iterator<Individual> instancesOfObject = KnowledgeEngine.ontoDB.getInstancesOfClass(className);
	
	/*
	  double minDis = 0;
	String res = "";
	for (int i = 0; i < resObj.objects.size(); i++) {
	    double temp = SpatialCalculator.distanceBetween(pose, resObj.objectsInfo.get(i).pose);

	    if(temp <= minDis) {
		minDis = temp;
		res = resObj.objects.get(i);
	    }
	}
	return res;
	*/
	GetObjectsOnMap.Response re = new GetObjectsOnMap.Response();
	re = OntoQueryUtil.getObjectsOnMapOfType(objectTypeURI, true);
	double minDis = 0;
	String res = "";
	if(re.objects.size() > 0) {
	    minDis = SpatialCalculator.distanceBetween(pose, re.objectsInfo.get(0).pose);
	    res = re.objects.get(0);
	}
	else {
	    return "";
	}

	for(int i = 1; i < re.objects.size(); i++) {
	    double temp = SpatialCalculator.distanceBetween(pose, re.objectsInfo.get(i).pose);
	    
	    if(temp <= minDis) {
		minDis = temp;
		res = re.objects.get(i);
	    }
	}
	return OntoQueryUtil.ObjectNameSpace + res;
    } 

    private static double distanceBetween(ros.pkg.geometry_msgs.msg.Pose obj1, ros.pkg.geometry_msgs.msg.Pose obj2) {
	
	math.geom3d.Point3D p1 = new math.geom3d.Point3D(obj1.position.x, obj1.position.y, obj1.position.z);
	math.geom3d.Point3D p2 = new math.geom3d.Point3D(obj2.position.x, obj2.position.y, obj2.position.z);
	return p1.getDistance(p2);
    }

    /**
     * 
     */
    public static String workspaceHolding(SRSSpatialInfo spaInfo) {

	GetWorkspaceOnMap.Response resWS = OntoQueryUtil.getWorkspaceOnMap(OntoQueryUtil.MapName, true);
	// pair-wise comparison --- if condition met, then update the knowledge base

	for (int j = 0; j < resWS.objectsInfo.size(); j++) {
	    if(SpatialCalculator.ifOnObject(spaInfo, resWS.objectsInfo.get(j), -1)) {
		return OntoQueryUtil.ObjectNameSpace + resWS.objects.get(j);
	    }
	}

	//if (j == resWS.objectsInfo.size()) {
	//System.out.println("NO MATCH.... found for : [" + spaInfo.l + "  " + spaInfo.h + "  " + spaInfo.w + "]");
	//return "";
	//}
	
	return "";
    }

}

