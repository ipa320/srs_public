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
import com.hp.hpl.jena.shared.JenaException;
import ros.*;
import ros.communication.*;
//import ros.pkg.srs_knowledge.srv.AskForActionSequence;  // deprecated
//import ros.pkg.srs_knowledge.srv.GenerateSequence;
import ros.pkg.srs_knowledge.srv.QuerySparQL;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.srs_knowledge.msg.SRSSpatialInfo;

import ros.pkg.srs_knowledge.srv.PlanNextAction;
import ros.pkg.srs_knowledge.srv.TaskRequest;
import ros.pkg.srs_knowledge.srv.GetObjectsOnMap;
import ros.pkg.srs_knowledge.srv.GetWorkspaceOnMap;
import ros.pkg.srs_knowledge.srv.GetObjectsOnTray;
import ros.pkg.srs_knowledge.srv.InsertInstance;
import ros.pkg.srs_knowledge.srv.DeleteInstance;
import ros.pkg.srs_knowledge.srv.UpdatePosInfo;
import ros.pkg.srs_knowledge.srv.GetRoomsOnMap;
import ros.pkg.srs_knowledge.srv.GetPredefinedPoses;
import ros.pkg.srs_knowledge.srv.GetWorkspaceForObject;

import com.hp.hpl.jena.rdf.model.Statement;
import org.srs.srs_knowledge.task.*;
import ros.pkg.geometry_msgs.msg.Pose2D;

import java.util.Properties;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList; 
import java.util.Iterator;

import org.srs.srs_knowledge.utils.*;

import tfjava.*;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import javax.vecmath.Point3d;
import javax.vecmath.Matrix4d;


public class KnowledgeEngine
{
    public static Ros ros;
    public static OntologyDB ontoDB;

    public static NodeHandle nodeHandle;

    public KnowledgeEngine()
    {
	this.defaultContextPath();
    }

    public KnowledgeEngine(String contextPath)
    {
	this.setContextPath(contextPath);
    }

    public void setContextPath(String path)
    {
	this.confPath = path;
    }

    public boolean init(String cfgFile)
    {
	ros = Ros.getInstance();
	ros.init(nodeName);
	ros.logInfo("INFO: Start RosJava_JNI service");
	
	nodeHandle = ros.createNodeHandle();
	
	try {
	    initProperties(cfgFile);
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	    return false;
	}	

	try{
	    initQuerySparQL();
	    initPlanNextAction();
	    initTaskRequest();
	    initGetObjectsOnMap();
	    initGetWorkspaceOnMap();
	    initGetObjectsOnTray();
	    initInsertInstance();
	    initDeleteInstance();
	    initUpdatePosInfo();
	    initGetRoomsOnMap();
	    initGetPredefinedPoses();
	    initGetWorkspaceForObject();
	}
	catch(RosException e){
	    System.out.println(e.getMessage());
	    return false;
	}

	////////  TO REMOVE ::: ONLY FOR TESTING
	//this.testFunction();
	////////  END:::: TESTING
	// init spatial symbolic relation
	OntoQueryUtil.computeOnSpatialRelation();

	ros.spin();

	return true;
    }

    private ArrayList<String> parseOntologyFileNames(String names)
    {
	ArrayList<String> nameList = new ArrayList<String>();

	StringTokenizer st = new StringTokenizer(names, " ");
	
	while(st.hasMoreTokens()) {
	    nameList.add(this.confPath + st.nextToken());
	}
	for(String v: nameList) {
	    System.out.println(v);
	}
	return nameList;
    }
    
    private void initProperties(String cfgFile) throws Exception
    {
	InputStream is = new FileInputStream(this.confPath + cfgFile);
	this.config = new Properties();
	this.config.load(is);

	String ontoDBFile = config.getProperty("ontologyFile", "house.owl");

	ArrayList<String> nameList = parseOntologyFileNames(ontoDBFile);

	ontoDB = new OntologyDB(nameList);

	//ontoDB = new OntologyDB(this.confPath + ontoDBFile);

	this.nodeName = config.getProperty("nodename", "knowledge_srs_node");

	taskRequestService = config.getProperty("taskRequestService", "task_request");
	planNextActionService = config.getProperty("planNextActionService", "plan_next_action");
	generateSequenceService = config.getProperty("generateSequenceService", "generate_sequence");
	querySparQLService = config.getProperty("querySparQLService", "query_sparql");
	getObjectsOnMapService = config.getProperty("getObjectsOnMapService", "get_objects_on_map");
	getWorkSpaceOnMapService = config.getProperty("getWorkSpaceOnMapService", "get_workspace_on_map");
	getObjectsOnTrayService = config.getProperty("getObjectsOnTrayService", "get_objects_on_tray");
	insertInstanceService = config.getProperty("insertInstanceService", "insert_instance");
	deleteInstanceService = config.getProperty("deleteInstanceService", "delete_instance");
	updatePosInfoService = config.getProperty("updatePosInfoService", "update_pos_info");
	getRoomsOnMapService = config.getProperty("getRoomsOnMapService", "get_rooms_on_map");
	getPredefinedPosesService = config.getProperty("getPredefinedPosesService", "get_predefined_poses");
	getWorkspaceForObjectService = config.getProperty("getWorkspaceForObjectService", "get_workspace_for_object");

	graspActionMode = this.readGraspModeParam("/srs/grasping_type");
	//graspActionMode = config.getProperty("grasp_mode", "move_and_grasp");

	//mapNamespacePrefix = config.getProperty("map_namespace", "ipa-kitchen-map");
	mapName = config.getProperty("map_name", "ipa-kitchen-map");

	String robotName = config.getProperty("robot_name",  System.getenv("ROBOT"));
	/*
	if(ontoDB.getNamespaceByPrefix(mapNamespacePrefix) != null) {
	    mapNamespace = ontoDB.getNamespaceByPrefix(mapNamespacePrefix);
	    System.out.println("Map Name Space: " + mapNamespace);
	    System.out.println("Map Name Space Prefix : " + mapNamespacePrefix);
	}
	*/

	mapNamespace = config.getProperty("env_namespace", "http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#");
	globalNamespace = config .getProperty("ont_namespace", "http://www.srs-project.eu/ontologies/srs.owl#");
	
	//ontoQueryUtil = new OntoQueryUtil(mapNamespace, globalNamespace);
	OntoQueryUtil.ObjectNameSpace = mapNamespace;
	OntoQueryUtil.GlobalNameSpace = globalNamespace;
	//OntoQueryUtil.MapName = mapNamespacePrefix;
	OntoQueryUtil.MapName = mapName;
	OntoQueryUtil.RobotName = robotName;
    }

    private String readGraspModeParam(String paramServer) {
	String graspMode = "Simple";
	try{
	    graspMode = nodeHandle.getStringParam(paramServer);
	    System.out.println("Read Parameter --- > " + graspMode);
	}
	catch(RosException e) {
	    System.out.println("Caught RosException -- > " + e.toString());
	}
	return graspMode;
    }

    public void testOnto(String className)
    {
	try{
	    Iterator<Individual> instances = ontoDB.getInstancesOfClass(className);

	    if(instances.hasNext()) {
		while (instances.hasNext()) {
		    Individual temp = (Individual)instances.next();
		    System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		}
	    }
	    else
		System.out.println("<EMPTY>");

	    System.out.println();

	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}

    }
    
    private QuerySparQL.Response handleQuerySparQL(QuerySparQL.Request req)
    {
	QuerySparQL.Response re = new QuerySparQL.Response();
	String queryString = req.query;
	//System.out.println(queryString);

	re.result = ontoDB.executeQuery(queryString);
	return re;
    }

    private void initQuerySparQL() throws RosException
    {
	ServiceServer.Callback<QuerySparQL.Request, QuerySparQL.Response> scb = new ServiceServer.Callback<QuerySparQL.Request, QuerySparQL.Response>() {
            public QuerySparQL.Response call(QuerySparQL.Request request) {
		return handleQuerySparQL(request);
            }
	};

	ServiceServer<QuerySparQL.Request, QuerySparQL.Response, QuerySparQL> srv = nodeHandle.advertiseService( querySparQLService , new QuerySparQL(), scb);
    }

    private PlanNextAction.Response newHandlePlanNextAction( PlanNextAction.Request request) throws NullPointerException {
	PlanNextAction.Response res = new PlanNextAction.Response();
	CUAction ca = new CUAction(); 
	
	if(currentTask == null) {
	    System.out.println("Current Task is NULL. Send task request first");
	    res.nextAction = new CUAction(); // empty task
	    return res;
	    //throw new NullPointerException("Current Task is NULL. Send task request first");
	}

	if(request.resultLastAction == 0) {
	    ArrayList<String> feedback = request.genericFeedBack;
	    ca = currentTask.getNextCUAction(true, feedback); // no error. generate new action
	}
	else if (request.resultLastAction == 2) {
	    ros.logInfo("INFO: possible hardware failure with robot. cancel current task");
	    ros.logInfo("INFO: Task termintated");
	    
	    currentTask = null;
	    // TODO:
	    //currentSessionId = 1;
	    
	    res.nextAction = new CUAction();
	    return res;
	}
	else{
	    ca = currentTask.getNextCUAction(false, null);
	}
	
	if(ca == null) {
	    currentTask = null;
	    System.out.println("No further action can be planned. Terminate the task. ");

	    res.nextAction = new CUAction(); // empty task
	    //res.nextAction.status = 0;
	    return res;	    
	}
	//if(at.getActionName().equals("finish_success")) {
	if(ca.status == 1) {
	    currentTask = null;
	    //currentSessionId = 1;
	    System.out.println("Reached the end of the task. No further action to be executed. ");
	    res.nextAction = ca;
	    return res;	    
	}
	else if( ca.status == -1) {
	    currentTask = null;

	    System.out.println("Reached the end of the task. No further action to be executed. ");
	    res.nextAction = ca;	    
	    return res;	    
	}

	res.nextAction = ca;

	//ros.logInfo("INFO: Generate sequence of length: ");
	return res;

    }

    private void initPlanNextAction() throws RosException
    {
	ServiceServer.Callback<PlanNextAction.Request, PlanNextAction.Response> scb = new ServiceServer.Callback<PlanNextAction.Request, PlanNextAction.Response>() {
            public PlanNextAction.Response call(PlanNextAction.Request request) {
		return newHandlePlanNextAction(request);
            }
	};
	
	ServiceServer<PlanNextAction.Request, PlanNextAction.Response, PlanNextAction> srv = nodeHandle.advertiseService(planNextActionService, new PlanNextAction(), scb);
    }

    private TaskRequest.Response handleTaskRequest(TaskRequest.Request request)
    {
	TaskRequest.Response res = new TaskRequest.Response();
	
	System.out.println("Received request for new task");
	
	if(request.task.equals("move")) {
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }

	    try{
		if(request.parameters.size() == 0) {
		    currentTask = new MoveTask(request.content, null);
		    System.out.println("Created CurrentTask " + "move " + request.content);
		}
		else {	
		    currentTask = new MoveTask((String)request.parameters.get(0), null);
		    System.out.println("Created CurrentTask " + "move " + (String)request.parameters.get(0));	    
		}
	    }
	    catch(Exception e) {
		System.out.println(">>>  " + e.getMessage());
		currentTask = null;
		res.result = 1;
		res.description = "No action";
	    }
	    
	}
	else if(request.task.equals("get")){
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }

	    try{
		if(request.parameters.size() == 0) {

		    //GetObjectTask got = new GetObjectTask(request.task, request.content, request.userPose, nodeHandle);
		    GetObjectTask got = null;
		    if(this.graspActionMode.equals("Simple")) {
			got = new GetObjectTask(request.task, request.content, GetObjectTask.GraspType.MOVE_AND_GRASP);
			currentTask = (Task)got;
		    }
		    else if(this.graspActionMode.equals("Planned")) {
			got = new GetObjectTask(request.task, request.content, GetObjectTask.GraspType.JUST_GRASP);
			currentTask = (Task)got;
		    }
		    else {
			/// default
			got = new GetObjectTask(request.task, request.content, GetObjectTask.GraspType.MOVE_AND_GRASP);
			currentTask = (Task)got;
		    }
		    System.out.println("Created CurrentTask " + "get " + request.content);	    
		}
		else {	
		    GetObjectTask got = new GetObjectTask(request.task, request.parameters.get(0));
		    currentTask = (Task)got;
		    System.out.println("Created CurrentTask " + "get " + request.parameters.get(0));	    
		}
	    }
	    catch(Exception e) {
		System.out.println(">>>  " + e.getMessage());
		currentTask = null;
		res.result = 1;
		res.description = "No action";
	    }

	    // TODO: for other types of task, should be dealt separately. 
	    // here is just for testing
	}
	else if(request.task.equals("search")){
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    try{
		if(request.parameters.size() == 0) {
		    //GetObjectTask got = new GetObjectTask(request.task, request.content, request.userPose, nodeHandle);
		    SearchObjectTask got = null;
		    if(this.graspActionMode.equals("Simple")) {
			got = new SearchObjectTask(request.task, request.content, GetObjectTask.GraspType.MOVE_AND_GRASP);
			currentTask = (Task)got;
		    }
		    else if(this.graspActionMode.equals("Planned")) {
			got = new SearchObjectTask(request.task, request.content, GetObjectTask.GraspType.JUST_GRASP);
			currentTask = (Task)got;
		    }
		    else {
			/// default
			got = new SearchObjectTask(request.task, request.content, GetObjectTask.GraspType.MOVE_AND_GRASP);
			currentTask = (Task)got;
		    }
		    System.out.println("Created CurrentTask " + "search " + request.content);	    
		}
		else {	
		    SearchObjectTask got = new SearchObjectTask(request.task, request.content);
		    currentTask = (Task)got;
		    System.out.println("Created CurrentTask " + "search " + request.content);	    
		}
	    }
	    catch(Exception e) {
		System.out.println(">>>  " + e.getMessage());
		currentTask = null;
		res.result = 1;
		res.description = "No action";
	    }

	}
	else if(request.task.equals("fetch")){
	    
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    try{
		if(request.parameters.size() == 0) {
		    //GetObjectTask got = new GetObjectTask(request.task, request.content, request.userPose, nodeHandle);
		    FetchObjectTask got = null;
		    if(this.graspActionMode.equals("Simple")) {
			got = new FetchObjectTask(request.task, request.content, request.userPose, GetObjectTask.GraspType.MOVE_AND_GRASP);
			currentTask = (Task)got;
		    }
		    else if(this.graspActionMode.equals("Planned")) {
			got = new FetchObjectTask(request.task, request.content, request.userPose, GetObjectTask.GraspType.JUST_GRASP);
			currentTask = (Task)got;
		    }
		    else {
			/// default
			got = new FetchObjectTask(request.task, request.content, request.userPose, GetObjectTask.GraspType.MOVE_AND_GRASP);
			currentTask = (Task)got;
		    }
		    System.out.println("Created CurrentTask " + "fetch " + request.content);	    
		}
		else if (request.parameters.size() == 2) {	
		    FetchObjectTask got = new FetchObjectTask(request.task, request.parameters.get(0), request.parameters.get(1));
		    currentTask = (Task)got;
		    System.out.println("Created CurrentTask " + "fetch " + request.parameters.get(0) + " to " + request.parameters.get(1));	    
		}
		else {
		    currentTask = null;
		    res.result = 1;
		    res.description = "No action";
		}
	    }
	    catch(Exception e) {
		System.out.println(">>>  " + e.getMessage());
		currentTask = null;
		res.result = 1;
		res.description = "No action";
	    }
	}
	else if(request.task.equals("deliver")){
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    try{
		if(request.parameters.size() == 0) {
		    
		    //GetObjectTask got = new GetObjectTask(request.task, request.content, request.userPose, nodeHandle);
		    GetObjectTask got = new GetObjectTask(request.task, request.content);
		    currentTask = (Task)got;
		    System.out.println("Created CurrentTask " + "get " + request.content);	    
		}
		else {	
		    GetObjectTask got = new GetObjectTask(request.task, request.parameters.get(0));
		    currentTask = (Task)got;
		    System.out.println("Created CurrentTask " + "get " + request.content);	    
		}
	    }
	    catch(Exception e) {
		System.out.println(">>>  " + e.getMessage());
		currentTask = null;
		res.result = 1;
		res.description = "No action";
	    }
	}
	else if(request.task.equals("charging")) {
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    currentTask = new ChargingTask();
	    System.out.println("Created CurrentTask " + "charge ");
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
	}
	else if(request.task.equals("check") || request.task.equals("verify")){
	    
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }

	    currentTask = new CheckWorkspaceTask(request.content);
	    System.out.println("Created CurrentTask " + "check workspace " + request.content);
	}
	else if(request.task.equals("stop")) {
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    currentTask = new StopTask();
	    System.out.println("Created CurrentTask " + " STOP ");
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
	}
	else {
	    // TODO: for other types of task, should be dealt separately. 
	    // here is just for testing
	    // task not created for some reason
	    //currentTask = new GetObjectTask(request.task, request.content, null, ontoDB, ontoQueryUtil, n);
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
	    currentTask = null;
	    res.result = 1;
	    res.description = "No action";
	}

	//if(currentTask.getActionSequence().size() == 0) {
	if(currentTask == null) {
	    // task not created for some reason
	    res.result = 1;
	    res.description = "No action";
	    System.out.println("No action. Task is null");
	}
	else if(currentTask.isEmpty()) {
	    // task not created for some reason
	    res.result = 1;
	    res.description = "No action";
	    System.out.println("No action. Task is empty");
	}
	else {
	    res.result = 0;
	    currentSessionId++;  // TODO: generate unique id
	    res.sessionId = currentSessionId;
	    res.description = "No";
	    System.out.println("SESSION ID IS--> " + res.sessionId);
	}
	//CUAction ca = new CUAction(); 
	//res.nextAction = ca;
	//ros.logInfo("INFO: Generate sequence of length: ");
	return res;
    }
    
    private void initTaskRequest() throws RosException
    {
	ServiceServer.Callback<TaskRequest.Request, TaskRequest.Response> scb = new ServiceServer.Callback<TaskRequest.Request, TaskRequest.Response>() {
            public TaskRequest.Response call(TaskRequest.Request request) {
		return handleTaskRequest(request);
            }
	};

	System.out.println(taskRequestService);
	ServiceServer<TaskRequest.Request, TaskRequest.Response, TaskRequest> srv = nodeHandle.advertiseService(taskRequestService, new TaskRequest(), scb);
    }


    private GetObjectsOnMap.Response handleGetObjectsOnMap(GetObjectsOnMap.Request req)
    {
	String map = req.map;
	boolean ifGeometryInfo = req.ifGeometryInfo;
	
	return OntoQueryUtil.getObjectsOnMap(map, ifGeometryInfo);
    }

    private void initGetObjectsOnMap() throws RosException
    {
	ServiceServer.Callback<GetObjectsOnMap.Request, GetObjectsOnMap.Response> scb = new ServiceServer.Callback<GetObjectsOnMap.Request, GetObjectsOnMap.Response>() {
            public GetObjectsOnMap.Response call(GetObjectsOnMap.Request request) {
		return handleGetObjectsOnMap(request);
            }
	};
	
	System.out.println(getObjectsOnMapService);
	ServiceServer<GetObjectsOnMap.Request, GetObjectsOnMap.Response, GetObjectsOnMap> srv = nodeHandle.advertiseService(getObjectsOnMapService, new GetObjectsOnMap(), scb);
    }


    private GetWorkspaceOnMap.Response handleGetWorkspaceOnMap(GetWorkspaceOnMap.Request req)
    {
	return OntoQueryUtil.getWorkspaceOnMap(req.map, req.ifGeometryInfo);
    }

    private float getFloatOfStatement(Statement stm) 
    {
	float t = -1000;
	try { 
	    t = stm.getFloat();
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
	return t;
    }

    private int getIntOfStatement(Statement stm)
    {
	int t = -1000;
	try { 
	    t = stm.getInt();
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
	return t;
    }

    private void initInsertInstance() throws RosException 
    {
	ServiceServer.Callback<InsertInstance.Request, InsertInstance.Response> scb = new ServiceServer.Callback<InsertInstance.Request, InsertInstance.Response>() {
	    public InsertInstance.Response call(InsertInstance.Request request) {
		return handleInsertInstance(request);
	    }
	};

	System.out.println(insertInstanceService);
	ServiceServer<InsertInstance.Request, InsertInstance.Response, InsertInstance> srv = nodeHandle.advertiseService(insertInstanceService, new InsertInstance(), scb);
    }

    private InsertInstance.Response handleInsertInstance(InsertInstance.Request request)
    {
	InsertInstance.Response res = new InsertInstance.Response();
	String objectName = request.objectName;
	String objectClass = request.objectClass;
	String hhId = request.houseHoldId;
	int id = -1000;
	try{
	    id = Integer.valueOf(hhId);
	}
	catch(NumberFormatException e) {
	    System.out.println(e.getMessage());
	    id = -1000;
	}

	try {
	    ontoDB.insertInstance(this.globalNamespace, objectClass, this.mapNamespace, objectName);
	    OntoQueryUtil.updateHHIdOfObject(id, this.globalNamespace, this.mapNamespace, objectName);
	    res.status = 0;
	}
	catch(DuplicatedEntryException de) {
	    res.status = 1;
	}
	catch(UnknownClassException ue) {
	    res.status = 2;
	}
	catch(Exception e) {
	    res.status = -1;
	}

	return res;
    }

    private void initDeleteInstance() throws RosException 
    {
	ServiceServer.Callback<DeleteInstance.Request, DeleteInstance.Response> scb = new ServiceServer.Callback<DeleteInstance.Request, DeleteInstance.Response>() {
	    public DeleteInstance.Response call(DeleteInstance.Request request) {
		return handleDeleteInstance(request);
	    }
	};

	System.out.println(deleteInstanceService);
	ServiceServer<DeleteInstance.Request, DeleteInstance.Response, DeleteInstance> srv = nodeHandle.advertiseService(deleteInstanceService, new DeleteInstance(), scb);
    }

    private DeleteInstance.Response handleDeleteInstance(DeleteInstance.Request request)
    {
	DeleteInstance.Response res = new DeleteInstance.Response();
	String objectName = request.objectName;
	String objectURI = this.mapNamespace;
	if(request.objectURI == null) {
	    objectURI = request.objectURI;
	}
	try {
	    ontoDB.deleteInstance( objectURI, request.objectName);
	    res.status = 0;
	}
	catch(NonExistenceEntryException ne) {
	    res.status = 1;
	}
	catch(UnknownException e) {
	    System.out.println(e.toString() + "  ---  " + e.getMessage());
	    res.status = -1;
	}
	catch(JenaException e) {
	    System.out.println(e.toString() + "  ---  " + e.getMessage());
	}
	catch(Exception e) {
	    System.out.println(e.toString() + "  ---  " + e.getMessage());
	    res.status = -1;
	}
	return res;
    }

    private void initUpdatePosInfo() throws RosException 
    {
	ServiceServer.Callback<UpdatePosInfo.Request, UpdatePosInfo.Response> scb = new ServiceServer.Callback<UpdatePosInfo.Request, UpdatePosInfo.Response>() {
	    public UpdatePosInfo.Response call(UpdatePosInfo.Request request) {
		return handleUpdatePosInfo(request);
	    }
	};

	System.out.println(updatePosInfoService);
	ServiceServer<UpdatePosInfo.Request, UpdatePosInfo.Response, UpdatePosInfo> srv = nodeHandle.advertiseService(updatePosInfoService, new UpdatePosInfo(), scb);
    }

    private UpdatePosInfo.Response handleUpdatePosInfo(UpdatePosInfo.Request request) 
    {
	UpdatePosInfo.Response res = new UpdatePosInfo.Response();
	String objectName = request.objectName;
	SRSSpatialInfo spa = request.spatialInfo;
	try {
	    // public boolean testUpdateObjectProperty(String objectNSURI, String objectName)
	    // OntoQueryUtil.Testupdateobjectproperty(this.globalNamespace, this.mapNamespace, objectName);
	    //public boolean updatePoseOfObject(Pose pos, String propertyNSURI, String objectNSURI, String objectName) throws NonExistenceEntryException {
	    OntoQueryUtil.updatePoseOfObject(spa.pose, this.globalNamespace, this.mapNamespace, objectName);
	    OntoQueryUtil.updateDimensionOfObject(spa.l, spa.w, spa.h, this.globalNamespace, this.mapNamespace, objectName);
	    res.success = true;
	}
	catch(Exception e) {
	    res.success = false;
	    System.out.println(e.toString() + " === " + e.getMessage());
	}
	
	return res;
    }

    private void initGetWorkspaceOnMap() throws RosException
    {
	ServiceServer.Callback<GetWorkspaceOnMap.Request, GetWorkspaceOnMap.Response> scb = new ServiceServer.Callback<GetWorkspaceOnMap.Request, GetWorkspaceOnMap.Response>() {
            public GetWorkspaceOnMap.Response call(GetWorkspaceOnMap.Request request) {
		return handleGetWorkspaceOnMap(request);
            }
	};
	
	System.out.println(getWorkSpaceOnMapService);
	ServiceServer<GetWorkspaceOnMap.Request, GetWorkspaceOnMap.Response, GetWorkspaceOnMap> srv = nodeHandle.advertiseService(getWorkSpaceOnMapService, new GetWorkspaceOnMap(), scb);
    }

    private GetObjectsOnTray.Response handleGetObjectsOnTray(GetObjectsOnTray.Request request)
    {
	GetObjectsOnTray.Response res = new GetObjectsOnTray.Response();
	String mapName = request.map;

	//String targetContent = "kitchen";
	// + "PREFIX ipa-kitchen-map: <http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#>\n"

	String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
	    + "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
	    + "PREFIX mapname: " + "<" + OntoQueryUtil.ObjectNameSpace + ">\n"
	    + "PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>\n";
	String queryString = "SELECT DISTINCT ?objs ?tray WHERE { "
	    + "?tray rdf:type srs:COBTray . "
	    + "<" + OntoQueryUtil.ObjectNameSpace + OntoQueryUtil.RobotName + ">"
	    + " srs:hasPart ?tray . " 
	    + "?objs srs:spatiallyRelated ?tray . "
	    + "}";

	if (this.ontoDB == null) {
	    ros.logInfo("INFO: Ontology Database is NULL. Nothing executed. ");
	    return res;
	}
	
	try {
	    ArrayList<QuerySolution> rset = ontoDB.executeQueryRaw(prefix + queryString);
	    if (rset.size() == 0) {
		ros.logInfo("No found from database");
	    }

	    for(int i = 0; i < rset.size(); i++) {
		QuerySolution qs = rset.get(i);
		RDFNode rn = qs.get("objs");
		Individual tmpInd = KnowledgeEngine.ontoDB.getIndividual(rn.toString());
		res.objects.add(rn.asResource().getLocalName());
		res.classesOfObjects.add(tmpInd.getRDFType(true).getLocalName());
	    }
	    /*
	    if (rset.size() == 0) {
		ros.logInfo("No found from database");
	    }
	    else {
		System.out.println("WARNING: Multiple options... ");
		
		QuerySolution qs = rset.get(0);
		RDFNode rn = qs.get("objs");
		
		//String objName = qs.getLiteral("objs").getString();
		
		//y = qs.getLiteral("y").getFloat();
		//theta = qs.getLiteral("theta").getFloat();
		//System.out.println("x is " + x + ". y is  " + y
		//		   + ". theta is " + theta);
	    }
	    */
	} 
	catch (Exception e) {
	    System.out.println("Exception -->  " + e.getMessage());
	}

	return res;
    }
    
    private void initGetObjectsOnTray() throws RosException
    {
	ServiceServer.Callback<GetObjectsOnTray.Request, GetObjectsOnTray.Response> scb = new ServiceServer.Callback<GetObjectsOnTray.Request, GetObjectsOnTray.Response>() {
            public GetObjectsOnTray.Response call(GetObjectsOnTray.Request request) {
		return handleGetObjectsOnTray(request);
            }
	};

	ServiceServer<GetObjectsOnTray.Request,GetObjectsOnTray.Response,GetObjectsOnTray> srv = nodeHandle.advertiseService(getObjectsOnTrayService, new GetObjectsOnTray(), scb);
    }

    private boolean loadPredefinedTasksForTest()
    {
	try{
	    System.out.println("Create Task Object");
	    //currentTask = new GetObjectTask(Task.TaskType.GET_OBJECT);
	    //currentTask = new GetObjectTask("get", null, null);
	    String taskFile = config.getProperty("taskfile", "task1.seq");
	    System.out.println(taskFile);
	    System.out.println(this.confPath);	    
	    
	    //if(currentTask.loadPredefinedSequence(this.confPath + taskFile)) {
	    if(currentTask.loadPredefinedSequence(this.confPath + taskFile)) {
		System.out.println("OK... ");
	    }
	    else  {
		System.out.println("Fail to load... ");
	    }
	}
	catch(Exception e) {
	    // should throw out. only keep for testing here
	    System.out.println(e.getMessage());
	    return false;
	}

	return true;
    }

    private void initGetRoomsOnMap() throws RosException 
    {
	ServiceServer.Callback<GetRoomsOnMap.Request, GetRoomsOnMap.Response> scb = new ServiceServer.Callback<GetRoomsOnMap.Request, GetRoomsOnMap.Response>() {
	    public GetRoomsOnMap.Response call(GetRoomsOnMap.Request request) {
		return handleGetRoomsOnMap(request);
	    }
	};

	System.out.println(getRoomsOnMapService);
	ServiceServer<GetRoomsOnMap.Request, GetRoomsOnMap.Response, GetRoomsOnMap> srv = nodeHandle.advertiseService(getRoomsOnMapService, new GetRoomsOnMap(), scb);
    }

    private GetRoomsOnMap.Response handleGetRoomsOnMap(GetRoomsOnMap.Request req)
    {
	GetRoomsOnMap.Response re = new GetRoomsOnMap.Response();

	String className = globalNamespace;
	String mapNS = mapNamespace;
		
	if(req.map != null) {
	    if(ontoDB.getNamespaceByPrefix(req.map) != null) {
		mapNS = ontoDB.getNamespaceByPrefix(req.map);
	    }
	}

	className = className + "RoomInAConstruction";
	//System.out.println(className);

	try{
	    Iterator<Individual> instances = ontoDB.getInstancesOfClass(className);
	    if(instances == null) {
		return re;
	    }

	    while (instances.hasNext()) { 
		Individual temp = (Individual)instances.next();
		//System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		if(temp.getNameSpace().equals(mapNamespace)) {
		    re.rooms.add(temp.getLocalName());
		    		    
		    if(req.ifGeometryInfo == true) { 
			SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
			com.hp.hpl.jena.rdf.model.Statement stm = ontoDB.getPropertyOf(globalNamespace, "xCoord", temp);
			spatialInfo.pose.position.x = getFloatOfStatement(stm);
			stm = ontoDB.getPropertyOf(globalNamespace, "yCoord", temp);
			spatialInfo.pose.position.y = getFloatOfStatement(stm);
			stm = ontoDB.getPropertyOf(globalNamespace, "zCoord", temp);
			spatialInfo.pose.position.z = getFloatOfStatement(stm);
			
			stm = ontoDB.getPropertyOf(globalNamespace, "widthOfObject", temp);
			spatialInfo.w = getFloatOfStatement(stm);
			stm = ontoDB.getPropertyOf(globalNamespace, "heightOfObject", temp);
			spatialInfo.h = getFloatOfStatement(stm);
			stm = ontoDB.getPropertyOf(globalNamespace, "lengthOfObject", temp);
			spatialInfo.l = getFloatOfStatement(stm);
			
			stm = ontoDB.getPropertyOf(globalNamespace, "qu", temp);
			spatialInfo.pose.orientation.w = getFloatOfStatement(stm);
			stm = ontoDB.getPropertyOf(globalNamespace, "qx", temp);
			spatialInfo.pose.orientation.x = getFloatOfStatement(stm);
			stm = ontoDB.getPropertyOf(globalNamespace, "qy", temp);
			spatialInfo.pose.orientation.y = getFloatOfStatement(stm);
			stm = ontoDB.getPropertyOf(globalNamespace, "qz", temp);
			spatialInfo.pose.orientation.z = getFloatOfStatement(stm);
			
			re.roomsInfo.add(spatialInfo);
		    }
		}
	    }       
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}

	return re;
    }

    private void initGetPredefinedPoses() throws RosException 
    {
	ServiceServer.Callback<GetPredefinedPoses.Request, GetPredefinedPoses.Response> scb = new ServiceServer.Callback<GetPredefinedPoses.Request, GetPredefinedPoses.Response>() {
	    public GetPredefinedPoses.Response call(GetPredefinedPoses.Request request) {
		return handleGetPredefinedPoses(request);
	    }
	};

	System.out.println(getPredefinedPosesService);
	ServiceServer<GetPredefinedPoses.Request, GetPredefinedPoses.Response, GetPredefinedPoses> srv = nodeHandle.advertiseService(getPredefinedPosesService, new GetPredefinedPoses(), scb);
    }

    private GetPredefinedPoses.Response handleGetPredefinedPoses(GetPredefinedPoses.Request req)
    {
	GetPredefinedPoses.Response re = new GetPredefinedPoses.Response();

	String className = globalNamespace;
	String mapNS = mapNamespace;
		
	if(req.map != null) {
	    if(ontoDB.getNamespaceByPrefix(req.map) != null) {
		mapNS = ontoDB.getNamespaceByPrefix(req.map);
	    }
	}

	className = className + "Point2D";

	try{
	    Iterator<Individual> instances = ontoDB.getInstancesOfClass(className);
	    if(instances == null) {
		return re;
	    }

	    while (instances.hasNext()) { 
		Individual temp = (Individual)instances.next();
		//System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		if(temp.getNameSpace().equals(mapNamespace)) {
		    Pose2D pos2d = new Pose2D();
		    com.hp.hpl.jena.rdf.model.Statement stm = ontoDB.getPropertyOf(globalNamespace, "xCoordinate", temp);
		    pos2d.x = getFloatOfStatement(stm);
		    stm = ontoDB.getPropertyOf(globalNamespace, "yCoordinate", temp);
		    pos2d.y = getFloatOfStatement(stm);
		    stm = ontoDB.getPropertyOf(globalNamespace, "orientationTheta", temp);
		    pos2d.theta = getFloatOfStatement(stm);

		    re.locations.add(temp.getLocalName());		    
		    re.poses.add(pos2d);
		}
	    }
	}	
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}

	return re;
    }

    private void initGetWorkspaceForObject() throws RosException 
    {
	ServiceServer.Callback<GetWorkspaceForObject.Request, GetWorkspaceForObject.Response> scb = new ServiceServer.Callback<GetWorkspaceForObject.Request, GetWorkspaceForObject.Response>() {
	    public GetWorkspaceForObject.Response call(GetWorkspaceForObject.Request request) {
		return handleGetWorkspaceForObject(request);
	    }
	};

	System.out.println(getWorkspaceForObjectService);
	ServiceServer<GetWorkspaceForObject.Request, GetWorkspaceForObject.Response, GetWorkspaceForObject> srv = nodeHandle.advertiseService(getWorkspaceForObjectService, new GetWorkspaceForObject(), scb);
    }

    private GetWorkspaceForObject.Response handleGetWorkspaceForObject(GetWorkspaceForObject.Request req)
    {
	GetWorkspaceForObject.Response re = new GetWorkspaceForObject.Response();
	try{
	    if(req.config == 0) {
		re.workspaces = OntoQueryUtil.getFurnituresLinkedToObject(req.objectType);
	    }
	    else if(req.config == 1) {
		re.workspaces = OntoQueryUtil.getWorkspaceNamesOfObject(req.objectType);
	    }
	}
	catch(Exception e) {
	    System.out.println("Please check if the object is spelled correctly. ");
	}
	return re;
    }

    private String defaultContextPath()
    {
	 this.confPath = this.getClass().getProtectionDomain().getCodeSource().getLocation().getPath();	
	 this.confPath = this.confPath + "../conf/";
	 return this.confPath;
    }
    
    public String getContextPath()
    {
	return this.confPath;
    }

    public void testFunction() {
	//boolean b = OntoQueryUtil.computeOnSpatialRelation();
	System.out.println("++++++++++++++++++++++++++++++++++");
	OntoQueryUtil.computeOnSpatialRelation();
	//System.out.println("++++++++++++++++++++++++++++++++++");

	//SpatialCalculator.testTF();
	//System.out.println(" ----- " + OntoQueryUtil.getFurnituresLinkedToObject("Milkbox"));
	//System.out.println(" ----- " + OntoQueryUtil.tempGetFurnituresLinkedToObject("Milkbox"));

	System.out.println("++++++++++++++++++++++++++++++++++");

	//System.out.println(" ----- " + OntoQueryUtil.getFurnituresLinkedToObject("FoodVessel"));
	//System.out.println(" ----- " + OntoQueryUtil.tempGetFurnituresLinkedToObject("FoodVessel"));
	//OntoQueryUtil.testRemoveProperty();
    }

    public static void main(String[] args)
    {

	String configFile = new String();
	System.out.print("There are " + args.length + " input arguments: ");

	if(args.length == 1) {
	    configFile = args[0];
	    System.out.println(configFile);
	}
	else {
	    String env =  System.getenv("ROBOT_ENV");
	    configFile = env + ".cfg";
	    //configFile = "srsknow.cfg";
	    System.out.println(configFile);
	}

	Properties conf = new Properties();
	KnowledgeEngine knowEng = new KnowledgeEngine();
	
	//knowEng.loadPredefinedTasksForTest();
	if (knowEng.init(configFile)) {
	    System.out.println("OK");
	
	}
	else {
	    System.out.println("Something wrong with initialisation");
	}
    }

    private Task currentTask;
    private int currentSessionId = 1;
    private String nodeName;

    private Properties config;

    private String taskRequestService = "task_request";
    private String planNextActionService = "plan_next_action";
    private String generateSequenceService = "generate_sequence";
    private String getObjectsOnTrayService = "get_objects_on_tray"; 
    private String querySparQLService = "query_sparql";
    private String getObjectsOnMapService = "get_objects_on_map";
    private String getWorkSpaceOnMapService = "get_workspace_on_map";
    private String insertInstanceService = "insert_instance";
    private String deleteInstanceService = "delete_instance";
    private String updatePosInfoService = "update_pos_info";
    private String getRoomsOnMapService = "get_rooms_on_map";
    private String getPredefinedPosesService = "get_predefined_poses";
    private String getWorkspaceForObjectService = "get_workspace_for_object";
    //private String mapNamespacePrefix = "ipa-kitchen-map";
    private String mapName = "ipa-kitchen-map";
    private String mapNamespace = "http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#";

    private String globalNamespace = "http://www.srs-project.eu/ontologies/srs.owl#";

    //private String graspActionMode = "move_and_grasp";
    private String graspActionMode = "Simple";
    private String confPath;
    //private OntoQueryUtil ontoQueryUtil;
    // 0: normal mode; 1: test mode (no inference, use predefined script instead)  ---- will remove this flag eventually. only kept for testing
    int flag = 1;
}
