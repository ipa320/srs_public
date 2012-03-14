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
import ros.pkg.srs_knowledge.srv.AskForActionSequence;  // deprecated
import ros.pkg.srs_knowledge.srv.GenerateSequence;
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

import com.hp.hpl.jena.rdf.model.Statement;
import org.srs.srs_knowledge.task.*;

import java.util.Properties;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList; 
import java.util.Iterator;

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
	try {
	    initProperties(cfgFile);
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	    return false;
	}
	
	ros = Ros.getInstance();
	ros.init(nodeName);
	ros.logInfo("INFO: Start RosJava_JNI service");
	
	nodeHandle = ros.createNodeHandle();

	try{
	    initGenerateSequence();
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
	}
	catch(RosException e){
	    System.out.println(e.getMessage());
	    return false;
	}

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

	mapNamespacePrefix = config.getProperty("map_namespace", "ipa-kitchen-map");
	
	if(ontoDB.getNamespaceByPrefix(mapNamespacePrefix) != null) {
	    mapNamespace = ontoDB.getNamespaceByPrefix(mapNamespacePrefix);
	    System.out.println("Map Name Space: " + mapNamespace);
	    System.out.println("Map Name Space Prefix : " + mapNamespacePrefix);
	}


	//ontoQueryUtil = new OntoQueryUtil(mapNamespace, globalNamespace);
	OntoQueryUtil.ObjectNameSpace = mapNamespace;
	OntoQueryUtil.GlobalNameSpace = globalNamespace;

	//testOnto("http://www.srs-project.eu/ontologies/srs.owl#MilkBox");

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

    private GenerateSequence.Response handleGenerateSequence(GenerateSequence.Request request)
    {
	GenerateSequence.Response res = new GenerateSequence.Response();
	ArrayList<CUAction> actSeq = new ArrayList<CUAction>();

	CUAction ca = new CUAction(); 
	actSeq.add(ca);
	actSeq.add(ca);
	
	res.actionSequence = actSeq;
	System.out.println(res.actionSequence.size());

	ros.logInfo("INFO: Generate sequence of length: " + actSeq.size());
	return res;
    }
    
    private void initGenerateSequence() throws RosException
    {
	ServiceServer.Callback<GenerateSequence.Request, GenerateSequence.Response> scb = new ServiceServer.Callback<GenerateSequence.Request,GenerateSequence.Response>() {
            public GenerateSequence.Response call(GenerateSequence.Request request) {
		return handleGenerateSequence(request);
            }
	};

	ServiceServer<GenerateSequence.Request,GenerateSequence.Response,GenerateSequence> srv = nodeHandle.advertiseService(generateSequenceService, new GenerateSequence(), scb);
    }

    private QuerySparQL.Response handleQuerySparQL(QuerySparQL.Request req)
    {
	QuerySparQL.Response re = new QuerySparQL.Response();
	String queryString = req.query;
	System.out.println(queryString);

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

	/*
	if(request.stateLastAction.length == 3) {
	    if(request.stateLastAction[0] == 0 && request.stateLastAction[1] == 0 && request.stateLastAction[2] == 0) {
		//ArrayList<String> feedback = new ArrayList<String>();
		ArrayList<String> feedback = request.genericFeedBack;
		ca = currentTask.getNextCUAction(true, feedback); // no error. generate new action
	    }
	    else if(request.stateLastAction[0] == 2 || request.stateLastAction[1] == 2 || request.stateLastAction[2] == 2) {
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
	}
	*/

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
	    currentTask = new MoveTask(request.content, null);
	    System.out.println("Created CurrentTask " + "move " + request.content);
	}
	else if(request.task.equals("get") || request.task.equals("search")){
	    
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }

	    GetObjectTask got = new GetObjectTask(request.task, request.content, request.userPose, nodeHandle);
	    currentTask = (Task)got;
	    System.out.println("Created CurrentTask " + "get " + request.content);	    

	    // TODO: for other types of task, should be dealt separately. 
	    // here is just for testing
	    
	    //currentTask = new TestTask();
	    //this.loadPredefinedTasksForTest();
	    
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
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
	GetObjectsOnMap.Response re = new GetObjectsOnMap.Response();

	String className = globalNamespace;
	String mapNS = mapNamespace;
		
	if(req.map != null) {
	    if(ontoDB.getNamespaceByPrefix(req.map) != null) {
		mapNS = ontoDB.getNamespaceByPrefix(req.map);
	    }
	}
	
	className = className + "FoodVessel";

	System.out.println(className + " --- ");
	Iterator<Individual> instances = ontoDB.getInstancesOfClass(className);
	if(instances == null) {
	    return re;
	}
	com.hp.hpl.jena.rdf.model.Statement stm;
	if(instances.hasNext()) {
	    while (instances.hasNext()) { 
		Individual temp = (Individual)instances.next();
		System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		if(temp.getNameSpace().equals(mapNamespace)) {
		    re.objects.add(temp.getLocalName());
		    re.classesOfObjects.add(temp.getRDFType(true).getLocalName());
		    try{
			
			stm = ontoDB.getPropertyOf(globalNamespace, "spatiallyRelated", temp);			
			re.spatialRelation.add(stm.getPredicate().getLocalName());
			re.spatialRelatedObject.add(stm.getObject().asResource().getLocalName());
		    }
		    catch(Exception e) {
			System.out.println("CAUGHT exception: " + e.toString());
			re.spatialRelation.add("NA");
			re.spatialRelatedObject.add("NA");
		    }
		    try{
			stm = ontoDB.getPropertyOf(globalNamespace, "houseHoldObjectID", temp);			
			re.houseHoldId.add(Integer.toString(getIntOfStatement(stm)));
		    }
		    catch(Exception e) {
			System.out.println("CAUGHT exception: " + e.toString());
			re.houseHoldId.add("NA");
		    }
		    if(req.ifGeometryInfo == true) { 
			SRSSpatialInfo spatialInfo = new SRSSpatialInfo();
			try{
			    
			    stm = ontoDB.getPropertyOf(globalNamespace, "xCoord", temp);
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
			}
			
			catch(Exception e) {
			    System.out.println("CAUGHT exception: " + e.getMessage()+ ".. added invalid values");
			    
			    spatialInfo.pose.position.x = -1000;
			    spatialInfo.pose.position.y = -1000;
			    spatialInfo.pose.position.z = -1000;
			    
			    spatialInfo.w = -1000;
			    spatialInfo.h = -1000;
			    spatialInfo.l = -1000;

			    spatialInfo.pose.orientation.w = -1000;
			    spatialInfo.pose.orientation.x = -1000;
			    spatialInfo.pose.orientation.y = -1000;
			    spatialInfo.pose.orientation.z = -1000;
			}

			re.objectsInfo.add(spatialInfo);
		    }
		    
		}
	    }
	}
	else {
	    System.out.println("<EMPTY>");
	}
	
	/*

	String targetContent = "kitchen";
	String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
	    + "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
	    + "PREFIX ipa-kitchen-map: <http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#>\n"
	    + "PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>\n";
	String queryString = "SELECT ?objs ?x ?y ?z ?w ?h ?l WHERE { "
	    + "?objs rdf:type srs:Dishwasher . "
	    + "?objs srs:xCoord ?x . "
	    + "?objs srs:yCoord ?y . " 
	    + "?objs srs:zCoord ?z . " 
	    + "?objs srs:widthOfObject ?w . " 
	    + "?objs srs:heightOfObject ?h . " 
	    + "?objs srs:lengthOfObject ?l . " 
	    + "}";
	System.out.println(prefix + queryString + "\n");
	
	if (this.ontoDB == null) {
	    ros.logInfo("INFO: Ontology Database is NULL. Nothing executed. ");
	    return re;
	}
	
	try {
	    ArrayList<QuerySolution> rset = ontoDB.executeQueryRaw(prefix
								   + queryString);
	    
	    if (rset.size() == 0) {
		ros.logInfo("No move target found from database");
	    }
	    else {
		System.out.println("WARNING: Multiple options... ");
		QuerySolution qs = rset.get(0);
		//x = qs.getLiteral("x").getFloat();
		//y = qs.getLiteral("y").getFloat();
		//theta = qs.getLiteral("theta").getFloat();
		//System.out.println("x is " + x + ". y is  " + y
		//		   + ". theta is " + theta);
	    }
	    
	} catch (Exception e) {
	    System.out.println("Exception -->  " + e.getMessage());
	    
	}

	//re.result = ontoDB.executeQuery(queryString);
	*/
	return re;
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
	GetWorkspaceOnMap.Response re = new GetWorkspaceOnMap.Response();

	String className = globalNamespace;
	String mapNS = mapNamespace;
		
	if(req.map != null) {
	    if(ontoDB.getNamespaceByPrefix(req.map) != null) {
		mapNS = ontoDB.getNamespaceByPrefix(req.map);
	    }
	}

	className = className + "FurniturePiece";
	//System.out.println(className);
	try{
	    Iterator<Individual> instances = ontoDB.getInstancesOfClass(className);
	    if(instances == null) {
		return re;
	    }

	    if(instances.hasNext()) {
		while (instances.hasNext()) { 
		    Individual temp = (Individual)instances.next();
		    //System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		    if(temp.getNameSpace().equals(mapNamespace)) {
			re.objects.add(temp.getLocalName());
			re.classesOfObjects.add(temp.getRDFType(true).getLocalName());

			if(req.ifGeometryInfo == true) { 
			    SRSSpatialInfo spatialInfo = new SRSSpatialInfo();

			    /*
			    com.hp.hpl.jena.rdf.model.Statement stm = ontoDB.getPropertyOf(globalNamespace, "xCoord", temp);
			    spatialInfo.point.x = getFloatOfStatement(stm);
			    stm = ontoDB.getPropertyOf(globalNamespace, "yCoord", temp);
			    spatialInfo.point.y = getFloatOfStatement(stm);
			    stm = ontoDB.getPropertyOf(globalNamespace, "zCoord", temp);
			    spatialInfo.point.z = getFloatOfStatement(stm);
			    */
			    
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
			    
			    /*
			    stm = ontoDB.getPropertyOf(globalNamespace, "r3d", temp);
			    spatialInfo.angles.r = getFloatOfStatement(stm);
			    stm = ontoDB.getPropertyOf(globalNamespace, "p3d", temp);
			    spatialInfo.angles.p = getFloatOfStatement(stm);
			    stm = ontoDB.getPropertyOf(globalNamespace, "y3d", temp);
			    spatialInfo.angles.y = getFloatOfStatement(stm);
			    */

			    stm = ontoDB.getPropertyOf(globalNamespace, "qu", temp);
			    spatialInfo.pose.orientation.w = getFloatOfStatement(stm);
			    stm = ontoDB.getPropertyOf(globalNamespace, "qx", temp);
			    spatialInfo.pose.orientation.x = getFloatOfStatement(stm);
			    stm = ontoDB.getPropertyOf(globalNamespace, "qy", temp);
			    spatialInfo.pose.orientation.y = getFloatOfStatement(stm);
			    stm = ontoDB.getPropertyOf(globalNamespace, "qz", temp);
			    spatialInfo.pose.orientation.z = getFloatOfStatement(stm);

			    re.objectsInfo.add(spatialInfo);

			    stm = ontoDB.getPropertyOf(globalNamespace, "houseHoldObjectID", temp);
			    re.houseHoldId.add(Integer.toString(getIntOfStatement(stm)));
			}
		    }
		}       
	    }
	    else
		System.out.println("<EMPTY>");
	        
	    //System.out.println();

	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}

	return re;
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

	String targetContent = "kitchen";
	String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
	    + "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
	    + "PREFIX ipa-kitchen-map: <http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#>\n"
	    + "PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>\n";
	String queryString = "SELECT ?objs ?tray WHERE { "
	    + "?tray rdf:type srs:CobTray . "
	    + "?objs srs:SpatiallyRelated ?tray . "
	    + "}";

	if (this.ontoDB == null) {
	    ros.logInfo("INFO: Ontology Database is NULL. Nothing executed. ");
	    return res;
	}
	
	try {
	    ArrayList<QuerySolution> rset = ontoDB.executeQueryRaw(prefix
								   + queryString);
	    
	    if (rset.size() == 0) {
		ros.logInfo("No found from database");
	    }
	    else {
		System.out.println("WARNING: Multiple options... ");
		QuerySolution qs = rset.get(0);
		String objName = qs.getLiteral("objs").getString();
		
		//y = qs.getLiteral("y").getFloat();
		//theta = qs.getLiteral("theta").getFloat();
		//System.out.println("x is " + x + ". y is  " + y
		//		   + ". theta is " + theta);
	    }
	    
	} catch (Exception e) {
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

	//ArrayList<ActionTuple> acts = currentTask.getActionSequence();

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

    /*
    public static void testTask(Properties conf)
    {
	try{
	    System.out.println("Create Task Object");
	    //Task task = new GetObjectTask(Task.TaskType.GET_OBJECT);
	    Task task = new GetObjectTask("get", null, null);

	    String taskFile = conf.getProperty("taskfile", "task1.seq");
	    System.out.println(taskFile);
	    if(task.loadPredefinedSequence(taskFile))   {
		System.out.println("OK... ");
	    }
	    else  {
		System.out.println("Fail to load... ");
	    }
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
    }
    */

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

    public static void main(String[] args)
    {
	String configFile = new String();
	System.out.print("There are " + args.length + " input arguments: ");

	if(args.length == 1) {
	    configFile = args[0];
	    System.out.println(configFile);
	}
	else {
	    configFile = "srsknow.cfg";
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

    private String mapNamespacePrefix = "ipa-kitchen-map";
    private String mapNamespace = "http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#";

    private String globalNamespace = "http://www.srs-project.eu/ontologies/srs.owl#";

    private String confPath;
    private OntoQueryUtil ontoQueryUtil;
    // 0: normal mode; 1: test mode (no inference, use predefined script instead)  ---- will remove this flag eventually. only kept for testing
    int flag = 1;
}
