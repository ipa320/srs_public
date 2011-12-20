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
import ros.pkg.srs_knowledge.srv.AskForActionSequence;  // deprecated
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

import java.util.Properties;

import java.io.IOException;
import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList; 
import java.util.Iterator;
//import org.apache.commons.logging.Log;

class KnowledgeEngine
{
    /*
    public KnowledgeEngine(String nodeName, String ontologyFile)
    {
	this.defaultContextPath();
	ontoDB = new OntologyDB(ontologyFile);
	this.nodeName = nodeName;
    }
    */
    /*
    public KnowledgeEngine(Properties conf)
    {
	this.defaultContextPath();
	String ontoDBFile = conf.getProperty("ontologyFile", "house.owl");

	ArrayList<String> nameList = parseOntologyFileNames(ontoDBFile);

	ontoDB = new OntologyDB(nameList);

	this.nodeName = conf.getProperty("nodename", "knowledge_srs_node");
	this.config = conf;

	taskRequestService = conf.getProperty("taskRequestService", "task_request");
	planNextActionService = conf.getProperty("planNextActionService", "plan_next_action");
	generateSequenceService = conf.getProperty("generateSequenceService", "generate_sequence");
	querySparQLService = conf.getProperty("querySparQLService", "query_sparql");
    }
    */

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
	
	n = ros.createNodeHandle();

	try{
	    initGenerateSequence();
	    initQuerySparQL();
	    initPlanNextAction();
	    initTaskRequest();
	    initGetObjectsOnMap();
	    initGetWorkspaceOnMap();
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

	mapNamespacePrefix = config.getProperty("map_namespace", "ipa-kitchen-map");
	if(ontoDB.getNamespaceByPrefix(mapNamespacePrefix) != null) {
	    mapNamespace = ontoDB.getNamespaceByPrefix(mapNamespacePrefix);
	}

	ontoQueryUtil = new OntoQueryUtil(mapNamespace, globalNamespace);
	
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

	ServiceServer<GenerateSequence.Request,GenerateSequence.Response,GenerateSequence> srv = n.advertiseService(generateSequenceService, new GenerateSequence(), scb);
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

	ServiceServer<QuerySparQL.Request, QuerySparQL.Response, QuerySparQL> srv = n.advertiseService( querySparQLService , new QuerySparQL(), scb);
    }

    private PlanNextAction.Response handlePlanNextAction( PlanNextAction.Request request) throws NullPointerException
    {
	PlanNextAction.Response res = new PlanNextAction.Response();
	CUAction ca = new CUAction(); 
	
	if(currentTask == null) {
	    System.out.println("Current Task is NULL. Send task request first");
	    res.nextAction = new CUAction(); // empty task
	    return res;
	    //throw new NullPointerException("Current Task is NULL. Send task request first");
	}

	ActionTuple at = null;

	if(request.stateLastAction.length == 3) {
	    if(request.stateLastAction[0] == 0 && request.stateLastAction[1] == 0 && request.stateLastAction[2] == 0) {
		at = currentTask.getNextAction(true); // no error. generate new action
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
		at = currentTask.getNextAction(false);
	    }
	}
	
	if(at == null) {
	    currentTask = null;
	    //currentSessionId = 1;
	    System.out.println("No further action can be planned. Terminate the task. ");

	    res.nextAction = new CUAction(); // empty task
	    return res;	    
	}
	if(at.getActionName().equals("finish_success")) {
	    currentTask = null;
	    //currentSessionId = 1;
	    System.out.println("Reached the end of the task. No further action to be executed. ");
	    res.nextAction = new CUAction(); // empty task
	    res.nextAction.status = 1;
	    return res;	    
	}
	else if( at.getActionName().equals("finish_fail")) {
	    currentTask = null;
	    // currentSessionId = 1;
	    System.out.println("Reached the end of the task. No further action to be executed. ");
	    res.nextAction = new CUAction(); // empty task
	    res.nextAction.status = -1;
	    return res;	    
	}

	ca = at.getCUAction();

	res.nextAction = ca;

	//ros.logInfo("INFO: Generate sequence of length: ");
	return res;
    }
    
    private void initPlanNextAction() throws RosException
    {
	ServiceServer.Callback<PlanNextAction.Request, PlanNextAction.Response> scb = new ServiceServer.Callback<PlanNextAction.Request, PlanNextAction.Response>() {
            public PlanNextAction.Response call(PlanNextAction.Request request) {
		return handlePlanNextAction(request);
            }
	};
	
	ServiceServer<PlanNextAction.Request, PlanNextAction.Response, PlanNextAction> srv = n.advertiseService(planNextActionService, new PlanNextAction(), scb);
    }

    private TaskRequest.Response handleTaskRequest(TaskRequest.Request request)
    {
	TaskRequest.Response res = new TaskRequest.Response();
	
	System.out.println("Received request for new task");
	
	if(request.task.equals("move")) {
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    currentTask = new MoveToTask(request.content, null, ontoDB);
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
	    System.out.println("Created CurrentTask " + "move " + request.content);
	}
	else if(request.task.equals("get") || request.task.equals("search")){
	    
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    GetObjectTask got = new GetObjectTask(request.task, request.content, null, ontoDB, ontoQueryUtil);

	    System.out.println("Created CurrentTask " + "get " + request.content);	    

	    // TODO: for other types of task, should be dealt separately. 
	    // here is just for testing
	    this.loadPredefinedTasksForTest(got);
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
	    currentTask = (Task)got;
	}
	else if(request.task.equals("charging")) {
	    if(ontoDB == null) {
		System.out.println(" ONTOLOGY FILE IS NULL ");
	    }
	    currentTask = new ChargingTask(null, null, ontoDB);
	    //currentTask = new MoveToTask(null, null, ontoDB);
	    System.out.println("Created CurrentTask " + "charge ");
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
	}
	else {
	    // TODO: for other types of task, should be dealt separately. 
	    // here is just for testing
	    // task not created for some reason
	    currentTask = new GetObjectTask(request.task, request.content, null, ontoDB, ontoQueryUtil);
	    //currentTask.setOntoQueryUtil(ontoQueryUtil);
	    res.result = 1;
	    res.description = "No action";
	}

	if(currentTask.getActionSequence().size() == 0) {
	    // task not created for some reason
	    res.result = 1;
	    res.description = "No action";
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
	ServiceServer<TaskRequest.Request, TaskRequest.Response, TaskRequest> srv = n.advertiseService(taskRequestService, new TaskRequest(), scb);
    }


    private GetObjectsOnMap.Response handleGetObjectsOnMap(GetObjectsOnMap.Request req)
    {
	GetObjectsOnMap.Response re = new GetObjectsOnMap.Response();
	
	String className = globalNamespace;
	if(req.map != null) {
	    if(ontoDB.getNamespaceByPrefix(req.map) != null) {
		className = ontoDB.getNamespaceByPrefix(req.map);
	    }
	}
	

	className = className + "FoodVessel";

	try{
	    Iterator<Individual> instances = ontoDB.getInstancesOfClass(className);
	    if(instances == null) {
		return re;
	    }

	    if(instances.hasNext()) {
		while (instances.hasNext()) { 
		    Individual temp = (Individual)instances.next();
		    System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
		    if(temp.getNameSpace().equals(mapNamespace)) {
			re.objects.add(temp.getLocalName());
			re.classesOfObjects.add(temp.getRDFType(true).getLocalName());
			
			com.hp.hpl.jena.rdf.model.Statement stm = ontoDB.getPropertyOf(globalNamespace, "spatiallyRelated", temp);			
			//System.out.println(" -->  " + stm);
			//System.out.println(" ===>  " + stm.getLiteral());
			re.spatialRelation.add(stm.getPredicate().getLocalName());
			re.spatialRelatedObject.add(stm.getObject().asResource().getLocalName());
			
			stm = ontoDB.getPropertyOf(globalNamespace, "houseHoldObjectID", temp);			
			re.houseHoldId.add(Integer.toString(getIntOfStatement(stm)));
		    }
		}
	    }
	    else
		System.out.println("<EMPTY>");
	        
        System.out.println();

	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
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
	ServiceServer<GetObjectsOnMap.Request, GetObjectsOnMap.Response, GetObjectsOnMap> srv = n.advertiseService(getObjectsOnMapService, new GetObjectsOnMap(), scb);
    }


    private GetWorkspaceOnMap.Response handleGetWorkspaceOnMap(GetWorkspaceOnMap.Request req)
    {
	GetWorkspaceOnMap.Response re = new GetWorkspaceOnMap.Response();

	String className = globalNamespace;
	if(req.map != null) {
	    System.out.println("<<<<  " + req.map + "  >>>>  ");
	    if(ontoDB.getNamespaceByPrefix(req.map) != null) {
		className = ontoDB.getNamespaceByPrefix(req.map);
	    }
	}

	className = className + "FurniturePiece";
	System.out.println(className);
	try{
	    Iterator<Individual> instances = ontoDB.getInstancesOfClass(className);
	    if(instances == null) {
		return re;
	    }

	    if(instances.hasNext()) {
		while (instances.hasNext()) { 
		    Individual temp = (Individual)instances.next();
		    System.out.println( temp.getNameSpace() + "   " + temp.getLocalName());
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
	        
        System.out.println();

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


    private void initGetWorkspaceOnMap() throws RosException
    {
	ServiceServer.Callback<GetWorkspaceOnMap.Request, GetWorkspaceOnMap.Response> scb = new ServiceServer.Callback<GetWorkspaceOnMap.Request, GetWorkspaceOnMap.Response>() {
            public GetWorkspaceOnMap.Response call(GetWorkspaceOnMap.Request request) {
		return handleGetWorkspaceOnMap(request);
            }
	};
	
	System.out.println(getWorkSpaceOnMapService);
	ServiceServer<GetWorkspaceOnMap.Request, GetWorkspaceOnMap.Response, GetWorkspaceOnMap> srv = n.advertiseService(getWorkSpaceOnMapService, new GetWorkspaceOnMap(), scb);
    }

    private boolean loadPredefinedTasksForTest(GetObjectTask got)
    {
	try{
	    System.out.println("Create Task Object");
	    //currentTask = new GetObjectTask(Task.TaskType.GET_OBJECT);
	    //currentTask = new GetObjectTask("get", null, null);
	    String taskFile = config.getProperty("taskfile", "task1.seq");
	    System.out.println(taskFile);
	    
	    //if(currentTask.loadPredefinedSequence(this.confPath + taskFile)) {
	    if(got.loadPredefinedSequence(this.confPath + taskFile)) {
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

	ArrayList<ActionTuple> acts = currentTask.getActionSequence();

	return true;
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
    private OntologyDB ontoDB;
    private Ros ros;
    private NodeHandle n;
    private String nodeName;

    private Properties config;

    private String taskRequestService = "task_request";
    private String planNextActionService = "plan_next_action";
    private String generateSequenceService = "generate_sequence";
    private String querySparQLService = "query_sparql";
    private String getObjectsOnMapService = "get_objects_on_map";
    private String getWorkSpaceOnMapService = "get_workspace_on_map";

    private String mapNamespacePrefix = "ipa-kitchen-map";
    private String mapNamespace = "http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#";

    private String globalNamespace = "http://www.srs-project.eu/ontologies/srs.owl#";

    private String confPath;
    private OntoQueryUtil ontoQueryUtil;
    // 0: normal mode; 1: test mode (no inference, use predefined script instead)  ---- will remove this flag eventually. only kept for testing
    int flag = 1;
}
