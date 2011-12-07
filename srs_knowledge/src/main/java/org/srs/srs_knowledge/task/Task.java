package org.srs.srs_knowledge.task;

import java.io.IOException;
import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;

public class Task
{
    public enum TaskType {GET_OBJECT, MOVETO_LOCATION, SEARCH_OBJECT, SCAN_AROUND, UNSPECIFIED};

    public Task(TaskType type)
    {
	acts = new ArrayList<ActionTuple>();
	//actionSequence = new ArrayList<CUAction>();
	setTaskType(type);
	currentAction = null;
    }

    public Task(String taskType, String targetContent, Pose2D userPose)
    {
	acts = new ArrayList<ActionTuple>();
	currentAction = null;
	if(taskType.toLowerCase().equals("get")) {
	    setTaskType(TaskType.GET_OBJECT);
	}
	else if(taskType.toLowerCase().equals("move")) {
	    setTaskType(TaskType.MOVETO_LOCATION);
	    
	    setTaskTarget(targetContent);
	    System.out.println("TASK.JAVA: Created CurrentTask " + "move " + targetContent);
	    createSimpleMoveTask();
	}
	else if(taskType.toLowerCase().equals("search")) {
	    setTaskType(TaskType.SEARCH_OBJECT);
	}
	else {
	    setTaskType(TaskType.UNSPECIFIED);
	}
    }

    public boolean createSimpleMoveTask()
    {
	//currentTask = new Task("move", "kitchen", null);
	// boolean addNewActionTuple(ActionTuple act)
	ActionTuple act = new ActionTuple();

	CUAction ca = new CUAction();
	MoveAction ma = new MoveAction();
	PerceptionAction pa = new PerceptionAction();
	GraspAction ga = new GraspAction();

	// TODO: should obtain information from Ontology here. But here we use temporary hard coded info for testing
	double x = 1;
	double y = 1;
	double theta = 0;

	/*TODO: Temporary: hard coded coordinate for predefined locations*/

	/*
	// cob_environments/cob_default_env_config/ipa-kitchen/navigation_goals.yaml 
	  home: [0, 0, 0]
	  order: [1, -0.5, -0.7]
	  kitchen: [-2.04, 0.3, 0]
	  new_kitchen: [-2.14, 0.0, 0]
	  kitchen_backwards: [-2.04, 0.3, 3.14]
	 */
	
	if (this.targetContent.equals("home")) {
	    this.targetContent = "[0 0 0]";
	}
	else if (this.targetContent.equals("order")) {
	    this.targetContent = "[1 -0.5 -0.7]";
	}
	else if (this.targetContent.equals("kitchen")) {
	    this.targetContent = "[-2.04 0.3 0]";
	}
	else if (this.targetContent.equals("new_kitchen")) {
	    this.targetContent = "[-2.14 0.0 0]";
	}
	else if (this.targetContent.equals("kitchen_backwards")) {
	    this.targetContent = "[-2.04 -0.3 3.14]";
	}
	
	if(this.targetContent.charAt(0) == '[' && this.targetContent.charAt(targetContent.length() - 1) == ']') {
	    StringTokenizer st = new StringTokenizer(targetContent, " []");
	    if(st.countTokens() == 3) {
		try {
		    x = Double.parseDouble(st.nextToken());
		    y = Double.parseDouble(st.nextToken());
		    theta = Double.parseDouble(st.nextToken());
		    System.out.println(x + "  " + y + " " + theta);
		}
		catch(Exception e){
		    System.out.println(e.getMessage());
		    return false;
		}
	    }
	}
	else {
	    return false;
	}

	ma.targetPose2D.x = x;
	ma.targetPose2D.y = y;
	ma.targetPose2D.theta = theta;

	ca.ma = ma;
	ca.pa = pa;
	ca.ga = ga;

	try {
	    ca.actionFlags = parseActionFlags("0 1 1");
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}

	act.setCUAction(ca);
	act.setActionId(1);
	addNewActionTuple(act);


	// add finish action __ success 

	act = new ActionTuple();

	ca = new CUAction();
	ma = new MoveAction();
	pa = new PerceptionAction();
	ga = new GraspAction();

	ca.ma = ma;
	ca.pa = pa;
	ca.ga = ga;

	try {
	    ca.actionFlags = parseActionFlags("0 1 1");
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}
	act.setActionName("finish_success");

	if(act.getActionName().equals("finish_success")){
	    ca.status = 1;	    
	}
	if(act.getActionName().equals("finish_fail")) {
	    ca.status = -1;
	}

	act.setCUAction(ca);
	act.setActionId(2);
	act.setParentId(1);
	act.setCondition(true);
	addNewActionTuple(act);

	// add finish action __ fail 

	act = new ActionTuple();

	ca = new CUAction();
	ma = new MoveAction();
	pa = new PerceptionAction();
	ga = new GraspAction();

	ca.ma = ma;
	ca.pa = pa;
	ca.ga = ga;

	try {
	    ca.actionFlags = parseActionFlags("0 1 1");
	}
	catch(Exception e) {
	    System.out.println(e.getMessage());
	}

	act.setActionName("finish_fail");

	//ca.status = -1;
	if(act.getActionName().equals("finish_success")){
	    ca.status = 1;	    
	}
	if(act.getActionName().equals("finish_fail")) {
	    ca.status = -1;
	}

	act.setCUAction(ca);
	act.setActionId(3);
	act.setParentId(1);
	act.setCondition(false);
	addNewActionTuple(act);







	System.out.println("number of actions: " + acts.size());
	return true;
    }

    public void setTaskId(int id)
    {
	this.taskId = id;
    }

    public int getTaskId()
    {
	return taskId;
    }

    public void setTaskTarget(String target)
    {
	this.targetContent = target;
    }

    public String getTaskTarget()
    {
	return this.targetContent;
    }

    public void setTaskType(TaskType type)
    {
	this.taskType = type;
    }

    public ArrayList<ActionTuple> getActionSequence()
    {
	return acts;
    }

    public ActionTuple getNextAction(boolean stateLastAction)
    {
	if(currentAction == null) {
	    for(int i = 0; i < acts.size(); i++) {
		if(acts.get(i).getActionId() == 1) {
		    currentAction = acts.get(i);
		    currentActionLoc = i;
		    System.out.println(i);
		    System.out.println(currentAction.getCUAction().ma.targetPose2D.x);
		    System.out.println(currentAction.getActionName());

		    return currentAction;
		}
	    }
	}
	else {
	    //ActionTuple at;
	    //int parentId = at.getParentId();
	    for(int i = 0; i < acts.size(); i++) {
		// if(acts.get(currentActionLoc).getId() == acts.get(i).getParentId() && stateLastAction == acts.get(i).getCondition()){    
		if(currentAction.getActionId() == acts.get(i).getParentId() && stateLastAction == acts.get(i).getCondition()){
		    currentAction = acts.get(i);
		    currentActionLoc = i;
		    System.out.println(i);
		    System.out.println(currentAction.getActionName());
		    
		    return currentAction;
		}
	    }
	    System.out.println("no action found");
	}
	return null;
    }
    
    public boolean addNewActionTuple(ActionTuple act)
    {
	return acts.add(act);
    }

    public boolean loadPredefinedSequence(String filename) throws IOException, Exception
    {
	//ArrayList<ActionTuple> acts = new ArrayList<ActionTuple>();
	File file = null;
	FileReader freader = null;
	LineNumberReader in = null;

	try{
	    file = new File(filename);
	    freader = new FileReader(file);
	    in = new LineNumberReader(freader);
	    String line = "";

	    while((line = in.readLine().trim()) != null) {
		//System.out.println("Line: " + in.getLineNumber() + ": " + line);
		if( line.equals("")) {
		    continue;
		}
		else if(line.substring(0,1).equals("#")) {
		    continue;
		}
		ActionTuple act = parseAction(line);
		if(act != null)
		    this.acts.add(act);
	    }
	}
	finally{
	    freader.close();
	    in.close();
	}
	System.out.println("Number of actions is: " + acts.size());
	return true;
    }

    public static ActionTuple parseAction(String actionDesc) throws Exception
    {
	ActionTuple act;
	act = new ActionTuple();
	String[] actions;
	CUAction ca = new CUAction();
	actions = actionDesc.split(";");
	if( actions.length != 9) {
	    throw new Exception("Wrong format");
	}
	String actionName = actions[0];
	int actionLevel = Integer.parseInt(actions[1]);
	int actionId = Integer.parseInt(actions[2]);

	int parentId = Integer.parseInt(actions[3]);
	String cond = actions[4];
	boolean condition = true;
	if(cond.equals("fail"))
	    condition = false; 
	else if(cond.equals("success"))
	    condition = true;
	else {
	    // condition = true;
	    System.out.println("Wrong format.");
	    throw new Exception("Wrong format");
	}
	
	String moveAction = actions[5];
	String perceptionAction = actions[6];
	String graspAction = actions[7];
	
	MoveAction ma = parseMoveAction(moveAction);
	PerceptionAction pa = parsePerceptionAction(perceptionAction);
	GraspAction ga = parseGraspAction(graspAction);
	ca.ma = ma;
	ca.pa = pa;
	ca.ga = ga;

	ca.status = 0;

	String actionFlags = actions[8];
	ca.actionFlags = parseActionFlags(actionFlags);
	
	act.setCUAction(ca);
	act.setActionName(actionName);
	act.setActionLevel(actionLevel);
	act.setActionId(actionId);
	act.setParentId(parentId);
	act.setCondition(condition);

	/*
	if(act.getActionName().equals("finish_success")){
	    ca.status = 1;
	}
	if(act.getActionName().equals("finish_fail")) {
	    ca.status = -1;
	}
	*/

	if(act.getActionName().indexOf("finish") != -1 && act.getCondition()){
	    ca.status = 1;
	}
	if(act.getActionName().indexOf("finish") != -1 && !act.getCondition()) {
	    ca.status = -1;
	}
	
	return act;
    }

    private  static MoveAction parseMoveAction(String moveAction) throws Exception
    {
	MoveAction ma = new MoveAction();
	String[] parameters = moveAction.split(" ");
	double x = Double.parseDouble(parameters[0]);
	double y = Double.parseDouble(parameters[1]);
	double theta = Double.parseDouble(parameters[2]);
	ma.targetPose2D.x = x;
	ma.targetPose2D.y = y;
	ma.targetPose2D.theta = theta;
	//System.out.println("X is   " + ma.targetPose2D.x + "   Y is   " + ma.targetPose2D.y);
	return ma;
    }

    private  static int[] parseActionFlags(String actionFlags) throws Exception
    {
	int[] _actionFlags = new int[3];
	
	String[] parameters = actionFlags.split(" ");
	//System.out.println(parameters[0] + " -- " + parameters[1] + " -- " + parameters[2]);
	_actionFlags[0] = Integer.parseInt(parameters[0].trim());
	_actionFlags[1] = Integer.parseInt(parameters[1].trim());
 	_actionFlags[2] = Integer.parseInt(parameters[2].trim());

	//System.out.println(actionFlags);
	/*
	for(int i = 0 ; i < 3; i++) {
	    System.out.println(i + " :  " + _actionFlags[i]);
	}
	*/
	return _actionFlags;
    }

    private static PerceptionAction parsePerceptionAction(String perceptionAction) throws Exception
    {
	PerceptionAction pa = new PerceptionAction();
	String[] parameters = perceptionAction.split(" ");
	String detectType = parameters[0];
	int id = Integer.parseInt(parameters[1]); // if detectType is object, then object id, and so on
	/*
	  string detectType
	  
	  ABoxObject aboxObject
	  TBoxObject tboxClass
	 */
	pa.detectType = detectType;
	pa.aboxObject.object_id = id;
	return pa;
    }

    private static GraspAction parseGraspAction(String graspAction) throws Exception
    {
	GraspAction ga = new GraspAction();
	String[] parameters = graspAction.split(" ");
	String _ifGrasp = parameters[0];
	ga.ifGrasp = false;
	if(_ifGrasp.equals("true")) {
	    ga.ifGrasp = true;
	}
	else if(_ifGrasp.equals("false")) {
	    ga.ifGrasp = false;
	}
	
	if(!ga.ifGrasp)   {
	    return ga;
	}

	//... other parameters

	return ga;
    }


    private TaskType taskType;
    private String targetContent;
    private int taskId;
    //private ArrayList<CUAction> actionSequence;
    private ArrayList<ActionTuple> acts;
    private int currentActionId = 1;
    private ActionTuple currentAction;
    private int currentActionLoc = 0; 
}
