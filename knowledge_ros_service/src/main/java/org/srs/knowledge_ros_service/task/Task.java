package org.srs.knowledge_ros_service.task;

import java.io.IOException;
import java.io.*;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.knowledge_ros_service.msg.*;


public class Task
{
    public enum TaskType {GET_OBJECT, MOVETO_LOCATION, DETECT_OBJECT, SCAN_AROUND};

    public Task(TaskType type)
    {
	acts = new ArrayList<ActionTuple>();
	//actionSequence = new ArrayList<CUAction>();
	setTaskType(type);
	currentAction = null;
    }

    public void setTaskId(int id)
    {
	this.taskId = id;
    }

    public int getTaskId()
    {
	return taskId;
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
		    System.out.println(currentAction.getActionName());

		    return acts.get(i);
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
		    System.out.println("NLKJALKJ");
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
	if( actions.length != 8) {
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
	
	act.setCUAction(ca);
	act.setActionName(actionName);
	act.setActionLevel(actionLevel);
	act.setActionId(actionId);
	act.setParentId(parentId);
	act.setCondition(condition);
	
	return act;
    }

    private  static MoveAction parseMoveAction(String moveAction) throws Exception
    {
	MoveAction ma = new MoveAction();
	String[] parameters = moveAction.split(" ");
	double x = Double.parseDouble(parameters[0]);
	double y = Double.parseDouble(parameters[1]);
	double theta = Double.parseDouble(parameters[2]);
	return ma;
    }

    private static PerceptionAction parsePerceptionAction(String perceptionAction) throws Exception
    {
	PerceptionAction pa = new PerceptionAction();
	String[] parameters = perceptionAction.split(" ");
	String detectType = parameters[0];
	int id = Integer.parseInt(parameters[1]); // if detectType is object, then object id, and so on
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
    private int taskId;
    //private ArrayList<CUAction> actionSequence;
    private ArrayList<ActionTuple> acts;
    private int currentActionId = 1;
    private ActionTuple currentAction;
    private int currentActionLoc = 0; 
}
