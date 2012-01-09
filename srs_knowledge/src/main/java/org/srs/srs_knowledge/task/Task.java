package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import org.srs.srs_knowledge.knowledge_engine.*;
import com.hp.hpl.jena.rdf.model.*;
import com.hp.hpl.jena.query.QueryExecutionFactory;
import com.hp.hpl.jena.query.ResultSet;
import com.hp.hpl.jena.query.QueryExecution;
import com.hp.hpl.jena.query.QuerySolution;
import ros.*;
import ros.communication.*;

public abstract class Task {
	public enum TaskType {
		GET_OBJECT, MOVETO_LOCATION, SEARCH_OBJECT, SCAN_AROUND, STOP_TASK, UNSPECIFIED
	};

    /*
	public Task(TaskType type) {
		acts = new ArrayList<ActionTuple>();
		// actionSequence = new ArrayList<CUAction>();
		setTaskType(type);
		currentAction = null;
		ontoDB = new OntologyDB();
	}
    */

    public Task() {
	// empty constructor.
	acts = new ArrayList<ActionTuple>();     // to be deprecated and replaced with allSubSeqs
	
	setTaskType(TaskType.UNSPECIFIED);
	currentAction = null;
	ontoDB = new OntologyDB();

    }

    public abstract boolean replan(OntologyDB onto, OntoQueryUtil ontoQuery);


    /*
	public Task(String taskType, String targetContent, Pose2D userPose) {
		ontoDB = new OntologyDB();
		this.init(taskType, targetContent, userPose);
	}
    */    
    /*	public Task(String taskType, String targetContent, Pose2D userPose,
			OntologyDB onto) {
		if (onto != null) {
			System.out.println("SET ONTOLOGY DB");
			this.ontoDB = onto;
		} 
		else {
			System.out.println("NULL ---- SET ONTOLOGY DB");
			this.ontoDB = new OntologyDB();
		}

		this.init(taskType, targetContent, userPose);
	}
    */	
    protected abstract boolean constructTask();// {
    //		return true;
    //	}

	public void setTaskId(int id) {
		this.taskId = id;
	}

	public int getTaskId() {
		return taskId;
	}

	public void setTaskTarget(String target) {
		this.targetContent = target;
	}

	public String getTaskTarget() {
		return this.targetContent;
	}

	public void setTaskType(TaskType type) {
		this.taskType = type;
	}

/*
	public ArrayList<ActionTuple> getActionSequence() {
		return acts;
	}
*/
    public CUAction getNextCUAction(boolean stateLastAction, ArrayList<String> feedback) {
	//CUAction ca = new CUAction();
	if (currentAction == null) {
	    for (int i = 0; i < acts.size(); i++) {
		if (acts.get(i).getActionId() == 1) {
		    currentAction = acts.get(i);
		    currentActionLoc = i;
		    //System.out.println(i);
		    //System.out.println(currentAction.getCUAction().ma.targetPose2D.x);
		    System.out.println(currentAction.getActionName());
		    
		    //return currentAction.getCUAction();
		    if (currentAction != null) 
			return currentAction.getCUAction();
		    else 
			return null;
		}
	    }
	} else {
	    // ActionTuple at;
	    // int parentId = at.getParentId();
	    for (int i = 0; i < acts.size(); i++) {
		// if(acts.get(currentActionLoc).getId() ==
		// acts.get(i).getParentId() && stateLastAction ==
		// acts.get(i).getCondition()){
		if (currentAction.getActionId() == acts.get(i).getParentId()
		    && stateLastAction == acts.get(i).getCondition()) {
		    currentAction = acts.get(i);
		    currentActionLoc = i;
		    System.out.println(i);
		    System.out.println(currentAction.getActionName());
		    if (currentAction != null) 
			return currentAction.getCUAction();
		    else 
			return null;
		}
	    }
	    System.out.println("no action found");
	}	
	
	return null;
    }
    /*
	public ActionTuple getNextAction(boolean stateLastAction) {
		if (currentAction == null) {
			for (int i = 0; i < acts.size(); i++) {
				if (acts.get(i).getActionId() == 1) {
					currentAction = acts.get(i);
					currentActionLoc = i;
					//System.out.println(i);
					//System.out.println(currentAction.getCUAction().ma.targetPose2D.x);
					System.out.println(currentAction.getActionName());

					return currentAction;
				}
			}
		} else {
			// ActionTuple at;
			// int parentId = at.getParentId();
			for (int i = 0; i < acts.size(); i++) {
				// if(acts.get(currentActionLoc).getId() ==
				// acts.get(i).getParentId() && stateLastAction ==
				// acts.get(i).getCondition()){
				if (currentAction.getActionId() == acts.get(i).getParentId()
						&& stateLastAction == acts.get(i).getCondition()) {
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
    */
	public boolean addNewActionTuple(ActionTuple act) {
		return acts.add(act);
	}

	public boolean loadPredefinedSequence(String filename) throws IOException,
			Exception {
		// ArrayList<ActionTuple> acts = new ArrayList<ActionTuple>();
		File file = null;
		FileReader freader = null;
		LineNumberReader in = null;

		try {
			file = new File(filename);
			freader = new FileReader(file);
			in = new LineNumberReader(freader);
			String line = "";

			while ((line = in.readLine().trim()) != null) {
				// System.out.println("Line: " + in.getLineNumber() + ": " +
				// line);
				if (line.equals("")) {
					continue;
				} else if (line.substring(0, 1).equals("#")) {
					continue;
				}
				ActionTuple act = parseAction(line);
				if (act != null)
					this.acts.add(act);
			}
		} finally {
			freader.close();
			in.close();
		}
		System.out.println("Number of actions is: " + acts.size());
		return true;
	}

	private static ActionTuple parseAction(String actionDesc) throws Exception {
		ActionTuple act;
		act = new ActionTuple();
		String[] actions;
		CUAction ca = new CUAction();
		actions = actionDesc.split(";");
		if (actions.length != 9) {
			throw new Exception("Wrong format");
		}
		String actionName = actions[0];
		int actionLevel = Integer.parseInt(actions[1]);
		int actionId = Integer.parseInt(actions[2]);

		int parentId = Integer.parseInt(actions[3]);
		String cond = actions[4];
		boolean condition = true;
		if (cond.equals("fail"))
			condition = false;
		else if (cond.equals("success"))
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
		 * if(act.getActionName().equals("finish_success")){ ca.status = 1; }
		 * if(act.getActionName().equals("finish_fail")) { ca.status = -1; }
		 */

		if (act.getActionName().indexOf("finish") != -1 && act.getCondition()) {
			ca.status = 1;
		}
		if (act.getActionName().indexOf("finish") != -1 && !act.getCondition()) {
			ca.status = -1;
		}

		return act;
	}

    private static MoveAction parseMoveAction(String moveAction)
	throws Exception {
	MoveAction ma = new MoveAction();
	String[] parameters = moveAction.split(" ");
	double x = Double.parseDouble(parameters[0]);
	double y = Double.parseDouble(parameters[1]);
	double theta = Double.parseDouble(parameters[2]);
	
	ma.targetPose2D.x = x;
	ma.targetPose2D.y = y;
		ma.targetPose2D.theta = theta;

		String ifWaitForObjectTaken = parameters[3];
		boolean ifWaitFOT = false;
		if (ifWaitForObjectTaken.equals("true")) {
		    ifWaitFOT = true;
		}
		ma.ifWaitObjectTaken = ifWaitFOT;
		// System.out.println("X is   " + ma.targetPose2D.x + "   Y is   " +
		// ma.targetPose2D.y);

		return ma;
	}

	protected static int[] parseActionFlags(String actionFlags) throws Exception {
		int[] _actionFlags = new int[3];

		String[] parameters = actionFlags.split(" ");
		// System.out.println(parameters[0] + " -- " + parameters[1] + " -- " +
		// parameters[2]);
		_actionFlags[0] = Integer.parseInt(parameters[0].trim());
		_actionFlags[1] = Integer.parseInt(parameters[1].trim());
		_actionFlags[2] = Integer.parseInt(parameters[2].trim());

		// System.out.println(actionFlags);
		/*
		 * for(int i = 0 ; i < 3; i++) { System.out.println(i + " :  " +
		 * _actionFlags[i]); }
		 */
		return _actionFlags;
	}

	private static PerceptionAction parsePerceptionAction(
			String perceptionAction) throws Exception {
		PerceptionAction pa = new PerceptionAction();
		String[] parameters = perceptionAction.split(" ");
		String detectType = parameters[0];
		int id = Integer.parseInt(parameters[1]); // if detectType is object,
													// then object id, and so on
		/*
		 * string detectType
		 * 
		 * ABoxObject aboxObject TBoxObject tboxClass
		 */
		pa.detectType = detectType;
		pa.aboxObject.object_id = id;
		return pa;
	}

	private static GraspAction parseGraspAction(String graspAction)
			throws Exception {
		GraspAction ga = new GraspAction();
		String[] parameters = graspAction.split(" ");
		String _ifGrasp = parameters[0];
		ga.ifGrasp = false;
		if (_ifGrasp.equals("true")) {
			ga.ifGrasp = true;
		} else if (_ifGrasp.equals("false")) {
			ga.ifGrasp = false;
		}

		if (!ga.ifGrasp) {
			return ga;
		}

		// ... other parameters

		return ga;
	}

    public boolean isEmpty() {
	try {
	    if(allSubSeqs.size() == 0 && acts.size() == 0) {
		return true;
	    }
	}
	catch(NullPointerException e) {
	    System.out.println(e.getMessage() + "\n" + e.toString());
	    return true;
	}
	return false;
    }


	protected TaskType taskType;
	protected String targetContent;
	protected int taskId;
	// private ArrayList<CUAction> actionSequence;
	protected ArrayList<ActionTuple> acts;
	protected int currentActionId = 1;
	protected ActionTuple currentAction;
	protected int currentActionLoc = 0;
	protected OntologyDB ontoDB;
    protected OntoQueryUtil ontoQueryUtil;
    protected NodeHandle nodeHandle;
    protected ArrayList<HighLevelActionSequence> allSubSeqs = new ArrayList<HighLevelActionSequence>();

}
