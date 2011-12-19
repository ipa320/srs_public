package org.srs.srs_knowledge.task;

import java.io.IOException;
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
	acts = new ArrayList<ActionTuple>();
	// actionSequence = new ArrayList<CUAction>();
	setTaskType(TaskType.UNSPECIFIED);
	currentAction = null;
	ontoDB = new OntologyDB();
    }
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
/*
	private void init(String taskType, String targetContent, Pose2D userPose) {
		acts = new ArrayList<ActionTuple>();
		currentAction = null;
		if (taskType.toLowerCase().equals("get")) {
			setTaskTarget(targetContent);
			setTaskType(TaskType.GET_OBJECT);
			//constructTask();
		} else if (taskType.toLowerCase().equals("move")) {
			setTaskType(TaskType.MOVETO_LOCATION);

			setTaskTarget(targetContent);
			System.out.println("TASK.JAVA: Created CurrentTask " + "move "
					+ targetContent);
			//constructTask();
			createSimpleMoveTaskNew();
		} else if (taskType.toLowerCase().equals("search")) {
			//constructTask();
			setTaskType(TaskType.SEARCH_OBJECT);
		} else if (taskType.toLowerCase().equals("stop")) {
			//constructTask();
			setTaskType(TaskType.STOP_TASK);
		} else if (taskType.toLowerCase().equals("getn")) {
			// new implementation of get action, specifically for milkbox in
			// this case
			setTaskType(TaskType.GET_OBJECT);
			setTaskTarget(targetContent);
			//System.out.println("TASK.JAVA: Created CurrentTask " + " get "
			//		+ targetContent);
			//this.createGetObjectTask();
		} else {
			setTaskType(TaskType.UNSPECIFIED);
		}
	}
*/
	protected boolean constructTask() {
		return true;
	}
    /*
	private boolean createSimpleMoveTaskNew() {
		// boolean addNewActionTuple(ActionTuple act)
		ActionTuple act = new ActionTuple();

		CUAction ca = new CUAction();
		MoveAction ma = new MoveAction();
		PerceptionAction pa = new PerceptionAction();
		GraspAction ga = new GraspAction();

		// TODO: should obtain information from Ontology here. But here we use
		// temporary hard coded info for testing
		double x = 1;
		double y = 1;
		double theta = 0;

		if (this.targetContent.charAt(0) == '['
				&& this.targetContent.charAt(targetContent.length() - 1) == ']') {
			StringTokenizer st = new StringTokenizer(targetContent, " [],");
			if (st.countTokens() == 3) {
				try {
					x = Double.parseDouble(st.nextToken());
					y = Double.parseDouble(st.nextToken());
					theta = Double.parseDouble(st.nextToken());
					System.out.println(x + "  " + y + " " + theta);
				} catch (Exception e) {
					System.out.println(e.getMessage());
					return false;
				}
			}
		} else {
			System.out.println("======MOVE COMMAND FORMAT=======");
			// TODO Ontology queries
			String prefix = "PREFIX srs: <http://www.srs-project.eu/ontologies/srs.owl#>\n"
					+ "PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>\n"
					+ "PREFIX ipa-kitchen-map: <http://www.srs-project.eu/ontologies/ipa-kitchen-map.owl#>\n";
			String queryString = "SELECT ?x ?y ?theta WHERE { "
					+ "ipa-kitchen-map:" + targetContent
					+ " srs:xCoordinate ?x . " + "ipa-kitchen-map:"
					+ targetContent + " srs:yCoordinate ?y . "
					+ "ipa-kitchen-map:" + targetContent
					+ " srs:orientationTheta ?theta .}";
			System.out.println(prefix + queryString + "\n");

			if (this.ontoDB == null) {
				System.out.println("Ontology Database is NULL");
				return false;
			}

			try {
				ArrayList<QuerySolution> rset = ontoDB.executeQueryRaw(prefix
						+ queryString);
				// ArrayList<String> reslist =
				// (ArrayList)(rset.getResultVars());
				// for(String v:reslist) {
				// System.out.println(v);
				// }
				try {
					if (rset.size() == 0) {
						System.out
								.println("ERROR: No move target found from database");
						return false;
					} else if (rset.size() == 1) {
						System.out
								.println("INFO: OK info retrieved from DB... ");
						QuerySolution qs = rset.get(0);
						x = qs.getLiteral("x").getFloat();
						y = qs.getLiteral("y").getFloat();
						theta = qs.getLiteral("theta").getFloat();
						System.out.println("x is " + x + ". y is  " + y
								+ ". theta is " + theta);
					} else {
						System.out.println("WARNING: Multiple options... ");
						QuerySolution qs = rset.get(0);
						x = qs.getLiteral("x").getFloat();
						y = qs.getLiteral("y").getFloat();
						theta = qs.getLiteral("theta").getFloat();
						System.out.println("x is " + x + ". y is  " + y
								+ ". theta is " + theta);
					}
				} catch (Exception e) {
					System.out.println(e.getMessage());
					return false;
				}
			} catch (Exception e) {
				System.out.println("Exception -->  " + e.getMessage());
				return false;
			}
			// return false;
		}

		ma.targetPose2D.x = x;
		ma.targetPose2D.y = y;
		ma.targetPose2D.theta = theta;

		ca.ma = ma;
		ca.pa = pa;
		ca.ga = ga;

		try {
			ca.actionFlags = parseActionFlags("0 1 1");
		} catch (Exception e) {
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
		} catch (Exception e) {
			System.out.println(e.getMessage());
		}
		act.setActionName("finish_success");

		if (act.getActionName().equals("finish_success")) {
			ca.status = 1;
		}
		if (act.getActionName().equals("finish_fail")) {
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
		} catch (Exception e) {
			System.out.println("Exception -->  " + e.getMessage());
		}

		act.setActionName("finish_fail");

		// ca.status = -1;
		if (act.getActionName().equals("finish_success")) {
			ca.status = 1;
		}
		if (act.getActionName().equals("finish_fail")) {
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
    */
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

	public ArrayList<ActionTuple> getActionSequence() {
		return acts;
	}

	public ActionTuple getNextAction(boolean stateLastAction) {
		if (currentAction == null) {
			for (int i = 0; i < acts.size(); i++) {
				if (acts.get(i).getActionId() == 1) {
					currentAction = acts.get(i);
					currentActionLoc = i;
					System.out.println(i);
					System.out
							.println(currentAction.getCUAction().ma.targetPose2D.x);
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

	protected TaskType taskType;
	protected String targetContent;
	protected int taskId;
	// private ArrayList<CUAction> actionSequence;
	protected ArrayList<ActionTuple> acts;
	protected int currentActionId = 1;
	protected ActionTuple currentAction;
	protected int currentActionLoc = 0;
	protected OntologyDB ontoDB;
}
