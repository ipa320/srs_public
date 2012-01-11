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
import ros.pkg.geometry_msgs.msg.Pose2D;
import ros.pkg.geometry_msgs.msg.Pose;
import java.util.Properties;

import java.io.*;
import java.util.StringTokenizer;
import java.util.ArrayList; 
import java.util.Iterator;


public class RosUtil {
    public static Pose getRobotCurrentPose() {
	//Ros ros = Ros.getInstance();
	//ros.init("testKKKK");
	Pose pos = new Pose();
	//NodeHandle nn = ros.createNodeHandle();
	// /robot_pose_ekf/odom_combined
	try {
	Subscriber.QueueingCallback<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped> callback =
	    new Subscriber.QueueingCallback<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped>();
	Subscriber<ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped> sub =
	    KnowledgeEngine.n.subscribe("/robot_pose_ekf/odom_combined", new ros.pkg.geometry_msgs.msg.PoseWithCovarianceStamped(), callback, 10);
	
	KnowledgeEngine.n.spinOnce();
	while (!callback.isEmpty()) {
	    pos = callback.pop().pose.pose;
	    System.out.println(pos);
	}
	KnowledgeEngine.n.shutdown();
	sub.shutdown();
	}
	catch(RosException e) {
	    System.out.println(e.toString());
	    return null;
	}
	catch(InterruptedException e)  {
	    System.out.println(e.toString());
	    return null;
	}
	return pos;
    }
}