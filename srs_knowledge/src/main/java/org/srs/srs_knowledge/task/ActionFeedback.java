package org.srs.srs_knowledge.task;

import java.io.*;
import java.util.StringTokenizer;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;
import ros.pkg.geometry_msgs.msg.Pose2D;
import ros.pkg.geometry_msgs.msg.Pose;
import org.srs.srs_knowledge.knowledge_engine.*;
import org.srs.srs_knowledge.task.Task;

public class ActionFeedback extends ArrayList<String> {
    public ActionFeedback(ArrayList<String> genericFeedback) {
	super(genericFeedback);
    }
    
    public static Pose toPose(ActionFeedback fb) {
	if(!fb.get(0).equals("detect")) {
	    return null;
	}
	try {
	    Pose pos = new Pose(); 
	    
	    System.out.println("detect: Object " + fb.get(1));
	    pos.position.x = Double.valueOf(fb.get(2));
	    pos.position.y = Double.valueOf(fb.get(3));
	    pos.position.z = Double.valueOf(fb.get(4));
	    pos.orientation.x = Double.valueOf(fb.get(5));
	    pos.orientation.y = Double.valueOf(fb.get(6));
	    pos.orientation.z = Double.valueOf(fb.get(7));
	    pos.orientation.w = Double.valueOf(fb.get(8));
	    return pos;
	}
	catch(Exception e) {
	    
	    System.out.println(e.toString());
	    return null;

	}
    }

}