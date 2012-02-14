package org.srs.srs_knowledge.task;

import java.io.*;
//import org.apache.commons.logging.Log;
import java.util.ArrayList;
import ros.pkg.srs_knowledge.msg.*;

/**
 * A container of CUAction and additional information
 */
public class ActionTuple
{
    public ActionTuple()
    {
	//ca = new CUAction();
    }

    public void setCUAction(CUAction ca)
    {
	this.ca = ca;
    }

    public void setActionName(String actionName)
    {
	this.actionName = actionName;
    }

    public void setActionLevel(int actionLevel)
    {
	this.actionLevel = actionLevel;
    }
    
    public void setActionId(int actionId)
    {
	this.actionId = actionId;
    }
    
    public void setParentId(int parentId)
    {
	this.parentId = parentId;
    }
    
    public void setCondition(boolean condition)
    {
	this.condition = condition;
    }

    public CUAction getCUAction()
    {
	return this.ca;
    }

    public String getActionName()
    {
	return actionName;
    }

    public int getActionLevel()
    {
	return actionLevel;
    }
    
    public int getActionId()
    {
	return actionId;
    }
    
    public int getParentId()
    {
	return this.parentId;
    }

    public boolean getCondition()
    {
	return condition;
    }


    private CUAction ca;

    private String actionName = "";
    private int actionId;
    private int parentId;
    private int actionLevel;
    private boolean condition; 

}