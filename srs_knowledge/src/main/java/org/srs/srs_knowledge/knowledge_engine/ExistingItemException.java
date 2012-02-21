package org.srs.srs_knowledge.knowledge_engine;

import java.lang.Exception;

public class ExistingItemException extends Exception 
{
    public ExistingItemException() 
    {
    }
    public ExistingItemException(String msg) 
    {
	super(msg);
    }
}