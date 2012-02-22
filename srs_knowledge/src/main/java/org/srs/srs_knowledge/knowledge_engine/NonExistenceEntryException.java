package org.srs.srs_knowledge.knowledge_engine;

import java.lang.Exception;

public class NonExistenceEntryException extends Exception 
{
    public NonExistenceEntryException() 
    {
    }
    public NonExistenceEntryException(String msg) 
    {
	super(msg);
    }
}