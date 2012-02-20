package org.srs.srs_knowledge.knowledge_engine;

import java.lang.Exception;

public class DuplicatedEntryException extends Exception 
{
    public DuplicatedEntryException() 
    {
    }
    public DuplicatedEntryException(String msg) 
    {
	super(msg);
    }
}