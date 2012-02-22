package org.srs.srs_knowledge.knowledge_engine;

import java.lang.Exception;

public class UnknownException extends Exception 
{
    public UnknownException() 
    {
    }
    public UnknownException(String msg) 
    {
	super(msg);
    }
}