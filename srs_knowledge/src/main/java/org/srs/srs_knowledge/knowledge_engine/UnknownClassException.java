package org.srs.srs_knowledge.knowledge_engine;

import java.lang.Exception;

public class UnknownClassException extends Exception 
{
    public UnknownClassException() 
    {
    }
    public UnknownClassException(String msg) 
    {
	super(msg);
    }
}