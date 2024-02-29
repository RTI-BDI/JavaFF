/************************************************************************
 * Strathclyde Planning Group,
 * Department of Computer and Information Sciences,
 * University of Strathclyde, Glasgow, UK
 * http://planning.cis.strath.ac.uk/
 * 
 * Copyright 2007, Keith Halsey
 * Copyright 2008, Andrew Coles and Amanda Smith
 *
 * (Questions/bug reports now to be sent to Andrew Coles)
 *
 * This file is part of JavaFF.
 * 
 * JavaFF is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * JavaFF is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with JavaFF.  If not, see <http://www.gnu.org/licenses/>.
 * 
 ************************************************************************/

package javaff.data.strips;

import javaff.data.PDDLPrinter;
import javaff.data.PDDLPrintable;
import javaff.data.Parameter;
import javaff.data.metric.FunctionSymbol;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.List;
import java.util.Iterator;
import java.io.PrintStream;

public class PredicateSymbol implements PDDLPrintable, Serializable, Cloneable
{
	protected String name;
	protected boolean staticValue;	
	// is it an internal self-made proposition to support the search (such as action "a" ended, e.g. "ga") 
	// or a valid predicate defined within the PDDL domain
	protected boolean domainDefined;

	protected ArrayList<Parameter> params = new ArrayList<>(); //The parameters (types) that this predicate symbol takes

	protected PredicateSymbol()
	{
		
	}

	public PredicateSymbol(String pName, boolean domainDefined)
	{
		name = pName;
		this.domainDefined = domainDefined;
	}

	public Object clone(){
		PredicateSymbol ps = new PredicateSymbol(name, domainDefined);
		ps.staticValue = staticValue;
		for(Parameter p : (List<Parameter>)params) {
			if (p instanceof Variable)
				ps.params.add((Parameter) ((Variable) p).clone());
			else if (p instanceof PDDLObject)
				ps.params.add((Parameter) ((PDDLObject) p).clone());
		}
		return ps;
	}

	public boolean isDomainDefined(){return domainDefined;}

	public String getName()
	{
		return name;
	}

	public ArrayList<String> getParams()
	{
		ArrayList<String> parameters = new ArrayList<String>();
		for(Parameter param : params)
			parameters.add(param.getName());
		return parameters;
	}
	
	public String toString()
	{
		return name;
	}

	public String toStringTyped()
	{
		String str = name;
		Iterator it = params.iterator();
		while (it.hasNext())
		{
			Variable v = (Variable) it.next();
			str += " " + v.toStringTyped();
		}
		return str;
	}

	public boolean isStatic()
	{
		return staticValue;
	}

	public void setStatic(boolean stat)
	{
		staticValue = stat;
	}

	public void addVar(Variable v)
	{
		params.add(v);
	}

	public boolean equals(Object obj)
	{
		if (obj instanceof PredicateSymbol)
		{
			PredicateSymbol ps = (PredicateSymbol) obj;
			return (name.equals(ps.name) && params.equals(ps.params));
		}
		else return false;
	}

	public int hashCode()
	{
		int hash = 8;
		hash = 31 * hash ^ name.hashCode();
		hash = 31 * hash ^ params.hashCode();
		return hash;
	}

	public void PDDLPrint(PrintStream p, int indent)
	{
		PDDLPrinter.printToString(this, p, true, true, indent);
	}
}
