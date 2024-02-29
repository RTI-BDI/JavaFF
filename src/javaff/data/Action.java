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

package javaff.data;

import javaff.data.strips.OperatorName;
import javaff.planning.STRIPSState;
import javaff.planning.State;

import java.io.Serializable;
import java.util.*;
import java.math.BigDecimal;

public abstract class Action implements Serializable, Cloneable
{
	public OperatorName name;
	public int instanceCounter = 1;
    public List params = new ArrayList(); // List of PDDLObjects

	public BigDecimal cost = new BigDecimal(0);

    public String toString()
    {
		String stringrep = name.toString();
		Iterator i = params.iterator();
		while (i.hasNext())
		{
			stringrep = stringrep + " " +  i.next();
		}
		return stringrep;
    }

	public String toStringBase()
	{
		String stringrep = name.toString();
		Iterator i = params.iterator();
		while (i.hasNext())
		{
			stringrep = stringrep + " " +  i.next();
		}
		return stringrep;
	}

	public abstract boolean isApplicable(State s);
	public abstract void apply(State s);
	public abstract Set getConditionalPropositions();
	public abstract Set getAddPropositions();
	public abstract Set getDeletePropositions();
	public abstract Set getComparators();
	public abstract Set getOperators();
	public abstract void staticify(Map fValues);

	public boolean equals(Object obj)
    {
		if (obj instanceof Action)
		{
			Action a = (Action) obj;
			return (name.equals(a.name) && params.equals(a.params)) && instanceCounter == a.instanceCounter;
		}
		else return false;
    }

    public int hashCode()
    {
		if(name == null || params == null)
			System.out.println("");
        return name.hashCode()*(1+instanceCounter) ^ params.hashCode();
    }
}
