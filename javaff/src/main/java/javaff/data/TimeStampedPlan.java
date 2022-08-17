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

import javaff.data.temporal.DurativeAction;
import javaff.data.temporal.SplitInstantAction;
import javaff.scheduling.MatrixSTN;

import java.sql.Time;
import java.util.Set;
import java.util.HashSet;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.Iterator;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.math.BigDecimal;

public class TimeStampedPlan implements Plan
{
	public SortedSet<TimeStampedAction> actions = new TreeSet();

	public TimeStampedAction addAction(Action a, BigDecimal t)
	{
		return addAction(a, t, null);
	}

	public TimeStampedAction addAction(Action a, BigDecimal t, BigDecimal d)
	{
		TimeStampedAction tsa = (new TimeStampedAction(a,t,d));
		actions.add(tsa);
		return tsa;
	}

		
	public void print(PrintStream p)
	{
		Iterator<TimeStampedAction> ait = actions.iterator();
		while (ait.hasNext())
		{
			TimeStampedAction a = ait.next();
			p.println(a);
		}
	}

	public String getPrintablePlan(boolean printExecStatus)
	{
		StringBuilder result = new StringBuilder();
		for (TimeStampedAction action : (Iterable<TimeStampedAction>) actions)
			if(printExecStatus)
				result.append(action.toStringWithExecStatus()).append("\n");
			else
				result.append(action).append("\n");

		return result.toString();
	}

	public void print(PrintWriter p)
	{
		Iterator<TimeStampedAction> ait = actions.iterator();
		while (ait.hasNext())
		{
			TimeStampedAction a = ait.next();
			p.println(a);
		}
	}
	
	public Set getActions()
	{
		Set s = new HashSet();
		Iterator<TimeStampedAction> ait = actions.iterator();
		while (ait.hasNext())
		{
			TimeStampedAction a =  ait.next();
			s.add(a.action);
		}
		return s;
	}

	public void markCommitted(DurativeAction da, BigDecimal startTime){
		for(TimeStampedAction ta : actions)
		{
			if(ta.time.compareTo(startTime) == 0 && ta.action.equals(da))
				ta.committed = true;
		}
	}

	public void markExecStatus(String durativeActionName, BigDecimal startTime, short execStatus){
		//System.out.println("Marking '" + durativeActionName + "' starting at " + startTime + " as " + execStatus);
		for(TimeStampedAction ta : actions)
		{
			if(ta.time.compareTo(startTime) == 0 && ta.action.toString().equals(durativeActionName))
				ta.status = execStatus;
		}
	}

	public SortedSet<TimeStampedAction> getSortedActions()
	{
		return actions;
	}

	public TimeStampedAction getTimeStampedAction(String actionFullName)
	{
		BigDecimal actionTime = BigDecimal.valueOf(Float.parseFloat(actionFullName.substring(actionFullName.lastIndexOf(":")+1))/1000.0f).setScale(MatrixSTN.SCALE,MatrixSTN.ROUND);
		String actionName = actionFullName.substring(actionFullName.indexOf("(")+1, actionFullName.lastIndexOf(")"));

		Iterator<TimeStampedAction> itsa = this.getSortedActions().iterator();
		while(itsa.hasNext()) {
			TimeStampedAction tsa = itsa.next();
			if (actionTime.compareTo(tsa.time) == 0 && actionName.equals((tsa.action).toString()))
				return tsa;
		}

		return null;
	}
	public TreeSet<SplitInstantAction> getSortedSplitInstantActions()
	{
		TreeSet<SplitInstantAction> splitInstantActions = new TreeSet<>();
		Iterator ait = actions.iterator();
		while (ait.hasNext())
		{
			TimeStampedAction a = (TimeStampedAction) ait.next();
			if (a.action instanceof DurativeAction)
			{
				DurativeAction da = (DurativeAction) a.action;
				splitInstantActions.add(da.startAction);
				splitInstantActions.add(da.endAction);
			}
		}
		return splitInstantActions;
	}
}
