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

import java.math.BigDecimal;

public class TimeStampedAction implements Comparable
{
	public Action action;
	public BigDecimal time;
	public BigDecimal duration;

	public boolean committed = false;

	public short status = 0;

	public TimeStampedAction(Action a, BigDecimal t, BigDecimal d)
	{
		action = a;
		time = t;
		duration = d;
	}

	public String toString()
	{
		String str = time +": ("+action+")";
		if (duration != null) str += " ["+duration+"]";
		return str;
	}

	public String toStringFullNameTimex1000()
	{
		return "("+action+"):"+((int)(time.floatValue()*1000));
	}

	public String toStringWithExecStatus()
	{
		String runningStatus = "";
		switch(status)
		{
			case 0:
				runningStatus = "WAITING";
				break;
			case 1:
				runningStatus = "RUNNING";
				break;
			case 2:
				runningStatus = "RUN_SUC";
				break;
			case 3:
				runningStatus = "SUCCESS";
				break;
			case 4:
				runningStatus = "FAILURE";
				break;
		}
		String str = runningStatus +
				"\t" + (committed? "COMMIT" : "NO_COM") +
			"\t" + time +": ("+action+")";
		if (duration != null) str += " ["+duration+"]";
		return str;
	}

	public int compareTo(Object o)
	{
		TimeStampedAction that = (TimeStampedAction) o;
		if (this.time.compareTo(that.time) != 0) return this.time.compareTo(that.time);
		return ((new Integer(this.action.hashCode())).compareTo(new Integer(that.action.hashCode())));
	}
}
