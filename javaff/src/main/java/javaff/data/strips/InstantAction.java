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

import javaff.data.GroundCondition;
import javaff.data.GroundEffect;
import javaff.data.Action;
import javaff.planning.STRIPSState;
import javaff.planning.State;

import java.util.Iterator;
import java.util.Set;
import java.util.Map;

public abstract class InstantAction extends Action
{
	public GroundCondition condition;
	public GroundEffect effect;

	public boolean isApplicable(State s)
	{
		return condition.isTrue(s) && s.checkAvailability(this);
	}

	/* Returns true if actions already applied in solution to get to s*/
	public boolean alreadyApplied(STRIPSState s){
		for(Action a : s.getTPSolution().getOrderedActions())
			if(this.equals(a))
				return true;
		return false;
	}

	public String toString()
	{
		String stringrep = name.toString() + instanceCounter;
		Iterator i = params.iterator();
		while (i.hasNext())
		{
			stringrep = stringrep + " " +  i.next();
		}
		return stringrep;
	}

	public void apply(State s)
	{
		effect.applyDels(s);
		effect.applyAdds(s);
	}

	public Set getConditionalPropositions()
	{
		return condition.getConditionalPropositions();
	}

	public Set getAddPropositions()
	{
		return effect.getAddPropositions();
	}

	public Set getDeletePropositions()
	{
		return effect.getDeletePropositions();
	}

	public Set getComparators()
	{
		return condition.getComparators();
	}

	public Set getOperators()
	{
		return effect.getOperators();
	}

	public void staticify(Map fValues)
	{
		condition = condition.staticifyCondition(fValues);
		effect = effect.staticifyEffect(fValues);
	}

	public boolean isEqualIgnoreStartEnd(Object obj){
		if (obj instanceof InstantAction)
		{
			InstantAction ia = (InstantAction) obj;
			String myName = this.name.name.substring(0,this.name.name.lastIndexOf("_"));
			String iaName = ia.name.name.substring(0,ia.name.name.lastIndexOf("_"));
			return (myName.equals(iaName) && params.equals(ia.params));
		}
		else return false;
	}
}
