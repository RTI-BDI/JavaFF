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

package javaff.search;

import javaff.planning.STRIPSState;
import javaff.planning.State;
import javaff.planning.Filter;
import javaff.planning.TemporalMetricState;

import java.math.BigDecimal;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.TreeSet;
import java.util.Hashtable;


public class BestFirstSearch extends Search
{
	protected BigDecimal bestHValue;
	protected BigDecimal bestHValueNoOpenAction;

	protected State bestIntermediateState;
	
	protected Hashtable closed;
	protected TreeSet open;
	protected Filter filter = null;
	protected float searchIntervalMs = 1000.0F;
	protected int maxPPlanSize = 32000;
	
	public BestFirstSearch(State s, float searchIntervalMs, int maxPPlanSize, TreeSet<State> open, Hashtable<Integer, State> closed)
    {
		this(s, searchIntervalMs, maxPPlanSize, new HValueComparator());

		this.open = open;
		this.closed = closed;
	}

	public BestFirstSearch(State s, float searchIntervalMs, int maxPPlanSize, Comparator c)
    {
		super(s);
		setComparator(c);

		this.searchIntervalMs = searchIntervalMs;
		this.maxPPlanSize = maxPPlanSize;
	}

	public void setFilter(Filter f)
	{
		filter = f;
	}

	public void updateOpen(State S)
    {
		open.addAll(S.getNextStates(filter.getActions(S)));
	}

	public State removeNext()
    {
		State S = (State) ((TreeSet) open).first();
		open.remove(S);
                /*
                System.out.println("================================");
		S.getSolution().print(System.out);
		System.out.println("----Helpful Actions-------------");
		javaff.planning.TemporalMetricState ms = (javaff.planning.TemporalMetricState) S;
		System.out.println(ms.helpfulActions);
		System.out.println("----Relaxed Plan----------------");
		ms.RelaxedPlan.print(System.out);
                */
		return S;
	}

	public boolean needToVisit(State s) {
		Integer Shash = new Integer(s.hashCode());
		State D = (State) closed.get(Shash);
		
		if (closed.containsKey(Shash) && D.equals(s)) return false;
		
		closed.put(Shash, s);
		return true;
	}

	public State search() {
		long startSearchTime = System.currentTimeMillis();
		bestHValue = start.getHValue(); // and take its heuristic value as the best so far
		bestHValueNoOpenAction = start.getHValue(); // best heuristic value so far for states with NO open action
		bestIntermediateState = start; // best intermediate state at start is start

		open.add(start);

		while (System.currentTimeMillis() - startSearchTime < this.searchIntervalMs && !open.isEmpty())
		{
			if(bestIntermediateState instanceof TemporalMetricState && ((TemporalMetricState) bestIntermediateState).getRealGValue() >= maxPPlanSize)
				break;
			else if(bestIntermediateState instanceof STRIPSState && ((STRIPSState) bestIntermediateState).getGValue().intValue() >= maxPPlanSize)
				break;
			State s = removeNext();
			if (needToVisit(s)) {
				++nodeCount;

				if (s.goalReached()) {
					return s;
				} else {

					//keep track of best intermediate state with no open actions
					TemporalMetricState tms = (TemporalMetricState) s;
					if(tms.openActions.isEmpty() && tms.getHValue().compareTo(bestHValueNoOpenAction) < 0)//update bestHValueNoOpenAction
					{
						bestHValueNoOpenAction = tms.getHValue();
						bestIntermediateState = tms;
					}

					if (tms.getHValue().compareTo(bestHValue) < 0) // if we've found a state with a better heuristic value than the best seen so far
						bestHValue = tms.getHValue(); // note the new best value

					updateOpen(s);
				}
			}
			
		}


		if(open.isEmpty())// no reason to move forward: unsat with EHC
		{
			return null;
		}
		// pick the state having bestHValueNoOpenAction and return that plan
		return bestIntermediateState;
	}

}
