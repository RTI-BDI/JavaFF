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

import javaff.data.temporal.DurativeAction;
import javaff.planning.State;
import javaff.planning.Filter;
import javaff.planning.TemporalMetricState;

import java.util.*;
import java.math.BigDecimal;

public class EnforcedHillClimbingSearch extends Search
{
	protected BigDecimal bestHValue;
	protected BigDecimal bestHValueNoOpenAction;

	protected Hashtable<Integer, State> closed;
	protected TreeSet<State> open;
	protected Filter filter = null;

	protected State bestIntermediateState;

	protected float searchIntervalMs = 1000.0F;

	public EnforcedHillClimbingSearch(State s)
	{
		this(s, new HValueComparator());
	}

	public EnforcedHillClimbingSearch(State s, float searchIntervalMs)
	{
		this(s, new HValueComparator());
		this.searchIntervalMs = searchIntervalMs;
	}

	public EnforcedHillClimbingSearch(State s, float searchIntervalMs, TreeSet<State> open, Hashtable<Integer, State> closed)
	{
		this(s, new HValueComparator());
		this.searchIntervalMs = searchIntervalMs;
		this.open = open;
		this.closed = closed;
	}

	public EnforcedHillClimbingSearch(State s, Comparator c)
	{
		super(s);
		this.bestIntermediateState = s;
		setComparator(c);

		closed = new Hashtable<>();
		open = new TreeSet<>();
	}

	public void setFilter(Filter f)
	{
		filter = f;
	}

	public State removeNext()
	{
		State S = (State) ((TreeSet) open).first();
		open.remove(S);

		return S;
	}

	public boolean needToVisit(State s) {
		Integer Shash = s.hashCode(); // compute hash for state
		State D = closed.get(Shash); // see if it's on the closed list

		if (closed.containsKey(Shash) && D.equals(s)) return false;  // if it is return false

		closed.put(Shash, s); // otherwise put it on
		return true; // and return true
	}

	public State search() {
		long startSearchTime = System.currentTimeMillis();

		if (start.goalReached()) { // wishful thinking
			return start;
		}

		needToVisit(start); // dummy call (adds start to the list of 'closed' states so we don't visit it again
		startSearchTime = System.currentTimeMillis();
		open.add(start); // add it to the open list
		bestHValue = start.getHValue(); // and take its heuristic value as the best so far
		bestHValueNoOpenAction = start.getHValue(); // best heuristic value so far for states with NO open action
		bestIntermediateState = start; // best intermediate state at start is start wtf??... :-)

		javaff.JavaFF.infoOutput.println(bestHValue);

		while (System.currentTimeMillis() - startSearchTime < this.searchIntervalMs && !open.isEmpty()) // whilst still states to consider
		{
			State s = removeNext(); // get the next one

			Set<State> successors = s.getNextStates(filter.getActions(s)); // and find its neighbourhood

			for (State succ : successors) {

				if (needToVisit(succ)) {
					TemporalMetricState tms = (TemporalMetricState) succ;
					if(tms.openActions.isEmpty() && tms.getHValue().compareTo(bestHValueNoOpenAction) < 0)//update bestHValueNoOpenAction
					{
						bestHValueNoOpenAction = tms.getHValue();
						bestIntermediateState = tms;
					}

					if (tms.goalReached()) { // if we've found a goal state - return it as the solution
						return tms;

					} else if (tms.getHValue().compareTo(bestHValue) < 0) {
						// if we've found a state with a better heuristic value than the best seen so far
						bestHValue = tms.getHValue(); // note the new best value
						javaff.JavaFF.infoOutput.println(bestHValue);
						open.clear(); // clear the open list
						open.add(tms); // put this on it
						break; // and skip looking at the other successors

					} else {
						open.add(tms); // otherwise, add to the open list
					}
				}
			}
		}

		if(open.isEmpty())// no reason to move forward: unsat with EHC
			return null;

		// pick the state having bestHValueNoOpenAction and return that plan
		return bestIntermediateState;
	}
}
