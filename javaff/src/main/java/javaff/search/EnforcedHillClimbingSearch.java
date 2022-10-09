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

import javaff.data.Action;
import javaff.data.metric.NamedFunction;
import javaff.data.strips.Proposition;
import javaff.data.temporal.DurativeAction;
import javaff.planning.STRIPSState;
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
	protected int maxPPlanSize = 32000;

	public EnforcedHillClimbingSearch(State s)
	{
		this(s, new HValueComparator());
	}

	public EnforcedHillClimbingSearch(State s, float searchIntervalMs, int maxPPlanSize)
	{
		this(s, new HValueComparator());
		this.searchIntervalMs = searchIntervalMs;
		this.maxPPlanSize = maxPPlanSize;
	}

	public EnforcedHillClimbingSearch(State s, float searchIntervalMs,int maxPPlanSize, TreeSet<State> open, Hashtable<Integer, State> closed)
	{
		this(s, new HValueComparator());
		this.searchIntervalMs = searchIntervalMs;
		this.maxPPlanSize = maxPPlanSize;
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
		State S = (State) open.first();
		boolean removed = open.remove(S);
		if(!removed)
		{
			System.out.println("I WAS NOT ABLE TO REMOVE the STATE in the open list");// SHOULD NOT HAPPEN ANYMORE AFTER BUG FIX IN REBASE
			/*
			for(State tms : (TreeSet<State>)open)
				if(((TemporalMetricState)tms).toString().equals(S.toString())) {
					System.out.println("TROVATO NELLA OPEN");
					System.out.println("riprovo remove " + open.remove(tms));
					BigDecimal d1 = S.getHValue();
					BigDecimal d2 = tms.getHValue();
					System.out.println("S.h = " + d1 + " tms.h = " + d2);
					int r = d1.compareTo(d2);
					System.out.println("A r="+r);
					if (r == 0)
					{
						d1 = S.getGValue();
						d2 = tms.getGValue();
						System.out.println("S.g = " + d1 + " tms.g = " + d2);
						r = d1.compareTo(d2);
						System.out.println("B r="+r);
						if (r == 0)
						{
							System.out.println("r="+r);
							if (tms.hashCode() > S.hashCode()) r = 1;
							else if (tms.hashCode() == S.hashCode() && tms.equals(S)) r=0;
							else r = -1;
						}
						System.out.println("C r="+r);
					}
					if(tms.equals(S))
						System.out.println("TROVATO NELLA OPEN equals non ROTTA");

				}

			for(State tms : (TreeSet<State>)open)
				if(((TemporalMetricState)tms).getUniqueId() == S.getUniqueId())
					System.out.println("TROVATO NELLA OPEN con UNIQUE ID");

			 */
		}
		//return open.pollFirst();
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

		int counter = 0;

		while (System.currentTimeMillis() - startSearchTime < this.searchIntervalMs && !open.isEmpty()) // whilst still states to consider
		{
			if(bestIntermediateState instanceof TemporalMetricState && ((TemporalMetricState) bestIntermediateState).getRealGValue() >= maxPPlanSize)
				break;
			else if(bestIntermediateState instanceof STRIPSState && ((STRIPSState) bestIntermediateState).getGValue().intValue() >= maxPPlanSize)
				break;
			//System.out.println("Still time to search");
			State s = removeNext(); // get the next one
			/*
			for(Proposition p : ((HashSet<Proposition>)((STRIPSState)s).facts))
				if(p.isDomainDefined() && p.getName().equals( "in"))
					System.out.println("\n Remove from open: "+p);
				else if (!p.isDomainDefined() && p.getName().equals("imove"))
					System.out.println("\n Remove from open: " + p);
			*/
			Set<Action> actions = filter.getActions(s);
			/*
			for(Action a : actions)
				System.out.println("\t- Selecting: " + a);
			 */
			Set<State> successors = s.getNextStates(actions); // and find its neighbourhood
			for (State succ : successors) {

				if (needToVisit(succ)) {
					/*
					for(Proposition p : ((HashSet<Proposition>)((STRIPSState)succ).facts))
						if(p.isDomainDefined() && p.getName().equals( "in"))
							System.out.println("\n Need to visit: "+p);
						else if (!p.isDomainDefined() && p.getName().equals("imove"))
							System.out.println("\n Need to visit: " + p);
					System.out.println("Current closed: "+closed.size());
					for(State ssClosed : ((Hashtable<Integer, State>)closed).values()) {
						for (Proposition p : ((HashSet<Proposition>) ((STRIPSState) ssClosed).facts))
							if (p.isDomainDefined() && p.getName().equals("in"))
								System.out.print("\t " + p);
							else if (!p.isDomainDefined() && p.getName().equals("imove"))
								System.out.print("\t " + p);
						System.out.println();
					}
					*/
					counter++;
					TemporalMetricState tms = (TemporalMetricState) succ;
					if(tms.openActions.isEmpty() && tms.getHValue().compareTo(bestHValueNoOpenAction) < 0)//update bestHValueNoOpenAction
					{
						bestHValueNoOpenAction = tms.getHValue();
						bestIntermediateState = tms;
						System.out.println("Found better tms: plan.size = " + tms.getSolution().getActions().size() + ", h = " + bestHValueNoOpenAction);
					}

					if (tms.goalReached()) { // if we've found a goal state - return it as the solution
						return tms;

					} else if (tms.getHValue().compareTo(bestHValue) < 0) {
						// if we've found a state with a better heuristic value than the best seen so far
						bestHValue = tms.getHValue(); // note the new best value
						//javaff.JavaFF.infoOutput.println(bestHValue);
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
