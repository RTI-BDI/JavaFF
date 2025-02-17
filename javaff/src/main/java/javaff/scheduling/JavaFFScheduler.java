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

package javaff.scheduling;

import javaff.data.*;
import javaff.data.metric.BinaryComparator;
import javaff.data.metric.ResourceOperator;
import javaff.data.metric.NumberFunction;
import javaff.data.metric.NamedFunction;
import javaff.data.metric.MetricSymbolStore;
import javaff.data.metric.TotalTimeFunction;
import javaff.data.strips.OperatorName;
import javaff.data.temporal.SplitInstantAction;
import javaff.data.temporal.StartInstantAction;
import javaff.planning.TemporalMetricState;

import java.util.*;
import java.math.BigDecimal;

public class JavaFFScheduler implements Scheduler
{
	protected GroundProblem problem;
	
	public JavaFFScheduler(GroundProblem p)
    {
		problem = p;
	}

	public TimeStampedPlan schedule(TotalOrderPlan top)
	{
		// ATTENTION PLEASE: really important to mark down exhaustively diff actions with same signature within the same plan,
		// otherwise rescheduling won't work!!! Using an instance counter to distinguish and match them appropriately

		HashMap<String, Integer> counters = new HashMap<>();
		for(Action a : top.getOrderedActions())
		{
			int counter = 1;
			if(a instanceof SplitInstantAction)
			{
				SplitInstantAction sia = (SplitInstantAction) a;
				if(!counters.containsKey(sia.toStringBase()))
					counters.put(sia.toStringBase(), counter);
				else{
					counter = ((Integer)counters.get(sia.toStringBase()))+1;
					counters.put(sia.toStringBase(), counter);
				}
				
				sia.instanceCounter = counter; // WITHOUT THIS YOU'RE FU***D
				sia.parent.startAction.instanceCounter = counter; // WITHOUT THIS YOU'RE FU***D
				sia.parent.endAction.instanceCounter = counter; // WITHOUT THIS YOU'RE FU***D
			}
		}

		PartialOrderPlan pop = GreedyPartialOrderLifter.lift(top, problem);

		MatrixSTN stn = new MatrixSTN(top);

		stn.addConstraints(pop.getTemporalConstraints());


		//Sort out the Durations
		Map states = new Hashtable(); //Maps (Actions => states (which the actions are applied in))
		Iterator ait = top.getOrderedActions().iterator();
		TemporalMetricState state = problem.getTemporalMetricInitialState();
		while (ait.hasNext())
		{
			Action a = (Action) ait.next();
			if (a instanceof StartInstantAction)
			{
				StartInstantAction sia = (StartInstantAction) a;
				List l = TemporalConstraint.getBounds(sia, sia.getSibling(), sia.parent.getMaxDuration(state), sia.parent.getMinDuration(state));
				stn.addConstraints(new HashSet(l));
			}
			states.put(a, state);
			state = (TemporalMetricState) state.apply(a);
		}

		
		
		boolean consistent = stn.consistent();

		// sort out the resources
		Map graphs = new Hashtable(); //Maps (NamedResources => PrecedenceGraphs)
		ait = top.getOrderedActions().iterator();
		while (ait.hasNext())
		{
			Action a = (Action) ait.next();
			
			Iterator bcit = a.getComparators().iterator();
			while (bcit.hasNext())
			{
				//WARNING WARNING WARNING - assumes comparators are of the form (NamedFunction </>/<=/>= StaticFunction)
                           BinaryComparator bc = (BinaryComparator) bcit.next(); 
				NamedFunction res = (NamedFunction) bc.first;
				PrecedenceResourceGraph prg = (PrecedenceResourceGraph) graphs.get(res);
				if (prg == null)
				{
					prg = new PrecedenceResourceGraph(stn);
					graphs.put(res,prg);
				}
				state = (TemporalMetricState) states.get(a);
				BigDecimal d = bc.second.getValue(state);
				prg.addCondition(new BinaryComparator( bc.type, res, new NumberFunction(d)), a);
			}

			Iterator roit = a.getOperators().iterator();
			while (roit.hasNext())
			{
				ResourceOperator ro = (ResourceOperator) roit.next();
				NamedFunction res = (NamedFunction) ro.resource;
				PrecedenceResourceGraph prg = (PrecedenceResourceGraph) graphs.get(res);
				if (prg == null)
				{
					prg = new PrecedenceResourceGraph(stn);
					graphs.put(res,prg);
				}
				prg.addOperator(new ResourceOperator( ro.type, res, ro.change.makeOnlyDurationDependent(state)), a);
			}

		}


		Iterator git = graphs.keySet().iterator();

		while (git.hasNext())
		{
			NamedFunction nf = (NamedFunction) git.next();
			PrecedenceResourceGraph prg = (PrecedenceResourceGraph) graphs.get(nf);
			prg.addOperator(new ResourceOperator(MetricSymbolStore.INCREASE, nf, new NumberFunction(nf.getValue(problem.getTemporalMetricInitialState()))), stn.START);
			boolean changesMade = true;
			while (changesMade)
			{
				changesMade = prg.meetConditions();
				stn.constrain();
			}
			changesMade = true;
			while (changesMade)
			{
				changesMade = prg.limitBounds();
				stn.constrain();
			}
			
		}

		
		Metric m = problem.metric;
		if (m != null && m.func instanceof NamedFunction && !(m.func instanceof TotalTimeFunction))
		{
			PrecedenceResourceGraph prg = (PrecedenceResourceGraph) graphs.get((NamedFunction) m.func);
			if(prg != null) {
				if (m.type == Metric.MAXIMIZE) prg.maximize();
				else if (m.type == Metric.MINIMIZE) prg.minimize();
			}
		}

		stn.constrain();

		stn.minimizeTime();
		stn.minimizeDuration();
		stn.constrain();
		
		TimeStampedPlan p = stn.getTimes();

		// (s1)... carrier_c in loading_c
		//  (s1) move1 (s2) -> pickup -> move2 -> putdown1
		// (s2)
		// (s2) = (s2_reale); s3 = s2.apply(pickup)
		// (s2)
		//  (s1) move1 (s2) -> (s2) pickup (s3) -> move2 -> putdown1
		// (s3)

		return p;
	}


}
