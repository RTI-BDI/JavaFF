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

package javaff.data.temporal;

import javaff.data.Parameter;
import javaff.data.metric.BinaryComparator;
import javaff.data.metric.ResourceOperator;
import javaff.data.strips.*;
import javaff.planning.TemporalMetricState;

import java.io.Serializable;
import java.math.BigDecimal;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.Iterator;

public class EndInstantAction extends SplitInstantAction implements Serializable
{
	public Object cloneBase(DurativeAction da){
		EndInstantAction eia = new EndInstantAction();
		// from base class Action
		eia.name = (OperatorName) name.clone();
		eia.instanceCounter = instanceCounter;
        eia.cost = cost.add(BigDecimal.ZERO);
		for(Parameter p : (List<Parameter>)params){
			if(p instanceof Variable)
				eia.params.add(((Variable) p).clone());
			else if(p instanceof PDDLObject)
				eia.params.add(((PDDLObject) p).clone());
		}

		// from base class InstantAction
		if(condition instanceof BinaryComparator)
			eia.condition = (BinaryComparator) ((BinaryComparator) condition).clone();
		else if(condition instanceof AND)
			eia.condition = (AND) ((AND) condition).clone();
		else if(condition instanceof Proposition)
			eia.condition = (Proposition) ((Proposition) condition).clone();
		else if(condition instanceof TrueCondition)
			eia.condition = TrueCondition.getInstance();

		if(effect instanceof ResourceOperator)
			eia.effect = (ResourceOperator) ((ResourceOperator) effect).clone();
		else if(effect instanceof AND)
			eia.effect = (AND) ((AND) effect).clone();
		else if(effect instanceof NOT)
			eia.effect = (NOT) ((NOT) effect).clone();
		else if(effect instanceof Proposition)
			eia.effect = (Proposition) ((Proposition) effect).clone();
		else if(effect instanceof NullEffect)
			eia.effect = NullEffect.getInstance();

		// from SplitInstantAction
		eia.predictedInstant =  predictedInstant != null? predictedInstant.add(BigDecimal.ZERO) : null;

		if(da == null) {
			eia.parent = new DurativeAction();

			eia.parent.name = (OperatorName) parent.name.clone();
			for(Parameter p : (List<Parameter>)parent.params){
				if(p instanceof Variable)
					eia.parent.params.add(((Variable) p).clone());
				else if(p instanceof PDDLObject)
					eia.parent.params.add(((PDDLObject) p).clone());
			}

			eia.parent.duration.ungroundDurativeAction = parent.duration.ungroundDurativeAction;
			if (parent.durationConstraint instanceof SimpleDurationConstraint)
				eia.parent.durationConstraint = (SimpleDurationConstraint) ((SimpleDurationConstraint) parent.durationConstraint).clone();
			eia.parent.durationConstraint.constraints = new HashSet();
			eia.parent.durationConstraint.constraints.addAll(parent.durationConstraint.constraints);

			if (parent.startCondition instanceof BinaryComparator)
				eia.parent.startCondition = (BinaryComparator) ((BinaryComparator) parent.startCondition).clone();
			else if (parent.startCondition instanceof AND)
				eia.parent.startCondition = (AND) ((AND) parent.startCondition).clone();
			else if (parent.startCondition instanceof Proposition)
				eia.parent.startCondition = (Proposition) ((Proposition) parent.startCondition).clone();
			else if (parent.startCondition instanceof TrueCondition)
				eia.parent.startCondition = TrueCondition.getInstance();

			if (parent.endCondition instanceof BinaryComparator)
				eia.parent.endCondition = (BinaryComparator) ((BinaryComparator) parent.endCondition).clone();
			else if (parent.endCondition instanceof AND)
				eia.parent.endCondition = (AND) ((AND) parent.endCondition).clone();
			else if (parent.endCondition instanceof Proposition)
				eia.parent.endCondition = (Proposition) ((Proposition) parent.endCondition).clone();
			else if (parent.endCondition instanceof TrueCondition)
				eia.parent.endCondition = TrueCondition.getInstance();

			if (parent.invariant instanceof BinaryComparator)
				eia.parent.invariant = (BinaryComparator) ((BinaryComparator) parent.invariant).clone();
			else if (parent.invariant instanceof AND)
				eia.parent.invariant = (AND) ((AND) parent.invariant).clone();
			else if (parent.invariant instanceof Proposition)
				eia.parent.invariant = (Proposition) ((Proposition) parent.invariant).clone();
			else if (parent.invariant instanceof TrueCondition)
				eia.parent.invariant = TrueCondition.getInstance();

			if (parent.startEffect instanceof ResourceOperator)
				eia.parent.startEffect = (ResourceOperator) ((ResourceOperator) parent.startEffect).clone();
			else if (parent.startEffect instanceof AND)
				eia.parent.startEffect = (AND) ((AND) parent.startEffect).clone();
			else if (parent.startEffect instanceof NOT)
				eia.parent.startEffect = (NOT) ((NOT) parent.startEffect).clone();
			else if (parent.startEffect instanceof Proposition)
				eia.parent.startEffect = (Proposition) ((Proposition) parent.startEffect).clone();
			else if (parent.startEffect instanceof NullEffect)
				eia.parent.startEffect = NullEffect.getInstance();

			if (parent.endEffect instanceof ResourceOperator)
				eia.parent.endEffect = (ResourceOperator) ((ResourceOperator) parent.endEffect).clone();
			else if (parent.endEffect instanceof AND)
				eia.parent.endEffect = (AND) ((AND) parent.endEffect).clone();
			else if (parent.endEffect instanceof NOT)
				eia.parent.endEffect = (NOT) ((NOT) parent.endEffect).clone();
			else if (parent.endEffect instanceof Proposition)
				eia.parent.endEffect = (Proposition) ((Proposition) parent.endEffect).clone();
			else if (parent.endEffect instanceof NullEffect)
				eia.parent.endEffect = NullEffect.getInstance();

			eia.parent.dummyJoin = (Proposition) parent.dummyJoin.clone();
			eia.parent.dummyGoal = (Proposition) parent.dummyGoal.clone();
			eia.parent.startAction = null;

		}else{
			eia.parent = da;
		}

		eia.parent.endAction = eia;

		return eia;
	}

	public Object clone(DurativeAction parent){
		return cloneBase(parent);
	}
	public Object clone(){
		EndInstantAction eia = (EndInstantAction) cloneBase(null);
		eia.parent.startAction = (StartInstantAction) ((StartInstantAction)getSibling()).clone(eia.parent);
		return eia;
	}
	
	public SplitInstantAction getSibling()
    {
		return parent.startAction;
	}

	public void applySplit(TemporalMetricState ts)
    {
		Set is = parent.invariant.getConditionalPropositions();
		
		Iterator iit = is.iterator();
		while (iit.hasNext())
		{
			ts.invariants.remove(iit.next());
		}
		ts.openActions.remove(parent);
		ts.actions.remove(this);
		ts.actions.add(getSibling());
	}

	public boolean exclusivelyInvariant(Proposition p)
    {
		return !parent.endCondition.getConditionalPropositions().contains(p) || !parent.endEffect.getAddPropositions().contains(p) || !parent.endEffect.getDeletePropositions().contains(p);
	}
}
