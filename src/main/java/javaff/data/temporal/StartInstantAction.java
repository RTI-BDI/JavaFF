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

public class StartInstantAction extends SplitInstantAction implements Serializable
{
	public Object cloneBase(DurativeAction da){
		StartInstantAction sia = new StartInstantAction();
		// from base class Action
		sia.name = (OperatorName) name.clone();
		sia.instanceCounter = instanceCounter;
        sia.cost = cost.add(BigDecimal.ZERO);
		for(Parameter p : (List<Parameter>)params){
			if(p instanceof Variable)
				sia.params.add(((Variable) p).clone());
			else if(p instanceof PDDLObject)
				sia.params.add(((PDDLObject) p).clone());
		}

		// from base class InstantAction
		if(condition instanceof BinaryComparator)
			sia.condition = (BinaryComparator) ((BinaryComparator) condition).clone();
		else if(condition instanceof AND)
			sia.condition = (AND) ((AND) condition).clone();
		else if(condition instanceof Proposition)
			sia.condition = (Proposition) ((Proposition) condition).clone();
		else if(condition instanceof TrueCondition)
			sia.condition = TrueCondition.getInstance();

		if(effect instanceof ResourceOperator)
			sia.effect = (ResourceOperator) ((ResourceOperator) effect).clone();
		else if(effect instanceof AND)
			sia.effect = (AND) ((AND) effect).clone();
		else if(effect instanceof NOT)
			sia.effect = (NOT) ((NOT) effect).clone();
		else if(effect instanceof Proposition)
			sia.effect = (Proposition) ((Proposition) effect).clone();
		else if(effect instanceof NullEffect)
			sia.effect = NullEffect.getInstance();

		// from SplitInstantAction
		sia.predictedInstant = predictedInstant != null? predictedInstant.add(BigDecimal.ZERO) : null;

		if(da == null) {
			sia.parent = new DurativeAction();

			sia.parent.name = (OperatorName) parent.name.clone();
			for(Parameter p : (List<Parameter>)parent.params){
				if(p instanceof Variable)
					sia.parent.params.add(((Variable) p).clone());
				else if(p instanceof PDDLObject)
					sia.parent.params.add(((PDDLObject) p).clone());
			}

			sia.parent.duration.ungroundDurativeAction = parent.duration.ungroundDurativeAction;
			if (parent.durationConstraint instanceof SimpleDurationConstraint)
				sia.parent.durationConstraint = (SimpleDurationConstraint) ((SimpleDurationConstraint) parent.durationConstraint).clone();
			sia.parent.durationConstraint.constraints = new HashSet();
			sia.parent.durationConstraint.constraints.addAll(parent.durationConstraint.constraints);

			if (parent.startCondition instanceof BinaryComparator)
				sia.parent.startCondition = (BinaryComparator) ((BinaryComparator) parent.startCondition).clone();
			else if (parent.startCondition instanceof AND)
				sia.parent.startCondition = (AND) ((AND) parent.startCondition).clone();
			else if (parent.startCondition instanceof Proposition)
				sia.parent.startCondition = (Proposition) ((Proposition) parent.startCondition).clone();
			else if (parent.startCondition instanceof TrueCondition)
				sia.parent.startCondition = TrueCondition.getInstance();

			if (parent.endCondition instanceof BinaryComparator)
				sia.parent.endCondition = (BinaryComparator) ((BinaryComparator) parent.endCondition).clone();
			else if (parent.endCondition instanceof AND)
				sia.parent.endCondition = (AND) ((AND) parent.endCondition).clone();
			else if (parent.endCondition instanceof Proposition)
				sia.parent.endCondition = (Proposition) ((Proposition) parent.endCondition).clone();
			else if (parent.endCondition instanceof TrueCondition)
				sia.parent.endCondition = TrueCondition.getInstance();

			if (parent.invariant instanceof BinaryComparator)
				sia.parent.invariant = (BinaryComparator) ((BinaryComparator) parent.invariant).clone();
			else if (parent.invariant instanceof AND)
				sia.parent.invariant = (AND) ((AND) parent.invariant).clone();
			else if (parent.invariant instanceof Proposition)
				sia.parent.invariant = (Proposition) ((Proposition) parent.invariant).clone();
			else if (parent.invariant instanceof TrueCondition)
				sia.parent.invariant = TrueCondition.getInstance();

			if (parent.startEffect instanceof ResourceOperator)
				sia.parent.startEffect = (ResourceOperator) ((ResourceOperator) parent.startEffect).clone();
			else if (parent.startEffect instanceof AND)
				sia.parent.startEffect = (AND) ((AND) parent.startEffect).clone();
			else if (parent.startEffect instanceof NOT)
				sia.parent.startEffect = (NOT) ((NOT) parent.startEffect).clone();
			else if (parent.startEffect instanceof Proposition)
				sia.parent.startEffect = (Proposition) ((Proposition) parent.startEffect).clone();
			else if (parent.startEffect instanceof NullEffect)
				sia.parent.startEffect = NullEffect.getInstance();

			if (parent.endEffect instanceof ResourceOperator)
				sia.parent.endEffect = (ResourceOperator) ((ResourceOperator) parent.endEffect).clone();
			else if (parent.endEffect instanceof AND)
				sia.parent.endEffect = (AND) ((AND) parent.endEffect).clone();
			else if (parent.endEffect instanceof NOT)
				sia.parent.endEffect = (NOT) ((NOT) parent.endEffect).clone();
			else if (parent.endEffect instanceof Proposition)
				sia.parent.endEffect = (Proposition) ((Proposition) parent.endEffect).clone();
			else if (parent.endEffect instanceof NullEffect)
				sia.parent.endEffect = NullEffect.getInstance();

			sia.parent.dummyJoin = (Proposition) parent.dummyJoin.clone();
			sia.parent.dummyGoal = (Proposition) parent.dummyGoal.clone();
			sia.parent.endAction = null;

		}else{
			sia.parent = da;
		}

		sia.parent.startAction = sia;

		return sia;
	}

	public Object clone(DurativeAction parent){
		return cloneBase(parent);
	}
	public Object clone(){
		StartInstantAction sia = (StartInstantAction) cloneBase(null);
		sia.parent.endAction = (EndInstantAction) ((EndInstantAction)getSibling()).clone(sia.parent);
		return sia;
	}

	public SplitInstantAction getSibling()
	{
		return parent.endAction;
	}

	public void applySplit(TemporalMetricState ts)
    {
		ts.invariants.addAll(parent.invariant.getConditionalPropositions());
		ts.openActions.add(parent);
		ts.actions.remove(this);
		ts.actions.add(getSibling());
	}

	public boolean exclusivelyInvariant(Proposition p)
    {
		return !parent.startCondition.getConditionalPropositions().contains(p) || !parent.startEffect.getAddPropositions().contains(p) || !parent.startEffect.getDeletePropositions().contains(p);
    }
}
