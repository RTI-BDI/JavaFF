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

import javaff.data.Parameter;
import javaff.data.metric.BinaryComparator;
import javaff.data.metric.ResourceOperator;
import javaff.data.temporal.StartInstantAction;

import java.math.BigDecimal;
import java.util.List;

public class STRIPSInstantAction extends InstantAction implements Cloneable
{
    public Object clone(){
        STRIPSInstantAction sia = new STRIPSInstantAction();

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
        
        return sia;
    }
}
