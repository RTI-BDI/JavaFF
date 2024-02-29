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

package javaff.data.metric;

import javaff.data.Parameter;
import javaff.data.strips.PDDLObject;
import javaff.data.strips.PredicateSymbol;
import javaff.data.strips.Variable;

import java.util.List;

public class FunctionSymbol extends PredicateSymbol implements Cloneable {
	public FunctionSymbol(String name, boolean domainDefined)
    {
		super(name, domainDefined);
	}

	public Object clone(){
		FunctionSymbol fs = new FunctionSymbol(name, domainDefined);
		fs.staticValue = staticValue;
		for(Parameter p : (List<Parameter>)params) {
			if (p instanceof Variable)
				fs.params.add((Parameter) ((Variable) p).clone());
			else if (p instanceof PDDLObject)
				fs.params.add((Parameter) ((PDDLObject) p).clone());
		}
		return fs;
	}
}
