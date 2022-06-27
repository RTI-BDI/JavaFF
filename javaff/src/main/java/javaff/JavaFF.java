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

package javaff;

import javaff.data.PDDLPrinter;
import javaff.data.UngroundProblem;
import javaff.data.GroundProblem;
import javaff.data.Plan;
import javaff.data.TotalOrderPlan;
import javaff.data.TimeStampedPlan;
import javaff.data.temporal.DurativeAction;
import javaff.parser.PDDL21parser;
import javaff.planning.State;
import javaff.planning.TemporalMetricState;
import javaff.planning.RelaxedTemporalMetricPlanningGraph;
import javaff.planning.HelpfulFilter;
import javaff.planning.NullFilter;
import javaff.scheduling.Scheduler;
import javaff.scheduling.JavaFFScheduler;
import javaff.search.Search;
import javaff.search.BestFirstSearch;
import javaff.search.EnforcedHillClimbingSearch;


import java.io.PrintStream;
import java.io.PrintWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.math.BigDecimal;
import java.util.*;

public class JavaFF
{
	public static BigDecimal EPSILON = new BigDecimal(0.01);
	public static BigDecimal MAX_DURATION = new BigDecimal("100000"); //maximum duration in a duration constraint
	public static boolean VALIDATE = false;
	public static Random generator = null;

	public static PrintStream planOutput = System.out;
	public static PrintStream parsingOutput = System.out;
	public static PrintStream infoOutput = System.out;
	public static PrintStream errorOutput = System.err;

	public static void main(String[] args){
			String domain = "";
			boolean errorDomain = false;

			try {
			     File myObj = new File(/*"/home/devis/Documents/pddl/printing/domain.pddl"*/"/home/devis/ros2_ws/install/ros2_bdi_tests/share/ros2_bdi_tests/pddl/printing-floor/printing-domain.pddl");
			     Scanner myReader = new Scanner(myObj);
			     while (myReader.hasNextLine()) {
			         domain += myReader.nextLine() + "\n";
			     }
			     myReader.close();
			     System.out.println("\n\nDOMAIN:\n" + domain);

			} catch (FileNotFoundException e) {
			     errorDomain = true;
			     System.out.println("An error occurred while reading domain file.");
			     e.printStackTrace();
			}

			String problem = " ( define ( problem problem_1 )\n" +
					" ( :domain printing-domain )\n" +
					" ( :objects\n" +
					" \tr_a r_b r_c r_d r_e r_f - room\n" +
					" \th11 h12 h13 h21 h22 h23 h31 h32 h33 h11_21 h13_23 h21_31 h23_33 - hallway_segment\n" +
					" \td0 d1 d2 d3 - dock\n" +
					//" \tr1 r3 - robot\n" +
					" \tr2 - robot\n" +
					" \tp1 p2 - printer\n" +
					" )\n" +
					" ( :init\n" +
					" \t( near r_a h11 )\n" +
					" \t( near h11 h12 )\n" +
					" \t( near h11 h11_21 )\n" +
					" \t( near h11 r_a )\n" +
					" \t( near h12 h11 )\n" +
					" \t( near h12 h13 )\n" +
					" \t( near h13 h12 )\n" +
					" \t( near h13 r_d )\n" +
					" \t( near h13 h13_23 )\n" +
					" \t( near r_d h13 )\n" +
					" \t( near r_b h11_21 )\n" +
					" \t( near h11_21 h11 )\n" +
					" \t( near h11_21 h21 )\n" +
					" \t( near h11_21 r_b )\n" +
					" \t( near h13_23 h13 )\n" +
					" \t( near h13_23 h23 )\n" +
					" \t( near h13_23 r_e )\n" +
					" \t( near r_e h13_23 )\n" +
					" \t( near h21 h11_21 )\n" +
					" \t( near h21 h22 )\n" +
					" \t( near h21 h21_31 )\n" +
					" \t( near h22 h21 )\n" +
					" \t( near h22 h23 )\n" +
					" \t( near h23 h13_23 )\n" +
					" \t( near h23 h22 )\n" +
					" \t( near h23 h23_33 )\n" +
					" \t( near r_c h21_31 )\n" +
					" \t( near h21_31 r_c )\n" +
					" \t( near h21_31 h21 )\n" +
					" \t( near h21_31 h31 )\n" +
					" \t( near h23_33 h23 )\n" +
					" \t( near h23_33 h33 )\n" +
					" \t( near h23_33 r_e )\n" +
					" \t( near r_e h23_33 )\n" +
					" \t( near h31 h21_31 )\n" +
					" \t( near h31 h32 )\n" +
					" \t( near h32 h31 )\n" +
					" \t( near h32 h33 )\n" +
					" \t( near h33 h23_33 )\n" +
					" \t( near h33 h32 )\n" +
					" \t( near h33 r_f )\n" +
					" \t( near r_f h33 )\n" +
					" \t( free r_a )\n" +
					" \t( free r_b )\n" +
					" \t( free r_c )\n" +
					" \t( free r_d )\n" +
					" \t( free r_e )\n" +
					" \t( free r_f )\n" +
					" \t( free h11 )\n" +
					" \t( free h12 )\n" +
					" \t( free h11_21 )\n" +
					" \t( free h13_23 )\n" +
					" \t( free h21 )\n" +
					" \t( free h23 )\n" +
					" \t( free h21_31 )\n" +
					" \t( free h23_33 )\n" +
					" \t( free h31 )\n" +
					" \t( free h32 )\n" +
					" \t( free h33 )\n" +
					" \t( p_in p1 h12 )\n" +
					" \t( p_in p2 h32 )\n" +
					" \t( available p1 )\n" +
					" \t( d_in d0 r_c )\n" +
					" \t( d_in d1 r_c )\n" +
					" \t( d_in d2 r_c )\n" +
					" \t( d_in d3 r_c )\n" +
					//" \t( r_in r1 r_d )\n" +
					//" \t( active r1 )\n" +
					//" \t( not_r_docked r1 )\n" +
					" \t( r_in r2 r_c )\n" +
					" \t( active r2 )\n" +
					" \t( r_docked r2 )\n" +
					//" \t( r_in r3 r_c )\n" +
					//" \t( active r3 )\n" +
					//" \t( r_docked r3 )\n" +
					" )\n" +
					" ( :goal \n" +
											"( and " +
												"(printed_docs_left_in r2 r_e) " +
											") " +
										") " +
					")";
			boolean errorProblem = false;

			//try {
			//	File myObj = new File("/home/devis/Documents/pddl/printing/problem.pddl");
			//	Scanner myReader = new Scanner(myObj);
			//	while (myReader.hasNextLine()) {
			//		problem += myReader.nextLine() + "\n";
			//	}
			//	myReader.close();
			//	System.out.println("\n\nPROBLEM:\n" + problem);

			//} catch (FileNotFoundException e) {
			//	errorProblem = true;
			//	System.out.println("An error occurred while reading problem file.");
			//	e.printStackTrace();
			//}

			if(!errorDomain && !errorProblem){
				GroundProblem groundProblem = JavaFF.computeGroundProblem(domain, problem);
				if(groundProblem != null)
				{
					TemporalMetricState currentState = JavaFF.computeInitialState(groundProblem);
					boolean unsat = false;
					LinkedList<State> open = new LinkedList<>();
					Hashtable<Integer, State> closed = new Hashtable<>();
					int i = 0;
					while(!currentState.goalReached() && !unsat){

						System.out.println("\n\n ROUND " + (i++));

						// move forward with the search for 500ms
						currentState = (TemporalMetricState) JavaFF.performFFSearch(currentState, 250, open, closed);
						//check whether unsat ~ empty open and search has return null
						unsat = open.isEmpty() && currentState == null;
						if(!unsat) {
							// build plan string from currentState
							String planString = JavaFF.buildPlan(groundProblem, currentState);
							System.out.println(planString);
							System.out.println("open.size=" + open.size() + "\t closed.size=" + closed.size());
							JavaFF.rebaseOnCurrentState(groundProblem, currentState);
							open = new LinkedList<>();
							closed = new Hashtable<>();
						}
					}
				}
			}
	}

	public static void rebaseOnCurrentState(GroundProblem groundProblem, TemporalMetricState currentState)
	{
		// rebase ground problem on current state
		groundProblem.initial = currentState.facts;
		groundProblem.state = currentState;
		groundProblem.functionValues = currentState.funcValues;

		// clean plan info
		currentState.cleanPlanInfo();
	}

	public static GroundProblem computeGroundProblem(String domain, String problem)
	{
		EPSILON = EPSILON.setScale(2,BigDecimal.ROUND_HALF_EVEN);
		MAX_DURATION = MAX_DURATION.setScale(2,BigDecimal.ROUND_HALF_EVEN);
		generator = new Random();

		TemporalMetricState initial = null;

		// ********************************
		// Parse and Ground the Problem
		// ********************************
		// long startTime = System.currentTimeMillis();

		UngroundProblem unground = PDDL21parser.parseDomainAndProblem(domain, problem);

		if (unground == null)
		{
			System.out.println("Parsing error - see console for details");
			return null;
		}


		//PDDLPrinter.printDomainFile(unground, System.out);
		//PDDLPrinter.printProblemFile(unground, System.out);

		GroundProblem ground = unground.ground();

		// long afterGrounding = System.currentTimeMillis();

		// ********************************
		// Search for a plan
		// ********************************
		
		return ground;
	}

	public static TemporalMetricState computeInitialState(GroundProblem ground){
		return ground.getTemporalMetricInitialState();
	}

	public static TemporalMetricState applyOpenActions (TemporalMetricState intermediateState)
	{
		if((intermediateState).openActions.isEmpty())//you've found a proper sequence to apply them all
			return intermediateState;

		for(DurativeAction da : ((Set<DurativeAction>) (intermediateState).openActions)) {
			if(da.isApplicable(intermediateState))
				applyOpenActions((TemporalMetricState) ((TemporalMetricState) intermediateState).apply(da.endAction));
		}

		return null; //no proper sequence to close out all end snap-actions
	}

	public static String buildPlan(GroundProblem ground, TemporalMetricState goalState)
	{
		String plan = "";

		//TODO (1) ASK MR if it's okay to have it here and not modify the "original" intermediate state
		//TODO (2) Apply Open Actions in the proper way, i.e. check why function above does not work!
		//flag for the user to decide, in any case MOVE IT FROM HERE!!!
		//for(DurativeAction da : ((Set<DurativeAction>) (goalState).openActions)) {
		//	goalState = ((TemporalMetricState) ((TemporalMetricState) goalState).apply(da.endAction));
		//}

		TotalOrderPlan top = null;
		if (goalState != null) top = (TotalOrderPlan) goalState.getSolution();
		if (top != null) plan = top.getPrintablePlan();

		// ********************************
		// Schedule a plan
		// ********************************

		TimeStampedPlan tsp = null;

		if (goalState != null)
		{

		 	infoOutput.println("Scheduling");

		 	Scheduler scheduler = new JavaFFScheduler(ground);
		 	tsp = scheduler.schedule(top);
		}

		if (tsp != null) plan = tsp.getPrintablePlan();

		return plan;
	}

	private static void writePlanToFile(Plan plan, File fileOut)
	{
		try
		{
			FileOutputStream outputStream = new FileOutputStream(fileOut);
			PrintWriter printWriter = new PrintWriter(outputStream);
			plan.print(printWriter);
			printWriter.close();
		}
		catch (IOException e)
		{
			errorOutput.println(e);
			e.printStackTrace();
		}

	}

	public static State performFFSearch(TemporalMetricState initialState, float searchIntervalMs, LinkedList<State> open, Hashtable<Integer, State> closed) {


		// Implementation of standard FF-style search
		infoOutput.println("Performing search as in FF - first considering EHC with only helpful actions");

		// Now, initialise an EHC searcher
		EnforcedHillClimbingSearch EHCS = new EnforcedHillClimbingSearch(initialState, searchIntervalMs, open, closed);

		EHCS.setFilter(HelpfulFilter.getInstance()); // and use the helpful actions neighbourhood

		// Try and find a plan using EHC
		State goalOrIntermediateState = EHCS.search();

		if (goalOrIntermediateState == null) // if we can't find one
		{
			infoOutput.println("EHC failed, using best-first search, with all actions");

			// create a Best-First Searcher
			// BestFirstSearch BFS = new BestFirstSearch(initialState);

			// ... change to using the 'all actions' neighbourhood (a null filter, as it removes nothing)

			// BFS.setFilter(NullFilter.getInstance());

			// and use that
			// goalOrIntermediateState = BFS.search();
		}

		return goalOrIntermediateState; // return the plan

	}
}