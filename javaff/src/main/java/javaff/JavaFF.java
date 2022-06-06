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

	// public static void main(String domain, String problem) {
		

		

	// 	// if (args.length < 2) {
	// 	// 	System.out.println("Parameters needed: domainFile.pddl problemFile.pddl [random seed] [outputfile.sol");

	// 	// } else {
	// 	// 	File domainFile = new File(args[0]);
	// 	// 	File problemFile = new File(args[1]);
	// 	// 	File solutionFile = null;
	// 	// 	if (args.length > 2)
	// 	// 	{
	// 	// 		generator = new Random(Integer.parseInt(args[2]));
	// 	// 	}

	// 	// 	if (args.length > 3)
	// 	// 	{
	// 	// 		solutionFile = new File(args[3]);
	// 	// 	}

	// 	// }

	// 	Plan plan = plan(domain,problem);

	// 	// if (solutionFile != null && plan != null) writePlanToFile(plan, solutionFile);
	// }

	public static void main(String[] args){
			String domain = ";; domain file: printing-domain.pddl\n" +
					"\n" +
					"(define (domain printing-domain)\n" +
					"\n" +
					"    (:requirements :strips :typing :fluents :durative-actions)\n" +
					"\n" +
					"    ;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;\n" +
					"    (:types\n" +
					"        room hallway_segment - area\n" +
					"        dock\n" +
					"        robot\n" +
					"        printer\n" +
					"    );; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;\n" +
					"\n" +
					"    (:predicates\n" +
					"        (near ?a1 ?a2 - area)\n" +
					"        (r_in ?r - robot ?a - area)\n" +
					"        (r_docked ?r - robot)\n" +
					"        (not_r_docked ?r - robot)\n" +
					"        (d_in ?d - dock ?r - room)\n" +
					"        (p_in ?p - printer ?h - hallway_segment)\n" +
					"        (free ?a - area)\n" +
					"        (inactive ?r - robot)\n" +
					"        (active ?r - robot)\n" +
					"        (available ?p - printer)\n" +
					"        (printed_docs_loaded ?r - robot)\n" +
					"        (printed_docs_left_in ?r - robot ?a - area)\n" +
					"        (fully_recharged ?r - robot)\n" +
					"    )\n" +
					"\n" +
					"    (:functions\n" +
					"        (battery_charge ?r - robot)\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action running\n" +
					"        :parameters (?r - robot)\n" +
					"        :duration (= ?duration 80)\n" +
					"        :condition (and \n" +
					"            (at start (inactive ?r))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at start (active ?r))\n" +
					"            (at start (not(inactive ?r)))\n" +
					"            (at end (not(active ?r)))\n" +
					"            (at end (inactive ?r))\n" +
					"        )\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action standby\n" +
					"        :parameters (?r - robot)\n" +
					"        :duration (= ?duration 80)\n" +
					"        :condition (and \n" +
					"            (at start (active ?r))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at start (inactive ?r))\n" +
					"            (at start (not(active ?r)))\n" +
					"            (at end (not(inactive ?r)))\n" +
					"            (at end (active ?r))\n" +
					"        )\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action move\n" +
					"        :parameters (?r - robot ?a1 ?a2 - area)\n" +
					"        :duration (= ?duration 4)\n" +
					"        :condition (and\n" +
					"            (at start (r_in ?r ?a1))\n" +
					"            (over all (active ?r))\n" +
					"            (over all (not_r_docked ?r))\n" +
					"            (over all (free ?a2))\n" +
					"            (over all (near ?a1 ?a2))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at start (not(r_in ?r ?a1)))\n" +
					"            (at end (r_in ?r ?a2))\n" +
					"        )\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action printing\n" +
					"        :parameters (?r - robot ?p - printer ?hs - hallway_segment)\n" +
					"        :duration (= ?duration 4)\n" +
					"        :condition (and\n" +
					"            (at start (available ?p))\n" +
					"            (over all (active ?r))\n" +
					"            (over all (not_r_docked ?r))\n" +
					"            (over all (r_in ?r ?hs))\n" +
					"            (over all (p_in ?p ?hs))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at start (not(available ?p)))\n" +
					"            (at end (printed_docs_loaded ?r))\n" +
					"        )\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action unload_printed_docs\n" +
					"        :parameters (?r - robot ?a - area)\n" +
					"        :duration (= ?duration 2)\n" +
					"        :condition (and\n" +
					"            (at start (printed_docs_loaded ?r))\n" +
					"            (over all (not_r_docked ?r))\n" +
					"            (over all (active ?r))\n" +
					"            (over all (r_in ?r ?a))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at end (not(printed_docs_loaded ?r)))\n" +
					"            (at end (printed_docs_left_in ?r ?a))\n" +
					"        )\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action recharge\n" +
					"        :parameters (?r - robot ?d - dock ?room - room)\n" +
					"        :duration (= ?duration 2)\n" +
					"        :condition (and\n" +
					"            (over all (d_in ?d ?room))\n" +
					"            (over all (r_docked ?r))\n" +
					"            (over all (inactive ?r))\n" +
					"            (over all (r_in ?r ?room))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at end (fully_recharged ?r))\n" +
					"        )\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action docking\n" +
					"        :parameters (?r - robot ?d - dock ?room - room)\n" +
					"        :duration (= ?duration 1)\n" +
					"        :condition (and\n" +
					"            (at start (not_r_docked ?r))\n" +
					"            (over all (active ?r))\n" +
					"            (over all (d_in ?d ?room))\n" +
					"            (over all (r_in ?r ?room))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at end (not(not_r_docked ?r)))\n" +
					"            (at end (r_docked ?r))\n" +
					"        )\n" +
					"    )\n" +
					"\n" +
					"    (:durative-action undocking\n" +
					"        :parameters (?r - robot ?d - dock ?room - room)\n" +
					"        :duration (= ?duration 1)\n" +
					"        :condition (and\n" +
					"            (at start (r_docked ?r))\n" +
					"            (over all (active ?r))\n" +
					"            (over all (d_in ?d ?room))\n" +
					"            (over all (r_in ?r ?room))\n" +
					"        )\n" +
					"        :effect (and\n" +
					"            (at end (not_r_docked ?r))\n" +
					"            (at end (not(r_docked ?r)))\n" +
					"        )\n" +
					"    )\n" +
					")\n";
			String problem = "( define ( problem printing1 )\n" +
					"( :domain printing-domain )\n" +
					"    ( :objects\n" +
					"        r1 r2 r3 - robot\n" +
					"        p1 p2 - printer\n" +
					"        r_a r_b r_c r_d r_e r_f - room\n" +
					"        h11 h12 h13 h21 h22 h23 h31 h32 h33 - hallway_segment\n" +
					"        h11_21 h13_23 h21_31 h23_33 - hallway_segment\n" +
					"        d0 d1 d2 d3 - dock\n" +
					"    )\n" +
					"    ( :init\n" +
					"        (near r_a h11)\n" +
					"\n" +
					"        (near h11 h12)\n" +
					"        (near h11 h11_21)\n" +
					"        (near h11 r_a)\n" +
					"\n" +
					"        (near h12 h11)\n" +
					"        (near h12 h13)\n" +
					"\n" +
					"        (near h13 h12)\n" +
					"        (near h13 r_d)\n" +
					"        (near h13 h13_23)\n" +
					"\n" +
					"        (near r_d h13)\n" +
					"\n" +
					"        (near r_b h11_21)\n" +
					"\n" +
					"        (near h11_21 h11)\n" +
					"        (near h11_21 h21)\n" +
					"        (near h11_21 r_b)\n" +
					"\n" +
					"        (near h13_23 h13)\n" +
					"        (near h13_23 h23)\n" +
					"        (near h13_23 r_e)\n" +
					"\n" +
					"        (near r_e h13_23)\n" +
					"\n" +
					"        (near h21 h11_21)\n" +
					"        (near h21 h22)\n" +
					"        (near h21 h21_31)\n" +
					"        \n" +
					"        (near h22 h21)\n" +
					"        (near h22 h23)\n" +
					"\n" +
					"        (near h23 h13_23)\n" +
					"        (near h23 h22)\n" +
					"        (near h23 h23_33)\n" +
					"\n" +
					"        (near r_c h21_31)\n" +
					"\n" +
					"        (near h21_31 r_c)\n" +
					"        (near h21_31 h21)\n" +
					"        (near h21_31 h31)\n" +
					"\n" +
					"        (near h23_33 h23)\n" +
					"        (near h23_33 h33)\n" +
					"        (near h23_33 r_e)\n" +
					"\n" +
					"        (near r_e h23_33)\n" +
					"\n" +
					"        (near h31 h21_31)\n" +
					"        (near h31 h32)\n" +
					"\n" +
					"        (near h32 h31)\n" +
					"        (near h32 h33)\n" +
					"\n" +
					"        (near h33 h23_33)\n" +
					"        (near h33 h32)\n" +
					"        (near h33 r_f)\n" +
					"\n" +
					"        (near r_f h33)\n" +
					"\n" +
					"        (free r_a)\n" +
					"        (free r_b)\n" +
					"        (free r_c)\n" +
					"        (free r_d)\n" +
					"        (free r_e)\n" +
					"        (free r_f)\n" +
					"\n" +
					"        (free h11)\n" +
					"        (free h12)\n" +
					"\n" +
					"        (free h11_21)\n" +
					"        (free h13_23)\n" +
					"\n" +
					"        (free h21)\n" +
					"        (free h23)\n" +
					"\n" +
					"        (free h21_31)\n" +
					"        (free h23_33)\n" +
					"\n" +
					"        (free h31)\n" +
					"        (free h32)\n" +
					"        (free h33)\n" +
					"\n" +
					"        (p_in p1 h12)\n" +
					"        (p_in p2 h32)\n" +
					"\n" +
					"        (available p1)\n" +
					"\n" +
					"        (d_in d0 r_c)\n" +
					"        (d_in d1 r_c)\n" +
					"        (d_in d2 r_c)\n" +
					"        (d_in d3 r_c)\n" +
					"\n" +
					"        (r_in r1 r_d)\n" +
					"        (inactive r1)\n" +
					"        (not_r_docked r1)\n" +
					"        \n" +
					"        (r_in r2 r_c)\n" +
					"        (inactive r2)\n" +
					"        (r_docked r2)\n" +
					"        \n" +
					"        (r_in r3 r_c)\n" +
					"        (inactive r3)\n" +
					"        (r_docked r3)\n" +
					"        \n" +
					"        (= (battery_charge r1) 30)\n" +
					"        (= (battery_charge r2) 100)\n" +
					"        (= (battery_charge r3) 100)\n" +
					"    )\n" +
					"    ( :goal\n" +
					"        ( and\n" +
					"            ;; (r_in r2 h21_31)\n" +
					"            (printed_docs_left_in r2 r_e)\n" +
					"        )\n" +
					"    )\n" +
					")";
			GroundProblem ground = Objects.requireNonNull(computeGroundProblem(domain, problem));
			TemporalMetricState initial = ground.getTemporalMetricInitialState();
			LinkedList<State> open = new LinkedList<>();
			Hashtable<Integer, State> closed = new Hashtable<>();

			int i = 0;
			TemporalMetricState goalOrIntermediateState = initial;
			do {
				System.out.println("\n\n ROUND " + (i++));
				goalOrIntermediateState = (TemporalMetricState) performFFSearch(goalOrIntermediateState, 400, open, closed);
				System.out.println(buildPlan(ground, goalOrIntermediateState));
				System.out.println("[main]: open.size="+open.size() + "\t closed.size=" + closed.size());
			}while(!goalOrIntermediateState.goalReached() || (open.isEmpty() && goalOrIntermediateState == null));
			/*
				System.out.println("\n\nSECONDO ROUND");
				TemporalMetricState goalOrIntermediateState2 = (TemporalMetricState) performFFSearch(goalOrIntermediateState, 600, open, closed);
				System.out.println(buildPlan(ground, goalOrIntermediateState2));

				System.out.println("\n\nTERZO ROUND");
				TemporalMetricState goalOrIntermediateState3 = (TemporalMetricState) performFFSearch(goalOrIntermediateState2, 600, open, closed);
				System.out.println(buildPlan(ground, goalOrIntermediateState3));
			*/
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
		for(DurativeAction da : ((Set<DurativeAction>) (goalState).openActions)) {
			goalState = ((TemporalMetricState) ((TemporalMetricState) goalState).apply(da.endAction));
		}

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
			// goalState = BFS.search();
		}

		return goalOrIntermediateState; // return the plan

	}
}