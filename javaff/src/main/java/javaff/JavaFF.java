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

import javaff.data.*;
import javaff.data.strips.Proposition;
import javaff.data.temporal.DurativeAction;
import javaff.data.temporal.EndInstantAction;
import javaff.data.temporal.SplitInstantAction;
import javaff.data.temporal.StartInstantAction;
import javaff.parser.PDDL21parser;
import javaff.planning.State;
import javaff.planning.TemporalMetricState;
import javaff.planning.RelaxedTemporalMetricPlanningGraph;
import javaff.planning.HelpfulFilter;
import javaff.planning.NullFilter;
import javaff.scheduling.Scheduler;
import javaff.scheduling.JavaFFScheduler;
import javaff.search.HValueComparator;
import javaff.search.Search;
import javaff.search.BestFirstSearch;
import javaff.search.EnforcedHillClimbingSearch;


import java.io.*;
import java.math.BigDecimal;
import java.sql.Time;
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


	public static void main(String[] args) throws IOException, ClassNotFoundException {

		 stdBody(650);
		// testBuildPlan();
		// testHvsL("/home/devis/lp5.txt","/home/devis/hp5.txt");
		/*
		for(int i=0; i<512; i++) {
			generator = new Random();
			stdbody(generator.nextInt(500)+250);
		}*/
	}

	private static void testHvsL(String solFile, String tpSolFile) throws IOException, ClassNotFoundException {

		FileInputStream fis0	= new FileInputStream(solFile);
		ObjectInputStream ois0 = new ObjectInputStream(fis0);
		TotalOrderPlan sol0 = (TotalOrderPlan) ois0.readObject(); // down-casting object
		Set<Action> sol = sol0.getActions();

		FileInputStream fis1	= new FileInputStream(solFile);
		ObjectInputStream ois1 = new ObjectInputStream(fis1);
		TotalOrderPlan tpSol1 = (TotalOrderPlan) ois1.readObject(); // down-casting object
		List<Action> tpSol = tpSol1.getOrderedActions();

		System.out.println("\nPLAN SNAP ACTION found ( from getSolution().getActions() ): " + sol.size());
		for(Action sia : ((HashSet<Action>)sol))
			System.out.println(sia.toString());

		System.out.println("\nPLAN SNAP ACTION found ( getTPSolution().getOrderedActions() ): " + tpSol.size());
		for(Action sia : ((List<Action>)tpSol))
			System.out.println(sia.toString());

		System.out.println("\nFill empty hashset");
		HashSet<Action> hashSet = new HashSet<>();
		for(Action a : tpSol)
			if(hashSet.contains(a))
				System.out.println("Contengo già " + a);
			else
			{
				hashSet.add(a);
				System.out.println("Aggiunto " + a);
			}

		fis0.close();fis1.close();
		ois0.close();ois1.close();
	}

	public static void testBuildPlan() throws IOException, ClassNotFoundException {
		String domain = "";
		boolean errorDomain = false;

		try {
			File myObj = new File("/home/devis/ros2_ws/install/ros2_bdi_tests/share/ros2_bdi_tests/pddl/gripper/gripper-domain_no_fluent_in_german.pddl");
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

		boolean errorProblem = false;
		String problem = "( define ( problem problem_1 )\n" +
				" ( :domain gripper-domain )\n" +
				" ( :objects\n" +
				" \tbox_a1 box_a2 box_b1 box_b2 box_c1 box_c2 - box\n" +
				" \tstart base_1 base_2 base_3 base_a base_b base_c - stackbase\n" +
				" \tdeposit_a deposit_b deposit_c - deposit\n" +
				" \tgripper_a - gripper\n" +
				" \tcarrier_a carrier_b carrier_c - carrier\n" +
				" )\n" +
				" ( :init\n" +
				" \t( upon gripper_a start )\n" +
				" \t( arm_free gripper_a )\n" +
				" \t( on box_a1 base_1 base_1 )\n" +
				" \t( on box_c2 box_a1 base_1 )\n" +
				" \t( on box_a2 box_c2 base_1 )\n" +
				" \t( in box_a1 base_1 )\n" +
				" \t( in box_c2 base_1 )\n" +
				" \t( in box_a2 base_1 )\n" +
				" \t( in base_1 base_1 )\n" +
				" \t( clear box_a2 )\n" +
				" \t( on box_b1 base_2 base_2 )\n" +
				" \t( on box_b2 box_b1 base_2 )\n" +
				" \t( on box_c1 box_b2 base_2 )\n" +
				" \t( in box_b1 base_2 )\n" +
				" \t( in box_b2 base_2 )\n" +
				" \t( in box_c1 base_2 )\n" +
				" \t( in base_2 base_2 )\n" +
				" \t( clear box_c1 )\n" +
				" \t( clear base_3 )\n" +
				" \t( clear base_c )\n" +
				" \t( clear base_b )\n" +
				" \t( clear base_a )\n" +
				" \t( in base_a base_a )\n" +
				" \t( in base_b base_b )\n" +
				" \t( in base_c base_c )\n" +
				" \t( in base_3 base_3 )\n" +
				" \t( carrier_in_deposit carrier_a deposit_a )\n" +
				" \t( carrier_in_deposit carrier_b deposit_b )\n" +
				" \t( carrier_in_deposit carrier_c deposit_c )\n" +
				" \t( carrier_can_come carrier_a base_a )\n" +
				" \t( carrier_can_come carrier_b base_b )\n" +
				" \t( carrier_can_come carrier_c base_c )\n" +
				" \t( carrier_can_go carrier_a deposit_a )\n" +
				" \t( carrier_can_go carrier_b deposit_b )\n" +
				" \t( carrier_can_go carrier_c deposit_c )\n" +
				" )\n" +
				" ( :goal \n" +
				"    (and (box_stored box_a2 deposit_a) (box_stored box_c2 deposit_c) (box_stored box_c1 deposit_c)) \n" +
				" )\n" +
				")\n";



		if(!errorDomain && !errorProblem){
			GroundProblem groundProblem = JavaFF.computeGroundProblem(domain, problem);
			if(groundProblem != null) {
				TemporalMetricState initialState = groundProblem.getTemporalMetricInitialState();
				System.out.println("\n\nInit State: " + initialState.toString());
				TemporalMetricState currState = (TemporalMetricState) initialState.clone();

				FileInputStream fis0	= new FileInputStream("/home/devis/hp0.txt");
				ObjectInputStream ois0 = new ObjectInputStream(fis0);
				TotalOrderPlan top0 = (TotalOrderPlan) ois0.readObject(); // down-casting object
				fis0.close(); ois0.close();

				/*
				FileInputStream fis2	= new FileInputStream("/home/devis/hp2.txt");
				ObjectInputStream ois2 = new ObjectInputStream(fis2);
				TotalOrderPlan top2 = (TotalOrderPlan) ois2.readObject(); // down-casting object
				fis2.close(); ois2.close();
				*/

				FileInputStream fis3	= new FileInputStream("/home/devis/hp3.txt");
				ObjectInputStream ois3 = new ObjectInputStream(fis3);
				TotalOrderPlan top3 = (TotalOrderPlan) ois3.readObject(); // down-casting object
				fis3.close(); ois3.close();

				FileInputStream fis4	= new FileInputStream("/home/devis/hp4.txt");
				ObjectInputStream ois4 = new ObjectInputStream(fis4);
				TotalOrderPlan top4 = (TotalOrderPlan) ois4.readObject(); // down-casting object
				fis4.close(); ois4.close();

				FileInputStream fis5	= new FileInputStream("/home/devis/hp5.txt");
				ObjectInputStream ois5 = new ObjectInputStream(fis5);
				TotalOrderPlan top5 = (TotalOrderPlan) ois5.readObject(); // down-casting object
				fis5.close(); ois5.close();

				for(Action a : top0.getOrderedActions())
					if(a instanceof StartInstantAction)
						currState = (TemporalMetricState) currState.apply(a);
				/*
				for(Action a : top2.getOrderedActions())
					if(a instanceof EndInstantAction)
						currState = (TemporalMetricState) currState.apply(a);
				 */
				for(Action a : top3.getOrderedActions())
					if(a instanceof EndInstantAction)
						currState = (TemporalMetricState) currState.apply(a);
				for(Action a : top4.getOrderedActions())
					if(a instanceof EndInstantAction)
						currState = (TemporalMetricState) currState.apply(a);

				//check state after sim p0 == 7c1e7dc
				System.out.println("\n\nAfter sim P0-P2-P3-P4, State: " + currState.toString());

				//sim rebase
				groundProblem.initial = currState.facts;
				groundProblem.state = currState;
				groundProblem.functionValues = currState.funcValues;
				currState.cleanPlanInfo();

				//sim p5
				for(Action a : top5.getOrderedActions())
					currState = (TemporalMetricState) currState.apply(a);

				//check state after p1 == a606d511
				System.out.println("\n\nAfter sim P5 State: " + currState.toString());


				//call buildPlan
				TimeStampedPlan tsp = JavaFF.buildPlan(groundProblem, currState);
				System.out.println(tsp.getPrintablePlan(false));
			}
		}
	}

	public static void stdBody(int searchInterval) throws IOException {
			String domain = "";
			boolean errorDomain = false;

			try {
				File myObj = new File("/home/devis/ros2_ws/install/ros2_bdi_tests/share/ros2_bdi_tests/pddl/gripper/gripper-domain.pddl");
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


			String problem = "( define ( problem problem_1 )\n" +
					" ( :domain gripper-domain )\n" +
					" ( :objects\n" +
					" \tbox_a1 box_a2 box_b1 box_b2 box_c1 box_c2 - box\n" +
					" \tstart base_1 base_2 base_3 base_a base_b base_c - stackbase\n" +
					" \tdeposit_a deposit_b deposit_c - deposit\n" +
					" \tgripper_a - gripper\n" +
					" \tcarrier_a carrier_b carrier_c - carrier\n" +
					" )\n" +
					" ( :init\n" +
					" \t( upon gripper_a start )\n" +
					" \t( on box_a1 base_1 base_1 )\n" +
					" \t( on box_c2 box_a1 base_1 )\n" +
					" \t( on box_a2 box_c2 base_1 )\n" +
					" \t( in box_a1 base_1 )\n" +
					" \t( in box_c2 base_1 )\n" +
					" \t( in box_a2 base_1 )\n" +
					" \t( in base_1 base_1 )\n" +
					" \t( clear box_a2 )\n" +
					" \t( on box_b1 base_2 base_2 )\n" +
					" \t( on box_b2 box_b1 base_2 )\n" +
					" \t( on box_c1 box_b2 base_2 )\n" +
					" \t( in box_b1 base_2 )\n" +
					" \t( in box_b2 base_2 )\n" +
					" \t( in box_c1 base_2 )\n" +
					" \t( in base_2 base_2 )\n" +
					" \t( clear box_c1 )\n" +
					" \t( clear base_3 )\n" +
					" \t( clear base_c )\n" +
					" \t( clear base_b )\n" +
					" \t( clear base_a )\n" +
					" \t( in base_a base_a )\n" +
					" \t( in base_b base_b )\n" +
					" \t( in base_c base_c )\n" +
					" \t( in base_3 base_3 )\n" +
					" \t( carrier_in_deposit carrier_a deposit_a )\n" +
					" \t( carrier_in_deposit carrier_b deposit_b )\n" +
					" \t( carrier_in_deposit carrier_c deposit_c )\n" +
					" \t( carrier_can_come carrier_a base_a )\n" +
					" \t( carrier_can_come carrier_b base_b )\n" +
					" \t( carrier_can_come carrier_c base_c )\n" +
					" \t( carrier_can_go carrier_a deposit_a )\n" +
					" \t( carrier_can_go carrier_b deposit_b )\n" +
					" \t( carrier_can_go carrier_c deposit_c )\n" +
					" \t( = ( holding_boxes gripper_a ) 0.0000000000 )\n" +
					" \t( = ( stacked start ) 0.0000000000 )\n" +
					" \t( = ( stacked base_1 ) 3.0000000000 )\n" +
					" \t( = ( stacked base_1 ) 3.0000000000 )\n" +
					" \t( = ( stacked base_2 ) 3.0000000000 )\n" +
					" \t( = ( stacked base_3 ) 0.0000000000 )\n" +
					" \t( = ( stacked base_a ) 0.0000000000 )\n" +
					" \t( = ( stacked base_b ) 0.0000000000 )\n" +
					" \t( = ( stacked base_c ) 0.0000000000 )\n" +
					" \t( = ( moving_boxes carrier_a ) 0.0000000000 )\n" +
					" \t( = ( moving_boxes carrier_b ) 0.0000000000 )\n" +
					" \t( = ( moving_boxes carrier_c ) 0.0000000000 )\n" +
					" )\n" +
					" ( :goal \n" +
					"    (and (carrier_moving carrier_a box_a2) (carrier_moving carrier_c box_c2)) \n" +
					" )\n" +
					")\n";

			/*
			String problem = "( define ( problem problem_1 )\n" +
					" ( :domain printing-domain )\n" +
					" ( :objects\n" +
					" \tr_a r_b r_c r_d r_e r_f - room\n" +
					" \th11 h12 h13 h21 h22 h23 h31 h32 h33 h11_21 h13_23 h21_31 h23_33 - hallway_segment\n" +
					" \td0 d1 d2 d3 - dock\n" +
					" \tr2 - robot\n" +
					" \tp1 p2 - printer\n" +
					" )\n" +
					" ( :init\n" +
					"\t(active r2)\n" +
					"\t(near h31 h32)\n" +
					"\t(free r_a)\n" +
					"\t(free r_e)\n" +
					"\t(near r_e h13_23)\n" +
					"\t(free h11)\n" +
					"\t(available p1)\n" +
					"\t(near h31 h21_31)\n" +
					"\t(r_docked r2)\n" +
					"\t(near h11_21 h21)\n" +
					"\t(free h13_23)\n" +
					"\t(near r_f h33)\n" +
					"\t(free r_d)\n" +
					"\t(free h21)\n" +
					"\t(near r_d h13)\n" +
					"\t(near h21_31 h31)\n" +
					"\t(near h11_21 h11)\n" +
					"\t(near h23_33 h23)\n" +
					"\t(d_in d0 r_c)\n" +
					"\t(r_in r2 r_c)\n" +
					"\t(near h11 r_a)\n" +
					"\t(near r_e h23_33)\n" +
					"\t(near h21 h21_31)\n" +
					"\t(near h22 h21)\n" +
					"\t(near h21_31 h21)\n" +
					"\t(near h13 h13_23)\n" +
					"\t(near r_c h21_31)\n" +
					"\t(near h33 h32)\n" +
					"\t(free r_c)\n" +
					"\t(d_in d2 r_c)\n" +
					"\t(near h33 h23_33)\n" +
					"\t(p_in p2 h32)\n" +
					"\t(free h31)\n" +
					"\t(free h32)\n" +
					"\t(near h13 h12)\n" +
					"\t(near h23_33 r_e)\n" +
					"\t(near h11_21 r_b)\n" +
					"\t(free h23_33)\n" +
					"\t(free h12)\n" +
					"\t(near h23 h23_33)\n" +
					"\t(near h23_33 h33)\n" +
					"\t(free h33)\n" +
					"\t(near h13 r_d)\n" +
					"\t(near r_b h11_21)\n" +
					"\t(near h12 h13)\n" +
					"\t(near h33 r_f)\n" +
					"\t(near h32 h31)\n" +
					"\t(near h13_23 h13)\n" +
					"\t(d_in d3 r_c)\n" +
					"\t(near h12 h11)\n" +
					"\t(near h23 h22)\n" +
					"\t(near h13_23 h23)\n" +
					"\t(free r_b)\n" +
					"\t(free h21_31)\n" +
					"\t(p_in p1 h12)\n" +
					"\t(near r_a h11)\n" +
					"\t(near h23 h13_23)\n" +
					"\t(free r_f)\n" +
					"\t(near h21_31 r_c)\n" +
					"\t(free h23)\n" +
					"\t(d_in d1 r_c)\n" +
					"\t(near h32 h33)\n" +
					"\t(near h21 h22)\n" +
					"\t(near h11 h11_21)\n" +
					"\t(near h13_23 r_e)\n" +
					"\t(free h11_21)\n" +
					"\t(near h11 h12)\n" +
					"\t(near h21 h11_21)\n" +
					"\t(near h22 h23)\n"+
					"\t(= (battery_charge r2) 85)\n"+
					" )\n" +
					" ( :goal\n" +
					" \t( and\n" +
					" \t\t( printed_docs_left_in r2 r_e )\n" +
					" \t)\n" +
					" )\n" +
					"(:metric maximize (+ (battery_charge r2) 0))\n" +
					" )";
			*/
			/*
			String problem = "(define (problem problem_1)\n" +
					"\t(:domain cleaner-domain)\n" +
					"\t(:objects\n" +
					"\t\tcleaner - robot\n" +
					"\t\tpluto - void\n" +
					"\t\tdock - waypoint\n" +
					"\t\tkitchen - waypoint\n" +
					"\t\tbedroom - waypoint\n" +
					"\t\tbathroom - waypoint)\n" +
					"\t(:init\n" +
					"\t\t(recharging_station dock)\n" +
					"\t\t(workfree cleaner)\n" +
					"\t\t(in cleaner dock)\n" +
					"\t\t(pred_a pluto)\n" +
					"\t\t(= (battery_charge cleaner) 90)\n" +
					"\t\t)\n" +
					"\t(:goal \n" +
					"\t\t(and\n" +
					"\t\t\t(cleaned dock)\n" +
					"\t\t\t(cleaned kitchen)\n" +
					"\t\t\t(in cleaner dock)\n" +
					"\t\t\t(cleaned bedroom)\n" +
					"\t\t\t(cleaned bathroom)\n" +
					"\t\t)\n" +
					"\t)\n"+
					")";
			*/
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
					//System.out.println("\n[1]Ground problem info:");
					//System.out.println("facts: " + groundProblem.initial.size());
					//for(Proposition f : (HashSet<Proposition>)groundProblem.initial)
					//	System.out.println("\t"+f.getName() + ", params = " + f.getStringParameters().toString());
					TemporalMetricState currentState = JavaFF.computeInitialState(groundProblem);
					int unsat = 0;
					TreeSet<State> open = new TreeSet<>(new HValueComparator());
					Hashtable<Integer, State> closed = new Hashtable<>();
					int i = 0;
					while(unsat < 2 && !currentState.goalReached()){

						System.out.println("\n\n ROUND " + (i++));


						// move forward with the search for 500ms
						System.out.println("[BEFORE SEARCH state="+currentState.toString()+"]: open.size=" + open.size() + "\t closed.size=" + closed.size());

						TemporalMetricState goalOrIntermediateState = unsat==0?
								(TemporalMetricState) JavaFF.performEHCSearch(currentState, searchInterval, open, closed)
								:
								(TemporalMetricState) JavaFF.performBFSSearch(currentState, searchInterval, open, closed);
						System.out.println("[AFTER SEARCH state="+goalOrIntermediateState.toString()+"]: open.size=" + open.size() + "\t closed.size=" + closed.size());

						//check whether unsat ~ empty open and search has return null
						if(open.isEmpty() && goalOrIntermediateState == null)
						{
							if(unsat == 0)//switch to BFS
								closed.remove(new Integer(currentState.hashCode()));//current was already explored in last EHC search
							unsat++;
						}
						boolean advancementMade = goalOrIntermediateState != null && goalOrIntermediateState.getSolution().getActions().size() > 0;
						if(unsat < 2 && goalOrIntermediateState != null && advancementMade) {
							currentState = goalOrIntermediateState;
							// build plan string from currentState
							TimeStampedPlan tsp = JavaFF.buildPlan(groundProblem, currentState);
							TreeSet<SplitInstantAction> orderedSplitInstantActions = (TreeSet<SplitInstantAction>)tsp.getSortedSplitInstantActions();
							Set operators = orderedSplitInstantActions.iterator().next().effect.getOperators();
							Set addPropositions = orderedSplitInstantActions.iterator().next().effect.getAddPropositions();
							Set delPropositions = orderedSplitInstantActions.iterator().next().effect.getDeletePropositions();
							if (tsp != null) System.out.println(tsp.getPrintablePlan(false));


							JavaFF.rebaseOnCurrentState(groundProblem, currentState, open, closed);
							System.out.println("[AFTER REBASE]: open.size=" + open.size() + "\t closed.size=" + closed.size());
						}

					}

					if(unsat >= 2)
						System.out.println("Problem unsat: I apologise if I cannot satisfy your demands, Master Devis");
				}
			}
	}

	public static void rebaseOnCurrentState(GroundProblem groundProblem, TemporalMetricState currentState, TreeSet<State> open, Hashtable<Integer, State> closed)
	{
		// rebase ground problem on current state
		groundProblem.initial = currentState.facts;
		groundProblem.state = currentState;
		groundProblem.functionValues = currentState.funcValues;

		// remove states in open list with different prefix and rebase plan of the ones with same prefix
		List<Action> committedOrderedActions = ((TotalOrderPlan)currentState.getSolution()).getOrderedActions();
		LinkedList<State> diffPrefixOpenStates = new LinkedList<>();
		int openPrefixNotMatching = 0;
		for(State openState : open)
		{
			TemporalMetricState openStateTMS = (TemporalMetricState) openState;
			List<Action> oSCommittedOrderedActions = ((TotalOrderPlan) openStateTMS.getSolution()).getOrderedActions();
			int j;
			boolean matching = true;
			for(j=0; j<oSCommittedOrderedActions.size() && j<committedOrderedActions.size(); j++)
				if(!committedOrderedActions.get(j).equals(oSCommittedOrderedActions.get(j)))
				{
					matching = false;
					break;
				}

			if(!matching || j != committedOrderedActions.size()) {
				openPrefixNotMatching++;
				diffPrefixOpenStates.add(openState);//not matching, remove state from open

			}else if(matching){//not matching, remove state from open
				openStateTMS.rebasePlan(committedOrderedActions);
			}
		}
		open.removeAll(diffPrefixOpenStates);


		// closed clean up at each search interval
		closed.clear();
		//System.out.println("openPrefixNotMatching=" + openPrefixNotMatching);
		//System.out.println("closedPrefixNotMatching=" + closedPrefixNotMatching);


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

	public static TimeStampedPlan buildPlan(GroundProblem ground, TemporalMetricState goalState)
	{
		String plan = "";

		TotalOrderPlan top = null;
		if (goalState != null) top = (TotalOrderPlan) goalState.getSolution();
		// ********************************
		// Schedule a plan
		// ********************************

		TimeStampedPlan tsp = null;

		if (goalState != null)
		{
			//infoOutput.println(plan);
		 	//infoOutput.println("Scheduling");

		 	Scheduler scheduler = new JavaFFScheduler(ground);
		 	tsp = scheduler.schedule(top);
		}

		// if (top != null) plan = top.getPrintablePlan();
		// 	System.out.println("SNAP ACTIONS plan with " + top.getPlanLength() + ": \n" + plan);
		//if (tsp != null) plan = tsp.getPrintablePlan();

		return tsp;
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

	public static State performEHCSearch(
			TemporalMetricState initialState,
			float searchIntervalMs,
			TreeSet<State> open,
			Hashtable<Integer, State> closed) {

		// Implementation of standard FF-style search
		//infoOutput.println("\n\nPerforming FF search - EHC with only helpful actions");

		// Now, initialise an EHC searcher
		EnforcedHillClimbingSearch EHCS = new EnforcedHillClimbingSearch(initialState, searchIntervalMs, open, closed);

		EHCS.setFilter(HelpfulFilter.getInstance()); // and use the helpful actions neighbourhood

		// Try and find a plan using EHC
		State goalOrIntermediateState = EHCS.search();

		return goalOrIntermediateState; // return the plan

	}

	public static State performBFSSearch(
			TemporalMetricState initialState,
			float searchIntervalMs,
			TreeSet<State> open,
			Hashtable<Integer, State> closed) {

		// Implementation of standard FF-style search
		//infoOutput.println("\n\nPerforming FF search - BFS with all applicable actions");

		// create a Best-First Searcher
		BestFirstSearch BFS = new BestFirstSearch(initialState, searchIntervalMs, open, closed);

		// ... change to using the 'all actions' neighbourhood (a null filter, as it removes nothing)
		BFS.setFilter(NullFilter.getInstance());

		// and use that
		State goalOrIntermediateState = BFS.search();

		return goalOrIntermediateState; // return the plan

	}
}