package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.Set;
import java.util.HashSet;
import java.math.BigDecimal;

import javaff.JavaFF;
import javaff.data.Action;
import javaff.data.GroundProblem;
import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;
import javaff.data.temporal.SplitInstantAction;
import javaff.data.temporal.DurativeAction;

import javaff.planning.TemporalMetricState;

import it.unitn.rti.bdi.javaff.TimeStampedPlanWithSearchBaseline;
import it.unitn.rti.bdi.javaff.SharedSearchData;
import it.unitn.rti.bdi.javaff.SearchDataUtils;


enum FFSearchStatus{EHC_SEARCHING, EHC_FAILED, BFS_SEARCHING, UNSAT};

public class SearchThread extends Thread{
  
  private static final float DEADLINE_EPSILON = 1.0f;
  private boolean killMySelf = false;
  private boolean debug = false;
  private boolean lookForImprovedSolutions = false;
  private SharedSearchData sharedSearchData;
  private Publisher<javaff_interfaces.msg.SearchResult> planPublisher;
  ArrayList<ros2_bdi_interfaces.msg.Belief> previousCommittedTrueBeliefsStart;

  public SearchThread(SharedSearchData sharedSearchData, Publisher<javaff_interfaces.msg.SearchResult> planPublisher, boolean debug){
    super();
    this.sharedSearchData = sharedSearchData;
    this.planPublisher = planPublisher;
    this.debug = debug;
  }

  public SearchThread(SharedSearchData sharedSearchData, Publisher<javaff_interfaces.msg.SearchResult> planPublisher, boolean debug, boolean lookForImprovedSolutions){
    super();
    this.sharedSearchData = sharedSearchData;
    this.planPublisher = planPublisher;
    this.debug = debug;
    this.lookForImprovedSolutions = lookForImprovedSolutions;
  }

  public void killMySelf(){killMySelf = true;}
  
  public void run(){
    this.sharedSearchData.open.clear();

    String prefix = lookForImprovedSolutions? "[searchThreadBFS]" : "[searchThread]";

    short searchRound = 0;
    short startPlanIndex = (short) this.sharedSearchData.searchResultMsg.getPlans().size();

    FFSearchStatus ffstatus = lookForImprovedSolutions? FFSearchStatus.BFS_SEARCHING : FFSearchStatus.EHC_SEARCHING;//start directly with a less greedy approach if looking for improved solutions

    this.sharedSearchData.searchResultMsg.setSearchBaseline(SearchDataUtils.getSearchBaseline(this.sharedSearchData.executingTspWSB));
    this.sharedSearchData.searchResultMsg.setBasePlanIndex((short)this.sharedSearchData.tspQueue.size());

    // we're starting a new search from scratch: we assume agent is not executing now
    // therefore its current state of execution is equivalent to the initial state of search
    // it's going to be updated as soon as the search starts
    this.sharedSearchData.execNextCommittedState = (TemporalMetricState) this.sharedSearchData.searchCurrentState.clone();
    this.sharedSearchData.execNextCommittedState.currInstant = BigDecimal.ZERO;
    
    // use in round > i, where i is the first round producing a plan to compute round precondition
    previousCommittedTrueBeliefsStart = null;

    int consecutiveEmptySearchRounds = 0;

    while(ffstatus != FFSearchStatus.UNSAT && !this.sharedSearchData.searchCurrentState.goalReached()){

      this.sharedSearchData.searchLock.lock();

        // check if search result are already outdated, i.e. searchBaseline updated (no p_index == -1), but old wrt. current exec status
        javaff_interfaces.msg.CommittedStatus currSearchBaseline = this.sharedSearchData.searchResultMsg.getSearchBaseline();
        if(currSearchBaseline.getExecutingPlanIndex() >= 0 && this.sharedSearchData.executingTspWSB!=null && this.sharedSearchData.executingTspWSB.outdatedSearchBaseline(currSearchBaseline))
        {
          System.out.println("\n"+prefix+" ROUND " + (searchRound) + " outdated search baseline: search will stop");  
          killMySelf = true;
        }

        if(!killMySelf)
        {
          System.out.println("\n"+prefix+" ROUND " + (searchRound) + " start");
       
          // retrieve preconditions from currentState that are going to apply for found plan
          ArrayList<ros2_bdi_interfaces.msg.Belief> committedTrueBeliefsStart = SearchDataUtils.getTrueBeliefs(this.sharedSearchData.searchCurrentState);
          ArrayList<ros2_bdi_interfaces.msg.Belief> filteredTrueBeliefs = SearchDataUtils.filterStatePrecondition(committedTrueBeliefsStart, previousCommittedTrueBeliefsStart);
          // String tBellog = ("ROUND " + searchRound + " committedTrueBeliefsStart:");
          // for(ros2_bdi_interfaces.msg.Belief b : committedTrueBeliefsStart)
          //   tBellog += ("\t- " + b.getName() + " " + b.getParams().stream().collect(Collectors.joining(" ")) + (b.getPddlType()==b.FUNCTION_TYPE? "\tval=" + b.getValue() : ""));
          // System.out.println("ROUND " + searchRound + " filtered true beliefs:");
          // for(ros2_bdi_interfaces.msg.Belief b : filteredTrueBeliefs)
          //   System.out.println("\t- " + b.getName() + " " + b.getParams().stream().collect(Collectors.joining(" "))  + (b.getPddlType()==b.FUNCTION_TYPE? "\tval=" + b.getValue() : ""));
    
          // move forward with the search for interval search time
          TemporalMetricState goalOrIntermediateState = (ffstatus == FFSearchStatus.EHC_SEARCHING)?
            (TemporalMetricState) JavaFF.performEHCSearch(this.sharedSearchData.searchCurrentState, this.sharedSearchData.searchParams.intervalSearchMS, this.sharedSearchData.open, this.sharedSearchData.closed)
            :
            (TemporalMetricState) JavaFF.performBFSSearch(this.sharedSearchData.searchCurrentState, this.sharedSearchData.searchParams.intervalSearchMS, this.sharedSearchData.open, this.sharedSearchData.closed);

          //check whether unsat ~ empty open and search has return null
          if(ffstatus == FFSearchStatus.EHC_SEARCHING)
          {
            ffstatus = (this.sharedSearchData.open.isEmpty() && goalOrIntermediateState == null)? FFSearchStatus.BFS_SEARCHING : FFSearchStatus.EHC_SEARCHING;
            
            if(ffstatus == FFSearchStatus.BFS_SEARCHING)//just switched to BFS searching
            {   
              // here switch to nextCommittedState to restart search from it and NOT from currentState, otherwise might be too late to find a viable solution
              
              // retrieve last exec committed action and put it as a search baseline, so that scheduler knows
              this.sharedSearchData.searchResultMsg.setSearchBaseline(SearchDataUtils.getSearchBaseline(this.sharedSearchData.executingTspWSB));
              
              // clear tspqueue index additional plans after currently committed one that are not valid anymore...
              while(this.sharedSearchData.searchResultMsg.getSearchBaseline().getExecutingPlanIndex() < (this.sharedSearchData.tspQueue.size() - 1)) 
                this.sharedSearchData.tspQueue.remove(this.sharedSearchData.tspQueue.size() - 1);

              // put next exec committedState as base for the next search iteration
              this.sharedSearchData.searchCurrentState = this.sharedSearchData.execNextCommittedState;

              // rebase on new search base state for next bfs search iterations
              JavaFF.rebaseOnCurrentState(this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState, this.sharedSearchData.open, this.sharedSearchData.closed);
            }

          }else if(ffstatus == FFSearchStatus.BFS_SEARCHING)
            ffstatus = (this.sharedSearchData.open.isEmpty() && goalOrIntermediateState == null)? FFSearchStatus.UNSAT : FFSearchStatus.BFS_SEARCHING;

          if(ffstatus != FFSearchStatus.UNSAT){
            if(goalOrIntermediateState != null && !goalOrIntermediateState.getSolution().getActions().isEmpty()){
              
              consecutiveEmptySearchRounds = 0;

              //System.out.println(tBellog);
              // update current state
              this.sharedSearchData.searchCurrentState = goalOrIntermediateState;

              // note: do not pass search round, as some round can return no result!!!
              short planIndex = (short) this.sharedSearchData.searchResultMsg.getPlans().size();

              // build plan string from currentState
              TimeStampedPlan tsp = JavaFF.buildPlan(planIndex, this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState);
              String planString = "";
              if (tsp != null && tsp.getSortedActions().size()>0) 
              {
                TimeStampedPlanWithSearchBaseline tspWSB = new TimeStampedPlanWithSearchBaseline(tsp, this.sharedSearchData.searchResultMsg.getSearchBaseline(), startPlanIndex);
                this.sharedSearchData.tspQueue.add(tspWSB);
                planString = tsp.getPrintablePlan(false);//get plan string

                plansys2_msgs.msg.Plan currentPlanMsg = SearchDataUtils.buildPsys2Plan(tsp);// build plan msg and publish it          
                
                // plan precondition gen.
                previousCommittedTrueBeliefsStart = committedTrueBeliefsStart;// update all precondition of the current plan (facts true in starting state)
                ArrayList<ros2_bdi_interfaces.msg.Belief> preconditionBeliefs = new ArrayList<ros2_bdi_interfaces.msg.Belief>();
                preconditionBeliefs.addAll(filteredTrueBeliefs);
                preconditionBeliefs.addAll(SearchDataUtils.computeImplicitPreconditions(tsp));
                ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions = SearchDataUtils.buildPreconditions(preconditionBeliefs); // TODO: filter might have some issues AND add first actions preconditions
                
                javaff_interfaces.msg.PartialPlan newPPlan = SearchDataUtils.buildNewPPlan(planIndex, this.sharedSearchData.fulfillingDesire,
                  planPreconditions, this.sharedSearchData.searchCurrentState, currentPlanMsg);
                
                //Search result containing all pplans up to now within this search
                this.sharedSearchData.searchResultMsg.getPlans().add(newPPlan);
                this.sharedSearchData.searchResultMsg.setStatus(this.sharedSearchData.searchCurrentState.goalReached()? this.sharedSearchData.searchResultMsg.SUCCESS : this.sharedSearchData.searchResultMsg.SEARCHING);
                
                if(!killMySelf && !lookForImprovedSolutions)//these search results are still valid and they need to be published immediately (if I'm looking for an improved solution, I should wait till the end)
                  this.planPublisher.publish(this.sharedSearchData.searchResultMsg);
                                
                //Log round result
                if(debug){  
                  System.out.println("\n"+prefix+" ROUND " + (searchRound) + " has produced plan " + tspWSB.planIndex + " (sID: " + tspWSB.searchID + ")");
                  System.out.println(planString);
                  System.out.println(prefix+" open.size="+this.sharedSearchData.open.size() + "\t closed.size=" + this.sharedSearchData.closed.size());
                }
                
                // rebase exclusively when plan presents some actions, otherwise next search cycle will start from where it left
                JavaFF.rebaseOnCurrentState(this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState, this.sharedSearchData.open, this.sharedSearchData.closed);
              }
            } else {
              consecutiveEmptySearchRounds++;//empty search round counter
            }
          }
        }
        
      this.sharedSearchData.searchLock.unlock();
      
      if(consecutiveEmptySearchRounds == this.sharedSearchData.searchParams.maxEmptySearchIntervals)
      {  
        System.out.println(prefix + ": too many consecutive empty search rounds: considering problem UNSAT");
        ffstatus = FFSearchStatus.UNSAT;
      }

      if(killMySelf)//if true mspawner thread has set it, put it again to false and terminate your execution
      {
        System.out.println(prefix + ": good moment for a suicide attempt ;-)");
        return;
      }
      
      searchRound++;
    }

    System.out.println(prefix + " search end, status: " + ffstatus);
    if(ffstatus == FFSearchStatus.UNSAT && !lookForImprovedSolutions)
    {
      //could return and pub. this: PPlans[Plan[ PlanItem{-1, "", -1} ]]  
      this.planPublisher.publish(SearchDataUtils.buildUnsatSearchResult());
    }
    else
    {
      short endPlanIndex = (short) this.sharedSearchData.searchResultMsg.getPlans().size();
      boolean goalReached = this.sharedSearchData.searchCurrentState != null? this.sharedSearchData.searchCurrentState.goalReached() : false;
      

      System.out.println(prefix + " search end, startPlanIndex: " + startPlanIndex + ", endPlanIndex: " + endPlanIndex);
      System.out.println(prefix + " search end, goalReached: " + goalReached);

      if(!lookForImprovedSolutions)
        this.sharedSearchData.goalReached = goalReached;
      else if(goalReached && this.sharedSearchData.executingTspWSB != null)
      {
        //I was looking for an improved solution and I might have found one... (I need to compare planned duration before publishing the results)
        
        boolean foundImprovedSolution = false;
        // is it a better solution (in the non committed space)??
        System.out.println(prefix + " lookForImprovedSolutions: tspQueue.size(): " + this.sharedSearchData.tspQueue.size());
        System.out.println(prefix + " lookForImprovedSolutions: executingTspWSB.planIndex: " + this.sharedSearchData.executingTspWSB.planIndex);
        System.out.println(prefix + " lookForImprovedSolutions: startPlanIndex: " + startPlanIndex);
        //start computing planned deadline in non committed space, moving forward till reaching the end of the tspQueue OR till reaching a new plan search result (based on searchID)
        float currentPlanDeadline = SearchDataUtils.computeNonCommittedPlannedDeadline(this.sharedSearchData.tspQueue, this.sharedSearchData.executingTspWSB.planIndex);
        float newPlanDeadline = SearchDataUtils.computeNonCommittedPlannedDeadline(this.sharedSearchData.tspQueue, startPlanIndex);
        System.out.println(prefix + " lookForImprovedSolutions: currentPlanDeadline: " + currentPlanDeadline + ", newPlanDeadline: " + newPlanDeadline);
        foundImprovedSolution = currentPlanDeadline > 0 && newPlanDeadline > 0 && (currentPlanDeadline - newPlanDeadline) > DEADLINE_EPSILON;
        if(foundImprovedSolution)
          this.planPublisher.publish(this.sharedSearchData.searchResultMsg);
      }
    }

  }

}