package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.Set;
import java.math.BigDecimal;

import javaff.JavaFF;
import javaff.data.Action;
import javaff.data.GroundProblem;
import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;

import javaff.data.temporal.SplitInstantAction;
import javaff.data.temporal.DurativeAction;
import javaff.planning.TemporalMetricState;

// import java.io.FileOutputStream;
// import java.io.ObjectOutputStream;

import it.unitn.rti.bdi.javaff.SharedSearchData;
import it.unitn.rti.bdi.javaff.SearchDataUtils;

enum FFSearchStatus{EHC_SEARCHING, EHC_FAILED, BFS_SEARCHING, UNSAT};

public class SearchThread extends Thread{

  private boolean killMySelf = false;
  private boolean debug = false;
  private SharedSearchData sharedSearchData;
  private Publisher<javaff_interfaces.msg.SearchResult> planPublisher;

  public SearchThread(SharedSearchData sharedSearchData, Publisher<javaff_interfaces.msg.SearchResult> planPublisher, boolean debug){
    super();
    this.sharedSearchData = sharedSearchData;
    this.planPublisher = planPublisher;
    this.debug = debug;
  }

  public void killMySelf(){killMySelf = true;}
  
  public void run(){
    this.sharedSearchData.open.clear();

    short searchRound = 0;
    FFSearchStatus ffstatus = FFSearchStatus.EHC_SEARCHING;

    // clear search result data
    this.sharedSearchData.searchResultMsg = new javaff_interfaces.msg.SearchResult();
    this.sharedSearchData.tspQueue = new ArrayList<TimeStampedPlan>();

    this.sharedSearchData.searchResultMsg.setSearchBaseline(SearchDataUtils.getSearchBaseline(this.sharedSearchData.tspQueue));

    // we're starting a new search from scratch: we assume agent is not executing now
    // therefore its current state of execution is equivalent to the initial state of search
    // it's going to be updated as soon as the search starts
    this.sharedSearchData.execNextCommittedState = (TemporalMetricState) this.sharedSearchData.searchCurrentState.clone();
    this.sharedSearchData.execNextCommittedState.currInstant = BigDecimal.ZERO;

    while(ffstatus != FFSearchStatus.UNSAT && !this.sharedSearchData.searchCurrentState.goalReached()){

        this.sharedSearchData.searchLock.lock();
        System.out.println("ROUND " + searchRound + " starting from state " + this.sharedSearchData.searchCurrentState.toString() + "\tUnique ID=" + (this.sharedSearchData.searchCurrentState.getUniqueId()));

        // retrieve preconditions from currentState that are going to apply for found plan
        ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions = SearchDataUtils.getCurrentStatePreconditions(this.sharedSearchData.searchCurrentState);

        ArrayList<ros2_bdi_interfaces.msg.Belief> committedTrueBeliefsStart = SearchDataUtils.getTrueBeliefs(this.sharedSearchData.searchCurrentState);

        // move forward with the search for interval search time
        TemporalMetricState goalOrIntermediateState = (ffstatus == FFSearchStatus.EHC_SEARCHING)?
          (TemporalMetricState) JavaFF.performEHCSearch(this.sharedSearchData.searchCurrentState, this.sharedSearchData.intervalSearchMS, this.sharedSearchData.open, this.sharedSearchData.closed)
          :
          (TemporalMetricState) JavaFF.performBFSSearch(this.sharedSearchData.searchCurrentState, this.sharedSearchData.intervalSearchMS, this.sharedSearchData.open, this.sharedSearchData.closed);

        //check whether unsat ~ empty open and search has return null
        if(ffstatus == FFSearchStatus.EHC_SEARCHING)
        {
          ffstatus = (this.sharedSearchData.open.isEmpty() && goalOrIntermediateState == null)? FFSearchStatus.BFS_SEARCHING : FFSearchStatus.EHC_SEARCHING;
          
          if(ffstatus == FFSearchStatus.BFS_SEARCHING)//just switched to BFS searching
          {   
            // here switch to nextCommittedState to restart search from it and NOT from currentState, otherwise might be too late to find a viable solution
            
            // retrieve last exec committed action and put it as a search baseline, so that scheduler knows
            this.sharedSearchData.searchResultMsg.setSearchBaseline(SearchDataUtils.getSearchBaseline(this.sharedSearchData.tspQueue));
            
            // clear tspqueue index additional plans after currently committed one that are not valid anymore...
            while(this.sharedSearchData.searchResultMsg.getSearchBaseline().getExecutingPlanIndex() < (this.sharedSearchData.tspQueue.size() - 1)) 
              this.sharedSearchData.tspQueue.remove(this.sharedSearchData.tspQueue.size() - 1);

            // put next exec committedState as base for the next search iteration
            this.sharedSearchData.searchCurrentState = this.sharedSearchData.execNextCommittedState;

             // rebase on new search base state for next bfs search iterations
             JavaFF.rebaseOnCurrentState(this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState, this.sharedSearchData.open, this.sharedSearchData.closed);
          }

        }else if(ffstatus == FFSearchStatus.BFS_SEARCHING)
          ffstatus = (this.sharedSearchData.open.isEmpty() && goalOrIntermediateState == null)? FFSearchStatus.UNSAT : FFSearchStatus.EHC_SEARCHING;

        if(ffstatus != FFSearchStatus.UNSAT){
          if(goalOrIntermediateState != null && !goalOrIntermediateState.getSolution().getActions().isEmpty()){
            /*
            try{  
              // Serializing 'a'
              FileOutputStream fos
                  = new FileOutputStream("/home/devis/p"+searchRound+".txt");
              ObjectOutputStream oos
                  = new ObjectOutputStream(fos);
              oos.writeObject(goalOrIntermediateState.getTPSolution());
            }catch(Exception e){
              e.printStackTrace();
            }
            */
            ArrayList<ros2_bdi_interfaces.msg.Belief> committedTrueBeliefsEnd = SearchDataUtils.getTrueBeliefs(goalOrIntermediateState);
          
            this.sharedSearchData.searchCurrentState = goalOrIntermediateState;
            // build plan string from currentState
            TimeStampedPlan tsp = JavaFF.buildPlan(this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState);
            String planString = "";
            if (tsp != null && tsp.getSortedActions().size()>0) 
            {
              this.sharedSearchData.tspQueue.add(tsp);
              planString = tsp.getPrintablePlan(false);//get plan string

              plansys2_msgs.msg.Plan currentPlanMsg = SearchDataUtils.buildPsys2Plan(tsp);// build plan msg and publish it          
              
              // note: do not pass search round, as some round can return no result!!!
              short planIndex = (short) this.sharedSearchData.searchResultMsg.getPlans().size();
              javaff_interfaces.msg.PartialPlan newPPlan = SearchDataUtils.buildNewPPlan(planIndex, this.sharedSearchData.fulfillingDesire,
                planPreconditions, this.sharedSearchData.searchCurrentState, currentPlanMsg);
              
              //Search result containing all pplans up to now within this search
              this.sharedSearchData.searchResultMsg.getPlans().add(newPPlan);
              this.sharedSearchData.searchResultMsg.setStatus(this.sharedSearchData.searchCurrentState.goalReached()? this.sharedSearchData.searchResultMsg.SUCCESS : this.sharedSearchData.searchResultMsg.SEARCHING);
              
              if(!killMySelf)//these search results are still valid
                this.planPublisher.publish(this.sharedSearchData.searchResultMsg);
                
              //Log round result
              if(debug){  
                /*
                System.out.println("ROUND " + searchRound + ": start state");
                for(ros2_bdi_interfaces.msg.Belief b : committedTrueBeliefsStart)
                  System.out.println("\t- " + b.getName() + " " + b.getParams().stream().collect(Collectors.joining(" ")));
                
                System.out.println("ROUND " + searchRound + ": end state");
                for(ros2_bdi_interfaces.msg.Belief b : committedTrueBeliefsEnd)
                  System.out.println("\t- " + b.getName() + " " + b.getParams().stream().collect(Collectors.joining(" ")));

                System.out.println("\nplan snap action num: " + this.sharedSearchData.searchCurrentState.getSolution().getActions().size());
                for(Action sia : ((List<Action>)this.sharedSearchData.searchCurrentState.getTPSolution().getOrderedActions()))
                  System.out.println(sia.toString());
                */
                System.out.println("\nROUND " + (searchRound) + " has produced plan " + planIndex);
                System.out.println(planString);
                System.out.println("open.size="+this.sharedSearchData.open.size() + "\t closed.size=" + this.sharedSearchData.closed.size());
              }
              
              // rebase exclusively when plan presents some actions, otherwise next search cycle will start from where it left
              JavaFF.rebaseOnCurrentState(this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState, this.sharedSearchData.open, this.sharedSearchData.closed);
            }
          }
        }
        
      this.sharedSearchData.searchLock.unlock();
      if(killMySelf)//if true mspawner thread has set it, put it again to false and terminate your execution
        return;
      

      System.out.println("ROUND " + searchRound + " ending in state " + this.sharedSearchData.searchCurrentState.toString() + "\tUnique ID=" + (this.sharedSearchData.searchCurrentState.getUniqueId()));

      searchRound++;
    }

    if(ffstatus == FFSearchStatus.UNSAT){
      //could return and pub. this: PPlans[Plan[ PlanItem{-1, "", -1} ]]  
      this.planPublisher.publish(SearchDataUtils.buildUnsatSearchResult());
    }

    this.sharedSearchData.goalReached = this.sharedSearchData.searchCurrentState != null? this.sharedSearchData.searchCurrentState.goalReached() : false;
  }

}