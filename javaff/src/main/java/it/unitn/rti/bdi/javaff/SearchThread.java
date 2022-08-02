package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;
import java.util.ArrayList;

import javaff.JavaFF;
import javaff.data.GroundProblem;
import javaff.data.TimeStampedPlan;
import javaff.planning.TemporalMetricState;

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

    // we're starting a new search from scratch: we assume agent is not executing now
    // therefore its current state of execution is equivalent to the initial state of search
    // it's going to be updated as soon as the search starts
    this.sharedSearchData.execNextCommittedState = this.sharedSearchData.searchCurrentState;

    while(ffstatus != FFSearchStatus.UNSAT && !this.sharedSearchData.searchCurrentState.goalReached()){
      
      this.sharedSearchData.searchLock.lock();

        // retrieve preconditions from currentState that are going to apply for found plan
        ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions = SearchDataUtils.getCurrentStatePreconditions(this.sharedSearchData.searchCurrentState);

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
            this.sharedSearchData.closed.remove(new Integer(this.sharedSearchData.searchCurrentState.hashCode()));// prevent BFS fails immediately: current was already explored in last EHC search
            //TODO handle here switch to nextCommittedState to restart search from it and NOT from currentState, otherwise might be too late to find a viable solution
          }

        }else if(ffstatus == FFSearchStatus.BFS_SEARCHING)
          ffstatus = (this.sharedSearchData.open.isEmpty() && goalOrIntermediateState == null)? FFSearchStatus.UNSAT : FFSearchStatus.EHC_SEARCHING;

        if(ffstatus != FFSearchStatus.UNSAT && goalOrIntermediateState != null){
          this.sharedSearchData.searchCurrentState = goalOrIntermediateState;
          // build plan string from currentState
          TimeStampedPlan tsp = JavaFF.buildPlan(this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState);
          String planString = "";
          if (tsp != null) 
          {
            this.sharedSearchData.tspQueue.add(tsp);
            planString = tsp.getPrintablePlan();//get plan string
          }

          if(debug){  
            System.out.println("\n\n ROUND " + (searchRound));
            System.out.println(planString);
            System.out.println("open.size="+this.sharedSearchData.open.size() + "\t closed.size=" + this.sharedSearchData.closed.size());
          }
          
          plansys2_msgs.msg.Plan currentPlanMsg = SearchDataUtils.buildPsys2Plan(tsp);// build plan msg and publish it
          
          if(currentPlanMsg.getItems().size() > 0)// build+pub new plan msg and rebase iff 
          {
            // note: do not pass search round, as some round can return no result!!!
            short planIndex = (short) this.sharedSearchData.searchResultMsg.getPlans().size();
            javaff_interfaces.msg.PartialPlan newPPlan = SearchDataUtils.buildNewPPlan(planIndex, this.sharedSearchData.fulfillingDesire,
              planPreconditions, this.sharedSearchData.searchCurrentState, currentPlanMsg);
            
            //Search result containing all pplans up to now within this search
            this.sharedSearchData.searchResultMsg.getPlans().add(newPPlan);
            this.sharedSearchData.searchResultMsg.setStatus(this.sharedSearchData.searchCurrentState.goalReached()? this.sharedSearchData.searchResultMsg.SUCCESS : this.sharedSearchData.searchResultMsg.SEARCHING);
            
            if(!killMySelf)//these search results are still valid
              this.planPublisher.publish(this.sharedSearchData.searchResultMsg);
            
            // rebase exclusively when plan presents some actions, otherwise next search cycle will start from where it left
            JavaFF.rebaseOnCurrentState(this.sharedSearchData.groundProblem, this.sharedSearchData.searchCurrentState, this.sharedSearchData.open, this.sharedSearchData.closed);
          }
        
        }
        

      this.sharedSearchData.searchLock.unlock();
      if(killMySelf)//if true mspawner thread has set it, put it again to false and terminate your execution
        return;

      searchRound++;
    }

    if(ffstatus == FFSearchStatus.UNSAT){
      //could return and pub. this: PPlans[Plan[ PlanItem{-1, "", -1} ]]  
      this.planPublisher.publish(SearchDataUtils.buildUnsatSearchResult());
    }
  }

}