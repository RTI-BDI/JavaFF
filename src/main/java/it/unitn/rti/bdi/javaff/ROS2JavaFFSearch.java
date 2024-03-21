package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.subscription.Subscription;
import java.util.ArrayList;
import java.util.Hashtable;
import java.util.TreeSet;
import java.util.stream.Collectors;
import java.math.BigDecimal;

import org.ros2.rcljava.node.BaseComposableNode;

import javaff.JavaFF;
import javaff.planning.TemporalMetricState;
import javaff.search.HValueComparator;
import javaff.data.GroundProblem;
import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;
import javaff.scheduling.MatrixSTN;

import it.unitn.rti.bdi.javaff.SearchParams;
import it.unitn.rti.bdi.javaff.SharedSearchData;
import it.unitn.rti.bdi.javaff.SearchDataUtils;
import it.unitn.rti.bdi.javaff.SearchThread;

enum ForecastPlanFailureRes{
  FAIL_C,
  FAIL_NC,
  NO_FAIL,
  NOT_COMP //not computed
}

public class ROS2JavaFFSearch extends BaseComposableNode{
    
    // Sibling node handling service request wrt. online planning
    private ROS2JavaFFServer serverNode;
    
    private SearchThread searchThread;

    private SearchThread searchThreadBFS;//to use just for improved solutions

    private SharedSearchData sharedSearchData;

    private Publisher<javaff_interfaces.msg.SearchResult> planPublisher;

    private Publisher<javaff_interfaces.msg.CommittedStatus> planCommittedStatusPublisher;

    private Subscription<javaff_interfaces.msg.ExecutionStatus> execStatusSubscriber;

    private String domain;

    private boolean debug;

    private int minCommitSteps;

    private int simToN;

    private javaff_interfaces.msg.ExecutionStatus lastExecStatusUpd;

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    private void execStatusCallback(final javaff_interfaces.msg.ExecutionStatus msg) {
      // Compare lastExecStatusUpd with msg to know: which actions have started and which have terminated
      
      // CHECK VALIDITY OF THE MESSAGE (i.e. old notification?)
      short planIndex = msg.getExecutingPlanIndex();
      if(planIndex < 0 || planIndex >= this.sharedSearchData.tspQueue.size())//invalid info corresponding to aborted or previous executions
        return;
      
      // Handle switch to subsequent computed plan, updating status of the last action(s) run in the previous and  
      if(this.sharedSearchData.nextCommittedState != null && lastExecStatusUpd != null)
        if(lastExecStatusUpd.getExecutingPlanIndex() < planIndex)
        {
          //reset to zero when new plan starts
          this.sharedSearchData.nextCommittedState.currInstant = BigDecimal.ZERO;
          
          //make sure that all actions of previous tsp are marked as success and not run_success TODO eval if comm. final SUCC in scheduler 
          if(lastExecStatusUpd.getExecutingPlanIndex() >= 0 && lastExecStatusUpd.getExecutingPlanIndex() < this.sharedSearchData.tspQueue.size())
          {
            javaff_interfaces.msg.ActionExecutionStatus aes = new javaff_interfaces.msg.ActionExecutionStatus();
            TimeStampedPlan lastTsp = this.sharedSearchData.tspQueue.get(lastExecStatusUpd.getExecutingPlanIndex());
            for(TimeStampedAction tsa : lastTsp.getSortedActions())
              if(tsa.status == aes.RUN_SUC)//IT'S IMPORTANT TO HAVE THIS step and diff. RUN_SUC from SUCCESS: at RUN_SUC action has terminated successfully, but end effects might have not been applied yet
                tsa.status = aes.SUCCESS;//When switching to SUCCESS I KNOW, end effects of last action in the plan has already been applied
          }
        }
      
      // KEEP TRACK OF CURRENTLY EXECUTING PLAN, must be in tspQueue
      this.sharedSearchData.executingTspWSB = this.sharedSearchData.tspQueue.get(planIndex); // update ptr to current tsp in execution within the tspQueue
      System.out.println("Received nr-" + msg.getNotificationReason() + " notification of plan " + this.sharedSearchData.executingTspWSB.planIndex + " execution");
      
      // UPDATE EXEC and COMMITTED status of actions in the currently running plan (COMPUTATION OF THEORETICAL COMMITTED STATE based on effects applied to the original state which spawned the search)
      for(javaff_interfaces.msg.ActionExecutionStatus aesMsg : msg.getExecutingActions())
      {
        BigDecimal startTimeBD = (new BigDecimal(aesMsg.getPlannedStartTime())).setScale(MatrixSTN.SCALE, MatrixSTN.ROUND);

        String fullActionNameTimex1000 = aesMsg.getExecutingAction() + ":"+ (int) (aesMsg.getPlannedStartTime()*1000);
        TimeStampedAction tsa = this.sharedSearchData.executingTspWSB.getTimeStampedAction(fullActionNameTimex1000);
        if(tsa != null)
        {
          if(aesMsg.getStatus() == aesMsg.RUNNING && tsa.status == aesMsg.WAITING && !tsa.committed)
          {
            // System.out.println("I heard: action '" + fullActionNameTimex1000 +"' of plan with i = " + planIndex + " is executing");
            TemporalMetricState nextCommittedState = (this.sharedSearchData.nextCommittedState.currInstant.compareTo(new BigDecimal(aesMsg.getPlannedStartTime())) < 0)?
                SearchDataUtils.computeNextCommittedState(
                  this.sharedSearchData.nextCommittedState, 
                  fullActionNameTimex1000,
                  this.sharedSearchData.executingTspWSB,
                  this.minCommitSteps)
                :
                null;
            if(nextCommittedState != null)
            {
              this.sharedSearchData.searchLock.lock();
              this.sharedSearchData.nextCommittedState = nextCommittedState;
              this.sharedSearchData.searchLock.unlock();
            }
          }
          
          // update action status in stored tsp
          tsa.status = aesMsg.getStatus() < tsa.status? tsa.status : aesMsg.getStatus(); // NOTE: first case, msg is old (e.g. receiving RUNNING, when I already know it has finished)
        }
      }
      
      if(msg.getNotificationReason() == msg.NEW_ACTION_STARTED && debug)
      {
        System.out.println("Current status - committed actions in plan " + 
          planIndex + ":\n" + 
          this.sharedSearchData.tspQueue.get(planIndex).getPrintablePlan(true));
      }

      // publish committed status
      publishCommittedStatus(this.sharedSearchData.executingTspWSB);
      
      // COMPUTE ACTUAL NEXT COMMITTED STATE
      GroundProblem updGroundProblem = JavaFF.computeGroundProblem(this.domain, msg.getPddlProblem());
      TemporalMetricState updCurrentState = JavaFF.computeInitialState(updGroundProblem);
      updCurrentState = SearchDataUtils.simCommitted(updCurrentState, planIndex, this.sharedSearchData.tspQueue);
      this.sharedSearchData.actualNextCommittedState = updCurrentState;
      
      if (msg.getNotificationReason() == msg.GOAL_BOOST)
        System.out.println("New BOOSTED goal: " + msg.getPddlProblem().substring(msg.getPddlProblem().indexOf("goal")));

      // FORECAST PLAN FAILURE
      ForecastPlanFailureRes planFailure = forecastPlanFailure(JavaFF.computeInitialState(updGroundProblem), msg.getNotificationReason(), msg.getSimToGoal());
      System.out.println("forecasting plan failure -> " + planFailure);
      
      // REACT TO PLAN FAILURE RESULT (continue, replan or improve)
      if(planFailure == ForecastPlanFailureRes.FAIL_NC)//failure in non committed actions -> replan
      {
        if(this.searchThreadBFS != null && this.searchThreadBFS.isAlive())
        {
          try{
            System.out.println("Search thread BFS already up: waiting for it to stop...");
            this.searchThreadBFS.killMySelf();
            this.searchThreadBFS.join();
          }catch(InterruptedException ie){
            System.out.println("Search for a plan has not been started successfully due to an internal error");
          }
        }

        if(msg.getSimToGoal() == msg.SIM_TO_GOAL_FORCE_REPLAN)
        {
          if(this.searchThread != null && this.searchThread.isAlive())
          {
            try{
              System.out.println("Search thread already up: force replan; waiting for it to stop...");
              this.searchThread.killMySelf();
              this.searchThread.join();
            }catch(InterruptedException ie){
              System.out.println("Search for a plan has not been started successfully due to an internal error");
            }
          }
        }
        
        if(this.searchThread == null || !this.searchThread.isAlive())
        {
          if(updCurrentState != null) // should never happen, if it wasn't for psys2 executor crashing
            startSearchFromNextCommittedState(updCurrentState);
        }
      }
      else if (planFailure == ForecastPlanFailureRes.NO_FAIL)// no failure -> improve solution
      {
        // Start search thread BFS
        if(this.searchThreadBFS == null || !this.searchThreadBFS.isAlive())
        {
          if(updCurrentState != null) // should never happen, if it wasn't for psys2 executor crashing
            if(!updCurrentState.goalReached()) // start search for improved solution if you're not already committed to the goal
              startSearchFromNextCommittedState(updCurrentState, true);
        }
      } 

      lastExecStatusUpd = msg;//store last upd
    }

    
    /* Forecast plan failure:
    * PARAM
    *   notificationReason  -> ACTION_STARTED, ACTION_FINISHED, GOAL_BOOST
    *   simToGoalReq        -> NO_SIM_TO_GOAL, SIM_TO_GOAL, SIM_TO_GOAL_FORCE_REPLAN
    *   simToN              -> -1 if sim to Goal, otherwise sim just successful N steps
    * RETURN
    * FAIL_C : case failure in committed actions -> too late to act
    * FAIL_NC: plan not valid, but we can replan
    * NO_FAIL: plan still valid and sound, i.e. able to sim. to goal
    * NOT_COMP: not computed, meaning that either:
        a) notification reason was an action which just finished (pointless to check at the end of an action, since situation will change again really quickly) or 
        b) sim to goal was not performed because simToGoal suggested not to (e.g. replan had found an alternative solution in found, which has already been put into the waiting queue)

        Note b) is a limitation of the current implementation, where we do not check for validity and sound of the plan after a successful early arrest request (because we do not track it here in the planner)
        We basically do not sim. in that situation, we know that correct replacement of the plan has been performed in time, so we wait the end of the committed actions in the current plan and
        then, as soon as the new one starts, forecast proceeds as usual 
    */
    private ForecastPlanFailureRes forecastPlanFailure(TemporalMetricState currState, final short notificationReason, final short simToGoalReq){
      final javaff_interfaces.msg.ExecutionStatus protoExecStatus = new javaff_interfaces.msg.ExecutionStatus();
      if(this.sharedSearchData.actualNextCommittedState == null)
      {
        System.out.println("Committed actions will fail, therefore current plan execution doomed to fail and too late to act!!");
        return ForecastPlanFailureRes.FAIL_C;
      }
      else if(notificationReason == protoExecStatus.NEW_ACTION_STARTED || notificationReason == protoExecStatus.GOAL_BOOST) // it's not valuable to start a search thread looking for alternative/improved solutions, if I just received a diff. type of notification (such as action finished, because results can become obsolete very quickly)
      {
        // force sim to goal should force its way into brutal replanning
        if (this.sharedSearchData.goalReached && simToGoalReq == protoExecStatus.SIM_TO_GOAL || simToGoalReq == protoExecStatus.SIM_TO_GOAL_FORCE_REPLAN)// if goal reached, try simulate current exec status to goal to see if it's still achievable through computed plan
        {
          if(simToN < 0)
          {
            boolean goalStillReachable = simToGoalReq != protoExecStatus.SIM_TO_GOAL_FORCE_REPLAN && SearchDataUtils.successSimToGoal(currState, this.sharedSearchData.executingTspWSB.planIndex, this.sharedSearchData.tspQueue);
            System.out.println("Sim to goal result: " + goalStillReachable);    
            return (goalStillReachable)? ForecastPlanFailureRes.NO_FAIL : ForecastPlanFailureRes.FAIL_NC;

          }
          else
          {
            boolean canPerformNSteps = simToGoalReq != protoExecStatus.SIM_TO_GOAL_FORCE_REPLAN && SearchDataUtils.successSimToN(currState, this.sharedSearchData.executingTspWSB.planIndex, this.sharedSearchData.tspQueue, this.simToN);
            System.out.println("Sim to N=" + simToN + " result: " + canPerformNSteps);    
            return (canPerformNSteps)?  ForecastPlanFailureRes.NO_FAIL:  ForecastPlanFailureRes.FAIL_NC;
          }
        }
          
      }
      
      // if here, forecast plan failure not computed
      return ForecastPlanFailureRes.NOT_COMP;
    }

    /*  Publish current committed status into a topic, just executing partial plan is taken into account 
        and all waiting actions in following plans are not accounted for
    */
    private void publishCommittedStatus(TimeStampedPlanWithSearchBaseline executingTspWSB){
      javaff_interfaces.msg.CommittedStatus executingTspCommittedStatus = new javaff_interfaces.msg.CommittedStatus();
      executingTspCommittedStatus.setExecutingPlanIndex(executingTspWSB.planIndex);
      ArrayList<javaff_interfaces.msg.ActionCommittedStatus> actionsCommittedStatus = new ArrayList<javaff_interfaces.msg.ActionCommittedStatus>();
      for(TimeStampedAction tsa : executingTspWSB.getSortedActions())
      {
        javaff_interfaces.msg.ActionCommittedStatus acs = new javaff_interfaces.msg.ActionCommittedStatus();
        acs.setCommittedAction(""+tsa.action);
        acs.setPlannedStartTime(tsa.time.floatValue());
        acs.setCommitted(tsa.committed);
        actionsCommittedStatus.add(acs);
      }

      executingTspCommittedStatus.setCommittedActions(actionsCommittedStatus);
      this.planCommittedStatusPublisher.publish(executingTspCommittedStatus);
    }

    /*
     * 
     * @name of the node
     * @namespace of the node
     * @domain PDDL 2.1 domain string to keep loaded for all future requests
     * @minCommitSteps minimum number of sequential actions that will be considered committed when action a which has just started running (therefore committed) starts
    */
    public ROS2JavaFFSearch(String name, String namespace, String domain, boolean debug, int minCommitSteps, int simToN) {
      super(name, namespace);
      this.domain = domain;
      this.debug = debug;
      this.minCommitSteps = minCommitSteps;
      this.simToN = simToN;

      this.sharedSearchData = new SharedSearchData();
      
      // publish updated search results
      this.planPublisher = this.node.<javaff_interfaces.msg.SearchResult>createPublisher(javaff_interfaces.msg.SearchResult.class, name + "/plan");

      // publish updated search results
      this.planCommittedStatusPublisher = this.node.<javaff_interfaces.msg.CommittedStatus>createPublisher(javaff_interfaces.msg.CommittedStatus.class, name + "/committed_status");
      
      // receive notification of execution status (i.e. when action x starts, it publishes to this topic)
      this.execStatusSubscriber = this.node.<javaff_interfaces.msg.ExecutionStatus>createSubscription(
        javaff_interfaces.msg.ExecutionStatus.class, 
        name + "/exec_status",
        this ::execStatusCallback);
    } 

    /*
     * Start search from scratch, considering all respective search parameters (interval search time, max plan size, maximum number of consecutive empty rounds in search)
    */
    public OperationResult startSearch(ros2_bdi_interfaces.msg.Desire fulfillingDesire, String problem, SearchParams searchParams){
      OperationResult returnObj = new OperationResult(false, "Search for a plan has not been started");
      
      try{

        // kill search thread if there is one already alive -> might cause npe: just ignore it
        try{
          if(this.searchThread.isAlive()){
            System.out.println("Search thread already up: waiting for it to stop...");
            this.searchThread.killMySelf();
            this.searchThread.join();
          }
        }catch(NullPointerException ne){}//handle first call in which thread has not been started yet or no one is running
        
        this.sharedSearchData.searchLock.lock();

        // START NEW SEARCH -> set new global target
        this.sharedSearchData.fulfillingDesire = fulfillingDesire;
        
        GroundProblem groundProblem = JavaFF.computeGroundProblem(this.domain, problem);
        TemporalMetricState initialTMS = JavaFF.computeInitialState(groundProblem);
        clearSharedSearchResultData();
        startNewSearch(groundProblem, initialTMS, searchParams);

        returnObj.result = true;  
        returnObj.msg = "Search for a plan has been started successfully";

      }catch(javaff.parser.TokenMgrError mgrError){
        String msgError = mgrError.toString();
        returnObj.msg = msgError; 
        if((msgError.contains("EOF") && msgError.contains("Lexical error at line 1")))
          returnObj.msg += "\n\nNOTE: Might be due to lack of proper end-line characters in " + ((msgError.contains("domain"))? "domain":"problem") + " string!";
        returnObj.result = false;  

      }catch(InterruptedException ie){
        returnObj.result = false;  
        returnObj.msg = "Search for a plan has not been started successfully due to an internal error";
      
      }finally{
        this.sharedSearchData.searchLock.unlock();
      }
      
      return returnObj;
    }

    /*
     * 
     * Start search from scratch after plan natural failure, treated as a search from scratch for now (better handling in future iterations)
     * Search params the same as the original search started to fulfill the goal with the most recent startSearch call
    */
    public OperationResult unexpectedState(String newStatePddlProblem){
      OperationResult returnObj = new OperationResult(false, "Search adjustments have been handled");

      this.sharedSearchData.searchLock.lock();
      try{
        //TODO advanced: check whether newState is in open or closed (for now they're empty: pointless)
        GroundProblem groundProblem = JavaFF.computeGroundProblem(this.domain, newStatePddlProblem);
        TemporalMetricState initialTMS = JavaFF.computeInitialState(groundProblem);
        clearSharedSearchResultData();
        startNewSearch(groundProblem, initialTMS, this.sharedSearchData.searchParams);
        returnObj.result = true;  
        returnObj.msg = "Search for an updated plan has been restarted successfully";

      }catch(javaff.parser.TokenMgrError mgrError){
        String msgError = mgrError.toString();
        returnObj.msg = msgError; 
        if((msgError.contains("EOF") && msgError.contains("Lexical error at line 1")))
          returnObj.msg += "\n\nNOTE: Might be due to lack of proper end-line characters in " + ((msgError.contains("domain"))? "domain":"problem") + " string!";
        returnObj.result = false;  

      
      }catch(InterruptedException ie){
        returnObj.result = false;  
        returnObj.msg = "Search for a plan has not been started successfully due to an internal error";
      
      }finally{
        this.sharedSearchData.searchLock.unlock();
      }

      return returnObj;
    }

    private void clearSharedSearchResultData(){
        // clear search result data
        this.sharedSearchData.searchResultMsg = new javaff_interfaces.msg.SearchResult();
        this.sharedSearchData.tspQueue = new ArrayList<TimeStampedPlanWithSearchBaseline>();
        this.sharedSearchData.executingTspWSB = null;
    }

    private void startNewSearch(GroundProblem groundProblem, TemporalMetricState initialState, SearchParams searchParams) throws javaff.parser.TokenMgrError, InterruptedException{
      startNewSearch(groundProblem, initialState, searchParams, false);
    }

    private void startNewSearch(GroundProblem groundProblem, TemporalMetricState initialState, SearchParams searchParams, boolean lookForImprovedSolutions) throws javaff.parser.TokenMgrError, InterruptedException{
      
      this.sharedSearchData.open = new TreeSet<>(new HValueComparator());
      this.sharedSearchData.closed = new Hashtable<>();
      // parse domain and problem, unground + ground processes, returning the initial state

      this.sharedSearchData.groundProblem = groundProblem;
      this.sharedSearchData.searchCurrentState = initialState;
      this.sharedSearchData.goalReached = lookForImprovedSolutions? this.sharedSearchData.goalReached : false;
      this.sharedSearchData.searchParams.intervalSearchMS = searchParams.intervalSearchMS > 100? searchParams.intervalSearchMS : 100;
      this.sharedSearchData.searchParams.maxPPlanSize = searchParams.maxPPlanSize > 0? searchParams.maxPPlanSize : 32000;
      this.sharedSearchData.searchParams.maxEmptySearchIntervals = searchParams.maxEmptySearchIntervals > 0? searchParams.maxEmptySearchIntervals : 1;
      
      lastExecStatusUpd = null;

      // start search thread from initial state and init. search data
      if(!lookForImprovedSolutions)
      {
        // Use "std" searchThread instance which starts with an EHC search, jumping to BFS search just if the first fails to find a goal 
        if(this.searchThread == null || !this.searchThread.isAlive()){
          //search thread not alive
          this.searchThread = new SearchThread(sharedSearchData, planPublisher, debug);
          this.searchThread.start();
        }
      }
      else
      {
        // Use searchThreadBFS and start directly with a BFS search (more expensive!!!!)
        if(this.searchThreadBFS == null || !this.searchThreadBFS.isAlive()){
          //search thread not alive
          this.searchThreadBFS = new SearchThread(sharedSearchData, planPublisher, debug, true);
          this.searchThreadBFS.start();
        }
      }

    }

    private void startSearchFromNextCommittedState(TemporalMetricState nextCommittedState){
      startSearchFromNextCommittedState(nextCommittedState, false);
    }

    private void startSearchFromNextCommittedState(TemporalMetricState nextCommittedState, boolean jumpToBFS){
    
      this.sharedSearchData.searchLock.lock();
      try{
		    nextCommittedState.cleanPlanInfo();
        
        this.sharedSearchData.groundProblem.initial = nextCommittedState.facts;
        this.sharedSearchData.groundProblem.state = nextCommittedState;
        this.sharedSearchData.groundProblem.functionValues = nextCommittedState.funcValues;
  
        startNewSearch(this.sharedSearchData.groundProblem, nextCommittedState, this.sharedSearchData.searchParams, jumpToBFS);  

      }catch(javaff.parser.TokenMgrError mgrError){
        String msgError = mgrError.toString();
        if((msgError.contains("EOF") && msgError.contains("Lexical error at line 1")))
          msgError += "\n\nNOTE: Might be due to lack of proper end-line characters in " + ((msgError.contains("domain"))? "domain":"problem") + " string!";
        System.err.println(msgError);
      
      }catch(InterruptedException ie){
        System.err.println("Search for a plan has not been started successfully due to an internal error");
      
      }finally{
        this.sharedSearchData.searchLock.unlock();
      }
    }
   
}