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

    private javaff_interfaces.msg.ExecutionStatus lastExecStatusUpd;

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    private void execStatusCallback(final javaff_interfaces.msg.ExecutionStatus msg) {
      // Compare lastExecStatusUpd with msg to know: which actions have started and which have terminated
      
      short planIndex = msg.getExecutingPlanIndex();
      if(planIndex < 0 || planIndex >= this.sharedSearchData.tspQueue.size())//invalid info corresponding to aborted or previous executions
        return;

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
              if(tsa.status == aes.RUN_SUC)
                tsa.status = aes.SUCCESS;
          }
        }
      

      this.sharedSearchData.executingTspWSB = this.sharedSearchData.tspQueue.get(planIndex); // update ptr to current tsp in execution within the tspQueue
      System.out.println("Received nr-" + msg.getNotificationReason() + " notification of plan " + this.sharedSearchData.executingTspWSB.planIndex + " execution");

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
      
      
      GroundProblem updGroundProblem = JavaFF.computeGroundProblem(this.domain, msg.getPddlProblem());
      TemporalMetricState updCurrentState = JavaFF.computeInitialState(updGroundProblem);
      updCurrentState = SearchDataUtils.simCommitted(updCurrentState, planIndex, this.sharedSearchData.tspQueue);
      this.sharedSearchData.actualNextCommittedState = updCurrentState;

      if(this.sharedSearchData.actualNextCommittedState == null)
        System.out.println("Committed actions, therefore current plan execution doomed to fail!!");

      else if(msg.getNotificationReason() == msg.NEW_ACTION_STARTED || msg.getNotificationReason() == msg.GOAL_BOOST) // it's not valuable to start a search thread looking for alternative/improved solutions, if I just received a diff. type of notification (such as action finished, because results can become obsolete very quickly)
      {
        System.out.println("sim to goal? " + msg.getSimToGoal());
        if (msg.getNotificationReason() == msg.GOAL_BOOST)
          System.out.println("New BOOSTED goal: " + msg.getPddlProblem().substring(msg.getPddlProblem().indexOf("goal")));

        // force sim to goal should force its way into brutal replanning
        if (this.sharedSearchData.goalReached && msg.getSimToGoal() == msg.SIM_TO_GOAL || msg.getSimToGoal() == msg.SIM_TO_GOAL_FORCE_REPLAN)// if goal reached, try simulate current exec status to goal to see if it's still achievable through computed plan
        {
          TemporalMetricState updCurrStateGoalSim = JavaFF.computeInitialState(updGroundProblem);
          boolean goalStillReachable = msg.getSimToGoal() != msg.SIM_TO_GOAL_FORCE_REPLAN && SearchDataUtils.successSimToGoal(updCurrStateGoalSim, planIndex, this.sharedSearchData.tspQueue);
          // System.out.println(msg.getPddlProblem().substring(msg.getPddlProblem().indexOf("init"), msg.getPddlProblem().indexOf("goal")));
          System.out.println("Sim: " + goalStillReachable);    
          
          if(!goalStillReachable)
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
          else 
          {
            // Start search thread BFS
            if(this.searchThreadBFS == null || !this.searchThreadBFS.isAlive())
            {
              if(updCurrentState != null) // should never happen, if it wasn't for psys2 executor crashing
                if(!updCurrentState.goalReached()) // start search for improved solution if you're not already committed to the goal
                  startSearchFromNextCommittedState(updCurrentState, true);
            }
          }  
        }
      }

      lastExecStatusUpd = msg;//store last upd
    }

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

    public ROS2JavaFFSearch(String name, String namespace, String domain, boolean debug, int minCommitSteps) {
      super(name, namespace);
      this.domain = domain;
      this.debug = debug;
      this.minCommitSteps = minCommitSteps;

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