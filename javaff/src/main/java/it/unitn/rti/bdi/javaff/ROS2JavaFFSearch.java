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
import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;
import javaff.scheduling.MatrixSTN;

import it.unitn.rti.bdi.javaff.SharedSearchData;
import it.unitn.rti.bdi.javaff.SearchDataUtils;
import it.unitn.rti.bdi.javaff.SearchThread;

public class ROS2JavaFFSearch extends BaseComposableNode{
    
    // Sibling node handling service request wrt. online planning
    private ROS2JavaFFServer serverNode;
    
    private SearchThread searchThread;

    private SharedSearchData sharedSearchData;

    private Publisher<javaff_interfaces.msg.SearchResult> planPublisher;

    private Subscription<javaff_interfaces.msg.ExecutionStatus> execStatusSubscriber;

    private String domain;

    private boolean debug;

    private javaff_interfaces.msg.ExecutionStatus lastExecStatusUpd;

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    private void execStatusCallback(final javaff_interfaces.msg.ExecutionStatus msg) {
      // Compare lastExecStatusUpd with msg to know: which actions have started and which have terminated
      
      short planIndex = msg.getExecutingPlanIndex();
      
      if(this.sharedSearchData.execNextCommittedState != null && lastExecStatusUpd != null)
        if(lastExecStatusUpd.getExecutingPlanIndex() < planIndex)
          this.sharedSearchData.execNextCommittedState.currInstant = BigDecimal.ZERO;//reset to zero when new plan starts
      
      TimeStampedPlan tsp = this.sharedSearchData.tspQueue.get(planIndex); 
      
      for(javaff_interfaces.msg.ActionExecutionStatus aesMsg : msg.getExecutingActions())
      {
        BigDecimal startTimeBD = (new BigDecimal(aesMsg.getPlannedStartTime())).setScale(MatrixSTN.SCALE, MatrixSTN.ROUND);

        String fullActionNameTimex1000 = aesMsg.getExecutingAction() + ":"+ (int) (aesMsg.getPlannedStartTime()*1000);
        TimeStampedAction tsa = tsp.getTimeStampedAction(fullActionNameTimex1000);
        if(aesMsg.getStatus() == aesMsg.RUNNING && tsa.status == aesMsg.WAITING && !tsa.committed)
        {
          // System.out.println("I heard: action '" + fullActionNameTimex1000 +"' of plan with i = " + planIndex + " is executing");
          TemporalMetricState nextCommittedState = (this.sharedSearchData.execNextCommittedState.currInstant.compareTo(new BigDecimal(aesMsg.getPlannedStartTime())) < 0)?
              SearchDataUtils.computeNextCommittedState(
                this.sharedSearchData.execNextCommittedState, 
                fullActionNameTimex1000,
                tsp)
              :
              null;
          if(nextCommittedState != null)
          {
            this.sharedSearchData.searchLock.lock();
            this.sharedSearchData.execNextCommittedState = nextCommittedState;
            this.sharedSearchData.searchLock.unlock();
          }
        }
        
        // update action status in stored tsp
        tsp.markExecStatus(aesMsg.getExecutingAction().substring(1,aesMsg.getExecutingAction().length()-1), startTimeBD, aesMsg.getStatus());
      }
  
      System.out.println("Current status - committed actions in plan " + planIndex + ":");
      System.out.println(this.sharedSearchData.tspQueue.get(planIndex).getPrintablePlan(true));
        
      if (this.sharedSearchData.goalReached)// if goal reached, try simulate current exec status to goal to see if it's still achievable through computed plan
      {
        boolean goalStillReachable = SearchDataUtils.successSimToGoal(this.domain, msg.getPddlProblem(), planIndex, this.sharedSearchData.tspQueue);
        // System.out.println(msg.getPddlProblem().substring(msg.getPddlProblem().indexOf("init"), msg.getPddlProblem().indexOf("goal")));
        System.out.println("Sim: " + goalStillReachable);    
        
        if(!goalStillReachable)
        {
          //TODO
        }  
      }

      lastExecStatusUpd = msg;//store last upd
    }

    public ROS2JavaFFSearch(String name, String namespace, String domain, boolean debug) {
      super(name, namespace);
      this.domain = domain;
      this.debug = debug;

      this.sharedSearchData = new SharedSearchData();
      
      // publish updated search results
      this.planPublisher = this.node.<javaff_interfaces.msg.SearchResult>createPublisher(javaff_interfaces.msg.SearchResult.class, name + "/plan");
      
      // receive notification of execution status (i.e. when action x starts, it publishes to this topic)
      this.execStatusSubscriber = this.node.<javaff_interfaces.msg.ExecutionStatus>createSubscription(
        javaff_interfaces.msg.ExecutionStatus.class, 
        name + "/exec_status",
        this ::execStatusCallback);
    } 

    public OperationResult startSearch(ros2_bdi_interfaces.msg.Desire fulfillingDesire, String problem, int intervalSearchMS){
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
        
        startNewSearch(problem, intervalSearchMS);

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
        startNewSearch(newStatePddlProblem, this.sharedSearchData.intervalSearchMS);
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

    private void startNewSearch(String problem, int intervalSearchMS) throws javaff.parser.TokenMgrError, InterruptedException{
      
      this.sharedSearchData.open = new TreeSet<>(new HValueComparator());
      this.sharedSearchData.closed = new Hashtable<>();
      // parse domain and problem, unground + ground processes, returning the initial state

      this.sharedSearchData.groundProblem = JavaFF.computeGroundProblem(this.domain, problem);
      this.sharedSearchData.searchCurrentState = JavaFF.computeInitialState(this.sharedSearchData.groundProblem);
      this.sharedSearchData.goalReached = false;
      this.sharedSearchData.intervalSearchMS = intervalSearchMS > 100? intervalSearchMS : 100;

      // start search thread from initial state and init. search data
      if(this.searchThread == null || !this.searchThread.isAlive()){
        //search thread not alive
        this.searchThread = new SearchThread(sharedSearchData, planPublisher, debug);
        this.searchThread.start();
      }

    }
   
}