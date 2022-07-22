package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;
import org.ros2.rcljava.subscription.Subscription;

// import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Hashtable;
import java.util.Set;
import java.util.TreeSet;
import java.util.HashSet;
import java.util.StringTokenizer;  


import org.ros2.rcljava.node.BaseComposableNode;

import javaff.JavaFF;
import javaff.data.GroundProblem;
import javaff.planning.State;
import javaff.planning.TemporalMetricState;
import javaff.data.strips.Proposition;
import javaff.search.HValueComparator;

import plansys2_msgs.msg.Plan;
import plansys2_msgs.msg.PlanItem;

enum FFSearchStatus{EHC_SEARCHING, EHC_FAILED, BFS_SEARCHING, UNSAT};

class SearchThread extends Thread{

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

  /*
   * Build from planString a Plansys2 Plan msg containing start time, action & duration for each action to be executed in the plan
   * 
   * planString has to contain line by line all actions to execute in this format "time: (action p1 p2 p3) [duration]"
  */
  private plansys2_msgs.msg.Plan buildPsys2Plan(String planString){
    plansys2_msgs.msg.Plan psys2Plan = new plansys2_msgs.msg.Plan();
    ArrayList<plansys2_msgs.msg.PlanItem> planItems = new ArrayList<>();

    StringTokenizer st = new StringTokenizer(planString,"\n");  
    while (st.hasMoreTokens()) { 
        // build empty plansys2 plan item obj 
        plansys2_msgs.msg.PlanItem planItem = new plansys2_msgs.msg.PlanItem();

        // retrieve next line of the plan
        String planItemString = st.nextToken();  
        
        // retrieve indexes for time, action, duration substrings
        int endOfTimeIndex = planItemString.indexOf(":") - 1;

        int startOfActionIndex = planItemString.indexOf("(");
        int endOfActionIndex = planItemString.indexOf(")")+1;

        int startDurationIndex = planItemString.indexOf("[")+1;
        int endDurationIndex = planItemString.indexOf("]")-1;

        // retrive actual action, duration, time data from plan line, converting them directly to correct type
        float time =  Float.valueOf(planItemString.substring(0, endOfTimeIndex)).floatValue();
        String action = planItemString.substring(startOfActionIndex, endOfActionIndex);
        float duration = Float.valueOf(planItemString.substring(startDurationIndex, endDurationIndex)).floatValue();

        // Set data to planItem and add it to items in plansys2 Plan.items msg
        planItem.setTime(time);
        planItem.setAction(action);
        planItem.setDuration(duration);
        planItems.add(planItem);
    }  

    psys2Plan.setItems(planItems);
    return psys2Plan;
  }

  /*
   * Return all domain defined predicates that are true (i.e. in facts) in the passed state as params for the method
   * Not domain defined predicates are the one indicating completed actions (e.g. gmove... if action move is defined)
  */
  ArrayList<ros2_bdi_interfaces.msg.Belief> getTrueBeliefs(TemporalMetricState state)
  {
    ArrayList<ros2_bdi_interfaces.msg.Belief> beliefs = new ArrayList<ros2_bdi_interfaces.msg.Belief>();
    for(Proposition p : ((HashSet<Proposition>)state.facts))
      if(p.isDomainDefined())
      {
        ros2_bdi_interfaces.msg.Belief belief = new ros2_bdi_interfaces.msg.Belief();
        belief.setName(p.getName());
        belief.setPddlType(belief.PREDICATE_TYPE);
        belief.setParams(p.getStringParameters());
        beliefs.add(belief);
      }
    
    return beliefs;
  }

  /*
   * Return all domain defined predicates that are true (i.e. in facts) in the passed state as params for the method
   * packing them as a preconditions DNF expression (having just one clause with all predicates in end)
  */
  ros2_bdi_interfaces.msg.ConditionsDNF getCurrentStatePreconditions(TemporalMetricState state)
  {
    ros2_bdi_interfaces.msg.ConditionsDNF preconditions = new ros2_bdi_interfaces.msg.ConditionsDNF();
    ArrayList<ros2_bdi_interfaces.msg.ConditionsConjunction> clauses = new ArrayList<ros2_bdi_interfaces.msg.ConditionsConjunction>();
    ros2_bdi_interfaces.msg.ConditionsConjunction clause = new ros2_bdi_interfaces.msg.ConditionsConjunction();
    ArrayList<ros2_bdi_interfaces.msg.Condition> literals = new ArrayList<ros2_bdi_interfaces.msg.Condition>();
    
    ArrayList<ros2_bdi_interfaces.msg.Belief> trueBeliefs = getTrueBeliefs(state);
    for(ros2_bdi_interfaces.msg.Belief belief : trueBeliefs)
    {
      ros2_bdi_interfaces.msg.Condition condition = new ros2_bdi_interfaces.msg.Condition();
      condition.setConditionToCheck(belief);
      condition.setCheck(condition.TRUE_CHECK);
      literals.add(condition);
    }
    clause.setLiterals(literals);
    clauses.add(clause);
    preconditions.setClauses(clauses);

    return preconditions;
  }

  private ros2_bdi_interfaces.msg.Desire buildTarget(
      int searchIteration,
      ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions,
      TemporalMetricState targetState,
      ros2_bdi_interfaces.msg.Desire fulfillingDesire,
      float planDeadline){

    ros2_bdi_interfaces.msg.Desire target = new ros2_bdi_interfaces.msg.Desire(); 
    
    // name = fulfillingDesire name concat. to reached search iteration
    target.setName(fulfillingDesire.getName() + searchIteration);

    //retrieve preconditions
    target.setPrecondition(planPreconditions);

    //retrieve goal value
    target.setValue(getTrueBeliefs(targetState));

    // set priority and deadline
    target.setPriority(fulfillingDesire.getPriority());
    target.setDeadline(planDeadline);
    
    return target;
  }

  /*
   * Create a msg with newly computed plan (to be updated back again to contain all partially computed plan in the search from the global initial and the global target)
  */
  private javaff_interfaces.msg.PartialPlan buildNewPPlan(
    short searchIteration,
    ros2_bdi_interfaces.msg.Desire fulfillingDesire,
    ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions, 
    TemporalMetricState currentState, 
    plansys2_msgs.msg.Plan currentPlanMsg){
    
    float maxEndTime = -1.0F;
    javaff_interfaces.msg.PartialPlan newPPlan = new javaff_interfaces.msg.PartialPlan();
    ArrayList<plansys2_msgs.msg.PlanItem> newItems = new ArrayList<plansys2_msgs.msg.PlanItem>();
    
    newPPlan.setIndex(searchIteration);

    // Iterate over old partial plans, determining highest start time so that it can be used as a lower bound after to identify the newly computed actions
    // for(plansys2_msgs.msg.Plan oldPlan : oldPartialPlansMsg.getPlans())
    //   for(plansys2_msgs.msg.PlanItem oldItem : oldPlan.getItems())
    //     if(oldItem.getTime() > maxStartTime)
    //       maxStartTime = oldItem.getTime();
    
    // Iterate for new items to compose a partial plan
    for(plansys2_msgs.msg.PlanItem newItem : currentPlanMsg.getItems())
    {  
      newItems.add(newItem);
      if(newItem.getTime() + newItem.getDuration() > maxEndTime)
        maxEndTime = newItem.getTime() + newItem.getDuration();//new plan deadline
    }

    if(!newItems.isEmpty()){//found some new item, i.e. a new partial plan to be executed will be added
      newPPlan.getPlan().setItems(newItems);
      newPPlan.setTarget(buildTarget(searchIteration, planPreconditions, currentState, fulfillingDesire, maxEndTime));
      //oldPartialPlansMsg.getPlans().add(newPPlan);
    }

    return newPPlan;
  }

  /*
   * Create a special msg when we meet an unexpected state PPlans[Plan[ PlanItem{-1, "", -1} ]]
  */
  private javaff_interfaces.msg.SearchResult buildUnsatSearchResult(){
    javaff_interfaces.msg.SearchResult searchResult = new javaff_interfaces.msg.SearchResult();
    searchResult.setStatus(searchResult.FAILED);
    return searchResult;
  }

  public void killMySelf(){killMySelf = true;}
  
  public void run(){
    this.sharedSearchData.open.clear();

    short i = 0;
    FFSearchStatus ffstatus = FFSearchStatus.EHC_SEARCHING;

    //clear search result
    this.sharedSearchData.searchResultMsg = new javaff_interfaces.msg.SearchResult();

    while(ffstatus != FFSearchStatus.UNSAT && !this.sharedSearchData.currentState.goalReached()){
      
      this.sharedSearchData.searchLock.lock();

        // retrieve preconditions from currentState that are going to apply for found plan
        ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions = getCurrentStatePreconditions(this.sharedSearchData.currentState);

        // move forward with the search for interval search time
        TemporalMetricState goalOrIntermediateState = (ffstatus == FFSearchStatus.EHC_SEARCHING)?
          (TemporalMetricState) JavaFF.performEHCSearch(this.sharedSearchData.currentState, this.sharedSearchData.intervalSearchMS, this.sharedSearchData.open, this.sharedSearchData.closed)
          :
          (TemporalMetricState) JavaFF.performBFSSearch(this.sharedSearchData.currentState, this.sharedSearchData.intervalSearchMS, this.sharedSearchData.open, this.sharedSearchData.closed);

        //check whether unsat ~ empty open and search has return null
        if(ffstatus == FFSearchStatus.EHC_SEARCHING)
        {
          ffstatus = (this.sharedSearchData.open.isEmpty() && goalOrIntermediateState == null)? FFSearchStatus.BFS_SEARCHING : FFSearchStatus.EHC_SEARCHING;
          
          if(ffstatus == FFSearchStatus.BFS_SEARCHING)//just switched to BFS searching
            this.sharedSearchData.closed.remove(new Integer(this.sharedSearchData.currentState.hashCode()));// prevent BFS fails immediately: current was already explored in last EHC search
        
        }else if(ffstatus == FFSearchStatus.BFS_SEARCHING)
          ffstatus = (this.sharedSearchData.open.isEmpty() && goalOrIntermediateState == null)? FFSearchStatus.UNSAT : FFSearchStatus.EHC_SEARCHING;

        if(ffstatus != FFSearchStatus.UNSAT && goalOrIntermediateState != null){
          this.sharedSearchData.currentState = goalOrIntermediateState;
          // build plan string from currentState
          String planString = JavaFF.buildPlan(this.sharedSearchData.groundProblem, this.sharedSearchData.currentState);
          if(debug){  
            System.out.println("\n\n ROUND " + (i));
            System.out.println(planString);
            System.out.println("open.size="+this.sharedSearchData.open.size() + "\t closed.size=" + this.sharedSearchData.closed.size());
          }
          // build plan msg and publish it
          plansys2_msgs.msg.Plan currentPlanMsg = buildPsys2Plan(planString);
          
          if(currentPlanMsg.getItems().size() > 0)// build+pub new plan msg and rebase iff 
          {
            javaff_interfaces.msg.PartialPlan newPPlan = buildNewPPlan(i, this.sharedSearchData.fulfillingDesire,
              planPreconditions, this.sharedSearchData.currentState, currentPlanMsg);
            
            //Search result containing all pplans up to now within this search
            this.sharedSearchData.searchResultMsg.getPlans().add(newPPlan);
            this.sharedSearchData.searchResultMsg.setStatus(this.sharedSearchData.currentState.goalReached()? this.sharedSearchData.searchResultMsg.SUCCESS : this.sharedSearchData.searchResultMsg.SEARCHING);
            
            if(!killMySelf)//these search results are still valid
              this.planPublisher.publish(this.sharedSearchData.searchResultMsg);
            
            // rebase exclusively when plan presents some actions, otherwise next search cycle will start from where it left
            JavaFF.rebaseOnCurrentState(this.sharedSearchData.groundProblem, this.sharedSearchData.currentState, this.sharedSearchData.open, this.sharedSearchData.closed);
          }
        
        }
        

      this.sharedSearchData.searchLock.unlock();
      if(killMySelf)//if true mspawner thread has set it, put it again to false and terminate your execution
        return;

      i++;
    }

    if(ffstatus == FFSearchStatus.UNSAT){
      //could return and pub. this: PPlans[Plan[ PlanItem{-1, "", -1} ]]  
      this.planPublisher.publish(buildUnsatSearchResult());
    }
  }

}

// Search data shared among ROS2JavaFFSearch and SearchThread running instance
class SharedSearchData{
  GroundProblem groundProblem;
  TemporalMetricState currentState;

  ros2_bdi_interfaces.msg.Desire fulfillingDesire;

  javaff_interfaces.msg.SearchResult searchResultMsg = new javaff_interfaces.msg.SearchResult();
  
  TreeSet<State> open = new TreeSet<>(new HValueComparator());
  Hashtable<Integer, State> closed = new Hashtable<>();
  ReentrantLock searchLock = new ReentrantLock(true);

  int intervalSearchMS;
}

public class ROS2JavaFFSearch extends BaseComposableNode{
    
    // Sibling node handling service request wrt. online planning
    private ROS2JavaFFServer serverNode;
    
    private SearchThread searchThread;

    private SharedSearchData sharedSearchData;

    private Publisher<javaff_interfaces.msg.SearchResult> planPublisher;

    private Subscription<javaff_interfaces.msg.ExecutionStatus> execStatusSubscriber;

    private String domain;

    private boolean debug;

    // private AtomicBoolean killSearchThread = new AtomicBoolean(false);

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    public ROS2JavaFFSearch(String name, String namespace, String domain, boolean debug) {
      super(name, namespace);
      this.domain = domain;
      this.debug = debug;

      this.sharedSearchData = new SharedSearchData();
      this.planPublisher = this.node.<javaff_interfaces.msg.SearchResult>createPublisher(javaff_interfaces.msg.SearchResult.class, name + "/plan");
      this.execStatusSubscriber = this.node.<javaff_interfaces.msg.ExecutionStatus>createSubscription(
        javaff_interfaces.msg.ExecutionStatus.class, 
        name + "/exec_status",
        msg -> System.out.println("I heard: action " + msg.getExecutingAction() + " of plan with i = " + msg.getExecutingPlanIndex() + " is executing"));
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

        // START NEW SEARCH
        this.sharedSearchData.fulfillingDesire = fulfillingDesire;
        // init again shared structures for search and partial plans composition (groundProblem and initial TMS are going to be computed just below)
        this.sharedSearchData.searchResultMsg = new javaff_interfaces.msg.SearchResult();
        
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
      // GroundProblem newGroundProblem =  JavaFF.computeGroundProblem(this.domain, newStatePddlProblem);
      // TemporalMetricState newState = JavaFF.computeInitialState(newGroundProblem);

      this.sharedSearchData.searchLock.lock();
      try{
        //TODO advanced: check whether newState is in open or closed (for now they empty: pointless)
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
      this.sharedSearchData.currentState = JavaFF.computeInitialState(this.sharedSearchData.groundProblem);

      this.sharedSearchData.intervalSearchMS = intervalSearchMS > 100? intervalSearchMS : 100;

      // start search thread from initial state and init. search data
      if(this.searchThread == null || !this.searchThread.isAlive()){
        //search thread not alive
        this.searchThread = new SearchThread(sharedSearchData, planPublisher, debug);
        this.searchThread.start();
      }

    }
   
}