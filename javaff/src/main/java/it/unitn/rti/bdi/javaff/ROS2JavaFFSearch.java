package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;

// import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Hashtable;
import java.util.StringTokenizer;  


import org.ros2.rcljava.node.BaseComposableNode;

import javaff.JavaFF;
import javaff.data.GroundProblem;
import javaff.planning.State;
import javaff.planning.TemporalMetricState;

import plansys2_msgs.msg.Plan;
import plansys2_msgs.msg.PlanItem;

class SearchThread extends Thread{

  private boolean killMySelf = false;
  private SharedSearchData sharedSearchData;
  private Publisher<javaff_interfaces.msg.PartialPlans> planPublisher;

  public SearchThread(SharedSearchData sharedSearchData, Publisher<javaff_interfaces.msg.PartialPlans> planPublisher){
    super();
    this.sharedSearchData = sharedSearchData;
    this.planPublisher = planPublisher;
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
        // System.out.println("\""+time+"\"");
        // System.out.println("\""+action+"\"");
        // System.out.println("\""+duration+"\"");

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
   * Check diff between old javaff_interfaces.msg.PartialPlans msg and newly instantiated plansys2_msgs.msg.Plan msg received from the search
   * to produce a new array of partial plans. In the general case, the append of the new plan will suffice, but when a completely new path has been computed,
   * old published paths that are still not executed might need to be replaced
  */
  private javaff_interfaces.msg.PartialPlans buildNewPartialPlansMsg(javaff_interfaces.msg.PartialPlans oldPartialPlansMsg, plansys2_msgs.msg.Plan currentPlanMsg){
    float maxStartTime = -1.0F;
    plansys2_msgs.msg.Plan newPPlan = new plansys2_msgs.msg.Plan();
    ArrayList<plansys2_msgs.msg.PlanItem> newItems = new ArrayList<plansys2_msgs.msg.PlanItem>();
    
    // Iterate over old partial plans, determining highest start time so that it can be used as a lower bound after to identify the newly computed actions
    for(plansys2_msgs.msg.Plan oldPlan : oldPartialPlansMsg.getPlans())
      for(plansys2_msgs.msg.PlanItem oldItem : oldPlan.getItems())
        if(oldItem.getTime() > maxStartTime)
          maxStartTime = oldItem.getTime();
    
    // Iterate for new items to compose a partial plan
    for(plansys2_msgs.msg.PlanItem newItem : currentPlanMsg.getItems())
      if(newItem.getTime() > maxStartTime)
        newItems.add(newItem);
    
    if(!newItems.isEmpty()){//found some new item, i.e. a new partial plan to be executed will be added
      newPPlan.setItems(newItems);
      oldPartialPlansMsg.getPlans().add(newPPlan);
    }

    return oldPartialPlansMsg;
  }

  public void killMySelf(){killMySelf = true;}
  
  public void run(){
    this.sharedSearchData.open.clear();

    int i = 0;
    boolean unsat = false;

    while(!this.sharedSearchData.currentState.goalReached() && !unsat){
      
      System.out.println("\n\n ROUND " + (i++));
      this.sharedSearchData.searchLock.lock();

        // move forward with the search for 500ms
        this.sharedSearchData.currentState = (TemporalMetricState) JavaFF.performFFSearch(this.sharedSearchData.currentState, 500, this.sharedSearchData.open, this.sharedSearchData.closed);
        
        // build plan string from currentState
        String planString = JavaFF.buildPlan(this.sharedSearchData.groundProblem, this.sharedSearchData.currentState);
        System.out.println(planString);
        System.out.println("open.size="+this.sharedSearchData.open.size() + "\t closed.size=" + this.sharedSearchData.closed.size());

        // build plan msg and publish it
        plansys2_msgs.msg.Plan currentPlanMsg = buildPsys2Plan(planString);
        
        this.sharedSearchData.partialPlansMsg = buildNewPartialPlansMsg(this.sharedSearchData.partialPlansMsg, currentPlanMsg);
        
        this.planPublisher.publish(this.sharedSearchData.partialPlansMsg);

        //check whether unsat ~ empty open and search has return null
        unsat = this.sharedSearchData.open.isEmpty() && this.sharedSearchData.currentState == null;

      this.sharedSearchData.searchLock.unlock();
      if(killMySelf)//if true mspawner thread has set it, put it again to false and terminate your execution
        return;

    }
  }

}

class SharedSearchData{
  GroundProblem groundProblem;
  TemporalMetricState currentState;

  javaff_interfaces.msg.PartialPlans partialPlansMsg = new javaff_interfaces.msg.PartialPlans();
  
  LinkedList<State> open = new LinkedList<>();
  Hashtable<Integer, State> closed = new Hashtable<>();
  ReentrantLock searchLock = new ReentrantLock(true);
}

public class ROS2JavaFFSearch extends BaseComposableNode{
    
    // Sibling node handling service request wrt. online planning
    private ROS2JavaFFServer serverNode;
    
    private SearchThread searchThread;

    private SharedSearchData sharedSearchData;

    private Publisher<javaff_interfaces.msg.PartialPlans> planPublisher;

    // private AtomicBoolean killSearchThread = new AtomicBoolean(false);

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    public ROS2JavaFFSearch(String name) {
      super(name);
      this.sharedSearchData = new SharedSearchData();
      this.planPublisher = this.node.<javaff_interfaces.msg.PartialPlans>createPublisher(javaff_interfaces.msg.PartialPlans.class, "plan");
      System.out.println("ns=" + this.node.getNamespace());
      // System.out.println("context= " + this.node.getContext());
    } 

    public OperationResult startSearch(String domain, String problem){
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

        // START NEW SEARCH

        // init again shared structures for search and partial plans composition (groundProblem and initial TMS are going to be computed just below)
        this.sharedSearchData.partialPlansMsg = new javaff_interfaces.msg.PartialPlans();
        this.sharedSearchData.open = new LinkedList<>();
        this.sharedSearchData.closed = new Hashtable<>();
        // parse domain and problem, unground + ground processes, returning the initial state
        this.sharedSearchData.groundProblem = JavaFF.computeGroundProblem(domain, problem);
        this.sharedSearchData.currentState = JavaFF.computeInitialState(this.sharedSearchData.groundProblem);

        // start search thread from initial state and init. search data
        this.searchThread = new SearchThread(sharedSearchData, planPublisher);
        this.searchThread.start();

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
      }
      
      return returnObj;
    }

    public void unexpectedState(String newState){
      this.sharedSearchData.searchLock.lock();
        this.sharedSearchData.open.clear();
        //this.sharedSearchData.open.add(newState);//TODO va fatto
      this.sharedSearchData.searchLock.unlock();
    }
   
}