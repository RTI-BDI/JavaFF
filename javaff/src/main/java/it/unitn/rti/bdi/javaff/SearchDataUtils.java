package it.unitn.rti.bdi.javaff;

import java.util.ArrayList;
import java.util.StringTokenizer;  
import java.util.TreeSet;
import java.util.HashSet;
import java.util.Iterator;

import java.math.BigDecimal;

import javaff.data.strips.Proposition;
import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;
import javaff.data.temporal.DurativeAction;
import javaff.data.temporal.SplitInstantAction;

import javaff.scheduling.MatrixSTN;

import javaff.planning.TemporalMetricState;

public class SearchDataUtils {
  
  /*
   * Build from planString a Plansys2 Plan msg containing start time, action & duration for each action to be executed in the plan
   * 
   * planString has to contain line by line all actions to execute in this format "time: (action p1 p2 p3) [duration]"
  */
  public static plansys2_msgs.msg.Plan buildPsys2Plan(String planString){
    plansys2_msgs.msg.Plan psys2Plan = new plansys2_msgs.msg.Plan();
    ArrayList<plansys2_msgs.msg.PlanItem> planItems = new ArrayList<>();

    StringTokenizer st = new StringTokenizer(planString,"\n");  
    while (st.hasMoreTokens()) { 
        // build empty plansys2 plan item obj 
        plansys2_msgs.msg.PlanItem planItem = new plansys2_msgs.msg.PlanItem();

        // retrieve next line of the plan
        String planItemString = st.nextToken();  
        
        // retrieve indexes for time, action, duration substrings
        int endOfTimeIndex = planItemString.indexOf(":");

        int startOfActionIndex = planItemString.indexOf("(");
        int endOfActionIndex = planItemString.indexOf(")")+1;

        int startDurationIndex = planItemString.indexOf("[")+1;
        int endDurationIndex = planItemString.indexOf("]");

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
   * Build from timestamped plan a Plansys2 Plan msg containing start time, action & duration for each action to be executed in the plan
   * 
   * tsp contains ordered timestamped actions with start time, action & duration
  */
  public static plansys2_msgs.msg.Plan buildPsys2Plan(TimeStampedPlan tsp){
    plansys2_msgs.msg.Plan psys2Plan = new plansys2_msgs.msg.Plan();
    ArrayList<plansys2_msgs.msg.PlanItem> planItems = new ArrayList<>();

    for(TimeStampedAction tsa : ((TreeSet<TimeStampedAction>)tsp.getSortedActions())){ 
        // build empty plansys2 plan item obj 
        plansys2_msgs.msg.PlanItem planItem = new plansys2_msgs.msg.PlanItem();

        // retrive actual action, duration, time data from plan line, converting them directly to correct type
        float time =  tsa.time.floatValue();        
        String action = "(" + tsa.action.toString() + ")";
        float duration = tsa.duration.floatValue();

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
  public static ArrayList<ros2_bdi_interfaces.msg.Belief> getTrueBeliefs(TemporalMetricState state)
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
  public static ros2_bdi_interfaces.msg.ConditionsDNF getCurrentStatePreconditions(TemporalMetricState state)
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

  public static ros2_bdi_interfaces.msg.Desire buildTarget(
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
   * Create a msg with newly computed plan (to be updated back again to contain all partially computed plan in the search 
   * from the global initial and the global target)
  */
  public static javaff_interfaces.msg.PartialPlan buildNewPPlan(
    short searchIteration,
    ros2_bdi_interfaces.msg.Desire fulfillingDesire,
    ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions, 
    TemporalMetricState currentState, 
    plansys2_msgs.msg.Plan currentPlanMsg){
    
    float maxEndTime = -1.0F;
    javaff_interfaces.msg.PartialPlan newPPlan = new javaff_interfaces.msg.PartialPlan();
    ArrayList<plansys2_msgs.msg.PlanItem> newItems = new ArrayList<plansys2_msgs.msg.PlanItem>();
    
    // Iterate for new items to compose a partial plan
    for(plansys2_msgs.msg.PlanItem newItem : currentPlanMsg.getItems())
    {  
      newItems.add(newItem);
      if(newItem.getTime() + newItem.getDuration() > maxEndTime)
        maxEndTime = newItem.getTime() + newItem.getDuration();//new plan deadline
    }

    if(!newItems.isEmpty()){//found some new item, i.e. a new partial plan to be executed will be added
      newPPlan.getPlan().setPlanIndex(searchIteration);
      newPPlan.getPlan().setItems(newItems);
      newPPlan.setTarget(buildTarget(searchIteration, planPreconditions, currentState, fulfillingDesire, maxEndTime));
    }

    return newPPlan;
  }

  /*
   * Compute next committed state with no open action based on received notification of started action
   * null if not possible or any errors arise
  */
  public static TemporalMetricState computeNextCommittedState(TemporalMetricState lastCommittedState, String actionStarted, TimeStampedPlan tsp){

    //compute action started time and remove parenthesis and time info from its string
    BigDecimal actionStartedTime = BigDecimal.valueOf(Float.parseFloat(actionStarted.substring(actionStarted.lastIndexOf(":")+1))/1000.0f).setScale(MatrixSTN.SCALE,MatrixSTN.ROUND);
    actionStarted = actionStarted.substring(actionStarted.indexOf("(")+1, actionStarted.lastIndexOf(")"));
  
    TemporalMetricState currCommittedState = (TemporalMetricState) lastCommittedState.clone();
  
    //find currTime of execution corresponding to start time of actionStarted
    BigDecimal currTime = BigDecimal.ZERO;
  
    //Find time stamped action in the timestamped plan and apply its start snap action
    for(TimeStampedAction tsa : ((TreeSet<TimeStampedAction>)tsp.getSortedActions()))
      if(actionStartedTime.compareTo(tsa.time) == 0 && actionStarted.equals(((DurativeAction)tsa.action).toString()))
      {
        currTime = tsa.time;//found starting time
        currCommittedState = (TemporalMetricState) currCommittedState.apply(((DurativeAction) tsa.action).startAction);//apply start instant snap action
        break;
      }

    if(currTime.equals(BigDecimal.ZERO) || currCommittedState.openActions.isEmpty())//error actionStarted not found in tsp -> cannot compute nextCommittedState
      return null;
  
    //Apply all instant snap actions ordered by predicted time that are above the currTime in the simulation and stop as soon as you reach a state with no open actions
    Iterator<SplitInstantAction> itsa = ((TreeSet<SplitInstantAction>)tsp.getSortedSplitInstantActions()).iterator();
    while(itsa.hasNext() && !currCommittedState.openActions.isEmpty())
    {
      SplitInstantAction sia = itsa.next();
      if(sia.predictedInstant.compareTo(currTime) > 0)
      {
        currTime = sia.predictedInstant;
        currCommittedState = (TemporalMetricState) currCommittedState.apply(sia);
      }
    }
  
    if(!currCommittedState.openActions.isEmpty())
      return null;
    else
      return currCommittedState;
  }


  /*
   * Create a special msg when we meet an unexpected state PPlans[Plan[ PlanItem{-1, "", -1} ]]
  */
  public static javaff_interfaces.msg.SearchResult buildUnsatSearchResult(){
    javaff_interfaces.msg.SearchResult searchResult = new javaff_interfaces.msg.SearchResult();
    searchResult.setStatus(searchResult.FAILED);
    return searchResult;
  }
}
