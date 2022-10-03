package it.unitn.rti.bdi.javaff;

import java.util.ArrayList;
import java.util.StringTokenizer;  
import java.util.Set;
import java.util.TreeSet;
import java.util.HashSet;
import java.util.Hashtable;
import java.util.Iterator;

import java.math.BigDecimal;
import static java.lang.Math.max;

import javaff.JavaFF;

import javaff.data.strips.AND;
import javaff.data.strips.Proposition;
import javaff.data.metric.ResourceOperator;
import javaff.data.metric.NamedFunction;
import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;
import javaff.data.temporal.DurativeAction;
import javaff.data.temporal.SplitInstantAction;
import javaff.data.temporal.StartInstantAction;
import javaff.data.temporal.EndInstantAction;
import javaff.data.GroundProblem;

import javaff.scheduling.MatrixSTN;

import javaff.planning.TemporalMetricState;

import it.unitn.rti.bdi.javaff.TimeStampedPlanWithSearchBaseline;

public class SearchDataUtils {
  
  public static void printSearchBaseline(javaff_interfaces.msg.CommittedStatus searchBaseline){
    System.out.println("SB: executing_plan_index: " + searchBaseline.getExecutingPlanIndex());
    System.out.println("Committed actions: ");
    for(javaff_interfaces.msg.ActionCommittedStatus acs : searchBaseline.getCommittedActions())
      System.out.println(acs.getCommitted() + "\t" + acs.getCommittedAction() + ":" + acs.getPlannedStartTime());
    
  }

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
    return getTrueBeliefs(state, false);
  }

  public static ArrayList<ros2_bdi_interfaces.msg.Belief> getTrueBeliefs(TemporalMetricState state, boolean justPredicates)
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
    
    if(!justPredicates)
      for(NamedFunction nf : ((Hashtable<NamedFunction, BigDecimal>)state.funcValues).keySet())
        if(nf.getPredicateSymbol().isDomainDefined())
        {
          ros2_bdi_interfaces.msg.Belief belief = new ros2_bdi_interfaces.msg.Belief();
          belief.setName(nf.getPredicateSymbol().getName());
          belief.setPddlType(belief.FUNCTION_TYPE);
          belief.setParams(nf.getStringParameters());
          belief.setValue(((Hashtable<NamedFunction, BigDecimal>)state.funcValues).get(nf).floatValue());
          beliefs.add(belief);
        }
    
    return beliefs;
  }

  private static boolean findBelief(ros2_bdi_interfaces.msg.Belief belief, ArrayList<ros2_bdi_interfaces.msg.Belief> beliefsList)
  {
    // assuming I will not find it in beliefsList
    boolean beliefFound = false;

    // Looking for it in beliefsList
    for(ros2_bdi_interfaces.msg.Belief searchBelief : beliefsList) {
      if(belief.getPddlType() == searchBelief.getPddlType() && belief.getName() == searchBelief.getName())
      {
        // Same NAME, same PDDLTYPE
        if(belief.getParams().size() == searchBelief.getParams().size())
        {
          // Same number of arguments
          if(belief.getPddlType() != belief.FUNCTION_TYPE || belief.getValue() == searchBelief.getValue())
          {
            // Same function value (if meaningful)
            boolean matchingParams = true;
            for(int i=0; matchingParams && i<belief.getParams().size(); i++)
              if(!belief.getParams().get(i).equals(searchBelief.getParams().get(i)))
                matchingParams = false;
            
            if(matchingParams)// if also params are matching, belief already found in previousBeliefs -> no reason to go on looking for it
              return true;
          }
        }
      } 
    }
    //End of linear looking for belief in previousBeliefs -> not found it
    return false;
  }

  /*
   * Return the beliefs given by the difference between currentBeliefs/previousBeliefs
   * return currentBeliefs if previousBeliefs is null
  */
  public static ArrayList<ros2_bdi_interfaces.msg.Belief> filterStatePrecondition(
    ArrayList<ros2_bdi_interfaces.msg.Belief> currentBeliefs, 
    ArrayList<ros2_bdi_interfaces.msg.Belief> previousBeliefs)
  {
    if(previousBeliefs == null)
      return currentBeliefs;

    // empty list of filtered beliefs
    ArrayList<ros2_bdi_interfaces.msg.Belief> filteredBeliefs = new ArrayList<ros2_bdi_interfaces.msg.Belief>();

    // for each currentBeliefs which is not in previousBeliefs push it to filteredBeliefs
    for(ros2_bdi_interfaces.msg.Belief belief : currentBeliefs)
      if(!findBelief(belief, previousBeliefs))
        filteredBeliefs.add(belief);

    return filteredBeliefs;    
  }

  /*
   * Return all domain defined predicates that are true (i.e. in facts) in the passed state as params for the method
   * packing them as a preconditions DNF expression (having just one clause with all predicates in end)
  */
  public static ros2_bdi_interfaces.msg.ConditionsDNF getCurrentStatePreconditions(TemporalMetricState state)
  {
    return buildPreconditions(getTrueBeliefs(state));
  }

  public static ArrayList<ros2_bdi_interfaces.msg.Belief> computeImplicitPreconditions(TimeStampedPlan tsp)
  {
    ArrayList<ros2_bdi_interfaces.msg.Belief> implicitPreconditions = new ArrayList<ros2_bdi_interfaces.msg.Belief>();
    //TODO AZ da TV: insert function logic in SearchDataUtils so that it can be used in planPreconditions computation
    
    
    //MatrixSTN.EPSILON;
    return implicitPreconditions;
  }

  /*
   * Return a ConditionsDNF msg packing all trueBeliefs in an AND clause
  */
  public static ros2_bdi_interfaces.msg.ConditionsDNF buildPreconditions(ArrayList<ros2_bdi_interfaces.msg.Belief> trueBeliefs)
  {
    ros2_bdi_interfaces.msg.ConditionsDNF preconditions = new ros2_bdi_interfaces.msg.ConditionsDNF();
    ArrayList<ros2_bdi_interfaces.msg.ConditionsConjunction> clauses = new ArrayList<ros2_bdi_interfaces.msg.ConditionsConjunction>();
    ros2_bdi_interfaces.msg.ConditionsConjunction clause = new ros2_bdi_interfaces.msg.ConditionsConjunction();
    ArrayList<ros2_bdi_interfaces.msg.Condition> literals = new ArrayList<ros2_bdi_interfaces.msg.Condition>();

    for(ros2_bdi_interfaces.msg.Belief belief : trueBeliefs)
    {
      ros2_bdi_interfaces.msg.Condition condition = new ros2_bdi_interfaces.msg.Condition();
      condition.setConditionToCheck(belief);
      condition.setCheck(belief.getPddlType() == belief.PREDICATE_TYPE? condition.TRUE_CHECK : condition.EQUALS_CHECK);
      literals.add(condition);
    }
    clause.setLiterals(literals);
    clauses.add(clause);
    preconditions.setClauses(clauses);

    return preconditions;
  }
  /*
   * TODO not in use it now, rest there for future iterations as pplan preconditions
  */
  // public static ros2_bdi_interfaces.msg.Desire buildTarget(
  //     int searchIteration,
  //     ros2_bdi_interfaces.msg.ConditionsDNF planPreconditions,
  //     TemporalMetricState targetState,
  //     ros2_bdi_interfaces.msg.Desire fulfillingDesire,
  //     float planDeadline){

  //   ros2_bdi_interfaces.msg.Desire target = new ros2_bdi_interfaces.msg.Desire(); 
    
  //   // name = fulfillingDesire name concat. to reached search iteration
  //   target.setName(fulfillingDesire.getName() + searchIteration);

  //   //retrieve preconditions
  //   target.setPrecondition(planPreconditions);

  //   //retrieve goal value
  //   target.setValue(getTrueBeliefs(targetState, true));

  //   // set priority and deadline
  //   target.setPriority(fulfillingDesire.getPriority());
  //   target.setDeadline(planDeadline);
    
  //   return target;
  // }

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
      //newPPlan.setTarget(buildTarget(searchIteration, planPreconditions, currentState, fulfillingDesire, maxEndTime));
      newPPlan.setTarget(fulfillingDesire);
    }

    return newPPlan;
  }

  /*
    Compute max committed number of completed and sequential durative actions (SplitInstantAction start + end) in orderedCommittedSplitInstantActions starting from currIndex
  */ 
  private static int computeMaxCommittedSequenceI(ArrayList<SplitInstantAction> orderedCommittedSplitInstantActions, int currIndex){
    if(currIndex >= orderedCommittedSplitInstantActions.size())//already overflowed arrat
      return 0;

    SplitInstantAction sia = orderedCommittedSplitInstantActions.get(currIndex);

    if(currIndex == orderedCommittedSplitInstantActions.size() - 1)
      if(sia instanceof StartInstantAction)
        return 0; // reached the end of the chain with a StartInstantAction
      else
        return 1; // reached the end of the chain with an EndInstantAction -> count this steps
        
    else
      if(sia instanceof StartInstantAction)
        return 0 + computeMaxCommittedSequenceI(orderedCommittedSplitInstantActions, currIndex + 1);
      else
      {
        int maxStepsAfter = 0;
        for(int i=currIndex+1; i<orderedCommittedSplitInstantActions.size(); i++)
        {
          SplitInstantAction followingSia = orderedCommittedSplitInstantActions.get(i);
          if(followingSia instanceof StartInstantAction && followingSia.predictedInstant.compareTo(sia.predictedInstant) > 0)
            maxStepsAfter = Math.max(maxStepsAfter, computeMaxCommittedSequenceI(orderedCommittedSplitInstantActions, i));
        }
        return 1 + maxStepsAfter; // count this step and add it to the max number of steps after it
      }
  }

  /*
    Compute max committed number of completed and sequential durative actions (SplitInstantAction start + end) in orderedCommittedSplitInstantActions
  */ 
  private static int computeMaxCommittedSequence(ArrayList<SplitInstantAction> orderedCommittedSplitInstantActions){
    if(orderedCommittedSplitInstantActions.size() == 0)
      return 0;
    else
      return computeMaxCommittedSequenceI(orderedCommittedSplitInstantActions, 0);
  }

  /*
   * Compute next committed state with no open action based on received notification of started action
   * null if not possible or any errors arise
  */
  public static TemporalMetricState computeNextCommittedState(TemporalMetricState lastCommittedState, String actionStarted, TimeStampedPlan tsp, int minCommitSteps){

    //compute action started time and remove parenthesis and time info from its string
    BigDecimal actionStartedTime = BigDecimal.valueOf(Float.parseFloat(actionStarted.substring(actionStarted.lastIndexOf(":")+1))/1000.0f).setScale(MatrixSTN.SCALE,MatrixSTN.ROUND);
    actionStarted = actionStarted.substring(actionStarted.indexOf("(")+1, actionStarted.lastIndexOf(")"));
  
    TemporalMetricState currCommittedState = (TemporalMetricState) lastCommittedState.clone();
  
    // find currTime of execution corresponding to start time of actionStarted
    BigDecimal currTime = BigDecimal.ZERO;
    // all sia in tsp
    TreeSet<SplitInstantAction> orderedSplitInstantActions = (TreeSet<SplitInstantAction>)tsp.getSortedSplitInstantActions();
    // all sia in tsp which are going to mark as committed in this round
    ArrayList<SplitInstantAction> orderedCommittedSplitInstantActions = new ArrayList<>();

    //Find time stamped action in the timestamped plan and apply its start snap action
    Iterator<SplitInstantAction> itsa = orderedSplitInstantActions.iterator();
    boolean foundStartAction = false;
    while(itsa.hasNext() && !foundStartAction)
    {
      SplitInstantAction sia = itsa.next();
      if(actionStartedTime.compareTo(sia.predictedInstant) == 0 && actionStarted.equals((sia.parent).toString()))
      {
        foundStartAction = true;

        currTime = sia.predictedInstant;//found starting time
        if(!sia.alreadyApplied(currCommittedState))
        {
          if(sia.isApplicable(currCommittedState))
          {
            currCommittedState = (TemporalMetricState) currCommittedState.apply(sia);//apply start instant snap action
            orderedCommittedSplitInstantActions.add(sia);
            //System.out.println("computeNextCommittedState: " + sia.toString() + " applied");
          }
          else
          {
            return null;//sequence not applicable anymore, failure bound to arise
          }
        }
        else
        {
          return null;
          // System.out.println("COMPUTING nextCommittedState: " + sia.toString() + " ALREADY applied");
        }
      }
    }

    if(currTime.equals(BigDecimal.ZERO) || currCommittedState.openActions.isEmpty())//error actionStarted not found in tsp -> cannot compute nextCommittedState
      return null;
    
    // reset iterator to the start because it might be that you have to still apply actions that start at the same time, but which have been put before in the treeset
    itsa = orderedSplitInstantActions.iterator();

    int sequentialCommitCounter = 0; // count max number of sequentially committed actions

    // apply all instant snap actions ordered by predicted time that are above the currTime in the simulation and stop as soon as you reach a state with no open actions
    // and you've applied the min number of sequential steps required by the user (if possible)
    while(itsa.hasNext() && (sequentialCommitCounter < minCommitSteps || !currCommittedState.openActions.isEmpty()))
    {
      SplitInstantAction sia = itsa.next();
      if(sia.predictedInstant.compareTo(currTime) >= 0)//we can pass by equivalent matches (i.e. two actions starting at the same time)
      {
        currTime = sia.predictedInstant;
        if(!sia.alreadyApplied(currCommittedState))
        {
          if(sia.isApplicable(currCommittedState))
          {
            currCommittedState = (TemporalMetricState) currCommittedState.apply(sia);
            orderedCommittedSplitInstantActions.add(sia);
            //System.out.println("computeNextCommittedState: " + sia.toString() + " applied");
            if(sia instanceof EndInstantAction)
            {
              //mark the timestamped action in the tsp as committed for execution
              tsp.markCommitted(sia.parent, sia.parent.startAction.predictedInstant);
              
              sequentialCommitCounter = computeMaxCommittedSequence(orderedCommittedSplitInstantActions);
              //System.out.println("computeNextCommittedState: max committed sequence up to now: " + sequentialCommitCounter + "; striving for " + minCommitSteps);
            }
          }
          else
          {
            System.out.println("ERROR IN COMPUTING nextCommittedState: " + sia.toString() + " is not applicable");
            return null;//sequence not applicable anymore, failure bound to arise
          }
        }
        // else
        // {
        //   System.out.println("COMPUTING nextCommittedState: " + sia.toString() + " ALREADY applied");
        // }
      }
    }
  
    if(!currCommittedState.openActions.isEmpty())
      return null;
    
    // mark current instant in next committed state
    currCommittedState.currInstant = currTime;
    return currCommittedState;
  }

  public static javaff_interfaces.msg.CommittedStatus getSearchBaseline(TimeStampedPlanWithSearchBaseline executingTspWSB)
  {
    javaff_interfaces.msg.CommittedStatus searchBaseline = new javaff_interfaces.msg.CommittedStatus();
    
    // baseline default value, execution not started yet, therefore no valid index of executing action
    searchBaseline.setExecutingPlanIndex((short)-1);
    if(executingTspWSB != null)
    {
      searchBaseline.setExecutingPlanIndex((short)executingTspWSB.planIndex);
      
      ArrayList<javaff_interfaces.msg.ActionCommittedStatus> acsList = new ArrayList<>();

      for(TimeStampedAction tsa : executingTspWSB.getSortedActions())
      {
        javaff_interfaces.msg.ActionCommittedStatus acsMsg = new javaff_interfaces.msg.ActionCommittedStatus();
        {
          // fill action committed status msg directly by mirroring current and last updated tsp
          acsMsg.setCommittedAction("(" + tsa.action + ")");
          acsMsg.setPlannedStartTime(tsa.time.floatValue());
          acsMsg.setCommitted(tsa.committed);
        }
        acsList.add(acsMsg);
      }
      searchBaseline.setCommittedActions(acsList);
    }

    return searchBaseline;
  }


  /*
   * Create a special msg when we meet an unexpected state PPlans[Plan[ PlanItem{-1, "", -1} ]]
  */
  public static javaff_interfaces.msg.SearchResult buildUnsatSearchResult(){
    javaff_interfaces.msg.SearchResult searchResult = new javaff_interfaces.msg.SearchResult();
    searchResult.setStatus(searchResult.FAILED);
    return searchResult;
  }

  /*
   * Simulate all/all_committed actions in tspQueue starting from currPlanIndex and taking into consideration the current status of each single tsa in the different tsp
  */
  private static TemporalMetricState simActions(TemporalMetricState updCurrentState, short currPlanIndex, 
    ArrayList<TimeStampedPlanWithSearchBaseline> tspQueue, boolean justSimCommitted)
  {
    javaff_interfaces.msg.ActionExecutionStatus aes = new javaff_interfaces.msg.ActionExecutionStatus();
    TemporalMetricState currentState = (TemporalMetricState) updCurrentState.clone();
    TimeStampedPlan tsp = tspQueue.get(currPlanIndex);
    TreeSet<SplitInstantAction> orderedSplitInstantActions = (TreeSet<SplitInstantAction>)tsp.getSortedSplitInstantActions();
    Iterator<SplitInstantAction> itsa = orderedSplitInstantActions.iterator();
    
    String msgLog = "";

    while(itsa.hasNext())
    {
      SplitInstantAction sia = itsa.next();
      String actionFullNameNoTime = "(" + sia.parent.toString() + ")";
      float actionPlannedTimeStart = (sia instanceof StartInstantAction)? sia.predictedInstant.floatValue()*1000.0f : sia.getSibling().predictedInstant.floatValue()*1000.0f;
      String actionFullName = actionFullNameNoTime + ":" + actionPlannedTimeStart;
      TimeStampedAction tsa = tsp.getTimeStampedAction(actionFullName);
      if(tsa == null)
      { 
        System.out.println(msgLog);
        System.out.println("SimActions " + (justSimCommitted? "JUST_COMMITTED" : "SIM_TO_GOAL") + ": tsa '" + actionFullName + "' not found!");
        return null;
      }

      
      boolean actionBTStillRunning = tsa.status == aes.RUNNING || tsa.status == aes.RUN_SUC;
      boolean actionBTWaiting = tsa.status == aes.WAITING || tsa.status == aes.RUN_SUC;

      if(actionBTStillRunning && sia instanceof StartInstantAction)
      {
        // apply all no-domain defined effect of sia Start snap action of a currently running action
        Set operators = sia.effect.getOperators();
        Set addPropositions = sia.effect.getAddPropositions();
        Set delPropositions = sia.effect.getDeletePropositions();
        for(Object op : operators)
          if(op instanceof ResourceOperator)
            if(! (((ResourceOperator) op).resource.getPredicateSymbol().isDomainDefined()))
              ((ResourceOperator) op).apply(currentState);
            

        for(Object addProp : addPropositions)
          if(addProp instanceof Proposition)
            if(! (((Proposition) addProp).getPredicateSymbol().isDomainDefined()))
              ((Proposition) addProp).apply(currentState);
            
        
        for(Object delProp : delPropositions)
          if(delProp instanceof Proposition)
            if(! (((Proposition) delProp).getPredicateSymbol().isDomainDefined()))
              ((Proposition) delProp).apply(currentState);
            
        msgLog += ("\nApplied non domain defined effects of " + sia);

      }else if( actionBTWaiting && (!justSimCommitted || justSimCommitted && tsa.committed) || actionBTStillRunning && sia instanceof EndInstantAction){
        // apply instant actions iff 
        // not started yet (hence, not even effect at start applied) 
        //  OR 
        // iff currently running and here we've reached the end snap action in the simulation

        if(sia.isApplicable(currentState))
          currentState = (TemporalMetricState) currentState.apply(sia);
        else{
          System.out.println(msgLog);
          System.out.println("SimActions " + (justSimCommitted? "JUST_COMMITTED" : "SIM_TO_GOAL") + ": sia not applicable: " + sia);
          return null;
        }

        msgLog += ("\nApplied effects of " + sia);
      }
    }

    if(justSimCommitted)//stop sim here
      return currentState;

    // sim to goal, taking into consideration all following enqueued plans
    
    // apply all enqueued tsp here (some might still be missing though: deal with it externally) 
    currPlanIndex++;
    while(currPlanIndex < tspQueue.size() && sameSearchBaseline(tspQueue.get(currPlanIndex-1).searchBaseline, tspQueue.get(currPlanIndex).searchBaseline))
    {
      tsp = tspQueue.get(currPlanIndex);
      orderedSplitInstantActions = (TreeSet<SplitInstantAction>)tsp.getSortedSplitInstantActions();
      itsa = orderedSplitInstantActions.iterator();
      while(itsa.hasNext())
      { 
        SplitInstantAction sia = itsa.next();
        if(sia.isApplicable(currentState))
          currentState = (TemporalMetricState) currentState.apply(sia);
        else{
          System.out.println(msgLog);
          System.out.println("SimActions " + (justSimCommitted? "JUST_COMMITTED" : "SIM_TO_GOAL") + ": sia not applicable: " + sia);
          return null;
        }
        
        msgLog += ("\nApplied effects of " + sia);
      }

      currPlanIndex++;
    }

    return currentState;
  }

  /*
   * Simulate current state to goal path applying the effects based on the state of the action in the corresponding tsp (i.e. WAITING, RUNNING, EXECUTED) 
   * and return whether the goal is still achievable in the response
  */
  public static boolean successSimToGoal(TemporalMetricState updCurrentState, short currPlanIndex, ArrayList<TimeStampedPlanWithSearchBaseline> tspQueue){
    TemporalMetricState currentState = simActions(updCurrentState, currPlanIndex, tspQueue, false);
    
    return (currentState==null)? false : currentState.goalReached();
  }

  /*
   * Simulate current state to next committed state path applying the effects based on the state of the action in the corresponding tsp (i.e. WAITING, RUNNING, EXECUTED) 
   * and return the achieved state (null in case it was not possible to achieve it)
  */
  public static TemporalMetricState simCommitted(TemporalMetricState updCurrentState, short currPlanIndex, ArrayList<TimeStampedPlanWithSearchBaseline> tspQueue){
    return simActions(updCurrentState, currPlanIndex, tspQueue, true);
  }


  public static boolean sameSearchBaseline(javaff_interfaces.msg.CommittedStatus oldSB, javaff_interfaces.msg.CommittedStatus newSB){
    if(oldSB.getExecutingPlanIndex() != newSB.getExecutingPlanIndex())
      return false;

    if(oldSB.getCommittedActions().size() != newSB.getCommittedActions().size())
      return false;

    for(int i=0; i<oldSB.getCommittedActions().size(); i++){
      javaff_interfaces.msg.ActionCommittedStatus a1 = oldSB.getCommittedActions().get(i);
      javaff_interfaces.msg.ActionCommittedStatus a2 = newSB.getCommittedActions().get(i);
      if(a1.getCommittedAction() != a2.getCommittedAction() || a1.getPlannedStartTime() != a2.getPlannedStartTime())
        return false;
      if(a1.getCommitted() != a2.getCommitted())
        return false;
    }
    return true;
  }


  /*
   * Compute est. deadline of non committed actions in tspQueue starting from tsp in startPlanIndex position
  */
  public static float computeNonCommittedPlannedDeadline(ArrayList<TimeStampedPlanWithSearchBaseline> tspQueue, short startPlanIndex){
    if(startPlanIndex >= tspQueue.size())
      return -1.0f;//error
    
    float computedDeadline = 0.0f; // deadline that needs to be returned at the end
    float planOffset = 0.0f; // plan offset such that if I have P5(seachID: 2)->P6(seachID: 2), deadlines of P6 takes as additive offset computeDeadline for P5
    float currentSearchID = tspQueue.get(startPlanIndex).searchID;
    for(short planIndex = startPlanIndex; planIndex < tspQueue.size() && tspQueue.get(planIndex).searchID == currentSearchID; planIndex++)
    {
      //iterate over all the tsp, starting from startPlanIndex till reaching the last tsp belonging to the same search results (based on searchID)
      TimeStampedPlanWithSearchBaseline tspWSB = tspQueue.get(planIndex);
      float committedOffset = 0.0f; // do not take into consideration time of already committed actions, compute it and remove it as negative offset
      for(TimeStampedAction tsa : tspWSB.getSortedActions()) // assumption: given ordering, all committed actions are put before non committed ones
      {
        if(tsa.committed)
          committedOffset = Math.max(committedOffset, planOffset + tsa.time.floatValue() + tsa.duration.floatValue());
        else
          computedDeadline = Math.max(computedDeadline, planOffset + tsa.time.floatValue() + tsa.duration.floatValue() - committedOffset);
      }
      planOffset = computedDeadline;
    }

    return computedDeadline;
  }
}
