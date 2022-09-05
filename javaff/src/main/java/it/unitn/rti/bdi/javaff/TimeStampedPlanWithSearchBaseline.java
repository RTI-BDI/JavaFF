package it.unitn.rti.bdi.javaff;

import java.util.TreeSet;

import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;

public class TimeStampedPlanWithSearchBaseline extends TimeStampedPlan{
    
    public javaff_interfaces.msg.CommittedStatus searchBaseline;

    public TimeStampedPlanWithSearchBaseline(TimeStampedPlan tsp, javaff_interfaces.msg.CommittedStatus searchBaseline)
    {
        this.searchBaseline = searchBaseline;
        this.actions = new TreeSet<TimeStampedAction>();
        this.actions.addAll(tsp.getSortedActions());
    }

    // compare current execution status of tsp with passed searchBaseline; 
    // returns true if the latter is outdated
    // (outdated means that we've already move over the threshold of committed actions)
    public boolean outdatedSearchBaseline(javaff_interfaces.msg.CommittedStatus searchBaseline)
    {
        if(this.planIndex > searchBaseline.getExecutingPlanIndex())
            return true;

        for(TimeStampedAction tsa : getSortedActions())
            if(tsa.committed)
            {
                // tsa should be committed in search baseline too, otherwise we've already moved over it
                for(javaff_interfaces.msg.ActionCommittedStatus acs : searchBaseline.getCommittedActions())
                    if(tsa.action.toString() == acs.getCommittedAction() && tsa.time.floatValue() == acs.getPlannedStartTime())
                        if(!acs.getCommitted())
                            return true;//outdated search baseline
            }

        return false;
    }
}
