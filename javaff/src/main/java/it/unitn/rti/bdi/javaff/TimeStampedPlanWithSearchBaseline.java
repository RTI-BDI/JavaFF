package it.unitn.rti.bdi.javaff;

import java.util.TreeSet;

import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;

public class TimeStampedPlanWithSearchBaseline extends TimeStampedPlan{
    
    public javaff_interfaces.msg.CommittedStatus searchBaseline;
    public short searchID;

    public TimeStampedPlanWithSearchBaseline(TimeStampedPlan tsp, javaff_interfaces.msg.CommittedStatus searchBaseline, short searchID)
    {
        this.searchBaseline = searchBaseline;
        this.planIndex = tsp.planIndex;
        this.actions = new TreeSet<TimeStampedAction>();
        this.actions.addAll(tsp.getSortedActions());
        this.searchID = searchID;
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
                String tsaFullNameTimex1000 = tsa.toStringFullNameTimex1000();
                // tsa should be committed in search baseline too, otherwise we've already moved over it
                for(javaff_interfaces.msg.ActionCommittedStatus acs : searchBaseline.getCommittedActions())
                {
                    String acsFullNameTimex1000 = acs.getCommittedAction() + ":" + ((int) (acs.getPlannedStartTime()*1000));

                    if(tsaFullNameTimex1000.equals(acsFullNameTimex1000))
                        if(!acs.getCommitted())
                            return true;//outdated search baseline
                }
            }

        return false;
    }
}
