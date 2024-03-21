package it.unitn.rti.bdi.javaff;

// Search parameters to tune behaviour of continual planner
public class SearchParams {
    public int intervalSearchMS; // search interval ms for each search round
    public int maxPPlanSize; // plan size max for each partial plan
    public int maxEmptySearchIntervals; // max number of consecutive search rounds producing no result (no better state is found)
    public SearchParams(int intervalMS, int maxPPlanSize, int maxEmptyIntervals)
    {
      this.intervalSearchMS = intervalMS;
      this.maxPPlanSize = maxPPlanSize;
      this.maxEmptySearchIntervals = maxEmptyIntervals;
    }
}
