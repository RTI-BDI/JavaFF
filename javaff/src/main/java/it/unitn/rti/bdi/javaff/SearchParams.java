package it.unitn.rti.bdi.javaff;

public class SearchParams {
    public int intervalSearchMS;
    public int maxPPlanSize;
    public int maxEmptySearchIntervals;
    public SearchParams(int intervalMS, int maxPPlanSize, int maxEmptyIntervals)
    {
      this.intervalSearchMS = intervalMS;
      this.maxPPlanSize = maxPPlanSize;
      this.maxEmptySearchIntervals = maxEmptyIntervals;
    }
}
