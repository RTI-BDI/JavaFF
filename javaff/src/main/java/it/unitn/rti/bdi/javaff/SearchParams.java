package it.unitn.rti.bdi.javaff;

public class SearchParams {
    public int intervalSearchMS;
    public int maxEmptySearchIntervals;
    public SearchParams(int intervalMS, int maxEmptyIntervals)
    {
      this.intervalSearchMS = intervalMS;
      this.maxEmptySearchIntervals = maxEmptyIntervals;
    }
}
