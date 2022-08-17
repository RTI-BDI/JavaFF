package it.unitn.rti.bdi.javaff;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.TreeSet;

import java.util.concurrent.locks.ReentrantLock;

import javaff.data.GroundProblem;
import javaff.data.TimeStampedPlan;
import javaff.planning.State;
import javaff.planning.TemporalMetricState;
import javaff.search.HValueComparator;

// Search data shared among ROS2JavaFFSearch and SearchThread running instance
public class SharedSearchData {
    public GroundProblem groundProblem;
    public TemporalMetricState searchCurrentState; // search current state
    
    public TemporalMetricState execNextCommittedState; // next state with no open action in execution
  
    public ros2_bdi_interfaces.msg.Desire fulfillingDesire;
    public ArrayList<TimeStampedPlan> tspQueue;
    public javaff_interfaces.msg.SearchResult searchResultMsg = new javaff_interfaces.msg.SearchResult();
    
    public TreeSet<State> open = new TreeSet<>(new HValueComparator());
    public Hashtable<Integer, State> closed = new Hashtable<>();
    public ReentrantLock searchLock = new ReentrantLock(true);
  
    public int intervalSearchMS;
}
