package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;

// import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.ArrayList;
import java.util.StringTokenizer;  

import javaff.JavaFF;
import javaff.data.GroundProblem;
import javaff.planning.TemporalMetricState;

import plansys2_msgs.msg.Plan;
import plansys2_msgs.msg.PlanItem;

class SearchThread extends Thread{

  private boolean killMySelf = false;
  private SharedSearchData sharedSearchData;
  private Publisher<plansys2_msgs.msg.Plan> planPublisher;

  public SearchThread(SharedSearchData sharedSearchData, Publisher<plansys2_msgs.msg.Plan> planPublisher){
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

  public void killMySelf(){killMySelf = true;}
  
  public void run(){
    this.sharedSearchData.open.clear();
    this.sharedSearchData.open.add("");//TODO update this

    while(true){
      try{
        this.sharedSearchData.searchLock.lock();
          //System.out.println("[Thread search]: waiting to acquire lock "  + this.sharedSearchData.searchLock.getQueueLength());
    
          //System.out.println("[Thread search]: Searching from " + this.sharedSearchData.initialState + " (open[0]=" + this.sharedSearchData.open.get(0) + ") ...");
          Thread.sleep(0);//searching phase
          String planString = JavaFF.plan(this.sharedSearchData.groundProblem, this.sharedSearchData.initialState);
          System.out.println("\n\nplanString: \n" + planString);        

          plansys2_msgs.msg.Plan planMessage = buildPsys2Plan(planString);
          this.planPublisher.publish(planMessage);

        this.sharedSearchData.searchLock.unlock();
        if(killMySelf)//if true mspawner thread has set it, put it again to false and terminate your execution
          return;

        //TODO publish results

      }catch(InterruptedException ie){
        ie.printStackTrace();
      } 
    }
  }
}

class SharedSearchData{
  GroundProblem groundProblem;
  TemporalMetricState initialState;
  
  ArrayList<String> open = new ArrayList<>();//TODO just for skeleton
  ReentrantLock searchLock = new ReentrantLock(true);
}

public class ROS2JavaFFSearch extends ROS2JavaFF{
    
    // Sibling node handling service request wrt. online planning
    private ROS2JavaFFServer serverNode;
    
    private SearchThread searchThread;

    private SharedSearchData sharedSearchData;

    private Publisher<plansys2_msgs.msg.Plan> planPublisher;

    // private AtomicBoolean killSearchThread = new AtomicBoolean(false);

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    public ROS2JavaFFSearch(String name, int i) {
      super(name, i);
      this.sharedSearchData = new SharedSearchData();
      this.planPublisher = this.node.<plansys2_msgs.msg.Plan>createPublisher(plansys2_msgs.msg.Plan.class, "plan");

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

        // parse domain and problem, unground + ground processes, returning the initial state
        this.sharedSearchData.groundProblem = JavaFF.computeGroundProblem(domain, problem);
        this.sharedSearchData.initialState = JavaFF.computeInitialState(this.sharedSearchData.groundProblem);
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
        this.sharedSearchData.open.add(newState);
      this.sharedSearchData.searchLock.unlock();
    }
}