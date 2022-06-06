package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;

// import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.ArrayList;

import javaff.JavaFF;
import javaff.data.GroundProblem;
import javaff.planning.TemporalMetricState;

class SearchThread extends Thread{

  private boolean killMySelf = false;
  private SharedSearchData sharedSearchData;
  private Publisher<example_interfaces.msg.String> planPublisher;

  public SearchThread(SharedSearchData sharedSearchData, Publisher<example_interfaces.msg.String> planPublisher){
    super();
    this.sharedSearchData = sharedSearchData;
    this.planPublisher = planPublisher;
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
          example_interfaces.msg.String planMessage = new example_interfaces.msg.String();
          planMessage.setData(planString);
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
  TemporalMetricState initialState;//TODO just for skeleton
  ArrayList<String> open = new ArrayList<>();//TODO just for skeleton
  ReentrantLock searchLock = new ReentrantLock(true);
}

public class ROS2JavaFFSearch extends ROS2JavaFF{
    
    // Sibling node handling service request wrt. online planning
    private ROS2JavaFFServer serverNode;
    
    private SearchThread searchThread;

    private SharedSearchData sharedSearchData;

    private Publisher<example_interfaces.msg.String> planPublisher;

    // private AtomicBoolean killSearchThread = new AtomicBoolean(false);

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    public ROS2JavaFFSearch(String name, int i) {
      super(name, i);
      this.sharedSearchData = new SharedSearchData();
      this.planPublisher = this.node.<example_interfaces.msg.String>createPublisher(example_interfaces.msg.String.class, "plan");
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