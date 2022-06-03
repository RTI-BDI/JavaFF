package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.publisher.Publisher;

// import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantLock;
import java.util.ArrayList;

import javaff.JavaFF;
import javaff.planning.TemporalMetricState;

class SearchThread extends Thread{

  private boolean killMySelf = false;
  private SharedSearchData sharedSearchData;
  private Publisher<std_msgs.msg.String> planPublisher;

  public SearchThread(SharedSearchData sharedSearchData, Publisher<std_msgs.msg.String> planPublisher){
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
          String planString = JavaFF.plan(this.sharedSearchData.initialState);
          std_msgs.msg.String planMessage = new std_msgs.msg.String();
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
  TemporalMetricState initialState;//TODO just for skeleton
  ArrayList<String> open = new ArrayList<>();//TODO just for skeleton
  ReentrantLock searchLock = new ReentrantLock(true);
}

public class ROS2JavaFFSearch extends ROS2JavaFF{
    
    // Sibling node handling service request wrt. online planning
    private ROS2JavaFFServer serverNode;
    
    private SearchThread searchThread;

    private SharedSearchData sharedSearchData;

    private Publisher<std_msgs.msg.String> planPublisher;

    // private AtomicBoolean killSearchThread = new AtomicBoolean(false);

    public void setServerNode(ROS2JavaFFServer serverNode){this.serverNode = serverNode;}

    public ROS2JavaFFSearch(String name, int i) {
      super(name, i);
      this.sharedSearchData = new SharedSearchData();
      this.planPublisher = this.node.<std_msgs.msg.String>createPublisher(std_msgs.msg.String.class, "plan");
    }

    public void startSearch(String domain, String problem) throws InterruptedException{
      try{
        if(this.searchThread.isAlive()){
          System.out.println("Search thread already up: waiting for it to stop...");
          this.searchThread.killMySelf();
          this.searchThread.join();
        }
      }catch(NullPointerException ne){}//handle first call in which thread has not been started yet or no one is running

      this.sharedSearchData.initialState = JavaFF.computeInitialState(domain, problem);
      this.searchThread = new SearchThread(sharedSearchData, planPublisher);
      this.searchThread.start();
    }

    public void unexpectedState(String newState){
      this.sharedSearchData.searchLock.lock();
        this.sharedSearchData.open.clear();
        this.sharedSearchData.open.add(newState);
      this.sharedSearchData.searchLock.unlock();
    }
}