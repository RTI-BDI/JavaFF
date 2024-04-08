package it.unitn.rti.bdi.javaff;

import java.io.File;  // Import the File class
import java.io.FileNotFoundException;  // Import this class to handle errors
import java.util.Scanner; // Import the Scanner class to read text files
import java.util.List;
import java.util.ArrayList;
import java.util.stream.Collectors;
import java.util.TreeSet;
import java.util.Set;
import java.util.Hashtable;
import java.util.HashSet;
import java.math.BigDecimal;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.SingleThreadedExecutor;
import org.ros2.rcljava.node.ComposableNode;
import org.ros2.rcljava.node.Node;
import org.ros2.rcljava.node.NodeOptions;

import javaff.JavaFF;
import javaff.planning.State;
import javaff.planning.TemporalMetricState;
import javaff.search.HValueComparator;
import javaff.data.GroundProblem;
import javaff.data.TimeStampedAction;
import javaff.data.TimeStampedPlan;

public class ROS2JavaFFOffline implements ComposableNode {
    
    private final String name;
    private final String namespace;

    protected final Node node;

    public Node getNode() {
      return node;
    }

    public ROS2JavaFFOffline(String name, String namespace) {

      this.name = name;
      this.namespace = namespace;
      this.node = RCLJava.createNode(this.name, this.namespace, RCLJava.getDefaultContext(), new NodeOptions());

    } 

    private static String readFile(String filepath) throws FileNotFoundException {
      File myObj = new File(filepath);
      Scanner myReader = new Scanner(myObj);
      String data = "";
  
      while (myReader.hasNextLine())
        data = data.concat(myReader.nextLine() + "\n");
      
      myReader.close();
  
      return data;
    }

    private void offlineSearch(String domain, String problem){
      GroundProblem groundProblem = JavaFF.computeGroundProblem(domain, problem);
      if(groundProblem != null)
      {

        TemporalMetricState currentState = JavaFF.computeInitialState(groundProblem);
        int unsat = 0;
        TreeSet<State> open = new TreeSet<>(new HValueComparator());
        Hashtable<Integer, State> closed = new Hashtable<>();
        int i = 0;
        short pCounter = 0;
        TemporalMetricState goalState = (TemporalMetricState) JavaFF.performOfflineSearch(currentState, open, closed);
        if (goalState != null)
        {
          TimeStampedPlan tsp = JavaFF.buildPlan(pCounter, groundProblem, goalState);
          if (tsp != null) System.out.println("Solution Found!\n" + tsp.getPrintablePlan(false));
          else System.out.println("Impossible to build plan");
        }
        else
          System.out.println("Problem unsat: I apologise if I cannot satisfy your demands, Master Devis");
      }
    }

    public static void main(final String[] args) throws Exception {   
      for(String a : args)
        System.out.println(a);
  
      // retrieve namespace from cli args[];
      String ns = String.join("", args);
      ns = ns.substring(ns.lastIndexOf("ns:=/") + "ns:=/".length());
      System.out.println("Setting up namespace \"" + ns + "\"");
      
      System.out.println("Reading domain file from \"" + args[0] + "\"");
      String domain = readFile(args[0]);

      System.out.println("Reading problem file from \"" + args[1] + "\"");
      String problem = readFile(args[1]);
  
      // Initialize RCL
      RCLJava.rclJavaInit();
  
      //SingleThreadedExecutor exec = new SingleThreadedExecutor();
      ROS2JavaFFOffline javaffSearchNode = new ROS2JavaFFOffline("javaff_search", ns);
      javaffSearchNode.offlineSearch(domain, problem);
      //exec.addNode(javaffSearchNode);
      //exec.spin();
      RCLJava.shutdown();
    }
   
}