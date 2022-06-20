/* 
*/

package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.MultiThreadedExecutor;

public class ROS2JavaFF{

  public static void main(final String[] args) throws Exception {
    //retrieve namespace from cli args[]
    String ns = String.join("", args);
    ns = ns.substring(ns.lastIndexOf("ns:=/") + "ns:=/".length());

    System.out.println("Setting up namespace \"" + ns + "\"");

    // Initialize RCL
    RCLJava.rclJavaInit();

    MultiThreadedExecutor exec = new MultiThreadedExecutor(2);
    
    ROS2JavaFFServer javaffServerNode = new ROS2JavaFFServer("javaff_server", ns);
    ROS2JavaFFSearch javaffSearchNode = new ROS2JavaFFSearch("javaff_search", ns);

    javaffSearchNode.setServerNode(javaffServerNode);
    javaffServerNode.setSearchNode(javaffSearchNode);

    exec.addNode(javaffServerNode);
    exec.addNode(javaffSearchNode);
    
    exec.spin();

  }
}

