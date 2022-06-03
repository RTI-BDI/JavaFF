/* 
*/

package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.executors.MultiThreadedExecutor;

public class ROS2JavaFF extends BaseComposableNode {

  public ROS2JavaFF(String name, int i) {
    super(name);
  }
  

  public static void main(final String[] args) throws Exception {
    // Initialize RCL
    RCLJava.rclJavaInit();

    MultiThreadedExecutor exec = new MultiThreadedExecutor(2);
    
    ROS2JavaFFServer javaffServerNode = new ROS2JavaFFServer("javaff_server", 1);
    ROS2JavaFFSearch javaffSearchNode = new ROS2JavaFFSearch("javaff_search", 2);

    javaffSearchNode.setServerNode(javaffServerNode);
    javaffServerNode.setSearchNode(javaffSearchNode);

    exec.addNode(javaffServerNode);
    exec.addNode(javaffSearchNode);
    
    exec.spin();

  }
}

