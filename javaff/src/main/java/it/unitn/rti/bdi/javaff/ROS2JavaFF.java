/* 
*/

package it.unitn.rti.bdi.javaff;

import java.io.File;  // Import the File class
import java.io.FileNotFoundException;  // Import this class to handle errors
import java.util.Scanner; // Import the Scanner class to read text files


import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.MultiThreadedExecutor;

public class ROS2JavaFF{

  private static String retrieveDomainFilepath(final String[] args){
    String fp = "";

    int encounteredSubseqQuestionMarks = 0;
    boolean reading = false;
    for(int i=0; i<args.length && (reading || fp.length() == 0); i++)
    { 
      // check whenever ??? is encountered
      while(i<args.length && args[i].equals("?"))
      {
        encounteredSubseqQuestionMarks++;
        i++;
      }
      
      // encountered ???, i.e. start or ending reading phase
      if(encounteredSubseqQuestionMarks == 3)
      {  
        reading = !reading;
        encounteredSubseqQuestionMarks = 0;
      }
      
      // if reading, concat 
      if(reading)
        fp = fp.concat(args[i]);
    }

    return fp;

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

  public static void main(final String[] args) throws Exception {
    //retrieve namespace from cli args[]
    String ns = String.join("", args);
    ns = ns.substring(ns.lastIndexOf("ns:=/") + "ns:=/".length());

    String domainFilepath = retrieveDomainFilepath(args);
    String domain = readFile(domainFilepath);

    // System.out.println("Setting up namespace \"" + ns + "\"");
    // System.out.println("Reading domain file from \"" + domainFilepath + "\"");
    // System.out.println("Loaded domain file: \"" + domain + "\"");

    // Initialize RCL
    RCLJava.rclJavaInit();

    MultiThreadedExecutor exec = new MultiThreadedExecutor(2);
    
    ROS2JavaFFServer javaffServerNode = new ROS2JavaFFServer("javaff_server", ns);
    ROS2JavaFFSearch javaffSearchNode = new ROS2JavaFFSearch("javaff_search", ns, domain);

    javaffSearchNode.setServerNode(javaffServerNode);
    javaffServerNode.setSearchNode(javaffSearchNode);

    exec.addNode(javaffServerNode);
    exec.addNode(javaffSearchNode);
    
    exec.spin();

  }
}

