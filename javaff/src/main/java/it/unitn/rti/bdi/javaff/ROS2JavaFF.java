/* 
*/

package it.unitn.rti.bdi.javaff;

import java.io.File;  // Import the File class
import java.io.FileNotFoundException;  // Import this class to handle errors
import java.util.Scanner; // Import the Scanner class to read text files


import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.executors.MultiThreadedExecutor;

public class ROS2JavaFF{

  private static String retrieveArgument(final String[] args, final String delimiter, final int delimiterCount){
    String arg = "";

    int encounteredSubseqQuestionMarks = 0;
    boolean reading = false;
    for(int i=0; i<args.length && (reading || arg.length() == 0); i++)
    { 
      // check whenever delimiter is encountered
      while(i<args.length && args[i].equals(delimiter))
      {
        encounteredSubseqQuestionMarks++;
        i++;
      }
      
      // encountered delimiter requested number of times, i.e. start or ending reading phase
      if(encounteredSubseqQuestionMarks == delimiterCount)
      {  
        reading = !reading;
        encounteredSubseqQuestionMarks = 0;
      }
      
      // if reading, concat 
      if(reading)
        arg = arg.concat(args[i]);
    }

    return arg;

  }

  private static String retrieveDomainFilepath(final String[] args){
      return retrieveArgument(args, "?", 3);
  }

  private static boolean retrieveDebug(final String[] args){
    String debug = retrieveArgument(args, "!", 3);
    return debug.equalsIgnoreCase("debug=True");
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
    // for(String a : args)
    //   System.out.println(a);

    //retrieve namespace from cli args[]
    String ns = String.join("", args);
    ns = ns.substring(ns.lastIndexOf("ns:=/") + "ns:=/".length());
    System.out.println("Setting up namespace \"" + ns + "\"");
    String domainFilepath = retrieveDomainFilepath(args);
    System.out.println("Reading domain file from \"" + domainFilepath + "\"");
    String domain = readFile(domainFilepath);
    boolean debugActive = retrieveDebug(args);
    System.out.println("Debug active = \"" + debugActive + "\"");

    //System.out.println("Loaded domain file: \"" + domain + "\"");

    // Initialize RCL
    RCLJava.rclJavaInit();

    MultiThreadedExecutor exec = new MultiThreadedExecutor(2);
    
    ROS2JavaFFServer javaffServerNode = new ROS2JavaFFServer("javaff_server", ns);
    ROS2JavaFFSearch javaffSearchNode = new ROS2JavaFFSearch("javaff_search", ns, domain, debugActive);

    javaffSearchNode.setServerNode(javaffServerNode);
    javaffServerNode.setSearchNode(javaffSearchNode);

    exec.addNode(javaffServerNode);
    exec.addNode(javaffSearchNode);
    
    exec.spin();

  }
}

