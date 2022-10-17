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

  private static int retrieveMinCommitSteps(final String[] args){
    String minCommitSteps = retrieveArgument(args, "@", 3);
    String steps = minCommitSteps.substring(("min_commit_steps").length()+1);
    return Integer.parseInt(steps);
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

  private static String addFullfilmentPredicates(String domain){
    int currIndex = domain.indexOf(":predicates");
    if(currIndex < 0)
        return "";
    currIndex +=  ":predicates".length();

    boolean parsingPredicate = false;
    String parsedPredicate = "";
    String fulfillmentPredicates = "";
    for(;currIndex<domain.length();currIndex++){
        if(domain.charAt(currIndex) == '(')
            parsingPredicate = true;
        
        if(parsingPredicate)
            parsedPredicate += domain.charAt(currIndex);
            
        if(domain.charAt(currIndex) == ')')
        {
            if(parsingPredicate)
            {
                //reached end of a predicate -> need to create a new one starting with "f_"
                parsingPredicate = false;
            
                int i = 1;
                while(!Character.isAlphabetic(parsedPredicate.charAt(i))){i++;}
                String fulfillmentPredicate = "(f_" + parsedPredicate.substring(i);
                fulfillmentPredicates += fulfillmentPredicate + "\n";
                parsedPredicate = "";
            }
            else
            {
                //reached end of predicates section
                return (domain.substring(0, currIndex) + //original domain till end of predicates parenthesis not included
                                fulfillmentPredicates + //fulfillment predicates
                                ")" + //predicates are finished
                                domain.substring(currIndex + 1));
            }
            
        }
    }
    return domain;
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
    domain = addFullfilmentPredicates(domain);
    //System.out.println(domain);
    boolean debugActive = retrieveDebug(args);
    System.out.println("Debug active = " + debugActive);
    int minCommitSteps = retrieveMinCommitSteps(args);
    System.out.println("Min commit steps = " + minCommitSteps);

    //System.out.println("Loaded domain file: \"" + domain + "\"");

    // Initialize RCL
    RCLJava.rclJavaInit();

    MultiThreadedExecutor exec = new MultiThreadedExecutor(2);
    
    ROS2JavaFFServer javaffServerNode = new ROS2JavaFFServer("javaff_server", ns);
    ROS2JavaFFSearch javaffSearchNode = new ROS2JavaFFSearch("javaff_search", ns, domain, debugActive, minCommitSteps);

    javaffSearchNode.setServerNode(javaffServerNode);
    javaffServerNode.setSearchNode(javaffSearchNode);

    exec.addNode(javaffServerNode);
    exec.addNode(javaffSearchNode);
    
    exec.spin();
    RCLJava.shutdown();
  }
}

