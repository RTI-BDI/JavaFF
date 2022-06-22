/* Copyright 2016-2017 Esteve Fernandez <esteve@apache.org>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.RCLJava;
import org.ros2.rcljava.client.Client;
import org.ros2.rcljava.node.Node;

import java.io.File;  // Import the File class
import java.io.FileNotFoundException;  // Import this class to handle errors
import java.util.Scanner; // Import the Scanner class to read text files

import java.util.concurrent.Future;

public class PlannerClient {
  public static void main(final String[] args) throws InterruptedException, Exception {
    // Initialize RCL
    RCLJava.rclJavaInit();
    // Let's create a new Node
    Node node = RCLJava.createNode("javaff_client");
;
    Client<javaff_interfaces.srv.JavaFFPlan> client =
        node.<javaff_interfaces.srv.JavaFFPlan>createClient(
            javaff_interfaces.srv.JavaFFPlan.class, "/cleaner/javaff_server/start_plan");

    javaff_interfaces.srv.JavaFFPlan_Request request =
        new javaff_interfaces.srv.JavaFFPlan_Request();

    // String domain = "";
    // boolean errorDomain = false;

    // try {
    //     File myObj = new File("/home/devis/Documents/pddl/printing/domain.pddl");
    //     Scanner myReader = new Scanner(myObj);
    //     while (myReader.hasNextLine()) {
    //         domain += myReader.nextLine() + "\n";
    //     }
    //     myReader.close();
    //     System.out.println("\n\nDOMAIN:\n" + domain);

    // } catch (FileNotFoundException e) {
    //     errorDomain = true;
    //     System.out.println("An error occurred while reading domain file.");
    //     e.printStackTrace();
    // }

    String problem = "";
    boolean errorProblem = false;

    try {
        File myObj = new File("/home/alex/cleaner_problem.pddl");
        Scanner myReader = new Scanner(myObj);
        while (myReader.hasNextLine()) {
            problem += myReader.nextLine() + "\n";
        }
        myReader.close();
        System.out.println("\n\nPROBLEM:\n" + problem);

    } catch (FileNotFoundException e) {
        errorProblem = true;
        System.out.println("An error occurred while reading problem file.");
        e.printStackTrace();
    }
   
    if(/*!errorDomain && */!errorProblem){
        request.setSearchInterval(500);
        request.setProblem(problem);

        if (client.waitForService()) {
        Future<javaff_interfaces.srv.JavaFFPlan_Response> future =
            client.asyncSendRequest(request);
        
        javaff_interfaces.srv.JavaFFPlan_Response response = future.get();

        System.out.println(
            "\nStart plan request: \naccepted = " + response.getAccepted() + "\nmsg = " + response.getMsg());
        }
    }

    RCLJava.shutdown();
  }
}
