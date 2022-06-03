package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.service.RMWRequestId;
import org.ros2.rcljava.service.Service;

public class ROS2JavaFFServer extends ROS2JavaFF{
    
    // Sibling node carrying on planning search tasks
    private ROS2JavaFFSearch searchNode;
    
    // start planning service
    private Service<javaff_interfaces.srv.JavaFFPlan> planService;

    // communicate unexpected state service
    private Service<javaff_interfaces.srv.UnexpectedState> unexpectedStateService;

    public ROS2JavaFFServer(String name, int i) {
        super(name, i);

        try{
            this.planService =
                this.node.<javaff_interfaces.srv.JavaFFPlan>createService(
                    javaff_interfaces.srv.JavaFFPlan.class, "start_plan",
                    (RMWRequestId header, javaff_interfaces.srv.JavaFFPlan_Request request,
                        javaff_interfaces.srv.JavaFFPlan_Response response)
                        -> this.handlePlanService(header, request, response));

            this.unexpectedStateService =
                this.node.<javaff_interfaces.srv.UnexpectedState>createService(
                    javaff_interfaces.srv.UnexpectedState.class, "unexpected_state",
                    (RMWRequestId header, javaff_interfaces.srv.UnexpectedState_Request request,
                        javaff_interfaces.srv.UnexpectedState_Response response)
                        -> this.handleUnexpectedStateService(header, request, response));

        }catch(Exception e){
          System.err.println("Service \"start_plan\" cannot be started");
          e.printStackTrace(System.out);
        }
       
    }

    public void setSearchNode(ROS2JavaFFSearch searchNode){this.searchNode = searchNode;}

    public void handlePlanService(final RMWRequestId header,
        final javaff_interfaces.srv.JavaFFPlan_Request request,
        final javaff_interfaces.srv.JavaFFPlan_Response response){
            //TODO remove print, just for debugging
            System.out.println("DOMAIN: " + request.getDomain());
            System.out.println("PROBLEM: " + request.getProblem());

            try{
                searchNode.startSearch(request.getDomain(), request.getProblem());

                //TODO get meaningful boolean from previous call
                response.setAccepted(true);
            }catch(InterruptedException ie){
                response.setAccepted(false);
            }   
            
    }

    public void handleUnexpectedStateService(final RMWRequestId header,
        final javaff_interfaces.srv.UnexpectedState_Request request,
        final javaff_interfaces.srv.UnexpectedState_Response response){
            searchNode.unexpectedState(request.getState());
            //TODO get meaningful boolean from previous call
            response.setHandled(true);
    }
}