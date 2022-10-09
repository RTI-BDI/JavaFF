package it.unitn.rti.bdi.javaff;

import org.ros2.rcljava.node.BaseComposableNode;
import org.ros2.rcljava.service.RMWRequestId;
import org.ros2.rcljava.service.Service;

import it.unitn.rti.bdi.javaff.SearchParams;

public class ROS2JavaFFServer extends BaseComposableNode{
    
    // Sibling node carrying on planning search tasks
    private ROS2JavaFFSearch searchNode;
    
    // start planning service
    private Service<javaff_interfaces.srv.JavaFFPlan> planService;

    // communicate unexpected state service
    private Service<javaff_interfaces.srv.UnexpectedState> unexpectedStateService;

    // get state srv (this is node a lifecycle node, but we want ROS2-BDI to use it just as an health status end-point
    // if it's not reachable, JavaFF nodes will be considered inactive)
    private Service<lifecycle_msgs.srv.GetState> getStateService;

    public ROS2JavaFFServer(String name, String namespace) {
        super(name, namespace);

        try{
            this.planService =
                this.node.<javaff_interfaces.srv.JavaFFPlan>createService(
                    javaff_interfaces.srv.JavaFFPlan.class, name + "/start_plan",
                    (RMWRequestId header, javaff_interfaces.srv.JavaFFPlan_Request request,
                        javaff_interfaces.srv.JavaFFPlan_Response response)
                        -> this.handlePlanService(header, request, response));

            this.unexpectedStateService =
                this.node.<javaff_interfaces.srv.UnexpectedState>createService(
                    javaff_interfaces.srv.UnexpectedState.class, name + "/unexpected_state",
                    (RMWRequestId header, javaff_interfaces.srv.UnexpectedState_Request request,
                        javaff_interfaces.srv.UnexpectedState_Response response)
                        -> this.handleUnexpectedStateService(header, request, response));

            this.getStateService =
                this.node.<lifecycle_msgs.srv.GetState>createService(
                    lifecycle_msgs.srv.GetState.class, name + "/get_state",
                    (RMWRequestId header, lifecycle_msgs.srv.GetState_Request request,
                        lifecycle_msgs.srv.GetState_Response response)
                        -> this.handleGetStateService(header, request, response));

        }catch(Exception e){
          System.err.println("Service \"start_plan\" cannot be started");
          e.printStackTrace(System.out);
        }
       
    }

    public void setSearchNode(ROS2JavaFFSearch searchNode){this.searchNode = searchNode;}

    public void handleGetStateService(final RMWRequestId header,
        final lifecycle_msgs.srv.GetState_Request request,
        final lifecycle_msgs.srv.GetState_Response response){
            lifecycle_msgs.msg.State state = new lifecycle_msgs.msg.State();
            
            state.setId(state.PRIMARY_STATE_ACTIVE);
            state.setLabel("active");

            response.setCurrentState(state);
    }

    public void handlePlanService(final RMWRequestId header,
        final javaff_interfaces.srv.JavaFFPlan_Request request,
        final javaff_interfaces.srv.JavaFFPlan_Response response){

        OperationResult startSearchStatus = searchNode.startSearch(request.getFulfillingDesire(), request.getProblem(), 
            new SearchParams(request.getSearchInterval(), request.getMaxPplanSize(), request.getMaxEmptySearchIntervals()));

        //get meaningful boolean from previous call
        response.setAccepted(startSearchStatus.result);
        response.setMsg(startSearchStatus.msg);
        
    }

    public void handleUnexpectedStateService(final RMWRequestId header,
        final javaff_interfaces.srv.UnexpectedState_Request request,
        final javaff_interfaces.srv.UnexpectedState_Response response){
            //System.out.println("Unexpected state service has been called; provided state: " + request.getPddlProblem());
            OperationResult unexpectedStateStatus = searchNode.unexpectedState(request.getPddlProblem());
            //TODO get meaningful boolean from previous call
            
            response.setHandled(unexpectedStateStatus.result);
    }
}