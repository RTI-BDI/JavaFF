# Message to contain a partial plan to be executed
# Target desire contains the state to be reached from the execution, as well as precondition necessary for triggering and executing
# the plan for its fulfillment

int16 index
plansys2_msgs/Plan plan
ros2_bdi_interfaces/Desire target