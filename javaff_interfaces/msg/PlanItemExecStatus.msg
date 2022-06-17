# Message contains plan item to be executed and its current status (i.e. in execution, waiting to be executed, successful execution or aborted)
# Here @status is at plan item (i.e. action) level

uint8 RUNNING=0
uint8 ABORT=1
uint8 SUCCESSFUL=2
uint8 WAITING=3

plansys2_msgs/PlanItem item
uint8 status