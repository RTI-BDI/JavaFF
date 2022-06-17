# Message contains plan to be executed and its current status (i.e. in execution, waiting to be executed, successful execution or aborted)
# Here @status is at partial plan level

uint8 RUNNING=0
uint8 ABORT=1
uint8 SUCCESSFUL=2
uint8 WAITING=3

PlanItemExecStatus[] pplan
uint8 status