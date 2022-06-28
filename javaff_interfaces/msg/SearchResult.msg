# Message to contain an array of plans ready to be executed, where plans[0] is supposed to be run and terminate before the execution
# of plans[1]

uint8 SEARCHING=0
uint8 FAILED=1
uint8 SUCCESS=2

PartialPlan[] plans
uint8 status
