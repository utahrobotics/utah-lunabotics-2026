class_name BucketActuatorsCommand
extends Command

## Move lift actuators, positive up, negative downed back

# TODO: could be connected to LiftActuatorCommand and have variable change target
# for now will remain separate commands

# In rust its an i8 (-128 to 127)
@export var lift: int = 0

# TODO: EXECUTE
func execute(actor: Node) -> void:
	var rustext = actor.get_node("../Main")
	rustext.set_bucket_actuators(lift)
	
