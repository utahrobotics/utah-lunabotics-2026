class_name LiftActuatorsCommand
extends Command

## Move lift actuators, positive up, negative downed back

# In rust its an i8 (-128 to 127)
@export var lift: int = 0


# TODO: EXECUTE
func execute(actor: Node) -> void:
	var rustext = actor.get_node("../Main")
	rustext.set_lift_actuators(lift)
	
