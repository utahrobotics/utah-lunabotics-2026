class_name SoftStopCommand
extends Command

## Request software stop mode

# TODO: EXECUTE
func execute(actor: Node) -> void:
	var rustExt = actor.get_node("../Main")
	rustExt.init_softstop()
	
