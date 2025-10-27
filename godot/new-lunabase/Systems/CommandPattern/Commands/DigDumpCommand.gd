class_name DigDumpCommand
extends Command

## Navigate to requested value

@export var target_pos: Vector2 = Vector2.ZERO

# TODO: EXECUTE
func execute(actor: Node) -> void:
	var rustext = actor.get_node("../Main")
	rustext.init_autonomous_digdump(target_pos.x,target_pos.y)
