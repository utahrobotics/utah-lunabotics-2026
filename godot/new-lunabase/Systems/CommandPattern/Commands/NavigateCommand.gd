class_name NavigateCommand
extends Command

## Go to autonomous target position

@export var target_pos: Vector2 = Vector2.ZERO

# Go to pos
# TODO: EXECUTE
func execute(actor: Node) -> void:
	var rustext = actor.get_node("../Main")
	rustext.init_autonomous_navigate(target_pos.x,target_pos.y)
	pass
