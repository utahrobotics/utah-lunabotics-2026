class_name NavigateCommand
extends Command

## Go to autonomous target position

@export var target_pos: Vector2 = Vector2.ZERO

# Go to pos
# TODO: EXECUTE
func execute(actor: Node) -> void:
	if actor.has_method("send_start_autonomy"):
		actor.send_start_autonomy();
	else:
		push_error("Actor does not have send_soft_stop method")
