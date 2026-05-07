class_name ResetObstaclesCommand
extends Command

func execute(actor: Node) -> void:
	if actor.has_method("send_reset_obstacles"):
		actor.send_reset_obstacles()
	else:
		push_error("Actor does not have send_reset_obstacles method")
