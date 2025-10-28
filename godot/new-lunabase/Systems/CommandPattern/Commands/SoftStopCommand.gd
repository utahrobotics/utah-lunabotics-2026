class_name SoftStopCommand
extends Command

## Request software stop mode

func execute(actor: Node) -> void:
	if actor.has_method("send_soft_stop"):
		actor.send_soft_stop()
	else:
		push_error("Actor does not have send_soft_stop method")
