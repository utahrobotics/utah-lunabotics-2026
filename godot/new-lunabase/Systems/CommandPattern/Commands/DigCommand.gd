class_name DigCommand
extends Command

## Request software stop mode

func execute(actor: Node) -> void:
	if actor.has_method("send_dig"):
		actor.send_dig()
	else:
		push_error("Actor does not have send_dig method")
