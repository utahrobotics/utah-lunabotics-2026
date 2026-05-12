class_name TestMotorsCommand
extends Command

## Request software stop mode

func execute(actor: Node) -> void:
	if actor.has_method("send_test_motors"):
		actor.send_test_motors()
	else:
		push_error("Actor does not have test_motors method")
