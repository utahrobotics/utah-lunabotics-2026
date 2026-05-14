class_name DumpCommand
extends Command

## Request software stop mode

func execute(actor: Node) -> void:
	if actor.has_method("send_dump"):
		actor.send_dump()
	else:
		push_error("Actor does not have dump method")
