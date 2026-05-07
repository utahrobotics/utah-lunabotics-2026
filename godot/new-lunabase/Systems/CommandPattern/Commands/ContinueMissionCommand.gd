class_name ContinueMissionCommand
extends Command

## Command to resume from stop to manual mode

func execute(actor: Node) -> void:
	if actor.has_method("send_continue_mission"):
		actor.send_continue_mission()
	else:
		push_error("Actor does not have send_continue_mission method")
