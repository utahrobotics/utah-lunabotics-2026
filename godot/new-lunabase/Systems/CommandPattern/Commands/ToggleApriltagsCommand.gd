extends Command
class_name ToggleApriltagsCommand

var enabled: bool

func _init(p_enabled: bool) -> void:
	enabled = p_enabled

func execute(actor: Node) -> void:
	if actor.has_method("send_toggle_apriltags"):
		actor.send_toggle_apriltags(enabled)
	else:
		push_error("Actor does not have send_toggle_apriltags method")
