extends Command
class_name SetSigmaRange

var new_range: float

func _init(p_new_range: float):
	new_range = p_new_range


func execute(actor: Node) -> void:
	if actor.has_method("send_set_sigma_range"):
		actor.send_set_sigma_range(new_range)
	else:
		push_error("Actor does not have send_set_sigma_range method")
