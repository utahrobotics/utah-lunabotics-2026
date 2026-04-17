extends Command
class_name SetSigmaSpatial

var new_spatial: float

func _init(p_new_spatial: float):
	new_spatial = p_new_spatial


func execute(actor: Node) -> void:
	if actor.has_method("send_set_sigma_spatial"):
		actor.send_set_sigma_spatial(new_spatial)
		print("sending set sigma spatial")
	else:
		push_error("Actor does not have send_set_sigma_spatial method")
