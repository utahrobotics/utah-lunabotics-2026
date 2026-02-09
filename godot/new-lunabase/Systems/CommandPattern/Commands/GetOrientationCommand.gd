class_name GetOrientation
extends Command



# For getting the location.
func execute(actor: Node) -> void:
	if actor.has_method("get_orientation"):
		actor.get_location()
		
	else:
		push_error("Actor does not have get_location method")
