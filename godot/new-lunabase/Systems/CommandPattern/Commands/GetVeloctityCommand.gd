class_name GetVelocity
extends Command



# For getting the location.
func execute(actor: Node) -> void:
	if actor.has_method("get_velocity"):
		actor.get_location()
		
	else:
		push_error("Actor does not have get_velocity method")
