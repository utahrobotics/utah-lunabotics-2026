class_name SetSpeedMultiplier
extends Command

## Skid steer command
@export var weight:float = 0.0

# For Adjusting SpeedMultiplier.
func execute(actor: Node) -> void:
	if actor.has_method("set_speed"):
		actor.set_speed(weight)
	else:
		push_error("Actor does not have execute_steering method")
