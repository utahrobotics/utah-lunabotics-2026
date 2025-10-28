class_name SteeringCommand
extends Command

## Skid steer command
@export var direction: Vector2 = Vector2.ZERO

# For Skid steer message, 1,1 is full speed forward, -1,-1 is full speed back
func execute(actor: Node) -> void:
	if actor.has_method("execute_steering"):
		actor.execute_steering(direction.x, direction.y)
	else:
		push_error("Actor does not have execute_steering method")
