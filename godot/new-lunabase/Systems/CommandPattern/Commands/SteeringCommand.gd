class_name SteeringCommand
extends Command

## Skid steer command
@export var direction: Vector2 = Vector2.ZERO

# For Skid steer message, 1,1 is full speed forward, -1,-1 is full speed back
# TODO: EXECUTE
func execute(actor: Node) -> void:
	pass
