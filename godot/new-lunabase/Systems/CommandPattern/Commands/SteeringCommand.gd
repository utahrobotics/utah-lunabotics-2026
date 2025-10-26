class_name SteeringCommand
extends Command

## Skid steer command
@export var direction: Vector2 = Vector2.ZERO

# For Skid steer message, 1,1 is full speed forward, -1,-1 is full speed back
# TODO: EXECUTE

var left: float = 0.0
var right: float = 0.0

func _init(leftVal: float, rightVal: float) -> void:
	left = leftVal
	right = rightVal

func execute(actor: Node) -> void:
	var rustext = actor.get_node("../Main")
	rustext.set_steering(left,right)
