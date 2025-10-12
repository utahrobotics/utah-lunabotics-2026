class_name TestMoveCommand
extends Command


## This is a test class meant to show off how action commands are created
##
## Handles movement commands for the test scene

@export var direction: Vector2 = Vector2.ZERO
@export var speed: float = 200.0

func execute(actor: Node) -> void:
	if actor.has_method("apply_movement"):
		actor.apply_movement(direction * speed)
