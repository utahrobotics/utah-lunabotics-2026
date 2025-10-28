class_name LiftActuatorsCommand
extends Command

## Move lift actuators, positive up, negative downed back

# In rust its an i8 (-128 to 127)
@export var lift: int = 0


func execute(actor: Node) -> void:
	if actor.has_method("send_lift_actuators"):
		# Convert from i8 range (-128 to 127) to float range (-1.0 to 1.0)
		var speed: float = lift / 127.0 if lift >= 0 else lift / 128.0
		actor.send_lift_actuators(speed)
	else:
		push_error("Actor does not have send_lift_actuators method")
