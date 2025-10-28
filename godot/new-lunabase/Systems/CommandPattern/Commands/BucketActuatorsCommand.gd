class_name BucketActuatorsCommand
extends Command

## Move bucket actuators, positive up, negative down

# TODO: could be connected to LiftActuatorCommand and have variable change target
# for now will remain separate commands

# In rust its an i8 (-128 to 127)
@export var bucket: int = 0

func execute(actor: Node) -> void:
	if actor.has_method("send_bucket_actuators"):
		# Convert from i8 range (-128 to 127) to float range (-1.0 to 1.0)
		var speed: float = bucket / 127.0 if bucket >= 0 else bucket / 128.0
		actor.send_bucket_actuators(speed)
	else:
		push_error("Actor does not have send_bucket_actuators method")
