class_name BucketShakeActuatorsCommand
extends Command

## Shake bucket actuators

# TODO: could be connected to LiftActuatorCommand and have variable change target
# for now will remain separate commands

# In rust its an i8 (-128 to 127)
@export var bucket: int = 0
# u8
@export var period: int = 0

func execute(actor: Node) -> void:
	if actor.has_method("send_shake_bucket_actuators"):
		# Convert from i8 range (-128 to 127) to float range (-1.0 to 1.0)
		var speed: float = bucket / 127.0 if bucket >= 0 else bucket / 128.0
		var frequency: float = 1.0 / (period / 1000.0)
		actor.send_shake_bucket_actuators(speed, frequency)
	else:
		push_error("Actor does not have send_shake_bucket_actuators method")
