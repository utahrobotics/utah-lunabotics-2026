class_name DumperActuatorCommand
extends Command

# Move dumper actuators, positive up, negative down

# In rust its an i8 (-128 to 127)
@export var dumper: int = 0
static var _warned_missing_method := false

func execute(actor: Node) -> void:
	if actor.has_method("send_dumper_actuators"):
		# Convert from i8 range (-128 to 127) to float range (-1.0 to 1.0)
		var speed: float = dumper / 127.0 if dumper >= 0 else dumper / 128.0
		actor.send_dumper_actuators(speed)
	else:
		if not _warned_missing_method:
			_warned_missing_method = true
			push_warning("Actor does not have send_dumper_actuators method")
