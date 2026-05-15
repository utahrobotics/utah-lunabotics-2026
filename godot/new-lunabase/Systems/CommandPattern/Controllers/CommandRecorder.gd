class_name CommandRecorder
extends Node

## Class that records, executes, and plays back the command history

var command_history: Array[Command] = []

# ACTOR NEEDS TO BE ASSIGNED (This is whatever object will act on execution)
## Currently assigned to GlobalLunabaseConnection
var actor: Node
var default_path := "res://Systems/CommandPattern/SavedHistory/defaultHistory.tres"

func _init(_actor, _default_path):
	actor = _actor
	default_path = _default_path


var record_start_time := 0
var replay_index := 0
var replay_start_time := 0
var is_replaying := false
@export var throttle_inputs := true
# Default throttle time set to 100ms
@export var throttle_time_ms = 50
@export var zero_keepalive_time_ms = 5000

var throttle_elapsed_ms := 0.0

# Per-stream coalescing: only keep the newest command for each throttle stream.
var pending_commands: Dictionary = {}

# Last command sent per stream, used for duplicate suppression and keepalive.
var last_sent_commands: Dictionary = {}
var last_sent_time_ms: Dictionary = {}


func _ready() -> void:
	_refresh_processing_state()

# Calls the command and appends it to the command history
func execute_and_store(cmd: Command):
	var stream_key = _get_throttle_stream_key(cmd)
	
	# Only throttle the high-frequency actuator streams.
	if throttle_inputs and stream_key != "":
		pending_commands[stream_key] = cmd
	else:
		_execute_and_log(cmd)

# Saves the current history to a given path
func save_history(path := default_path):
	var history_res := HistoryResource.new()
	history_res.commands = command_history
	var result = ResourceSaver.save(history_res, path)
	
	if result != OK:
		push_error("Failed to save history")
	else:
		print("History saved to", path)

# Loads history from given path
func load_history(path := default_path):
	if ResourceLoader.exists(path):
		print("Loading history from ", path)
		var history_res: HistoryResource = ResourceLoader.load(path)
		command_history = history_res.commands.duplicate()
	else:
		print("Couldn't find history")

# Plays currently loaded command history
func start_replay():
	if command_history.is_empty():
		print("No commands to replay")
		return
	replay_index = 0
	replay_start_time = Time.get_ticks_msec()
	is_replaying = true
	
	print("Replay started")
	_refresh_processing_state()

func clear_history():
	print("clearing command history")
	stop_replay()
	command_history.clear()
	pending_commands.clear()
	last_sent_commands.clear()
	last_sent_time_ms.clear()
	throttle_elapsed_ms = 0.0

func stop_replay():
	is_replaying = false
	_refresh_processing_state()
	print("Replay stopped!")

func _process(_delta):
	if throttle_inputs:
		handle_throttle(_delta)
		_send_zero_keepalives_if_needed()

	if not is_replaying:
		return
	
	var elapsed_time = Time.get_ticks_msec() - replay_start_time
	
	# Tries to run commands at logged times 
	# TODO: Is there a better way to do this?
	while replay_index < command_history.size() and command_history[replay_index].timestamp <= elapsed_time:
		print("Replaying command at index", replay_index, " at ", elapsed_time, "ms (Originally logged at ", command_history[replay_index].timestamp, "ms)")
		command_history[replay_index].execute(actor)
		replay_index += 1

	# Ends replay at end of command history array
	if replay_index >= command_history.size():
		print("Replay finished")
		is_replaying = false
		_refresh_processing_state()
	
	
func handle_throttle(_delta):
	throttle_elapsed_ms += _delta * 1000.0
	
	# Returns if it isn't time to send packets
	if throttle_elapsed_ms < throttle_time_ms:
		return
	
	# Deterministic stream order.
	for stream_key in ["steering", "lift", "bucket", "dumper"]:
		if not pending_commands.has(stream_key):
			continue
		if not _is_stream_supported(stream_key):
			continue
		var command: Command = pending_commands[stream_key]
		if _should_send(stream_key, command):
			_execute_and_log(command)
			last_sent_commands[stream_key] = command
			last_sent_time_ms[stream_key] = Time.get_ticks_msec()
	
	pending_commands.clear()
	throttle_elapsed_ms = 0.0


func _send_zero_keepalives_if_needed() -> void:
	var now_ms: int = Time.get_ticks_msec()
	for stream_key in ["steering", "lift", "bucket", "dumper"]:
		if not last_sent_commands.has(stream_key):
			continue
		if not _is_stream_supported(stream_key):
			continue
		var last_cmd: Command = last_sent_commands[stream_key]
		if not _is_zero_command(last_cmd):
			continue
		var elapsed_since_last: int = now_ms - int(last_sent_time_ms.get(stream_key, now_ms))
		if elapsed_since_last < zero_keepalive_time_ms:
			continue
		var keepalive_cmd: Command = _build_zero_command_for_stream(stream_key)
		if keepalive_cmd == null:
			continue
		_execute_and_log(keepalive_cmd)
		last_sent_commands[stream_key] = keepalive_cmd
		last_sent_time_ms[stream_key] = now_ms


func _execute_and_log(cmd: Command) -> void:
	print("executing command!", cmd)
	if command_history.is_empty():
		print("Starting new command history")
		replay_index = 0
		record_start_time = Time.get_ticks_msec()
	cmd.timestamp = Time.get_ticks_msec() - record_start_time
	cmd.execute(actor)
	command_history.append(cmd)


func _get_throttle_stream_key(cmd: Command) -> String:
	if cmd is SteeringCommand:
		return "steering"
	if cmd is LiftActuatorsCommand:
		return "lift"
	if cmd is BucketActuatorsCommand:
		return "bucket"
	if cmd is DumperActuatorCommand:
		return "dumper"
	return ""


func _should_send(stream_key: String, cmd: Command) -> bool:
	if not last_sent_commands.has(stream_key):
		return true
	var previous: Command = last_sent_commands[stream_key]
	if _commands_equivalent(previous, cmd):
		# Zeros: do not spam; receiver + zero_keepalive handles refresh.
		if _is_zero_command(cmd):
			return false
		# Same non-zero held: re-send each throttle window so hardware keeps seeing commands.
		var now_ms: int = Time.get_ticks_msec()
		var elapsed_since_last: int = now_ms - int(last_sent_time_ms.get(stream_key, 0))
		return elapsed_since_last >= throttle_time_ms
	return true


func _commands_equivalent(a: Command, b: Command) -> bool:
	if a == null or b == null:
		return false
	if a.get_class() != b.get_class():
		return false
	if a is SteeringCommand and b is SteeringCommand:
		return (a as SteeringCommand).direction.is_equal_approx((b as SteeringCommand).direction)
	if a is LiftActuatorsCommand and b is LiftActuatorsCommand:
		return (a as LiftActuatorsCommand).lift == (b as LiftActuatorsCommand).lift
	if a is BucketActuatorsCommand and b is BucketActuatorsCommand:
		return (a as BucketActuatorsCommand).bucket == (b as BucketActuatorsCommand).bucket
	if a is DumperActuatorCommand and b is DumperActuatorCommand:
		return (a as DumperActuatorCommand).dumper == (b as DumperActuatorCommand).dumper
	return false


func _is_zero_command(cmd: Command) -> bool:
	if cmd is SteeringCommand:
		return (cmd as SteeringCommand).direction.is_equal_approx(Vector2.ZERO)
	if cmd is LiftActuatorsCommand:
		return (cmd as LiftActuatorsCommand).lift == 0
	if cmd is BucketActuatorsCommand:
		return (cmd as BucketActuatorsCommand).bucket == 0
	if cmd is DumperActuatorCommand:
		return (cmd as DumperActuatorCommand).dumper == 0
	return false


func _build_zero_command_for_stream(stream_key: String) -> Command:
	match stream_key:
		"steering":
			var steering := SteeringCommand.new()
			steering.direction = Vector2.ZERO
			return steering
		"lift":
			var lift := LiftActuatorsCommand.new()
			lift.lift = 0
			return lift
		"bucket":
			var bucket := BucketActuatorsCommand.new()
			bucket.bucket = 0
			return bucket
		"dumper":
			var dumper := DumperActuatorCommand.new()
			dumper.dumper = 0
			return dumper
		_:
			return null


func _is_stream_supported(stream_key: String) -> bool:
	match stream_key:
		"steering":
			return actor != null and actor.has_method("execute_steering")
		"lift":
			return actor != null and actor.has_method("send_lift_actuators")
		"bucket":
			return actor != null and actor.has_method("send_bucket_actuators")
		"dumper":
			return actor != null and actor.has_method("send_dumper_actuators")
		_:
			return false


func _refresh_processing_state() -> void:
	# Keep processing while replaying or while input throttling/keepalive is enabled.
	set_process(is_replaying or throttle_inputs)
