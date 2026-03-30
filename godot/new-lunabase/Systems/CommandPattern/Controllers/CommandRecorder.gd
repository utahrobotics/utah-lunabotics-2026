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
@export var throttle_time_ms = 200

var time := 0.0

var throttled_inputs : Array[Command]

# Calls the command and appends it to the command history
func execute_and_store(cmd: Command):
	if command_history.is_empty():
		print("Starting new command history")
		replay_index = 0
		record_start_time = Time.get_ticks_msec()
		
	cmd.timestamp = Time.get_ticks_msec() - record_start_time
	
	if throttle_inputs:
		throttled_inputs.append(cmd)
	else:
		# Input isn't throttled and is executed immediately
		cmd.execute(actor)
	command_history.append(cmd)

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
	set_process(true)

func clear_history():
	print("clearing command history")
	stop_replay()
	command_history.clear()

func stop_replay():
	is_replaying = false
	set_process(false)
	print("Replay stopped!")

func _process(_delta):
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
		set_process(false)
	
	if throttled_inputs:
		handle_throttle(_delta)

func handle_throttle(_delta):
	time += _delta
	
	# Returns if it isn't time to send packets
	if time < throttle_time_ms:
		return
	
	for command in throttled_inputs:
		# TODO Add checker. If input was previouslyt set to 0 and current command is also 0 then don't execute.
		# TODO possibly create an array of commands with 0 values, if command is 0 and in array then drop. If command is non 0 and is in array then delete entry in array
		print("command executed")
		command.execute(actor)
	
	# Remove all throttled inputs
	throttled_inputs.clear()
	time = 0
