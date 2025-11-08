class_name LunabaseHumanController extends Node

@export var actor_path: NodePath
var actor: Node

@export var default_path := "res://Systems/CommandPattern/SavedHistory/lunabotHistory.tres"

@export var deadzone: float = 0.2

var command_recorder: CommandRecorder

# track previous state to detect button releases
var prev_lift_input: float = 0.0
var prev_bucket_input: float = 0.0

# previous steering to avoid sending duplicate commands
var prev_left_speed: float = 0.0
var prev_right_speed: float = 0.0


func _ready() -> void:
	if actor_path:
		actor = get_node(actor_path)
	
	if not actor:
		push_error("LunabaseHumanController: No actor assigned or path invalid!")
		return
	
	command_recorder = CommandRecorder.new(actor, default_path)
	add_child(command_recorder)


func _process(_delta: float) -> void:
	if not actor:
		return
	
	# === STEERING (Keyboard + Gamepad) ===
	# GAMEPAD INPUT IS UNTESTED, I DONT OWN A CONTROLLER. (this will need testing and tweaking likely)
	var forward_trigger := Input.get_joy_axis(0, JOY_AXIS_TRIGGER_RIGHT)
	var backward_trigger := Input.get_joy_axis(0, JOY_AXIS_TRIGGER_LEFT)
	var joy_turn := Input.get_joy_axis(0, JOY_AXIS_LEFT_X)
	
	var keyboard_forward := 1.0 if Input.is_action_pressed("move_forward") else 0.0
	var keyboard_backward := 1.0 if Input.is_action_pressed("move_backward") else 0.0
	var keyboard_turn := Input.get_axis("move_left", "move_right")
	
	var forward_input: float = max(forward_trigger, keyboard_forward)
	var backward_input: float = max(backward_trigger, keyboard_backward)
	var turn_input: float = joy_turn if abs(joy_turn) > deadzone else keyboard_turn
	
	# apply deadzone to turn input
	if abs(turn_input) < deadzone:
		turn_input = 0.0
	
	# forward backward speed
	var forward_backward: float = forward_input - backward_input
	
	# diff steering calculation
	var left_speed: float = forward_backward + turn_input
	var right_speed: float = forward_backward - turn_input
	
	left_speed = clamp(left_speed, -1.0, 1.0)
	right_speed = clamp(right_speed, -1.0, 1.0)
	
	# Only send steering command if it changed (avoid spamming)
	# I have had problems where if the trigger is partially pressed it sends a bajillion commands with tiny changes
	# this will need to be tested and perhapse rate limited if need be
	if abs(left_speed - prev_left_speed) > 0.01 or abs(right_speed - prev_right_speed) > 0.01:
		var cmd := SteeringCommand.new()
		cmd.direction = Vector2(left_speed, right_speed)
		command_recorder.execute_and_store(cmd)
		prev_left_speed = left_speed
		prev_right_speed = right_speed
	
	# === LIFT ACTUATORS (Keyboard Q/E + D-pad) ===
	# Sim env doesnt have actuators yet, this will need testing in real life
	var lift_input: float = 0.0
	

	if Input.is_action_pressed("lift_up"):
		lift_input = 1.0
	elif Input.is_action_pressed("lift_down"):
		lift_input = -1.0
	elif Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_UP):
		lift_input = 1.0
	elif Input.is_joy_button_pressed(0, JOY_BUTTON_DPAD_DOWN):
		lift_input = -1.0
	
	if lift_input != prev_lift_input:
		var cmd := LiftActuatorsCommand.new()
		cmd.lift = int(lift_input * 127.0)
		command_recorder.execute_and_store(cmd)
		prev_lift_input = lift_input
	
	# === BUCKET ACTUATORS (Keyboard Z/C + Y/A buttons) ===
	# This might be a bit clunky tbh, someone who is a gamer might have better ideas
	var bucket_input: float = 0.0
	
	if Input.is_action_pressed("bucket_up"):
		bucket_input = 1.0
	elif Input.is_action_pressed("bucket_down"):
		bucket_input = -1.0
	elif Input.is_joy_button_pressed(0, JOY_BUTTON_Y):
		bucket_input = 1.0
	elif Input.is_joy_button_pressed(0, JOY_BUTTON_A):
		bucket_input = -1.0
	
	if bucket_input != prev_bucket_input:
		var cmd := BucketActuatorsCommand.new()
		cmd.bucket = int(bucket_input * 127.0)
		command_recorder.execute_and_store(cmd)
		prev_bucket_input = bucket_input
	
	if Input.is_action_just_pressed("continue_mission"):
		command_recorder.execute_and_store(ContinueMissionCommand.new())
	if Input.is_action_just_pressed("soft_stop"):
		command_recorder.execute_and_store(SoftStopCommand.new())
	if Input.is_action_just_pressed("autonomy"):
		command_recorder.execute_and_store(NavigateCommand.new())
