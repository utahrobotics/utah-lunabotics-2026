class_name LunabaseHumanController extends Node

@export var actor_path: NodePath
var actor: Node

@export var default_path := "res://Systems/CommandPattern/SavedHistory/lunabotHistory.tres"

@export var deadzone: float = 0.2

@onready var speed_slider = $"../VBoxContainer/MainContent/HBoxContainer/RightColumn/SpeedControl/MarginContainer/VBox/SpeedMultiplierSlider"

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

	# Clamping values from 0 to 1 to keep cohesion
	var forward_input : float = clampf(Input.get_action_strength("move_forward"), 0.0, 1.0)
	var backward_input : float = clampf(Input.get_action_strength("move_backward"), 0.0, 1.0)
	
	var turn_input: float = Input.get_action_strength("move_right") - Input.get_action_strength("move_left")
	
	# apply deadzone to turn input
	#TODO: Check if this is necessary, I think the input map already
	# handles deadzones which can be tweaked. Right now all inputs have a
	# deadzone of 0.2
	if abs(turn_input) < deadzone:
		turn_input = 0.0
		
	
	# forward backward speed
	var forward_backward: float = forward_input - backward_input
	
	# diff steering calculation
	
	# The inputs of moving the left or right wheels individually
	var left_wheel : float = Input.get_action_strength("left_wheel")
	var right_wheel : float = Input.get_action_strength("right_wheel")
	
	# Gets left and right wheel speed if using the forward/backward input
	var calculated_left_speed: float = forward_backward + turn_input
	var calculated_right_speed: float = forward_backward - turn_input
	
	# Should return the current used value
	var left_speed: float = left_wheel if left_wheel != 0.0 else calculated_left_speed
	var right_speed: float = right_wheel if right_wheel != 0.0 else calculated_right_speed
	
	left_speed = clamp(left_speed, -1, 1)
	right_speed = clamp(right_speed, -1, 1)
	
	# Only send steering command if it changed (avoid spamming)
	# I have had problems where if the trigger is partially pressed it sends a bajillion commands with tiny changes
	#not true anymore now hehehe -max
	# this will need to be tested and perhapse rate limited if need be
	#if abs(left_speed - prev_left_speed) > 0.01 or abs(right_speed - prev_right_speed) > 0.01:
	
	var steercmd := SteeringCommand.new()
	steercmd.direction = Vector2(left_speed, right_speed)
	command_recorder.execute_and_store(steercmd)
	prev_left_speed = left_speed
	prev_right_speed = right_speed
		
	
	# === LIFT ACTUATORS (Keyboard Q/E + D-pad) ===
	# Sim env doesnt have actuators yet, this will need testing in real life
	var lift_input: float = 0.0
	
	if Input.is_action_pressed("lift_up"):
		lift_input = 1.0
	elif Input.is_action_pressed("lift_down"):
		lift_input = -1.0
	
	
	var liftcmd := LiftActuatorsCommand.new()
	liftcmd.lift = int(lift_input * 127.0)
	command_recorder.execute_and_store(liftcmd)
	prev_lift_input = lift_input
		
	#=====Speed Slider increment and decrement
	const SPEED_SLIDER_STEP := 100
	if Input.is_action_pressed("increment_speed"):
		speed_slider.value = clamp(speed_slider.value + 
		SPEED_SLIDER_STEP, 
		speed_slider.min_value,speed_slider.max_value)
	
	if Input.is_action_pressed("decrement_speed"):
		speed_slider.value = clamp(speed_slider.value -
		SPEED_SLIDER_STEP,
		speed_slider.min_value,
		speed_slider.max_value)
	
	# === BUCKET ACTUATORS (Keyboard Z/C + Y/A buttons) ===
	# This might be a bit clunky tbh, someone who is a gamer might have better ideas
	var bucket_input: float = 0.0
	
	if Input.is_action_pressed("bucket_up"):
		bucket_input = 1.0
	elif Input.is_action_pressed("bucket_down"):
		bucket_input = -1.0
	
	#if bucket_input != prev_bucket_input:
	var buckcmd := BucketActuatorsCommand.new()
	buckcmd.bucket = int(bucket_input * 127.0)
	command_recorder.execute_and_store(buckcmd)
	prev_bucket_input = bucket_input
	
	if Input.is_action_just_pressed("continue_mission"):
		command_recorder.execute_and_store(ContinueMissionCommand.new())
	if Input.is_action_just_pressed("soft_stop"):
		command_recorder.execute_and_store(SoftStopCommand.new())
	if Input.is_action_just_pressed("autonomy"):
		command_recorder.execute_and_store(NavigateCommand.new())
