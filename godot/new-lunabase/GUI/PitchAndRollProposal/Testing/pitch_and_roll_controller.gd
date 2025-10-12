class_name TestHumanController extends Node

## This is the human controller for the test scene
##
## Controllers should have a child CommandRecorder (This is currently done by instancing one). 
## Controllers are the ones responsible for handling input and the CommandRecorder 
## handles execution and logging of commands.
## While this is a test class, the get_movement_vector() function could be reusued
## for the actual input handler.

# The node that takes in the command
@export var actor: Node

# Path to save the command history
@export var default_path := "res://Systems/CommandPattern/SavedHistory/defaultHistory.tres"

# Joystick deadzone
@export var deadzone : float = 0.2

var command_recorder : CommandRecorder

func _ready() -> void:
	# Creates CommandRecorder and adds it as a child
	command_recorder = CommandRecorder.new(actor, default_path)
	add_child(command_recorder)
	

func _process(_delta):
	var direction = get_movement_vector()

	# Gets called to record and execute movement vector
	if direction != Vector2.ZERO:
		var cmd := PitchRollCommand.new()
		cmd.pitch = direction.y / 100
		cmd.roll = direction.x / 100
		command_recorder.execute_and_store(cmd)

# Returns movement vector either through analog or digital through the input map
func get_movement_vector() -> Vector2:
	var input_vector := Vector2.ZERO
	input_vector = Vector2(Input.get_axis("move_left", "move_right"), Input.get_axis("move_forward", "move_backward"))

	# Analog input from left stick on controller
	var joy_vector := Vector2(
		Input.get_joy_axis(0, JOY_AXIS_LEFT_X),
		Input.get_joy_axis(0, JOY_AXIS_LEFT_Y)
	)

	if joy_vector.length() > deadzone:
		input_vector = joy_vector

	return input_vector.normalized()
