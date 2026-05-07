class_name LunabaseTouchscreenController extends Node

# x = turn, y = up negative
var joystick_vector: Vector2 = Vector2.ZERO

# (-1, 0, 1).
var touch_lift: float = 0.0
var touch_bucket: float = 0.0

@export var actor_path: NodePath
@export var is_actor_lunabase_connection := true

@export var default_path := "res://Systems/CommandPattern/SavedHistory/lunabotHistory.tres"

@export var deadzone: float = 0.2

var speed_slider: HSlider

var actor: Node

var can_send_inputs := true

var command_recorder: CommandRecorder



func _ready() -> void:
	if actor_path:
		actor = get_node(actor_path)

	if is_actor_lunabase_connection:
		actor = GlobalLunabaseConnection

	if not actor:
		push_error("LunabaseTouchscreenController: No actor assigned or path invalid")
		return

	var main_control := get_parent().get_parent() as Node
	var human: LunabaseHumanController = null
	if main_control:
		speed_slider = main_control.get_node_or_null(
			"VBoxContainer/MainContent/HBoxContainer/RightColumn/SpeedControl/MarginContainer/VBox/SpeedMultiplierSlider"
		) as HSlider
		human = main_control.get_node_or_null("LunabaseHumanController") as LunabaseHumanController

	if human and human.command_recorder:
		command_recorder = human.command_recorder
	else:
		command_recorder = CommandRecorder.new(actor, default_path)
		add_child(command_recorder)
	SettingsMenu.toggle_can_accept_inputs.connect(set_can_send_inputs)


func set_can_send_inputs(_can_send_inputs):
	can_send_inputs = _can_send_inputs


func _process(_delta: float) -> void:
	if not actor or not can_send_inputs:
		return

	var j := joystick_vector
	var forward_input: float = clampf(maxf(0.0, -j.y), 0.0, 1.0)
	var backward_input: float = clampf(maxf(0.0, j.y), 0.0, 1.0)
	var turn_input: float = j.x

	if abs(turn_input) < deadzone:
		turn_input = 0.0

	var forward_backward: float = forward_input - backward_input
	var calculated_left_speed: float = forward_backward + turn_input
	var calculated_right_speed: float = forward_backward - turn_input

	var left_speed: float = calculated_left_speed
	var right_speed: float = calculated_right_speed

	left_speed = clamp(left_speed, -1, 1)
	right_speed = clamp(right_speed, -1, 1)

	var steer_cmd := SteeringCommand.new()
	steer_cmd.direction = Vector2(left_speed, right_speed)
	command_recorder.execute_and_store(steer_cmd)

	var lift_input: float = touch_lift
	var lift_cmd := LiftActuatorsCommand.new()
	lift_cmd.lift = int(lift_input * 127.0)
	command_recorder.execute_and_store(lift_cmd)

	const SPEED_SLIDER_STEP := 100
	if speed_slider and Input.is_action_pressed("increment_speed"):
		speed_slider.value = clamp(
			speed_slider.value + SPEED_SLIDER_STEP,
			speed_slider.min_value,
			speed_slider.max_value
		)
	if speed_slider and Input.is_action_pressed("decrement_speed"):
		speed_slider.value = clamp(
			speed_slider.value - SPEED_SLIDER_STEP,
			speed_slider.min_value,
			speed_slider.max_value
		)

	var bucket_input: float = touch_bucket
	var bucket_cmd := BucketActuatorsCommand.new()
	bucket_cmd.bucket = int(bucket_input * 127.0)
	command_recorder.execute_and_store(bucket_cmd)

	if Input.is_action_just_pressed("continue_mission"):
		command_recorder.execute_and_store(ContinueMissionCommand.new())
	if Input.is_action_just_pressed("soft_stop"):
		command_recorder.execute_and_store(SoftStopCommand.new())
	if Input.is_action_just_pressed("autonomy"):
		command_recorder.execute_and_store(NavigateCommand.new())
