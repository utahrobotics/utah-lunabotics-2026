class_name ControlSchemeResource extends Resource

@export var scheme_name: String
@export var left_wheel_fwd: Array[InputEvent]
@export var left_wheel_back: Array[InputEvent]
@export var right_wheel_fwd: Array[InputEvent]
@export var right_wheel_back: Array[InputEvent]
@export var move_left: Array[InputEvent]
@export var move_right: Array[InputEvent]
@export var move_forward: Array[InputEvent]
@export var move_backward: Array[InputEvent]
@export var continue_mission: Array[InputEvent]
@export var soft_stop: Array[InputEvent]
@export var lift_up: Array[InputEvent]
@export var lift_down: Array[InputEvent]
@export var bucket_up: Array[InputEvent]
@export var bucket_down: Array[InputEvent]
@export var dumper_up: Array[InputEvent]
@export var dumper_down: Array[InputEvent]
@export var autonomy: Array[InputEvent]
@export var increment_speed: Array[InputEvent]
@export var decrement_speed: Array[InputEvent]

func update_action(action_name: String, events: Array[InputEvent]):
	if action_name in self:
		set(action_name, events)

func get_all_actions() -> Dictionary:
	var all_actions: Dictionary = {}
	all_actions["left_wheel_fwd"] = left_wheel_fwd
	all_actions["left_wheel_back"] = left_wheel_back
	all_actions["right_wheel_fwd"] = right_wheel_fwd
	all_actions["right_wheel_back"] = right_wheel_back
	all_actions["move_left"] = move_left
	all_actions["move_right"] = move_right
	all_actions["move_forward"] = move_forward
	all_actions["move_backward"] = move_backward
	all_actions["continue_mission"] = continue_mission
	all_actions["soft_stop"] = soft_stop
	all_actions["lift_up"] = lift_up
	all_actions["lift_down"] = lift_down
	all_actions["bucket_up"] = bucket_up
	all_actions["bucket_down"] = bucket_down
	all_actions["dumper_up"] = dumper_up
	all_actions["dumper_down"] = dumper_down
	all_actions["autonomy"] = autonomy
	all_actions["increment_speed"] = increment_speed
	all_actions["decrement_speed"] = decrement_speed
	return all_actions
