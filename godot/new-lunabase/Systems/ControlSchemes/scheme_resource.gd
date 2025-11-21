class_name ControlSchemeResource extends Resource

@export var scheme_name : String

@export var left_wheel  : Array[InputEvent]
@export var right_wheel  : Array[InputEvent]
@export var move_left  : Array[InputEvent]
@export var move_right  : Array[InputEvent]
@export var move_forward  : Array[InputEvent]
@export var move_backward  : Array[InputEvent]
@export var continue_mission  : Array[InputEvent]
@export var stop_mission  : Array[InputEvent]
@export var lift_up  : Array[InputEvent]
@export var lift_down  : Array[InputEvent]
@export var bucket_up  : Array[InputEvent]
@export var bucket_down  : Array[InputEvent]
@export var autonomy  : Array[InputEvent]

func get_all_actions() -> Dictionary:
	var all_actions : Dictionary = {}
	all_actions["left_wheel"] = left_wheel
	all_actions["right_wheel"] = right_wheel
	all_actions["move_left"] = move_left
	all_actions["move_right"] = move_right
	all_actions["move_forward"] = move_forward
	all_actions["move_backward"] = move_backward
	all_actions["continue_mission"] = continue_mission
	all_actions["stop_mission"] = stop_mission
	all_actions["lift_up"] = lift_up
	all_actions["lift_down"] = lift_down
	all_actions["bucket_up"] = bucket_up
	all_actions["bucket_down"] = bucket_down
	all_actions["autonomy"] = autonomy
	return all_actions
