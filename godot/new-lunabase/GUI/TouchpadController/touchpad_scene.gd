extends CanvasLayer

@onready var left_joy: VirtualJoystickPlus = $VirtualJoystickPlus
var movement_vector : Vector2
enum control_mode {bucket_up_down, speed_up_down}
@onready var label: Label = $IncDecPanel/VBoxContainer/HBoxContainer/TitlePanel/Label

#TODO Implement
#@export var left_wheel_axis  : Array[InputEvent]
#@export var right_wheel_axis  : Array[InputEvent]
#@export var move_left  : Array[InputEvent]
#@export var move_right  : Array[InputEvent]
#@export var move_forward  : Array[InputEvent]
#@export var move_backward  : Array[InputEvent]
#@export var continue_mission  : Array[InputEvent]
#@export var soft_stop  : Array[InputEvent]
#@export var lift_up  : Array[InputEvent]
#@export var lift_down  : Array[InputEvent]
#@export var bucket_up  : Array[InputEvent]
#@export var bucket_down  : Array[InputEvent]
#@export var autonomy  : Array[InputEvent]
#@export var increment_speed  : Array[InputEvent]
#@export var decrement_speed  : Array[InputEvent]


func _process(delta: float) -> void:
	movement_vector = left_joy.get_value()

var modes = ["bucket_height", "movement_speed", "lift_angle"]
var current_mode_index : int = 0

var stats = {
	"bucket_height": 0,
	"movement_speed": 5,
	"lift_angle": 0,
	"autonomy_level": 1
}

func _on_nextbutton_pressed() -> void:
	current_mode_index = (current_mode_index + 1) % modes.size()
	update_ui()

func _on_previous_button_pressed() -> void:
	current_mode_index = (current_mode_index - 1 + modes.size()) % modes.size()
	update_ui()

func _on_increment_button_down() -> void:
	var key = modes[current_mode_index]
	stats[key] += 1 
	print("Increased ", key, " to ", stats[key])

func _on_decrement_button_down() -> void:
	var key = modes[current_mode_index]
	stats[key] -= 1
	print("Decreased ", key, " to ", stats[key])

func update_ui():
	label.text = "Controlling: " + modes[current_mode_index].capitalize()
