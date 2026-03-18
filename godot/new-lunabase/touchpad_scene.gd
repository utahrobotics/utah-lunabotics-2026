extends CanvasLayer

@onready var left_joy: VirtualJoystickPlus = $VirtualJoystickPlus
@onready var right_joy: VirtualJoystickPlus = $VirtualJoystickPlus2

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
	var left_value = left_joy.get_value()
	var right_value = right_joy.get_value()
	
