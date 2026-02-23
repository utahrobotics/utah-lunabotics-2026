class_name ControlSchemeSwitcher extends Control

@onready var controller_scheme_option_button: OptionButton = $Panel/VBoxContainer/HBoxContainer/ControllerSchemeOptionButton
@onready var keyboard_scheme_option_button: OptionButton = $Panel/VBoxContainer/HBoxContainer2/KeyboardSchemeOptionButton

@export var controller_path : String = "res://Systems/ControlSchemes/schemes/ControllerSchemes"
@export var keyboard_path : String = "res://Systems/ControlSchemes/schemes/KeyboardSchemes"

signal updated_control_scheme

var controller_schemes : Array[ControlSchemeResource]
var keyboard_schemes : Array[ControlSchemeResource]

var keyboard_prev_index := -1
var controller_prev_index := -1

func _ready() -> void:
	populate_schemes()

func populate_schemes():
	controller_scheme_option_button.clear()
	keyboard_scheme_option_button.clear()
	
	# Load controller schemes
	for file in ResourceLoader.list_directory(controller_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(controller_path + "/" + file)
		controller_schemes.append(new_resource)
		controller_scheme_option_button.add_item(new_resource.scheme_name)

	
	for file in ResourceLoader.list_directory(keyboard_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(keyboard_path + "/" + file)
		keyboard_schemes.append(new_resource)
		keyboard_scheme_option_button.add_item(new_resource.scheme_name)
	_on_keyboard_scheme_option_button_item_selected(0)
	_on_controller_scheme_option_button_item_selected(0)

func _process(_delta: float) -> void:
	if Input.is_action_just_pressed("left_wheel"):
		print("left wheel pressed")

func erase_all_action_events():
	var actions = InputMap.get_actions()
	for action in actions:
		InputMap.action_erase_events(action)

func unload_scheme(scheme_to_unload : ControlSchemeResource):
	var actions := scheme_to_unload.get_all_actions()
	var action_keys = actions.keys()
	
	for action in action_keys:
		for event in actions.get(action):
			if event is InputEvent and InputMap.has_action(action):
				print("removing event ", event, "from action ", action)
				InputMap.action_erase_event(action, event)

func populate_from_scheme(new_scheme : ControlSchemeResource):
	var new_actions := new_scheme.get_all_actions()
	var action_keys = new_actions.keys()
	
	for action in action_keys:
		for event in new_actions.get(action):
			if event is InputEvent and not null:
				print("adding event ", event, "to action ", action)
				InputMap.action_add_event(action, event)
	
	updated_control_scheme.emit()

func _on_keyboard_scheme_option_button_item_selected(index: int) -> void:
	if keyboard_prev_index >= 0:
		unload_scheme(keyboard_schemes[keyboard_prev_index])
	populate_from_scheme(keyboard_schemes[index])
	keyboard_prev_index = index


func _on_controller_scheme_option_button_item_selected(index: int) -> void:
	if controller_prev_index >= 0:
		unload_scheme(controller_schemes[controller_prev_index])
	populate_from_scheme(controller_schemes[index])
	controller_prev_index = index


func _on_default_button_pressed() -> void:
	populate_schemes()
