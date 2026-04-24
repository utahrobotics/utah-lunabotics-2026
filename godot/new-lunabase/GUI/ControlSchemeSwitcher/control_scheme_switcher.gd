class_name ControlSchemeSwitcher extends Control

@onready var local_scheme_option_button: OptionButton = $VBoxContainer/HBoxContainer4/LocalSchemeOptionButton
@onready var controller_scheme_option_button: OptionButton = $VBoxContainer/HBoxContainer/ControllerSchemeOptionButton
@onready var keyboard_scheme_option_button: OptionButton = $VBoxContainer/HBoxContainer2/KeyboardSchemeOptionButton
@onready var touch_controls_check: CheckButton = $VBoxContainer/HBoxContainer3/TouchControlsCheckButton

@export var controller_path : String = "res://Systems/ControlSchemes/schemes/ControllerSchemes"
@export var keyboard_path : String = "res://Systems/ControlSchemes/schemes/KeyboardSchemes"
@export var custom_path : String = "user://bindings"
const REBINDING_SCENE = preload("res://Systems/ControlSchemes/RebindingScene/RebindingScene.tscn")

signal updated_control_scheme
signal close_settings_menu

var controller_schemes : Array[ControlSchemeResource]
var keyboard_schemes : Array[ControlSchemeResource]
var custom_schemes: Array[ControlSchemeResource]

var keyboard_prev_index := -1
var controller_prev_index := -1

var custom_scheme_filenames: Array[String] = []

func _ready() -> void:
	populate_schemes()
	sync_touch_controls_from_settings()

func sync_touch_controls_from_settings() -> void:
	if touch_controls_check:
		touch_controls_check.set_pressed_no_signal(SettingsMenu.touch_screen_controls_enabled)


func _on_touch_controls_check_toggled(pressed: bool) -> void:
	SettingsMenu.set_touch_screen_controls(pressed)

func populate_schemes(apply_default_keyboard_and_controller: bool = true) -> void:
	keyboard_prev_index = -1
	controller_prev_index = -1

	controller_scheme_option_button.clear()
	keyboard_scheme_option_button.clear()
	local_scheme_option_button.clear()
	
	controller_schemes.clear()
	keyboard_schemes.clear()
	custom_schemes.clear()
	custom_scheme_filenames.clear()
	
	# Load controller schemes
	for file in ResourceLoader.list_directory(controller_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(controller_path + "/" + file)
		controller_schemes.append(new_resource)
		controller_scheme_option_button.add_item(new_resource.scheme_name)
	
	for file in ResourceLoader.list_directory(keyboard_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(keyboard_path + "/" + file)
		keyboard_schemes.append(new_resource)
		keyboard_scheme_option_button.add_item(new_resource.scheme_name)
	
	for file in ResourceLoader.list_directory(custom_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(custom_path + "/" + file)
		custom_schemes.append(new_resource)
		custom_scheme_filenames.append(file)
		local_scheme_option_button.add_item(new_resource.scheme_name)
	
	if apply_default_keyboard_and_controller:
		if keyboard_schemes.size() > 0:
			_on_keyboard_scheme_option_button_item_selected(0)
		if controller_schemes.size() > 0:
			_on_controller_scheme_option_button_item_selected(0)

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

func _on_create_binding_button_pressed() -> void:
	var bindings_scene = REBINDING_SCENE.instantiate()
	get_tree().root.add_child(bindings_scene)
	bindings_scene.binding_saved.connect(_on_binding_saved)
	close_settings_menu.emit()

func _on_binding_saved(saved_path: String) -> void:
	var kb_sel: int = keyboard_scheme_option_button.selected if keyboard_scheme_option_button.item_count > 0 else 0
	var ctrl_sel: int = controller_scheme_option_button.selected if controller_scheme_option_button.item_count > 0 else 0

	populate_schemes(false)

	if keyboard_schemes.size() > 0:
		var kb_i: int = clampi(kb_sel, 0, keyboard_schemes.size() - 1)
		keyboard_scheme_option_button.select(kb_i)
		_on_keyboard_scheme_option_button_item_selected(kb_i)
	if controller_schemes.size() > 0:
		var c_i: int = clampi(ctrl_sel, 0, controller_schemes.size() - 1)
		controller_scheme_option_button.select(c_i)
		_on_controller_scheme_option_button_item_selected(c_i)

	_select_custom_scheme_by_saved_path(saved_path)


func _select_custom_scheme_by_saved_path(saved_path: String) -> void:
	if saved_path.is_empty():
		return
	var base: String = saved_path.get_file()
	for i in custom_scheme_filenames.size():
		if custom_scheme_filenames[i] == base:
			local_scheme_option_button.select(i)
			_on_local_scheme_option_button_item_selected(i)
			return
	if ResourceLoader.exists(saved_path):
		var res: ControlSchemeResource = ResourceLoader.load(saved_path) as ControlSchemeResource
		if res:
			for i in custom_schemes.size():
				if custom_schemes[i].scheme_name == res.scheme_name:
					local_scheme_option_button.select(i)
					_on_local_scheme_option_button_item_selected(i)
					return
	push_warning("ControlSchemeSwitcher: saved binding not found in list: %s" % base)


func _on_local_scheme_option_button_item_selected(index: int) -> void:
	if index < 0 or index >= custom_schemes.size():
		return
	erase_all_action_events()
	populate_from_scheme(custom_schemes[index])
