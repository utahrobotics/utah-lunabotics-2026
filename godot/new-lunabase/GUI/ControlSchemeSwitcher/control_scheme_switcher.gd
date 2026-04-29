class_name ControlSchemeSwitcher extends Control

enum ActiveInputSource {
	KEYBOARD_CONTROLLER,
	CUSTOM_SCHEME,
	CONFIG_FILE,
}

@onready var local_scheme_option_button: OptionButton = $VBoxContainer/HBoxContainer4/LocalSchemeOptionButton
@onready var controller_scheme_option_button: OptionButton = $VBoxContainer/HBoxContainer/ControllerSchemeOptionButton
@onready var keyboard_scheme_option_button: OptionButton = $VBoxContainer/HBoxContainer2/KeyboardSchemeOptionButton
@onready var touch_controls_check: CheckButton = $VBoxContainer/HBoxContainer3/TouchControlsCheckButton
@onready var active_source_option_button: OptionButton = $VBoxContainer/HBoxContainerSource/ActiveSourceOptionButton
@onready var active_source_label: Label = $VBoxContainer/ActiveSchemeLabel

@export var controller_path : String = "res://Systems/ControlSchemes/schemes/ControllerSchemes"
@export var keyboard_path : String = "res://Systems/ControlSchemes/schemes/KeyboardSchemes"
@export var custom_path : String = "user://bindings"
const REBINDING_SCENE = preload("res://Systems/ControlSchemes/RebindingScene/RebindingScene.tscn")
const ControlConfigLoaderScript = preload("res://Systems/ControlSchemes/control_config_loader.gd")
const UI_STATE_PATH := "user://control_scheme_switcher_state.cfg"

signal updated_control_scheme
signal close_settings_menu

var controller_schemes : Array[ControlSchemeResource]
var keyboard_schemes : Array[ControlSchemeResource]
var custom_schemes: Array[ControlSchemeResource]

var keyboard_prev_index := -1
var controller_prev_index := -1
var current_source: ActiveInputSource = ActiveInputSource.KEYBOARD_CONTROLLER

var custom_scheme_filenames: Array[String] = []
var _pending_source: int = ActiveInputSource.KEYBOARD_CONTROLLER
var _pending_keyboard_index: int = 0
var _pending_controller_index: int = 0
var _pending_custom_index: int = 0

func _ready() -> void:
	_setup_active_source_dropdown()
	_load_ui_state()
	populate_schemes()
	sync_touch_controls_from_settings()
	_update_source_ui_state()
	_update_active_scheme_label()

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
	local_scheme_option_button.add_item("None")
	
	# Load controller schemes
	for file in ResourceLoader.list_directory(controller_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(controller_path + "/" + file)
		if new_resource == null:
			continue
		controller_schemes.append(new_resource)
		controller_scheme_option_button.add_item(new_resource.scheme_name)
	
	for file in ResourceLoader.list_directory(keyboard_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(keyboard_path + "/" + file)
		if new_resource == null:
			continue
		keyboard_schemes.append(new_resource)
		keyboard_scheme_option_button.add_item(new_resource.scheme_name)
	
	for file in ResourceLoader.list_directory(custom_path):
		var new_resource : ControlSchemeResource = ResourceLoader.load(custom_path + "/" + file)
		if new_resource == null:
			continue
		custom_schemes.append(new_resource)
		custom_scheme_filenames.append(file)
		local_scheme_option_button.add_item(new_resource.scheme_name)
	
	if keyboard_schemes.size() > 0:
		var kb_i := clampi(_pending_keyboard_index, 0, keyboard_schemes.size() - 1) if not apply_default_keyboard_and_controller else 0
		keyboard_scheme_option_button.select(kb_i)
	if controller_schemes.size() > 0:
		var ctrl_i := clampi(_pending_controller_index, 0, controller_schemes.size() - 1) if not apply_default_keyboard_and_controller else 0
		controller_scheme_option_button.select(ctrl_i)
	if local_scheme_option_button.item_count > 0:
		var custom_i := clampi(_pending_custom_index, 0, local_scheme_option_button.item_count - 1)
		local_scheme_option_button.select(custom_i)
	current_source = clampi(_pending_source, 0, ActiveInputSource.CONFIG_FILE) as ActiveInputSource
	active_source_option_button.select(current_source)
	_apply_current_source()

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
	if current_source == ActiveInputSource.KEYBOARD_CONTROLLER:
		_apply_keyboard_controller_combo()
	_save_ui_state()
	_update_active_scheme_label()


func _on_controller_scheme_option_button_item_selected(index: int) -> void:
	if current_source == ActiveInputSource.KEYBOARD_CONTROLLER:
		_apply_keyboard_controller_combo()
	_save_ui_state()
	_update_active_scheme_label()


func _on_default_button_pressed() -> void:
	populate_schemes()
	current_source = ActiveInputSource.KEYBOARD_CONTROLLER
	active_source_option_button.select(current_source)
	_apply_current_source()
	_save_ui_state()

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
		if current_source == ActiveInputSource.KEYBOARD_CONTROLLER:
			_on_controller_scheme_option_button_item_selected(c_i)

	_select_custom_scheme_by_saved_path(saved_path)
	_update_active_scheme_label()


func _select_custom_scheme_by_saved_path(saved_path: String) -> void:
	if saved_path.is_empty():
		return
	var base: String = saved_path.get_file()
	for i in custom_scheme_filenames.size():
		if custom_scheme_filenames[i] == base:
			local_scheme_option_button.select(i + 1)
			current_source = ActiveInputSource.CUSTOM_SCHEME
			active_source_option_button.select(current_source)
			_apply_current_source()
			return
	if ResourceLoader.exists(saved_path):
		var res: ControlSchemeResource = ResourceLoader.load(saved_path) as ControlSchemeResource
		if res:
			for i in custom_schemes.size():
				if custom_schemes[i].scheme_name == res.scheme_name:
					local_scheme_option_button.select(i + 1)
					current_source = ActiveInputSource.CUSTOM_SCHEME
					active_source_option_button.select(current_source)
					_apply_current_source()
					return
	push_warning("ControlSchemeSwitcher: saved binding not found in list: %s" % base)


func _on_local_scheme_option_button_item_selected(index: int) -> void:
	if current_source == ActiveInputSource.CUSTOM_SCHEME:
		_apply_custom_scheme()
	_save_ui_state()
	_update_active_scheme_label()


func _setup_active_source_dropdown() -> void:
	active_source_option_button.clear()
	active_source_option_button.add_item("Keyboard + Controller")
	active_source_option_button.add_item("Custom Scheme")
	active_source_option_button.add_item("Config File (control_config.json)")
	active_source_option_button.select(current_source)


func _on_active_source_option_button_item_selected(index: int) -> void:
	current_source = index as ActiveInputSource
	_apply_current_source()
	_save_ui_state()


func _apply_current_source() -> void:
	match current_source:
		ActiveInputSource.KEYBOARD_CONTROLLER:
			_apply_keyboard_controller_combo()
		ActiveInputSource.CUSTOM_SCHEME:
			_apply_custom_scheme()
		ActiveInputSource.CONFIG_FILE:
			_apply_config_file()
	_update_source_ui_state()
	_update_active_scheme_label()


func _apply_keyboard_controller_combo() -> void:
	erase_all_action_events()
	if keyboard_scheme_option_button.selected >= 0 and keyboard_scheme_option_button.selected < keyboard_schemes.size():
		populate_from_scheme(keyboard_schemes[keyboard_scheme_option_button.selected])
	if controller_scheme_option_button.selected >= 0 and controller_scheme_option_button.selected < controller_schemes.size():
		populate_from_scheme(controller_schemes[controller_scheme_option_button.selected])


func _apply_custom_scheme() -> void:
	var selected_custom := local_scheme_option_button.selected - 1
	if selected_custom < 0 or selected_custom >= custom_schemes.size():
		return
	erase_all_action_events()
	populate_from_scheme(custom_schemes[selected_custom])


func _apply_config_file() -> void:
	ControlConfigLoaderScript.ensure_loaded()
	updated_control_scheme.emit()


func _update_source_ui_state() -> void:
	var kb_ctrl_active := current_source == ActiveInputSource.KEYBOARD_CONTROLLER
	var custom_active := current_source == ActiveInputSource.CUSTOM_SCHEME
	keyboard_scheme_option_button.disabled = not kb_ctrl_active
	controller_scheme_option_button.disabled = not kb_ctrl_active
	local_scheme_option_button.disabled = not custom_active


func _update_active_scheme_label() -> void:
	match current_source:
		ActiveInputSource.KEYBOARD_CONTROLLER:
			var kb_name := "None"
			var ctrl_name := "None"
			if keyboard_scheme_option_button.selected >= 0 and keyboard_scheme_option_button.selected < keyboard_schemes.size():
				kb_name = keyboard_schemes[keyboard_scheme_option_button.selected].scheme_name
			if controller_scheme_option_button.selected >= 0 and controller_scheme_option_button.selected < controller_schemes.size():
				ctrl_name = controller_schemes[controller_scheme_option_button.selected].scheme_name
			active_source_label.text = "Active input source: Keyboard + Controller | Keyboard: %s | Controller: %s" % [kb_name, ctrl_name]
		ActiveInputSource.CUSTOM_SCHEME:
			var custom_name := "None selected"
			var selected_custom := local_scheme_option_button.selected - 1
			if selected_custom >= 0 and selected_custom < custom_schemes.size():
				custom_name = custom_schemes[selected_custom].scheme_name
			active_source_label.text = "Active input source: Custom Scheme | Scheme: %s" % custom_name
		ActiveInputSource.CONFIG_FILE:
			active_source_label.text = "Active input source: Config File | Path: control_config.json (exe folder in build, user:// in editor)"


func _save_ui_state() -> void:
	var cfg := ConfigFile.new()
	cfg.set_value("control_scheme_switcher", "active_source", int(current_source))
	cfg.set_value("control_scheme_switcher", "keyboard_index", keyboard_scheme_option_button.selected)
	cfg.set_value("control_scheme_switcher", "controller_index", controller_scheme_option_button.selected)
	cfg.set_value("control_scheme_switcher", "custom_index", local_scheme_option_button.selected)
	cfg.save(UI_STATE_PATH)


func _load_ui_state() -> void:
	var cfg := ConfigFile.new()
	var err := cfg.load(UI_STATE_PATH)
	if err != OK:
		return
	_pending_source = int(cfg.get_value("control_scheme_switcher", "active_source", ActiveInputSource.KEYBOARD_CONTROLLER))
	_pending_keyboard_index = int(cfg.get_value("control_scheme_switcher", "keyboard_index", 0))
	_pending_controller_index = int(cfg.get_value("control_scheme_switcher", "controller_index", 0))
	_pending_custom_index = int(cfg.get_value("control_scheme_switcher", "custom_index", 0))
