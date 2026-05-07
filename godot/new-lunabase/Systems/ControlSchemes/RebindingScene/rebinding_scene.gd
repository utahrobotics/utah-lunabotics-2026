extends Control
@onready var current_control_interface_v_box: VBoxContainer = $LeftColumn/CurrentControlsPanel/MarginContainer/CurrentControlInterfaceVBox
@onready var label = $StatusLabel
@export var current_scheme: ControlSchemeResource

var is_rebinding: bool = false
var current_action_to_bind: String = ""

@onready var naming_panel: Panel = $NamingPanel

@onready var skip_binding_button: Button = $LeftColumn/InformationPanel/MarginContainer/VBox/SkipBindingButton
@onready var confirm_bindings: Button = $LeftColumn/InformationPanel/MarginContainer/VBox/ConfirmBindings
@onready var save_bindings_button: Button = $LeftColumn/InformationPanel/MarginContainer/VBox/SaveBindingsButton
@onready var line_edit: LineEdit = $NamingPanel/MarginContainer/VBoxContainer/LineEdit

signal new_binding
signal input_received
signal new_message

signal binding_saved(saved_path: String)
const JOYPAD_AXIS_BIND_THRESHOLD := 0.5
const JOYPAD_AXIS_RELEASE_THRESHOLD := 0.2

var waiting_for_axis_release := false
var pending_axis_device := -1
var pending_axis := -1

func _ready() -> void:
	setup()

func setup():
	naming_panel.hide()
	is_rebinding = false
	current_action_to_bind = ""
	waiting_for_axis_release = false
	pending_axis_device = -1
	pending_axis = -1
	current_scheme = ControlSchemeResource.new()
	rebind_all_controls()
	update_ui()
	current_control_interface_v_box.setup()

func update_ui():
	skip_binding_button.visible = is_rebinding
	confirm_bindings.visible = not is_rebinding
	save_bindings_button.visible = not is_rebinding

func rebind_all_controls():
	var actions = current_scheme.get_all_actions()
	
	for action_name in actions.keys():
		label.text = "Press input for: " + action_name
		current_action_to_bind = action_name
		is_rebinding = true
		
		await self.input_received 
		
	label.text = "Binding Created!"
	is_rebinding = false
	update_ui()

func save_binding_to_file():
	var scheme_name = line_edit.text if line_edit.text != "" else "custom_binding"
	scheme_name += ".tres"
	
	var dir_path = "user://bindings/"
	var file_path = dir_path + scheme_name
	
	current_scheme.scheme_name = scheme_name
	
	if not DirAccess.dir_exists_absolute(dir_path):
		DirAccess.make_dir_recursive_absolute(dir_path)
	
	var error = ResourceSaver.save(current_scheme, file_path)
	
	var msg = ""
	if error != OK:
		msg = "Save failed with error code: " + str(error)
	else:
		msg = "Resource saved successfully to: " + ProjectSettings.globalize_path(file_path)
		binding_saved.emit(file_path)
	print(msg)
	var rich_text_msg = "[color=3399ff] %s [/color]" % msg
	new_message.emit(msg)
	naming_panel.hide()

func _input(event: InputEvent) -> void:
	if not is_rebinding:
		return
	if waiting_for_axis_release:
		_handle_axis_release(event)
		return
	# MouseButton is intentionally excluded so UI remains usable.
	if not _is_bindable_event(event):
		return
	var bind_event := _build_bind_event(event)
	if bind_event == null:
		return
	apply_binding(current_action_to_bind, bind_event)
	if bind_event is InputEventJoypadMotion:
		waiting_for_axis_release = true
		pending_axis_device = bind_event.device
		pending_axis = bind_event.axis
		label.text = "Release stick to continue..."
	else:
		input_received.emit()
	get_viewport().set_input_as_handled()


func _is_bindable_event(event: InputEvent) -> bool:
	if event is InputEventKey:
		return event.is_pressed() and not event.echo
	if event is InputEventJoypadButton:
		return event.is_pressed()
	if event is InputEventJoypadMotion:
		return abs(event.axis_value) >= JOYPAD_AXIS_BIND_THRESHOLD
	return false


func _build_bind_event(event: InputEvent) -> InputEvent:
	if event is InputEventJoypadMotion:
		var motion := InputEventJoypadMotion.new()
		motion.device = event.device
		motion.axis = event.axis
		# Save a deterministic direction for the action (-1 or 1)
		motion.axis_value = 1.0 if event.axis_value >= 0.0 else -1.0
		return motion
	return event


func _handle_axis_release(event: InputEvent) -> void:
	if not (event is InputEventJoypadMotion):
		return
	if event.device != pending_axis_device or event.axis != pending_axis:
		return
	if abs(event.axis_value) > JOYPAD_AXIS_RELEASE_THRESHOLD:
		return
	waiting_for_axis_release = false
	pending_axis_device = -1
	pending_axis = -1
	input_received.emit()
	get_viewport().set_input_as_handled()

func apply_binding(action_name: String, event: InputEvent):
	if not InputMap.has_action(action_name):
		InputMap.add_action(action_name)
	
	InputMap.action_erase_events(action_name)
	InputMap.action_add_event(action_name, event)
	
	var new_events: Array[InputEvent] = [event]
	current_scheme.update_action(action_name, new_events)
	new_binding.emit(action_name, event.as_text())
	
	print("Bound %s to %s" % [action_name, event.as_text()])

func skip_binding():
	if not is_rebinding:
		return
	print("Skipped %s" % [current_action_to_bind])
	InputMap.action_erase_events(current_action_to_bind)
	new_binding.emit(current_action_to_bind, "skipped")
	input_received.emit()

func _on_skip_binding_button_pressed() -> void:
	skip_binding()

func _on_rebind_pressed() -> void:
	setup()

func _on_confirm_bindings_pressed() -> void:
	SettingsMenu.toggle_can_accept_inputs.emit(true)
	SettingsMenu.updated_control_scheme.emit()
	self.queue_free()

func _on_save_bindings_button_pressed() -> void:
	open_text_edit()

func open_text_edit():
	naming_panel.show()

func _on_confirm_name_pressed() -> void:
	save_binding_to_file()


func _on_line_edit_text_submitted(new_text: String) -> void:
	save_binding_to_file()
