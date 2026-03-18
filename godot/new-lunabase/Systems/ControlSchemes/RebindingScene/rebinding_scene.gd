extends Control

@onready var label = $StatusLabel
@export var current_scheme: ControlSchemeResource
var is_rebinding: bool = false
var current_action_to_bind: String = ""

@onready var skip_binding_button: Button = $LeftColumn/InformationPanel/MarginContainer/VBox/SkipBindingButton
@onready var confirm_bindings: Button = $LeftColumn/InformationPanel/MarginContainer/VBox/ConfirmBindings
@onready var save_bindings_button: Button = $LeftColumn/InformationPanel/MarginContainer/VBox/SaveBindingsButton

signal new_binding
signal input_received
signal new_message

func _ready() -> void:
	rebind_all_controls()
	update_ui()

func update_ui():
	skip_binding_button.visible = is_rebinding
	confirm_bindings.visible = not is_rebinding
	save_bindings_button.visible = not is_rebinding

func rebind_all_controls():
	var actions = current_scheme.get_all_actions()
	
	for action_name in actions.keys():
		label.text = "Press key for: " + action_name
		current_action_to_bind = action_name
		is_rebinding = true
		
		await self.input_received 
		
	label.text = "Binding Created!"
	is_rebinding = false
	update_ui()

func save_binding_to_file():
	var current_time_string := Time.get_datetime_string_from_system()
	var scheme_name := current_time_string +  "_binding.tres"
	var file_path = "user://" + scheme_name
	
	var error = ResourceSaver.save(current_scheme, file_path)
	var msg = ""
	if error != OK:
		msg = "Save failed with error code: " + error
	else:
		msg = "Resource saved successfully to: " + ProjectSettings.globalize_path(file_path)
	print(msg)
	var rich_text_msg = "[color=3399ff] %s [/color]" % msg
	new_message.emit(msg)


func _input(event: InputEvent) -> void:
	if not is_rebinding:
		return

	# MouseButton will not be a remappable thing due to it killing the UI, also why would you do that?
	if event is InputEventKey or event is InputEventJoypadButton:
		if event.is_pressed():
			apply_binding(current_action_to_bind, event)

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
	get_tree().reload_current_scene()

#TODO COMPLETE
func _on_confirm_bindings_pressed() -> void:
	pass # Replace with function body.

func _on_save_bindings_button_pressed() -> void:
	save_binding_to_file()
