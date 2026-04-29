class_name SettingsMenuHandler extends Node
const ControlConfigLoaderScript = preload("res://Systems/ControlSchemes/control_config_loader.gd")

@onready var panel: Panel = $Control/Panel
@onready var control_scheme_switcher: ControlSchemeSwitcher = $Control/Panel/Panel

signal toggle_can_accept_inputs
signal updated_control_scheme
signal touch_screen_controls_changed(enabled: bool)

var touch_screen_controls_enabled: bool = false
var disable_ui_inputs := true
const UI_NAV_ACTIONS: PackedStringArray = [
	"ui_up",
	"ui_down",
	"ui_left",
	"ui_right",
	"ui_accept",
	"ui_cancel",
	"ui_focus_next",
	"ui_focus_prev",
]

func set_touch_screen_controls(enabled: bool) -> void:
	if touch_screen_controls_enabled == enabled:
		return
	touch_screen_controls_enabled = enabled
	touch_screen_controls_changed.emit(enabled)

func _ready() -> void:
	ControlConfigLoaderScript.ensure_loaded()
	if disable_ui_inputs:
		_disable_non_pointer_ui_navigation()
	control_scheme_switcher.updated_control_scheme.connect(_on_controls_updated)
	control_scheme_switcher.close_settings_menu.connect(_on_settings_button_toggled)


func _disable_non_pointer_ui_navigation() -> void:
	for action_name in UI_NAV_ACTIONS:
		if not InputMap.has_action(action_name):
			continue
		var action_events := InputMap.action_get_events(action_name)
		for event in action_events:
			if event is InputEventKey or event is InputEventJoypadButton or event is InputEventJoypadMotion:
				InputMap.action_erase_event(action_name, event)

func _on_controls_updated():
	if disable_ui_inputs:
			_disable_non_pointer_ui_navigation()
	updated_control_scheme.emit()

func toggle_menu_visibility(menu : Control, make_visible : bool):
	if make_visible:
		menu.show()
	else:
		menu.hide()

func _on_settings_button_toggled(toggled_on: bool = false) -> void:
	toggle_menu_visibility(panel, toggled_on)
	if toggled_on:
		control_scheme_switcher.sync_touch_controls_from_settings()
