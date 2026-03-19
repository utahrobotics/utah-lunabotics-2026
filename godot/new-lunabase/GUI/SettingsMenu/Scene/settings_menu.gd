class_name SettingsMenuHandler extends Node

@onready var panel: Panel = $Control/Panel
@onready var control_scheme_switcher: ControlSchemeSwitcher = $Control/Panel/Panel

signal toggle_can_accept_inputs
signal updated_control_scheme

func _ready() -> void:
	control_scheme_switcher.updated_control_scheme.connect(_on_controls_updated)
	control_scheme_switcher.close_settings_menu.connect(_on_settings_button_toggled)

func _on_controls_updated():
	updated_control_scheme.emit()

func toggle_menu_visibility(menu : Control, make_visible : bool):
	if make_visible:
		menu.show()
	else:
		menu.hide()

func _on_settings_button_toggled(toggled_on: bool = false) -> void:
	toggle_menu_visibility(panel, toggled_on)
