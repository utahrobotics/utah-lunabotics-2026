class_name SettingsMenuHandler extends Node

@onready var panel: Panel = $Control/Panel
@onready var control_scheme_switcher: ControlSchemeSwitcher = $Control/Panel/ControlSchemeSwitcher

func toggle_menu_visibility(menu : Control, make_visible : bool):
	if make_visible:
		menu.show()
	else:
		menu.hide()


func _on_settings_button_toggled(toggled_on: bool) -> void:
	toggle_menu_visibility(panel, toggled_on)
