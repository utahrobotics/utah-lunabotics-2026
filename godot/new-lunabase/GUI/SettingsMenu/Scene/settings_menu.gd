extends Control

@onready var panel: Panel = $Panel

func toggle_menu_visibility(menu : Control, make_visible : bool):
	if make_visible:
		menu.show()
	else:
		menu.hide()


func _on_settings_button_toggled(toggled_on: bool) -> void:
	toggle_menu_visibility(panel, toggled_on)
