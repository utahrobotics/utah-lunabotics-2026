extends CanvasLayer

const SPEED_SLIDER_STEP := 100

@onready var left_joy: VirtualJoystickPlus = $VirtualJoystickPlus
@onready var touchscreen: LunabaseTouchscreenController = $TouchscreenController
@onready var label: Label = $IncDecPanel/VBoxContainer/HBoxContainer/TitlePanel/Label

var modes: PackedStringArray = ["movement_speed", "lift_angle", "bucket_height"]
var current_mode_index: int = 0

var _human: LunabaseHumanController


func _ready() -> void:
	var p := get_parent()
	if p:
		_human = p.get_node_or_null("LunabaseHumanController") as LunabaseHumanController
	visibility_changed.connect(_sync_human_suppression)
	SettingsMenu.touch_screen_controls_changed.connect(_set_touch_layer_enabled)
	visible = SettingsMenu.touch_screen_controls_enabled
	_sync_human_suppression()
	call_deferred("update_ui")
	call_deferred("_hook_speed_slider")


func _set_touch_layer_enabled(enabled: bool) -> void:
	visible = enabled


func _sync_human_suppression() -> void:
	if _human:
		_human.suppress_for_touch_ui = visible
	if not visible and touchscreen:
		touchscreen.touch_lift = 0.0
		touchscreen.touch_bucket = 0.0
		touchscreen.joystick_vector = Vector2.ZERO


func _hook_speed_slider() -> void:
	var slider := touchscreen.speed_slider
	if slider and not slider.value_changed.is_connected(_on_speed_slider_value_changed):
		slider.value_changed.connect(_on_speed_slider_value_changed)


func _on_speed_slider_value_changed(_value: float) -> void:
	if modes[current_mode_index] == "movement_speed":
		update_ui()


func _exit_tree() -> void:
	if _human:
		_human.suppress_for_touch_ui = false


func _process(_delta: float) -> void:
	if not visible:
		touchscreen.joystick_vector = Vector2.ZERO
		return
	touchscreen.joystick_vector = left_joy.get_value()


func _on_nextbutton_pressed() -> void:
	current_mode_index = (current_mode_index + 1) % modes.size()
	update_ui()


func _on_previous_button_pressed() -> void:
	current_mode_index = (current_mode_index - 1 + modes.size()) % modes.size()
	update_ui()


func _on_increment_button_down() -> void:
	match modes[current_mode_index]:
		"movement_speed":
			_adjust_speed_slider(1)
		"lift_angle":
			touchscreen.touch_lift = 1.0
		"bucket_height":
			touchscreen.touch_bucket = 1.0


func _on_decrement_button_down() -> void:
	match modes[current_mode_index]:
		"movement_speed":
			_adjust_speed_slider(-1)
		"lift_angle":
			touchscreen.touch_lift = -1.0
		"bucket_height":
			touchscreen.touch_bucket = -1.0


func _on_increment_button_up() -> void:
	match modes[current_mode_index]:
		"lift_angle":
			touchscreen.touch_lift = 0.0
		"bucket_height":
			touchscreen.touch_bucket = 0.0
		_:
			pass


func _on_decrement_button_up() -> void:
	match modes[current_mode_index]:
		"lift_angle":
			touchscreen.touch_lift = 0.0
		"bucket_height":
			touchscreen.touch_bucket = 0.0
		_:
			pass


func _adjust_speed_slider(direction: int) -> void:
	var slider := touchscreen.speed_slider
	if not slider:
		return
	slider.value = clamp(
		slider.value + float(direction * SPEED_SLIDER_STEP),
		slider.min_value,
		slider.max_value
	)
	update_ui()


func update_ui() -> void:
	var mode: String = modes[current_mode_index]
	var suffix := ""
	if mode == "movement_speed" and touchscreen.speed_slider:
		suffix = " (%d)" % int(touchscreen.speed_slider.value)
	var readable := mode.replace("_", " ").capitalize()
	label.text = "Controlling: %s%s" % [readable, suffix]
