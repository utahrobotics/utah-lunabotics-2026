class_name ControlConfigLoader
extends RefCounted

const USER_CONFIG_PATH := "user://control_config.json"
const EXTERNAL_CONFIG_FILENAME := "control_config.json"
const VERSION := 1
const FLAGS_KEY := "flags"
const DEFAULT_FLAGS := {
	"apply_steering_axis_deadzone": true,
	"steering_axis_deadzone": 0.2,
	"apply_trigger_deadzone": true,
	"trigger_deadzone": 0.05,
	"invert_lift_default_direction": false,
	"invert_bucket_default_direction": true,
	"invert_dumper_default_direction": false,
}

const ACTIONS: PackedStringArray = [
	"left_wheel_fwd",
	"left_wheel_back",
	"right_wheel_fwd",
	"right_wheel_back",
	"move_left",
	"move_right",
	"move_forward",
	"move_backward",
	"continue_mission",
	"soft_stop",
	"lift_up",
	"lift_down",
	"bucket_up",
	"bucket_down",
	"dumper_up",
	"dumper_down",
	"lift_analog_up",
	"lift_analog_down",
	"lift_reverse",
	"bucket_analog_up",
	"bucket_analog_down",
	"bucket_reverse",
	"dumper_analog_up",
	"dumper_analog_down",
	"dumper_reverse",
	"autonomy",
	"increment_speed",
	"decrement_speed",
]


static func ensure_loaded() -> void:
	var config_path := _resolve_config_path()
	if not FileAccess.file_exists(config_path):
		_write_current_inputmap_to_file(config_path)
		return
	_apply_from_file(config_path)


static func _apply_from_file(path: String) -> void:
	var file := FileAccess.open(path, FileAccess.READ)
	if file == null:
		return
	var parsed = JSON.parse_string(file.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY:
		return
	var actions: Dictionary = parsed.get("actions", {})
	for action_name in actions.keys():
		if not InputMap.has_action(action_name):
			continue
		InputMap.action_erase_events(action_name)
		for token in actions[action_name]:
			var event := _token_to_event(str(token))
			if event != null:
				InputMap.action_add_event(action_name, event)


static func _write_current_inputmap_to_file(path: String) -> void:
	var actions: Dictionary = {}
	for action_name in ACTIONS:
		if not InputMap.has_action(action_name):
			continue
		var tokens: Array[String] = []
		for event in InputMap.action_get_events(action_name):
			var token := _event_to_token(event)
			if token != "":
				tokens.append(token)
		actions[action_name] = tokens
	var payload := {
		"version": VERSION,
		"readme": "Tokens: keyboard_<key> | gamepad_button_<index> | gamepad_axis_<index>_positive|negative",
		FLAGS_KEY: DEFAULT_FLAGS.duplicate(true),
		"actions": actions,
	}
	var file := FileAccess.open(path, FileAccess.WRITE)
	if file:
		file.store_string(JSON.stringify(payload, "\t"))


static func apply_controller_flags(controller: Node) -> void:
	if controller == null:
		return
	var config_path := _resolve_config_path()
	if not FileAccess.file_exists(config_path):
		return
	var file := FileAccess.open(config_path, FileAccess.READ)
	if file == null:
		return
	var parsed = JSON.parse_string(file.get_as_text())
	if typeof(parsed) != TYPE_DICTIONARY:
		return
	var flags: Dictionary = parsed.get(FLAGS_KEY, {})
	if typeof(flags) != TYPE_DICTIONARY:
		return
	_apply_bool_flag(controller, flags, "apply_steering_axis_deadzone")
	_apply_float_flag(controller, flags, "steering_axis_deadzone")
	_apply_bool_flag(controller, flags, "apply_trigger_deadzone")
	_apply_float_flag(controller, flags, "trigger_deadzone")
	_apply_bool_flag(controller, flags, "invert_lift_default_direction")
	_apply_bool_flag(controller, flags, "invert_bucket_default_direction")
	_apply_bool_flag(controller, flags, "invert_dumper_default_direction")


static func _apply_bool_flag(controller: Node, flags: Dictionary, key: String) -> void:
	if not controller.has_method("get") or not controller.has_method("set"):
		return
	if not flags.has(key):
		return
	controller.set(key, bool(flags[key]))


static func _apply_float_flag(controller: Node, flags: Dictionary, key: String) -> void:
	if not controller.has_method("get") or not controller.has_method("set"):
		return
	if not flags.has(key):
		return
	controller.set(key, float(flags[key]))


static func _resolve_config_path() -> String:
	if OS.has_feature("editor"):
		return USER_CONFIG_PATH
	var external := _get_external_config_path()
	if external == "":
		return USER_CONFIG_PATH
	return external


static func _get_external_config_path() -> String:
	var exe_path := OS.get_executable_path()
	if exe_path == "":
		return ""
	var exe_dir := exe_path.get_base_dir()
	if exe_dir == "":
		return ""
	return exe_dir.path_join(EXTERNAL_CONFIG_FILENAME)


static func _event_to_token(event: InputEvent) -> String:
	if event is InputEventKey:
		var label := OS.get_keycode_string(event.keycode).to_lower().replace(" ", "_")
		if label == "":
			label = "keycode_%d" % event.keycode
		return "keyboard_%s" % label
	if event is InputEventJoypadButton:
		return "gamepad_button_%d" % event.button_index
	if event is InputEventJoypadMotion:
		var sign := "positive" if event.axis_value >= 0.0 else "negative"
		return "gamepad_axis_%d_%s" % [event.axis, sign]
	return ""


static func _token_to_event(token: String) -> InputEvent:
	if token.begins_with("keyboard_"):
		var k := token.trim_prefix("keyboard_")
		if k.begins_with("keycode_"):
			var direct := int(k.trim_prefix("keycode_"))
			if direct != 0:
				var key_event := InputEventKey.new()
				key_event.keycode = direct
				return key_event
		var guess := OS.find_keycode_from_string(k.replace("_", " ").to_upper())
		if guess != 0:
			var key_event2 := InputEventKey.new()
			key_event2.keycode = guess
			return key_event2
	if token.begins_with("gamepad_button_"):
		var b := InputEventJoypadButton.new()
		b.button_index = int(token.trim_prefix("gamepad_button_"))
		return b
	if token.begins_with("gamepad_axis_"):
		var tail := token.trim_prefix("gamepad_axis_")
		var bits := tail.split("_")
		if bits.size() >= 2:
			var m := InputEventJoypadMotion.new()
			m.axis = int(bits[0])
			m.axis_value = 1.0 if bits[1] == "positive" else -1.0
			return m
	return null
