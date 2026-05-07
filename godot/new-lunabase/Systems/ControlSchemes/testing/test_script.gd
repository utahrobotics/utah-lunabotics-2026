# InputProfileManager.gd
extends Node

const CONTROL_SCHEMES_PATH = "user://control_schemes/"

func export_current_input_map(profile_name: String) -> void:
	var config = ConfigFile.new()
	
	for action in InputMap.get_actions():
		var events = []
		for event in InputMap.action_get_events(action):
			events.append(event_to_dict(event))
		config.set_value("input", action, events)
	
	# Ensure directory exists
	var dir = DirAccess.open("user://")
	dir.make_dir_recursive("control_schemes")
	
	config.save(CONTROL_SCHEMES_PATH + profile_name + ".cfg")

func load_input_profile(profile_name: String) -> void:
	var config = ConfigFile.new()
	var err = config.load(CONTROL_SCHEMES_PATH + profile_name + ".cfg")
	
	if err != OK:
		push_error("Failed to load input profile: " + profile_name)
		return
	
	for action in InputMap.get_actions():
		InputMap.action_erase_events(action)
	
	for action in config.get_section_keys("input"):
		if InputMap.has_action(action):
			var events = config.get_value("input", action)
			for event_dict in events:
				var event = dict_to_event(event_dict)
				if event:
					InputMap.action_add_event(action, event)

func event_to_dict(event: InputEvent) -> Dictionary:
	# Convert event to dictionary for saving
	# Implementation depends on which event types you need to support
	return {}

func dict_to_event(dict: Dictionary) -> InputEvent:
	# Convert dictionary back to event
	# Implementation depends on which event types you need to support
	return null
