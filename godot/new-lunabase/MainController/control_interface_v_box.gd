extends VBoxContainer
@onready var lunabase_human_controller: LunabaseHumanController = $"../../../../../../../LunabaseHumanController"
@onready var controls_label: RichTextLabel = $ScrollContainer/ControlsLabel

func _ready() -> void:
	populate_control_text()
	SettingsMenu.updated_control_scheme.connect(populate_control_text)


func populate_control_text():
	var controls = ""
	controls_label.clear()
	var regex = RegEx.new()
	regex.compile(r"\(([^)]*)\)")
	
	await get_tree().create_timer(1).timeout
	for action_name in InputMap.get_actions():
		if action_name.contains("ui"):
			continue
		var events = InputMap.action_get_events(action_name)
		print(action_name)
		controls += "[color=3399ff]" + action_name +  "[/color]\n "
		for event in events:
			var result = event.as_text()
			if event is InputEventJoypadButton or event is InputEventJoypadMotion:
				var readable := regex.search_all(result)
				result = readable[0].get_string()
			controls += result + "\n"
			print(event.as_text()) 
		controls += "[color=3399ff]------------[/color]\n"
		print("__________")
	controls_label.append_text(controls)
