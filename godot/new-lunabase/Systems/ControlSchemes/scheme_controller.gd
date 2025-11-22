extends Node

# For testing
@export var new_control_scheme : ControlSchemeResource

func _ready() -> void:
	tester()
	

func tester():
	erase_all_action_events()
	populate_from_scheme(new_control_scheme)
	print("ready to test")

func erase_all_action_events():
	var actions = InputMap.get_actions()
	for action in actions:
		InputMap.action_erase_events(action)

func populate_from_scheme(new_scheme : ControlSchemeResource):
	var new_actions := new_scheme.get_all_actions()
	var action_keys = new_actions.keys()
	
	for action in action_keys:
		for event in new_actions.get(action):
			if event is InputEvent and not null:
				print("adding event ", event, "to action ", action)
				InputMap.action_add_event(action, event)

func _process(_delta: float) -> void:
	#TESTING
	
	if Input.is_action_pressed("move_left"):
		print("moving left!")
	
	if Input.is_action_pressed("move_right"):
		print("moving right!")

# TODO have resource that exports all current input 
# maps to an array. then be able to create a scripts from that array that creates
# a custom resource where you can create new mappings. Then have some input manager
# that lets the user creeate new input maps and easily bind to new ones
	
