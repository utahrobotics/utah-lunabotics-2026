extends Button

@onready var rust = $".."
func _pressed() ->void:
	var command = ContinureMissionCommand.new()
	
	command.execute(rust)
	print("Initiated Manual Control")
	
	
