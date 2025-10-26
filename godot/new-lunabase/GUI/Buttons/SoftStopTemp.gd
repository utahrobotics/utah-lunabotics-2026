extends Button

@onready var rust = $".."
func _pressed() ->void:
	var command = SoftStopCommand.new()
	
	command.execute(rust)
	print("Initiated Soft Stop")
	
	
	

	
