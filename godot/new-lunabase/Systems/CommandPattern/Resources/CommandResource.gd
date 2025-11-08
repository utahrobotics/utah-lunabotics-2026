class_name Command
extends Resource

## The main Command class meant to be extended by true commands
##
## Holds a timestamp on when the action was executed and an execute() function.
## The execute() function should be implemented in your extended command.
## See test_move_command.gd for an example

# This is to keep track of when the command is issued
@export var timestamp: float = 0.0  

# Implement the execute command in the extended resource
func execute(_actor: Node) -> void:
	push_error("EXECUTE HAS NOT BEEN IMPLEMENTED")
