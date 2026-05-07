class_name HistoryResource
extends Resource

## The HistoryResource is used to log all commands in sequential order
##
## I opted to use a resource for this because it can be easily serializeable.

# TODO: not important but we could create a "sequence builder" that could create
# command histories without live input and instead program them without having to
# create tons of resource since right now every command is logged.'

@export var commands: Array[Command] = []
