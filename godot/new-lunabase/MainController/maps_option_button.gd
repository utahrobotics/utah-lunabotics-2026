extends OptionButton
@onready var arena_map: Control = %ArenaMap

func _ready():
	item_selected.connect(handle_map_change)

func handle_map_change(new_item):
	print("new item pressed ", new_item)
	arena_map.update_map(new_item)
