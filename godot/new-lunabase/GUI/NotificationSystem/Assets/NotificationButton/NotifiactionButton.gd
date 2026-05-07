class_name NotificationButton extends Button

## Creates a notification. 
##
## On click it will delete itself. If lifetime < 0 then
## it won't be automatically deleted

signal deleted

# Text to display
@export var notification_text : String = "TEST NOTIFICATION"
# Time for notifiaction to stay on screen
# If set to a negative value it will live forever
@export var lifetime : float = -1.0

func _init(new_text : String = "TEST NOTIFICATION", life : float = -1.0) -> void:
	notification_text = new_text
	lifetime = life

func _ready() -> void:
	text = notification_text
	
	if lifetime > 0:
		await get_tree().create_timer(lifetime).timeout
		delete_noticiation()

func delete_noticiation():
	deleted.emit()
	# Possibly look into writing all notifications into a file?
	self.queue_free()

func _pressed() -> void:
	delete_noticiation()
