extends Node2D

func _ready() -> void:
	NotificationCanvasGlobal.add_notification("THIS IS A TEST", 3)
	
	# MODIFY THIS TO CREATE MORE NOTIFICATIONS!
	for x in range(10):
		var new_text = "This is " + str(x)
		NotificationCanvasGlobal.add_notification(new_text)
