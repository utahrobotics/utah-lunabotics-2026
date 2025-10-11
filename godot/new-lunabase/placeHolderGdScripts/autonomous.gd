extends CheckButton



func _on_toggled(toggled_on: bool) -> void:
	if(toggled_on == true):
		print("Autonomous ON")
	else: 
		print("Autonomous OFF")

	# Replace with function body.
