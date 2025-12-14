extends MeshInstance3D

@onready var ui_attitude: Control = $"../CanvasLayer/Control/UIAttitude"
@onready var pitch_and_roll_proposal_testing: Node3D = $".."

func _process(_delta: float) -> void:
	var forward = -basis.z
	var up = -basis.y
	var pitch = rad_to_deg(atan2(forward.y, sqrt(forward.z * forward.z + forward.x * forward.x)))
	var roll = rad_to_deg(atan2(up.x, -up.y))
	var bearing = 0 # I DON'T KNOW IF WE WILL USE BEARING INTERFACE
	ui_attitude.update_interface({
		"pitch": pitch,
		"roll": roll,
		"bearing": bearing,
	})


func set_pitch_roll(pitch, roll):
	rotation.x += pitch
	rotation.z -= roll
