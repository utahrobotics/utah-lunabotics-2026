extends Control

var orientation: PackedFloat32Array

func _ready()-> void:
	if GlobalLunabaseConnection == null:
		return
		
	
	orientation = GlobalLunabaseConnection.get_orientation()
	update_interface(orientation)
	

func _process(_delta: float)-> void:
	if GlobalLunabaseConnection == null:
		return
		
	
	orientation = GlobalLunabaseConnection.get_orientation()
	update_interface(orientation)
	
func update_interface(values: PackedFloat32Array):
	var x: float = values[0]
	var y: float = values[1]
	var z: float = values[2]
	var w: float = values[3]
	
	var roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
	var pitch = asin(2.0 * (w*y - z*x))
	#might include later implementing rotation to left indicator later cuz it was wonky af
	var yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


	
	
	
	$Panel/Horizon/InnerPanel/CenterRef.rotation = -roll
	var max_pixels:float = 71.0
	var pitch_norm: float = clamp(pitch / (PI / 2.0), -1.0, 1.0)

	$Panel/Horizon/InnerPanel/CenterRef/Ground.position.y = pitch_norm * max_pixels
	
	$Panel/Compass/InnerPanel/CenterRef.rotation = -yaw
