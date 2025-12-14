extends Control
#onready var connection: Node = $LunabaseConnection
var connectionToBase: LunabaseConnection
var orientation: PackedFloat32Array


func set_conn(connection: LunabaseConnection) -> void:
	connectionToBase = connection
	
	orientation = connectionToBase.get_orientation()
	update_interface(orientation)
	
func _ready()-> void:
	if connectionToBase == null:
		return
		
	
	orientation = connectionToBase.get_orientation()
	update_interface(orientation)
	

func _process(_delta: float)-> void:
	if connectionToBase == null:
		return
		
	
	orientation = connectionToBase.get_orientation()
	update_interface(orientation)
	
func update_interface(values: PackedFloat32Array):
	var w: float = values[0]
	var x: float = values[1]
	var y: float = values[2]
	var z: float = values[3]

	var roll  = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
	var pitch = asin(clamp(2*(w*y - z*x), -1.0, 1.0))
	#might include later implementing rotation to left indicator later cuz it was wonky af
	
	#var yaw   = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))


	
	
	
	#$Panel/Horizon/InnerPanel/CenterRef.rotation = -roll
	var max_pixels:float = 71.0
	var pitch_norm: float = clamp(pitch / (PI / 2.0), -1.0, 1.0)

	$Panel/Horizon/InnerPanel/CenterRef/Ground.position.y = pitch_norm * max_pixels
	
	$Panel/Compass/InnerPanel/CenterRef.rotation = roll
