extends Control

@onready var LocationLunabotArtemis := $Artemis/origin/LocationArtemis
@onready var LocationLunabotUcf := $Ucf/Origin/LocationUcf
@onready var connection := GlobalLunabaseConnection
@onready var Ucf: TextureRect = $Ucf
@onready var Artemis: TextureRect = $Artemis
@onready var TrailArtemis:= $Artemis/origin/TrailMarkerArtemis
@onready var TrailUcf:= $Ucf/Origin/TrailMarkerUcf

@export var artemis_pixels_per_unit: float = 100.0
@export var ucf_pixels_per_unit: float = 100.0
@export var snail_trail_length := 1000.0

enum MAP {OFF, ARTEMIS, UCF}

var connectionToBase: LunabaseConnection
var location: PackedFloat32Array

var current_map : MAP = MAP.OFF
var _original_sizes := {}

func _ready() -> void:
	TrailUcf.default_color = Color(0.76, 0.0, 0.0, 1.0)
	TrailArtemis.default_color = Color(0.629, 0.0, 0.0, 1.0)
	_original_sizes[Artemis] = Artemis.size
	_original_sizes[Ucf] = Ucf.size
	update_state()
	resized.connect(_on_resized)
	_on_resized()

func _process(_delta: float) -> void:
	var locationValues = connection.get_location()
	updateLunabotLocation(locationValues)
	_update_yaw()

func _on_resized() -> void:
	_fit_map(Artemis)
	_fit_map(Ucf)

func _fit_map(tex: TextureRect) -> void:
	var original : Vector2 = _original_sizes.get(tex, Vector2.ZERO)
	if original == Vector2.ZERO:
		return
	var available := size
	if available.x <= 0 or available.y <= 0:
		return
	var s := minf(available.x / original.x, available.y / original.y)
	tex.scale = Vector2(s, s)
	tex.size = original

func update_map(new_map):
	current_map = new_map
	update_state()

func update_state():
	Artemis.visible = (current_map == MAP.ARTEMIS)
	Ucf.visible = (current_map == MAP.UCF)

func update_ufc(x, y):
	var p2 := _world_to_map_pixels(x, y, ucf_pixels_per_unit)
	LocationLunabotUcf.position = p2
	TrailUcf.add_point(p2)
	if(TrailUcf.points.size() > 1000):
		TrailUcf.remove_point(0)

func update_artmis(x, y):
	var p := _world_to_map_pixels(x, y, artemis_pixels_per_unit)
	LocationLunabotArtemis.position = p
	TrailArtemis.add_point(p)
	if(TrailArtemis.points.size() > snail_trail_length): # change the > x if you want the snail trail to last longer
		TrailArtemis.remove_point(0) #or just comment out if you want it to stay

func updateLunabotLocation(values: PackedFloat32Array ) -> void:
	if(values.size() < 2):
		return 
	var x: float = values[0] 
	var y: float = values[1] 
	
	if current_map == MAP.ARTEMIS:
		update_artmis(x,y)
	elif current_map == MAP.UCF:
		update_ufc(x, y)


func _update_yaw() -> void:
	var ori = connection.get_orientation()
	if ori.size() < 4:
		return
	var x: float = ori[0]
	var y: float = ori[1]
	var z: float = ori[2]
	var w: float = ori[3]
	var yaw := atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
	var map_yaw := -yaw - PI / 2.0
	LocationLunabotArtemis.rotation = map_yaw
	LocationLunabotUcf.rotation = map_yaw

func _world_to_map_pixels(world_x: float, world_y: float, pixels_per_unit: float) -> Vector2:
	return Vector2(world_x * pixels_per_unit, -world_y * pixels_per_unit)
