extends Control

@onready var panel: Panel = $Panel
@onready var button: Button = $Panel/Button 
@onready var ArenaText: RichTextLabel = $Panel/ArenaText
@onready var LocationLunabotArtemis := $Artemis/origin/LocationArtemis
@onready var LocationLunabotUcf := $Ucf/Origin/LocationUcf
@onready var connection := GlobalLunabaseConnection
@onready var Ucf: TextureRect = $Ucf
@onready var Artemis: TextureRect = $Artemis
@onready var TrailArtemis:= $Artemis/origin/TrailMarkerArtemis
@onready var TrailUcf:= $Ucf/Origin/TrailMarkerUcf



var arena = false

var connectionToBase: LunabaseConnection
var location: PackedFloat32Array


func _on_button_toggled(is_button_toggled: bool) -> void:
	Ucf.visible = is_button_toggled && !arena
	Artemis.visible = is_button_toggled && arena

	
func _ready() -> void:
	TrailUcf.default_color = Color(0.76, 0.0, 0.0, 1.0) 
	TrailArtemis.default_color = Color(0.629, 0.0, 0.0, 1.0) 
	pass
	
func _process(_delta: float) -> void:
	var locationValues = connection.get_location()
	updateLunabotLocation(locationValues)
	

func _on_button_for_arena_toggled(toggled_on: bool) -> void:
	if (toggled_on):
		arena = true
	else :
		arena = false



func updateLunabotLocation(values: PackedFloat32Array ) -> void:
	if(values.size() < 2):
		return 
	var x: float = values[0] 
	var y: float = values[1] 

	
	if (arena == true):
		LocationLunabotArtemis.position = Vector2(x * 100,-y * 100)
		TrailArtemis.add_point(Vector2(x*100,-y*100))
		if(TrailArtemis.points.size() > 1000): # change the > x if you want the snail trail to last longer
			TrailArtemis.remove_point(0)
		
	if(arena == false):
		LocationLunabotUcf.position = Vector2(x * 100,-y * 100 )
		TrailUcf.add_point(Vector2(x*100,-y*100))
		if(TrailUcf.points.size() > 1000):
			TrailUcf.remove_point(0)
		
