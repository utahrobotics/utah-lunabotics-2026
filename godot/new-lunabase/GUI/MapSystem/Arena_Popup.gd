extends Control

@onready var panel: Panel = $Panel
@onready var button: Button = $Panel/Button 
@onready var ArenaText: RichTextLabel = $Panel/ArenaText
@onready var LocationLunabotArtemis := $Artemis/origin/LocationArtemis
@onready var LocationLunabotUcf := $Ucf/Origin/LocationUcf
@onready var connection := GlobalLunabaseConnection
@onready var Ucf: TextureRect = $Ucf
@onready var Artemis: TextureRect = $Artemis

var marker_pos = Vector2.ZERO


var arena = false

var connectionToBase: LunabaseConnection
var location: PackedFloat32Array


func _on_button_toggled(is_button_toggled: bool) -> void:
	Ucf.visible = is_button_toggled && !arena
	Artemis.visible = is_button_toggled && arena

	
func _ready() -> void:
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
	#not bound to traditional x and y persay check 
	#what each arena defines x and y as :j
	
	
	if (arena == true):
		LocationLunabotArtemis.position = Vector2(x * 153,y * 153)
		marker_pos = Vector2(x * 153, y * 153)
		
	if(arena == false):
		LocationLunabotUcf.position = Vector2(x * 69,-y * 69 )
		marker_pos = Vector2(x * 69, y* 69)
		
		
