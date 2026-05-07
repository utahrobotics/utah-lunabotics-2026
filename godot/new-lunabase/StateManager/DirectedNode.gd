class_name DirectedNode 
extends Node2D

@export var current_node : Node2D
@export var linked_nodes : Array[DirectedNode] = []
@export var isActive : bool = false
@onready var panel = $PanelContainer

func _ready():
	current_node = self

func set_active_node():
	isActive = true


func _process(_delta):
	checkIfActive()
	
	

func checkIfActive() -> void:
	var stylebox = StyleBoxFlat.new()
	stylebox.bg_color = Color("crimson")
	stylebox.set_corner_radius_all(6)
	if isActive:
		stylebox.set_border_width_all(10)
		stylebox.border_color = Color("black")
		
		panel.add_theme_stylebox_override("panel",stylebox)
	else:
	
		
		panel.add_theme_stylebox_override("panel",stylebox)
		
		


		
