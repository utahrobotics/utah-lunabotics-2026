extends Node2D
#statemanager

@export var behavior_tree_nodes : Array[DirectedNode]
var timer: float = 0.0
var interval: float = 4.0
var state: LunabaseConnection = GlobalLunabaseConnection

func set_node_active(num: int) :

 for node in behavior_tree_nodes:
   node.isActive = false

 behavior_tree_nodes[num].isActive = true;

func _ready():
   set_node_active(0)

func _process(_delta):
   pass
