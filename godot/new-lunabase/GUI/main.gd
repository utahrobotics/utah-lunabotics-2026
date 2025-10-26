extends LunabaseConnection
@onready var stateViewer = $stateString;



func _ready():
	pass
	

func _process(_delta):
	var currentLunaBotState = retrieve_state()
	stateViewer.text = "Current Stage:" + str(currentLunaBotState);
	
