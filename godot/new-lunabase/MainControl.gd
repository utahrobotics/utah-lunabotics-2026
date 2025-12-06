extends Control

@onready var connection: Node = $LunabaseConnection
@onready var controller: LunabaseHumanController = $LunabaseHumanController
@onready var ip_input: LineEdit = $VBoxContainer/TopPanel/IPInput
@onready var connect_button: Button = $VBoxContainer/TopPanel/ConnectButton
@onready var packet_label: Label = $VBoxContainer/TopPanel/PacketLabel
@onready var stage_label: Label = $VBoxContainer/TopPanel/StageLabel
@onready var soft_stop_button: Button = $VBoxContainer/BottomPanel/SoftStopButton
@onready var manual_button: Button = $VBoxContainer/BottomPanel/ManualButton
@onready var autonomous_button: Button = $VBoxContainer/BottomPanel/AutonomousButton
@onready var errored_tasks_label: Label = $VBoxContainer/CenterContainer/HBoxContainer/ErroredTasksPanel/MarginContainer/ScrollContainer/VBox/ErroredTasksLabel
@onready var speed_label: Label = $VBoxContainer/TopPanel/SpeedLabel
@onready var speed_slider: HSlider = $SpeedMultiplierSlider
@onready var location_label: Label = $LocationLabel
@onready var orientation_label: Label = $OrientationLabel

var command_recorder: CommandRecorder

#Speed Slider

var set_weight:= SetSpeedMultiplier.new()
var weight: float

#location label

var get_location:= GetLocation.new()
var location: PackedFloat32Array


# Should match the LunabotStage enum in the Rust extension
enum LunabotStage {
	MANUAL = 0,
	SOFT_STOP = 1,
	AUTONOMY = 2,
}

func _ready() -> void:
	connection.stage_changed.connect(_on_stage_changed)
	
	connect_button.pressed.connect(_on_connect_pressed)
	soft_stop_button.pressed.connect(_on_soft_stop_pressed)
	manual_button.pressed.connect(_on_manual_pressed)
	autonomous_button.pressed.connect(_on_autonomous_pressed)
	
	
	speed_slider.value = 0;
	var new_weight = connection.set_speed(weight)
	
	speed_label.text = "SpeedMultiplier set to " + str(new_weight)
	
	


func _process(delta: float) -> void:
	var ms_since_packet: int = connection.get_ms_since_last_packet()
	var packet_time_sec: float = ms_since_packet / 1000.0
	
	# Format time text
	var time_text: String
	if packet_time_sec < 1.0:
		time_text = "%dms" % ms_since_packet
	elif packet_time_sec < 60.0:
		time_text = "%.1fs" % packet_time_sec
	else:
		time_text = ">60s"
	packet_label.text = "Last Packet: " + time_text
	
	var t: float
	if packet_time_sec < 0.1:
		t = 0.0
	elif packet_time_sec < 0.5:
		t = (packet_time_sec - 0.1) / 0.4
	else:
		t = 1.0
	packet_label.modulate = Color.GREEN.lerp(Color.RED, t)
	
	var stage: int = connection.get_lunabot_stage()
	match stage:
		LunabotStage.MANUAL:
			stage_label.text = "Stage: Manual"
			stage_label.modulate = Color.GREEN
		LunabotStage.SOFT_STOP:
			stage_label.text = "Stage: SoftStop"
			stage_label.modulate = Color.YELLOW
		LunabotStage.AUTONOMY:
			stage_label.text = "Stage: Autonomy"
			stage_label.modulate = Color.CYAN
	
	#location  maybe temporary 
	var location = connection.get_location()
	location_label.text = "location: [%.2f, %.2f, %.2f]" % [location[0], location[1], location[2]]
	
	var orientation = connection.get_orientation()
	orientation_label.text = "orientation: [%.2f, %.2f, %.2f, %.2f]" % [orientation[0], orientation[1], orientation[2], orientation[3]
]
	
	
	# Update errored tasks
	var errored_tasks: Dictionary = connection.get_errored_tasks()
	if errored_tasks.is_empty():
		errored_tasks_label.text = "No errors"
	else:
		var error_text := ""
		for task_name in errored_tasks.keys():
			var error_msg: String = errored_tasks[task_name]
			error_text += task_name + ":\n  " + error_msg + "\n\n"
		errored_tasks_label.text = error_text.strip_edges()
		
	


func _on_stage_changed(stage: int) -> void:
	print("Stage changed to: ", stage)


func _on_connect_pressed() -> void:
	var address := ip_input.text
	if address.is_empty():
		push_error("Address cannot be empty")
		return
	
	print("Connecting to: ", address)
	# this should probably an action also idk
	connection.reconnect(address)


func _on_soft_stop_pressed() -> void:
	print("Sending SoftStop command")
	controller.command_recorder.execute_and_store(SoftStopCommand.new())


func _on_manual_pressed() -> void:
	print("Sending ContinueMission command (Manual mode)")
	controller.command_recorder.execute_and_store(ContinueMissionCommand.new())


func _on_autonomous_pressed() -> void:
	print("Sending Navigate command (Autonomous mode)")
	controller.command_recorder.execute_and_store(NavigateCommand.new())
	

func _on_speed_multiplier_slider_value_changed(value: float) -> void:
	# Update slider weight
	var new_weight = connection.set_speed(value)

	# Update UI
	speed_label.text = "SpeedMultiplier set to " + str(new_weight)
	print("Speed Multiplier set to ", new_weight)
