extends Control

@onready var connection:= GlobalLunabaseConnection
@onready var controller:= $LunabaseHumanController
@onready var ip_input: LineEdit = $VBoxContainer/TopBar/MarginContainer/TopPanel/ConnectionGroup/IPInput
@onready var connect_button: Button = $VBoxContainer/TopBar/MarginContainer/TopPanel/ConnectionGroup/ConnectButton
@onready var packet_label: Label = $VBoxContainer/TopBar/MarginContainer/TopPanel/StatusGroup/PacketLabel
@onready var stage_label: Label = $VBoxContainer/TopBar/MarginContainer/TopPanel/StatusGroup/StageLabel
@onready var soft_stop_button: Button = $VBoxContainer/BottomBar/MarginContainer/BottomPanel/SoftStopButton
@onready var test_motors_button: Button = $VBoxContainer/BottomBar/MarginContainer/BottomPanel/TestMotorsButton
@onready var manual_button: Button = $VBoxContainer/BottomBar/MarginContainer/BottomPanel/ManualButton
@onready var autonomous_button: Button = $VBoxContainer/BottomBar/MarginContainer/BottomPanel/AutonomousButton
@onready var errored_tasks_label: Label = $VBoxContainer/MainContent/HBoxContainer/LeftColumn/ErrorsPanel/MarginContainer/VBox/ScrollContainer/ErroredTasksLabel
@onready var speed_label: Label = $VBoxContainer/TopBar/MarginContainer/TopPanel/StatusGroup/SpeedLabel
@onready var speed_slider: HSlider = $VBoxContainer/MainContent/HBoxContainer/RightColumn/SpeedControl/MarginContainer/VBox/SpeedMultiplierSlider
@onready var reset_obstacles_button: Button = $VBoxContainer/MainContent/HBoxContainer/RightColumn/SpeedControl/MarginContainer/VBox/ResetObstaclesButton
@onready var toggle_apriltags_button: Button = $VBoxContainer/MainContent/HBoxContainer/RightColumn/SpeedControl/MarginContainer/VBox/EnableApriltagsButton
@onready var sigma_spatial_input: SpinBox = $VBoxContainer/MainContent/HBoxContainer/RightColumn/SpeedControl/MarginContainer/VBox/SigmaSpatialInput
@onready var sigma_range_input: SpinBox = $VBoxContainer/MainContent/HBoxContainer/RightColumn/SpeedControl/MarginContainer/VBox/SigmaRangeInput

@onready var location_label: Label = $VBoxContainer/TopBar/MarginContainer/TopPanel/StatusGroup/location_label
@onready var bt_status_label: Label = $VBoxContainer/MainContent/HBoxContainer/CenterColumn/StatusPanel/MarginContainer/BTStatusLabel
@onready var PitchAndRollGUI: Control = $VBoxContainer/MainContent/HBoxContainer/CenterColumn/UIAttitude
var command_recorder: CommandRecorder

#Speed Slider

var set_weight := SetSpeedMultiplier.new()
var weight: float

#location label

var get_location := GetLocation.new()


# Should match the LunabotStage enum in the Rust extension
enum LunabotStage {
	MANUAL = 0,
	SOFT_STOP = 1,
	AUTONOMY = 2,
	TEST_MOTORS = 3,
}

func _ready() -> void:
	connection.stage_changed.connect(_on_stage_changed)
	connect_button.pressed.connect(_on_connect_pressed)
	ip_input.gui_input.connect(_on_ip_input_gui_input)
	soft_stop_button.pressed.connect(_on_soft_stop_pressed)
	manual_button.pressed.connect(_on_manual_pressed)
	autonomous_button.pressed.connect(_on_autonomous_pressed)
	reset_obstacles_button.pressed.connect(_on_reset_obstacles_pressed)
	toggle_apriltags_button.toggled.connect(_on_toggle_apriltags_pressed)
	sigma_spatial_input.value_changed.connect(_on_set_sigma_spatial_changed)
	sigma_range_input.value_changed.connect(_on_set_sigma_range_changed)
	test_motors_button.pressed.connect(_on_test_motors_pressed)
	
	
	speed_slider.value = 600;
	weight = 600;
	var new_weight = GlobalLunabaseConnection.set_speed(weight)
	
	speed_label.text = "SpeedMultiplier set to " + str(new_weight)
	

func _process(delta: float) -> void:
	var ms_since_packet: int = GlobalLunabaseConnection.get_ms_since_last_packet()
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
	
	var stage: int = GlobalLunabaseConnection.get_lunabot_stage()
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
		LunabotStage.TEST_MOTORS:
			stage_label.text = "Stage: Test Motors"
			stage_label.modulate = Color.RED
			
	
	#location  
	var location = connection.get_location()
	location_label.text = "location: [%.2f, %.2f, %.2f]" % [location[0], location[1], location[2]]
 
	# Update errored tasks
	var errored_tasks: Dictionary = GlobalLunabaseConnection.get_errored_tasks()
	if errored_tasks.is_empty():
		errored_tasks_label.text = "No errors"
	else:
		var error_text := ""

		var sorted_tasks := errored_tasks.keys()
		sorted_tasks.sort()

		for task_name in sorted_tasks:
			var error_msg: String = errored_tasks[task_name]
			error_text += task_name + ":\n  " + error_msg + "\n\n"

		errored_tasks_label.text = error_text.strip_edges()
		
	var bt_status: String = GlobalLunabaseConnection.get_bt_status()
	
	if bt_status.is_empty():
		bt_status_label.text = "BT STATUS: UNKNOWN"
	else:
		bt_status_label.text = "BT STATUS: " + bt_status


		
	
func _on_stage_changed(stage: int) -> void:
	print("Stage changed to: ", stage)


func _on_connect_pressed() -> void:
	var address := ip_input.text
	if address.is_empty():
		push_error("Address cannot be empty")
		return
	
	print("Connecting to: ", address)
	# this should probably an action also idk
	GlobalLunabaseConnection.reconnect(address)


func _on_soft_stop_pressed() -> void:
	print("Sending SoftStop command")
	controller.command_recorder.execute_and_store(SoftStopCommand.new())


func _on_manual_pressed() -> void:
	print("Sending ContinueMission command (Manual mode)")
	controller.command_recorder.execute_and_store(ContinueMissionCommand.new())


func _on_autonomous_pressed() -> void:
	print("Sending Navigate command (Autonomous mode)")
	controller.command_recorder.execute_and_store(NavigateCommand.new())

func _on_reset_obstacles_pressed() -> void:
	print("Sending ResetObstacles command")
	controller.command_recorder.execute_and_store(ResetObstaclesCommand.new())
	
func _on_toggle_apriltags_pressed(is_pressed: bool) -> void:
	controller.command_recorder.execute_and_store(ToggleApriltagsCommand.new(is_pressed))
	
func _on_set_sigma_spatial_changed(new_spatial: float) -> void:
	controller.command_recorder.execute_and_store(SetSigmaSpatial.new(new_spatial))
	
func _on_set_sigma_range_changed(new_range: float) -> void:
	controller.command_recorder.execute_and_store(SetSigmaRange.new(new_range))

	
func _on_speed_multiplier_slider_value_changed(value: float) -> void:
	# Update slider weight
	var new_weight = GlobalLunabaseConnection.set_speed(value)

	# Update UI
	speed_label.text = "SpeedMultiplier set to " + str(new_weight)

func _on_test_motors_pressed() -> void:
	controller.command_recorder.execute_and_store(TestMotorsCommand.new())

func _on_ip_input_gui_input(event: InputEvent) -> void:
	if not (event is InputEventKey):
		return
	var key_event := event as InputEventKey
	if not key_event.pressed or key_event.echo:
		return
	if key_event.keycode != KEY_BACKSPACE and key_event.keycode != KEY_DELETE:
		return

	if ip_input.has_selection():
		ip_input.delete_text(ip_input.get_selection_from_column(), ip_input.get_selection_to_column())
		ip_input.deselect()
		accept_event()
		return

	var caret: int = ip_input.caret_column
	if key_event.keycode == KEY_BACKSPACE:
		if caret <= 0:
			accept_event()
			return
		ip_input.delete_text(caret - 1, caret)
		ip_input.caret_column = caret - 1
		accept_event()
		return

	# KEY_DELETE
	if caret < ip_input.text.length():
		ip_input.delete_text(caret, caret + 1)
	accept_event()
