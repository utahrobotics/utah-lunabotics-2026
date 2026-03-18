extends VBoxContainer
@onready var controls_label: RichTextLabel = $ScrollContainer/ControlsLabel
@onready var rebinding_scene: Control = $"../../../.."

var current_bindings := {}
var regex = RegEx.new()
var controls = ""

func _ready() -> void:
	setup()
	rebinding_scene.new_binding.connect(add_binding_text)
	rebinding_scene.new_message.connect(append_text)
	regex.compile(r"\(([^)]*)\)")

func setup():
	controls = ""
	controls_label.clear()
	current_bindings.clear()

func add_binding_text(action: String, input: String):
	print("RECEIVED: %s action and %s input" % [action, input])
	current_bindings.set(action, input)
	update_binding_text(action, input)

func update_binding_text(action : String, input: String):
	controls += "[color=3399ff]" + action +  "[/color]\n "
	controls += input + "\n"
	controls += "[color=3399ff]------------[/color]\n"
	controls_label.clear()
	controls_label.text = controls

func append_text(new_text : String):
	controls_label.append_text("\n")
	controls_label.append_text(new_text)
