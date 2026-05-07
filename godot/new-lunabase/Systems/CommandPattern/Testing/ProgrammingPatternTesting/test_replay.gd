extends Node

# IGNORE THIS CODE, I JUST USE IT TO TEST FUNCTIONS THROUGH BUTTONS

@onready var test_controller: TestHumanController = $"../../CommandController"
@onready var character_body_2d: CharacterBody2D = $"../../CharacterBody2D"

# Sets the test player back at the scene origin
func reset_char():
	character_body_2d.position = Vector2.ZERO

func reset_scene():
	reset_char()
	test_controller.command_recorder.clear_history()


func _on_button_reset_scene_pressed() -> void:
	reset_scene()

func _on_button_playback_pressed() -> void:
	reset_char()
	print("starting to load history...")
	test_controller.command_recorder.start_replay()
