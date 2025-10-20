class_name NotificationCanvas extends Control

@onready var total_label: Label = $TotalLabel
@onready var v_box_container: VBoxContainer = $ScrollContainer/VBoxContainer

func add_notification(notif_text = "NOT SET", lifetime = -1):
	var new_notif := NotificationButton.new(notif_text, lifetime)
	new_notif.deleted.connect(update_total, true)
	v_box_container.add_child(new_notif)
	update_total()

func update_total(delayed := true):
	if delayed:
		await get_tree().create_timer(0.1).timeout
	
	var total = v_box_container.get_child_count()
	total_label.text = "Total: " + str(total)

func clear_all_notifications():
	for child in v_box_container.get_children():
		child.queue_free()
	update_total()

func _on_clear_all_button_pressed() -> void:
	clear_all_notifications()
