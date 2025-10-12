extends Control

func update_interface(values: Dictionary):
	$Panel/Horizon/InnerPanel/CenterRef.rotation_degrees = -values["roll"]
	$Panel/Horizon/InnerPanel/CenterRef/Ground.position.y = (values["pitch"]/90.0)*71.0
	$Panel/Compass/InnerPanel/CenterRef.rotation_degrees = (-values["bearing"])
