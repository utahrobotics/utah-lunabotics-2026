extends CharacterBody2D

# Applies movement to test characterbody in the test scene

func apply_movement(_velocity: Vector2):
	self.velocity = _velocity
	move_and_slide()
