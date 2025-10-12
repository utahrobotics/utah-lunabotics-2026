class_name PitchRollCommand extends Command

@export var pitch: float
@export var roll: float

func execute(actor):
	actor.set_pitch_roll(pitch, roll)
