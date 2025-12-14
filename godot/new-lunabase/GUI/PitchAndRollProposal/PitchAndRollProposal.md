# Proposal
Simple proposal that uses the command pattern to show a pitch and roll example.
This comes from this project which uses the MIT license
https://github.com/fbcosentino/godot-simplified-flightsim

# Testing the scene
To test it out use WASD or a joystick on the PitchAndRollProposalTesting.tscn scene
in the GUI/PitchAndRollProposal/ folder. It uses the command pattern to capture input.

# Considerations for actual implementation
The code connecting to it is a bit messy right now, but in the actual implementation we will only
need to pass values taken from the robot (I assume) all we need to know is what rotational info
we are receiving to correctly pass it to the GUI
