# Introduction
This system is meant to help have multiple control schemes for both keyboard and generic controllerss.

# Currently
- Controller and keyboard schemes are independent of each other
- A scheme can hold multiple inputs for the same action
- Left wheel / Right wheel movement and forward / backward movement are independent of each other
- These schemes can be switched out at any time
- All inputs have a default deadzone of 0.2 (this can be modified in Project -> Project Settings -> Input Map)
- New inputs can be created in their corresponding folder and godot should pick them up automatically

# Limitations
Any modifications to the input map such as adding or modifying input behavior will require the scheme_resource.gd item to be modified to represent these changes

Creating a new input system isn't terrible but it isn't super friendly. It would be a Nice To Have if there was a scene where you can modify inputs by pressing them in order similar to other games. (This will need some thought since I want to leave the door open to having multiple possible inputs)
