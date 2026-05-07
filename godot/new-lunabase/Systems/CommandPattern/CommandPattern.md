# Command Pattern
The command pattern is used to record user input and log it regardless of input device. This will allow us
to have clear commands that can be recorded and played back. This will also allow us to handle input device independently
of command implementation. Controllers can also be created for both users and game AI if needed.

## TODO
Creating the actual robot-specific commands still needs to be done. Implementing those commands to 
whatever signal is sent to the robot also needs to be implemented.

# Approach
A Controller must be created (See the TestController folder for an example) to handle inputs and then execute and log them as commands that the 
actor (robot) will then use. This was done using Resources. A command history is built and can be saved as a file.
These files can then be played back.

# Extending the command pattern
To create a new command, simply create a script that extends the base command class. Then, implement your logic
in the execute() function. See the Testing/TestCommands folder for examples.

# Other considerations
I initially wanted to use @abstract to create the command class, but resources proved to be better since they could be saved and replayed easily.

# Try it out!
To try out the command pattern go to the Testing folder and play the TestSceneCommandPattern scene. 
Move the sprite around with WASD or a joystick, then press playback to see your actions repeat.
Press Reset to try again!
