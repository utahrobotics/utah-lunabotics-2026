# Installation
As of now, only the stable release of Godot 4.5 is needed

# Contributing

Some things to keep in mind to help avoid headaches and keep all our code nice and friendly :)

- Follow [Godot's GDScript style guide](https://docs.godotengine.org/en/stable/tutorials/scripting/gdscript/gdscript_styleguide.html).
- Keep components modular and scoped.
- Organize each feature (e.g., GUI, system) in its own folder.
- Include a README in your folder if:
  - Your component has dependencies.
  - It needs explanation for usage or integration.


Example: If you are working on a complex GUI component, this should be done in its own folder with all its scenes, assets, and scripts within it. Naturally you can have subfolder to keep organization neat. Adding a readme is greatly appreciated especially if you depend on other components.

> Similar components like GUI could all be in a parent folder (GUI)

# Other considerations

As of now I am using the Compatibility renderer for speed. This choice is open to discussion. Feel free to propose changes to this readme.