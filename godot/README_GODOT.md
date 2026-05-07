# Installation
As of now, only the stable release of Godot 4.6 is needed

Since we are using GDExtension to use Rust be sure to do set it up

Go to
> lunabase-lib
and run cargo build. Do this any time the lib.rs for godot is updated

# Contributing

Some things to keep in mind to help avoid headaches and keep all our code nice and friendly :)

- Follow [Godot's GDScript style guide](https://docs.godotengine.org/en/stable/tutorials/scripting/gdscript/gdscript_styleguide.html).
- Keep components modular and scoped.
- Organize each feature (e.g., GUI, system) in its own folder.
- Include a README in your folder if:
  - Your component has dependencies.
  - It needs explanation for usage or integration.

Example: If you're working on a GUI component, it should live in its own folder named after the component. This folder should contain all related scenes, assets, and scripts. You can use subfolders to keep things tidy and organized.
- Try to keep components modular and scoped to their purpose.
- No need to be overly granular, but clarity and separation help avoid future headaches.
- Similar components (like GUI elements) can be grouped under a parent folder such as GUI/.

# Other considerations

As of now I am using the Compatibility renderer for speed. This choice is open to discussion. Feel free to propose changes to this readme.
