pub mod safestop;
pub mod state_viewer;

use godot::prelude::*;


struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension{

}