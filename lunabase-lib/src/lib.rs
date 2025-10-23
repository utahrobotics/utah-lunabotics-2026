use godot::prelude::*;
use quic::QuicClient;

struct LunabaseExtension;

#[gdextension]
unsafe impl ExtensionLibrary for LunabaseExtension {}

#[derive(GodotClass)]
#[class(base=Node)]
struct LunabaseConnection {
    base: Base<Node>,
}

#[godot_api]
impl INode for LunabaseConnection {
    fn init(base: Base<Node>) -> Self {
        LunabaseConnection { base }
    }
}
