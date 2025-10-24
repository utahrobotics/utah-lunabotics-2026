use std::net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4};

use common::QuicMessage;
use godot::{classes::Os, prelude::*};
use quic::QuicClient;

struct LunabaseExtension;

#[gdextension]
unsafe impl ExtensionLibrary for LunabaseExtension {}

#[derive(GodotClass)]
#[class(base=Node)]
struct LunabaseConnection {
    base: Base<Node>,
    client: QuicClient<QuicMessage>
}

#[godot_api]
impl INode for LunabaseConnection {
    fn init(base: Base<Node>) -> Self {
        let lunabot_address_str = Os::singleton()
            .get_cmdline_user_args()
            .get(0)
            .map(|x| x.to_string())
            .unwrap_or_else(|| "192.168.0.102".into());
        let lunabot_address = {
            if let Ok(addr) = lunabot_address_str.parse::<Ipv4Addr>() {
                godot_warn!("Connecting to: {lunabot_address_str}");
                Some(addr)
            } else {
                godot_error!("Failed to parse address: {lunabot_address_str}");
                None
            }
        };
        let socket_addr = SocketAddr::V4(SocketAddrV4::new(lunabot_address.unwrap(), common::ports::TELEOP));
        let client = {
            let c = QuicClient::connect(socket_addr);
            if let Err(ref e) = c {
                godot_error!("Failed to create client: {}", e);
            }
            c
        };
        LunabaseConnection { 
            base,
            client: client.unwrap()
        }
    }

}
