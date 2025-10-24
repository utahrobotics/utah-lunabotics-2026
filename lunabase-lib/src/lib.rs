use std::net::{Ipv4Addr, SocketAddr, SocketAddrV4};

use common::{QuicMessage, Steering};
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


#[godot_api]
impl LunabaseConnection {
    #[func]
    fn execute_steering(&self, left: f64, right: f64) {
        // lets not use the weight 
        // send could block depending on the packet size so we might need to just queue up the steering msg
        // and then spawn a thread in the init function that just continuously checks if there is a new message in the queue and sends it to 
        // the lunabot.
        match self.client.send(QuicMessage::FromClient(common::FromLunabase::Steering(Steering::new(left, right, 1.0)))) {
            Ok(_) => {
                
            },
            Err(e) => {
                godot_warn!("Failed to send packet: {e}");
                // use Carlos's notification system here, 
                //or maybe return an error message from here and notify the user
                // from godot instead of rust if that is easier
            },
        }
    }
}
