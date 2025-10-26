


use godot::prelude::*;
use godot::classes::*;
use common::{
    FromLunabase, FromLunabot, LunabotStage, Steering, THALASSIC_CELL_SIZE, THALASSIC_HEIGHT,
    THALASSIC_WIDTH,
};
use cakap2::{
    packet::{Action, ReliableIndex},
    Event, PeerStateMachine, RecommendedAction,
};
use bitcode::encode;

use std::time::Instant;
use std::{
    net::{UdpSocket, SocketAddr, SocketAddrV4, IpAddr, Ipv4Addr},
    time::Duration, 
    collections::VecDeque,
};


struct MyExtension;

#[gdextension]
unsafe impl ExtensionLibrary for MyExtension{

}

struct botconnInner{
cakap_sm: PeerStateMachine,
udp: UdpSocket,
to_lunabot: VecDeque<Action>,

}
#[derive(GodotClass)]
#[class(base = Node)]
struct lunabotConn{
 inner: Option<botconnInner>,
 base: Base<Node>,


}
thread_local! {
    static PONG_MESSAGE: Box<[u8]> = {
        encode(&FromLunabase::Pong).into()
    };
}

#[godot_api]
impl INode for lunabotConn{
 fn init(base: Base<Node>) -> Self{
     
     if Engine::singleton().is_editor_hint() {
            return Self {
                inner: None,
                base,
            };
    }
   let lunabot_address_str = Os::singleton()
            .get_cmdline_user_args()
            .get(0)
            .map(|x| x.to_string())
            .unwrap_or_else(|| "192.168.0.102".into());
        
    let lunabot_address = {
            if let Ok(addr) = lunabot_address_str.parse::<IpAddr>() {
                godot_warn!("Connecting to: {lunabot_address_str}");
                Some(addr)
            } else {
                godot_error!("Failed to parse address: {lunabot_address_str}");
                None
            }
        };
    
    let udp = UdpSocket::bind(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            #[cfg(not(feature = "production"))]
            common::ports::LUNABASE_SIM_TELEOP,
            #[cfg(feature = "production")]
            common::ports::TELEOP,
        ))
        .expect("Failed to bind to teleop port");

        udp.set_nonblocking(true)
            .expect("Failed to set non-blocking");

        let cakap_sm = PeerStateMachine::new(Duration::from_millis(150), 1024, 1400);
      Self{
       base,
       inner: Some(botconnInner{
        udp,
        cakap_sm,
        to_lunabot: VecDeque::new(),
       }
    )
    
  }

    
  }
 fn process(&mut self,delta:f64){
    let mut received = false;
    let mut buf = [0u8; 1500];
    let data = &buf;

 if let Some(inner) = self.inner.as_mut(){
    while let Some(msg) = inner.cakap_sm.poll(){
      self.handle_messages(msg, &mut received);
  }

 }
    }


 
} 
impl lunabotConn {
    fn handle_messages(&mut self, msg: FromLunabot,  received: &mut bool) {
        *received = true;
     if let FromLunabot::Ping(stage) = msg{
        match stage {
           LunabotStage::Manual => {
               self.base_mut().emit_signal("entered_manual", &[])
            }
            LunabotStage::SoftStop => {
                self.base_mut().emit_signal("entered_safestop", &[])
            }
            LunabotStage::Autonomy =>{
                self.base_mut().emit_signal("entered_autonomy", &[])
            }
                            };
     }
     if let Some(inner) = self.inner.as_mut(){
    
                PONG_MESSAGE.with(|pong| {
                    if let Ok(packet) = inner
                        .cakap_sm
                        .get_packet_builder()
                        .new_unreliable(pong.to_vec().into())
                    {
                        inner.to_lunabot.push_back(Action::SendUnreliable(packet));
                    } else {
                        godot_warn!("Failed to create Pong packet");
                    }
                });
     }  

    }
  
}
   
