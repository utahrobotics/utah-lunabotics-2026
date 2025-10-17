

use common::{LUNABOT_STAGE, LunabotStage};
use std::thread;
use std::time::Duration;
use godot::prelude::*;
use godot::classes::*;

#[derive(GodotClass)]
#[class(base = Node)]
pub struct StateViewer {
    base: Base<Node>,
    last_stage: LunabotStage,
}
#[godot_api]
impl INode for StateViewer { 
    fn init(base: Base<Node>) -> Self {
         Self { 
            
            base,
             last_stage : LUNABOT_STAGE.load(),
            
             }
         }
     }

#[godot_api]
impl StateViewer{
    #[func]
    fn checkout_stage(&self)  {
    
      let last_stage = LUNABOT_STAGE.load();

        match last_stage {
        LunabotStage::Manual => godot_print!("Manual stage"),
        LunabotStage::SoftStop => godot_print!("SoftStop"),
        LunabotStage::Autonomy => godot_print!("Autonomy stage"),

}

    }
}
