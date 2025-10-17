use common::LunabotStage;
use godot::prelude::*;
use godot::classes::*;
use common::{FromLunabot, LUNABOT_STAGE};

#[derive(GodotClass)]
#[class(base = Button)]
pub struct safestop {
    base: Base<Button>,
}

#[godot_api]
impl IButton for safestop{
  fn init(base: Base<Button>) -> Self{
    Self {base}
  }
 fn pressed(&mut self){
    
    LUNABOT_STAGE.store(LunabotStage::SoftStop);
 }

}