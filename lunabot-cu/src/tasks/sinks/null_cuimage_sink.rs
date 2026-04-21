#[cfg(feature="production")]
pub mod implementation {
    use cu_gstreamer::CuGstBuffer;
    use cu29::{cutask::{CuSinkTask, Freezable}, input_msg};
    use cu29::prelude::*;
    
    pub struct NullCuImageSink {

    } 

    impl Freezable for NullCuImageSink {}

    impl CuSinkTask for NullCuImageSink {
        type Input<'m> = input_msg!('m, CuGstBuffer);
    
        type Resources<'r> = ();
    
        fn new(_config: Option<&cu29::prelude::ComponentConfig>, _resources: Self::Resources<'_>) -> cu29::CuResult<Self>
        where
            Self: Sized {
            Ok(
                NullCuImageSink {  }
            )
        }
    
        fn process<'i>(&mut self, _clock: &cu29::prelude::RobotClock, _input: &Self::Input<'i>) -> cu29::CuResult<()> {
            Ok(())
        }
    }
}


#[cfg(not(feature="production"))]
pub mod implementation {
    use cu29::{cutask::{CuSinkTask, Freezable}, input_msg};
    use cu29::prelude::*;
    use crate::tasks::auto_gstreamer::CuGstBuffer;

    pub struct NullCuImageSink {

    }

    impl Freezable for NullCuImageSink {}

    impl CuSinkTask for NullCuImageSink {
        type Input<'m> = input_msg!('m, CuGstBuffer);
    
        type Resources<'r> = ();
    
        fn new(_config: Option<&cu29::prelude::ComponentConfig>, _resources: Self::Resources<'_>) -> cu29::CuResult<Self>
        where
            Self: Sized {
            Ok(
                NullCuImageSink {  }
            )
        }
    
        fn process<'i>(&mut self, _clock: &cu29::prelude::RobotClock, input: &Self::Input<'i>) -> cu29::CuResult<()> {
            println!("calling null cuimage sink");
            Ok(
                ()
            )
        }
    }
}