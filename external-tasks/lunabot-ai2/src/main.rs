#![feature(unsigned_signed_diff)]
#![feature(mixed_integer_ops_unsigned_sub)]

use context::HostHandle;

mod context;
mod software_stop;
mod teleop;


#[tokio::main]
async fn main() -> ! {
    let mut host_handle = HostHandle::new();

    loop {
        software_stop::software_stop(&mut host_handle).await;
        teleop::teleop(&mut host_handle).await;
    }
}