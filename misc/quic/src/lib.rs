mod utils;
use std::{marker::PhantomData, net::SocketAddrV4, str::FromStr, sync::Arc, time::Duration};

use bincode::{Decode, Encode};
use quinn::{Connection, RecvStream, SendStream};
use tasker::parking_lot::Mutex;

use crate::utils::make_server_endpoint;

pub struct InnerShared {
    pub connection: Option<Connection>,
    pub send_stream: Option<SendStream>,
    pub recv_stream: Option<RecvStream>,
}

pub struct QuicServer<Msg: Encode + Decode<()>> {
    pub ping_delay: Duration,
    pub shared: Arc<Mutex<InnerShared>>,
    _boo: PhantomData<Msg>,
}

impl<Msg: Encode + Decode<()>> QuicServer<Msg> {
    /// Listens on 0.0.0.0:{port}
    /// Accepts incomming connections
    pub fn listen(
        port: u32,
        ping_delay: Duration,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let (endpoint, _cert) = make_server_endpoint(std::net::SocketAddr::V4(
            SocketAddrV4::from_str(&format!("0.0.0.0:{port}")).unwrap(),
        ))?;
        let shared = Arc::new(Mutex::new(InnerShared {
            connection: None,
            send_stream: None,
            recv_stream: None,
        }));

        let shared_c1 = Arc::clone(&shared);
        tasker::get_tokio_handle().spawn(async move {
            loop {
                let Some(incoming) = endpoint.accept().await else {
                    eprintln!("No Incoming Connection");
                    continue;
                };
                match incoming.accept() {
                    Ok(ongoing) => {
                        // 0 rrt should make re connections faster
                        let connection = ongoing.into_0rtt();
                        if let Err(ongoing) = connection {
                            eprintln!(
                                "0-RRT Connection failed, attempting to establish normal connection"
                            );
                            match ongoing.await {
                                Ok(connection) => {
                                    match connection.accept_bi().await {
                                        Ok((send_stream, recv_stream)) => {
                                            shared_c1.lock().send_stream = Some(send_stream);
                                            shared_c1.lock().recv_stream = Some(recv_stream);
                                        }
                                        Err(e) => {
                                            eprintln!(
                                                "Failed to accept bi directional stream: {e}"
                                            );
                                        }
                                    }
                                    shared_c1.lock().connection = Some(connection);
                                }
                                Err(e) => {
                                    eprintln!("Failed to establish connection: {e}");
                                }
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Failed to accept incoming connection: {e}");
                    }
                }
            }
        });
        Ok(Self {
            ping_delay,
            shared,
            _boo: PhantomData {},
        })
    }

    pub fn recv(&self) -> Result<Msg, Box<dyn std::error::Error + Send + Sync>> {
        todo!()
    }

    pub fn try_recv(&self) -> Result<Msg, Box<dyn std::error::Error + Send + Sync>> {
        todo!()
    }

    pub fn send(&self, packet: Msg) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let bytes = bincode::encode_to_vec(packet, bincode::config::standard())?;
        todo!()
    }

    pub fn try_send(&self, packet: Msg) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        let bytes = bincode::encode_to_vec(packet, bincode::config::standard())?;
        todo!()
    }
}
