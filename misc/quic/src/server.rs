use std::{net::SocketAddrV4, str::FromStr, sync::Mutex, time::Duration};

use bincode::{Decode, Encode};
use quinn::{ConnectionError, Endpoint, RecvStream, SendStream};

use crate::{KeepAlive, StreamHandle, start_server_keep_alive, utils::make_server_endpoint};

pub struct QuicServer {
    endpoint: Endpoint,
    pub server_cert: Vec<u8>,
}

impl QuicServer {
    pub fn new(
        port: u16,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>> {
        let (endpoint, cert) = make_server_endpoint(std::net::SocketAddr::V4(
            SocketAddrV4::from_str(&format!("0.0.0.0:{port}")).unwrap(),
        ))?;
        Ok(Self {
            endpoint,
            server_cert: cert.to_vec(),
        })
    }

    /// Wait for an incoming connection and return a [`ServerConnection`].
    pub async fn accept<KA>(
        &self,
        keep_alive_interval: Duration,
        initial_keep_alive_msg: KA,
    ) -> Result<ServerConnection<KA>, Box<dyn std::error::Error + Send + Sync>>
    where
        KA: Encode + Decode<()> + Clone + Send + Sync + 'static,
    {
        let incoming = self.endpoint.accept().await.ok_or("endpoint closed")?;
        let connection = incoming.await?;
        let keep_alive = start_server_keep_alive(
            connection.clone(),
            initial_keep_alive_msg,
            keep_alive_interval,
        );
        Ok(ServerConnection {
            connection,
            pending: Mutex::new(Vec::new()),
            keep_alive,
        })
    }
}

pub struct ServerConnection<KA> {
    connection: quinn::Connection,
    pending: Mutex<Vec<(u8, SendStream, RecvStream)>>,
    pub keep_alive: KeepAlive<KA>,
}

impl<KA> ServerConnection<KA> {
    /// Accept a bidirectional stream of the given type.
    ///
    /// Reads the stream type ID byte and matches it against `S::ID`.
    /// Streams with non-matching IDs are buffered for later calls.
    pub async fn accept_bi<FromServer, FromClient, const ID: u8>(
        &self,
    ) -> Result<StreamHandle<FromServer, FromClient>, ConnectionError>
    where
        FromServer: Encode + Decode<()>,
        FromClient: Encode + Decode<()>,
    {
        {
            let mut pending = self.pending.lock().unwrap();
            if let Some(idx) = pending.iter().position(|(id, _, _)| *id == ID) {
                let (_, send, recv) = pending.remove(idx);
                return Ok(StreamHandle::new(send, recv));
            }
        }
        loop {
            let (send, mut recv) = self.connection.accept_bi().await?;
            let mut id_buf = [0u8; 1];
            recv.read_exact(&mut id_buf)
                .await
                .map_err(|_| ConnectionError::LocallyClosed)?;
            if id_buf[0] == ID {
                return Ok(StreamHandle::new(send, recv));
            }
            self.pending.lock().unwrap().push((id_buf[0], send, recv));
        }
    }

    /// Open a typed bidirectional stream to the peer.
    ///
    /// Writes the stream type ID byte first, then returns a typed [`StreamHandle`].
    pub async fn open_bi<FromClient, FromServer, const ID: u8>(
        &self,
    ) -> Result<StreamHandle<FromClient, FromServer>, ConnectionError>
    where
        FromServer: Encode + Decode<()>,
        FromClient: Encode + Decode<()>,
    {
        let (mut send, recv) = self.connection.open_bi().await?;
        send.write_all(&[ID])
            .await
            .map_err(|_| ConnectionError::LocallyClosed)?;
        Ok(StreamHandle::new(send, recv))
    }
}
