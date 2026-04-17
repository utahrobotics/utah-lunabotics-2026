use std::{
    net::SocketAddr,
    sync::{Arc, Mutex},
};

use bincode::{Decode, Encode};
use quinn::{
    ClientConfig, ConnectionError, Endpoint, RecvStream, SendStream, TransportConfig, VarInt,
};

use crate::{
    KeepAlive, StreamHandle, start_client_keep_alive,
    utils::{IDLE_TIMEOUT_MS, SkipServerVerification, make_client_endpoint},
};

pub struct QuicClient<KA> {
    endpoint: Endpoint,
    connection: quinn::Connection,
    pending: Mutex<Vec<(u8, SendStream, RecvStream)>>,
    pub keep_alive: KeepAlive<KA>,
}

impl<KA> QuicClient<KA> {
    /// Access the inner [`quinn::Connection`].
    ///
    /// Useful for cloning the refcounted connection handle so other tasks
    /// can wait on `connection.closed()` without holding the [`QuicClient`].
    pub fn quinn(&self) -> &quinn::Connection {
        &self.connection
    }

    /// Connect to a QUIC server, verifying the server certificate.
    pub async fn connect(
        bind_addr: SocketAddr,
        server_addr: SocketAddr,
        server_name: &str,
        server_cert_der: &[u8],
        initial_keep_alive_msg: KA,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>>
    where
        KA: Encode + Decode<()> + Clone + Send + Sync + 'static,
    {
        let endpoint = make_client_endpoint(bind_addr, &[server_cert_der])?;
        let connection = endpoint.connect(server_addr, server_name)?.await?;
        let keep_alive = start_client_keep_alive(connection.clone(), initial_keep_alive_msg);

        Ok(Self {
            endpoint,
            connection,
            pending: Mutex::new(Vec::new()),
            keep_alive,
        })
    }

    /// Connect to a QUIC server without verifying the server certificate.
    pub async fn connect_insecure(
        bind_addr: SocketAddr,
        server_addr: SocketAddr,
        server_name: &str,
        initial_keep_alive_msg: KA,
    ) -> Result<Self, Box<dyn std::error::Error + Send + Sync>>
    where
        KA: Encode + Decode<()> + Clone + Send + Sync + 'static,
    {
        let crypto = quinn::rustls::ClientConfig::builder()
            .dangerous()
            .with_custom_certificate_verifier(SkipServerVerification::new())
            .with_no_client_auth();

        let quic_crypto = quinn::crypto::rustls::QuicClientConfig::try_from(crypto)?;
        let mut config = ClientConfig::new(Arc::new(quic_crypto));
        let mut transport = TransportConfig::default();
        transport.max_idle_timeout(Some(VarInt::from_u32(IDLE_TIMEOUT_MS).into()));
        config.transport_config(Arc::new(transport));

        let mut endpoint = Endpoint::client(bind_addr)?;
        endpoint.set_default_client_config(config);
        let connection = endpoint.connect(server_addr, server_name)?.await?;
        let keep_alive = start_client_keep_alive(connection.clone(), initial_keep_alive_msg);
        Ok(Self {
            endpoint,
            connection,
            keep_alive,
            pending: Mutex::new(Vec::new()),
        })
    }

    /// Open a typed bidirectional stream to the server.
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

    /// Accept a bidirectional stream of the given type opened by the server.
    /// ID is sent/recved as the first byte when opening a stream to verify the types are correct.
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

    /// Close the connection gracefully.
    pub fn close(&self) {
        self.connection.close(0u32.into(), b"done");
    }
}

impl<KA> Drop for QuicClient<KA> {
    fn drop(&mut self) {
        self.endpoint.close(0u32.into(), b"client dropped");
    }
}
