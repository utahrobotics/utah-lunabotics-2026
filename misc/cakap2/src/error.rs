use std::fmt::Display;

#[derive(Debug, PartialEq, Eq)]
pub enum CakapError {
    /// A packet from the peer was too small to be processed.
    PacketTooSmall,
    /// A packet from the peer was too large to be processed.
    PacketTooLong,
    /// A packet from the peer was invalid.
    InvalidPacket,
}

impl Display for CakapError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::PacketTooSmall => write!(f, "Packet from peer was too small to be processed"),
            Self::PacketTooLong => write!(f, "Packet from peer was too large to be processed"),
            Self::InvalidPacket => write!(f, "Packet from peer was invalid"),
        }
    }
}

impl std::error::Error for CakapError {}

#[derive(Debug, thiserror::Error)]
pub enum BuildPacketError {
    #[error("Buffer too large, max packet size: {max_packet_size}")]
    BufferTooLarge {
        buffer: Vec<u8>,
        max_packet_size: usize,
    },
    #[error("Buffer is empty")]
    EmptyBuffer { buffer: Vec<u8> },
}
