use std::{
    fmt::Debug,
    num::NonZeroU64,
    ops::Deref,
    sync::{atomic::Ordering, Arc},
};

use crate::{error::BuildPacketError, Shared};

#[derive(Debug)]
pub enum Action {
    SendReliable(ReliablePacket),
    CancelReliable(ReliableIndex),
    CancelAllReliable,
    SendUnreliable(UnreliablePacket),
}

impl From<ReliablePacket> for Action {
    fn from(packet: ReliablePacket) -> Self {
        Self::SendReliable(packet)
    }
}

impl From<UnreliablePacket> for Action {
    fn from(packet: UnreliablePacket) -> Self {
        Self::SendUnreliable(packet)
    }
}

#[derive(Debug)]
pub struct ReliablePacket {
    pub(crate) index: ReliableIndex,
    pub(crate) data: Box<[u8]>,
}

impl ReliablePacket {
    pub fn get_index(&self) -> ReliableIndex {
        self.index
    }
}

#[derive(Clone, Debug)]
pub struct UnreliablePacket {
    pub(crate) data: Box<[u8]>,
}

#[derive(Clone, Copy, Debug)]
pub struct ReliableIndex(pub(crate) NonZeroU64);

pub struct PacketBody {
    pub data: Vec<u8>,
}

impl PacketBody {
    fn into_bytes(mut self, extra: &[u8]) -> Box<[u8]> {
        self.data.extend_from_slice(extra);
        self.data.into_boxed_slice()
    }
}

impl From<Vec<u8>> for PacketBody {
    fn from(data: Vec<u8>) -> Self {
        Self { data }
    }
}

impl FromIterator<u8> for PacketBody {
    fn from_iter<T: IntoIterator<Item = u8>>(iter: T) -> Self {
        Self {
            data: iter.into_iter().collect(),
        }
    }
}

/// Used to create reliable and unreliable packets.
#[derive(Clone, Debug)]
pub struct PacketBuilder {
    pub(crate) shared: Arc<Shared>,
}

impl PacketBuilder {
    /// Sends the given bytes unreliably.
    ///
    /// Packets cannot have a zero-sized payload.
    pub fn new_unreliable(&self, body: PacketBody) -> Result<UnreliablePacket, BuildPacketError> {
        if body.data.is_empty() {
            return Err(BuildPacketError::EmptyBuffer { buffer: body.data });
        }
        if body.data.len() > self.shared.max_packet_size {
            return Err(BuildPacketError::BufferTooLarge {
                buffer: body.data,
                max_packet_size: self.shared.max_packet_size,
            });
        }

        Ok(UnreliablePacket {
            data: body.into_bytes(&[0; 8]),
        })
    }

    /// Sends the given bytes reliably.
    ///
    /// Packets cannot have a zero-sized payload.
    ///
    /// # Safety
    /// Strictly speaking, unexpected behavior can occur if this method is called 2^63 - 1 times per struct due to overflow.
    /// However, this is hopefully not a practical concern.
    pub fn new_reliable(&self, body: PacketBody) -> Result<ReliablePacket, BuildPacketError> {
        if body.data.is_empty() {
            return Err(BuildPacketError::EmptyBuffer { buffer: body.data });
        }
        if body.data.len() > self.shared.max_packet_size {
            return Err(BuildPacketError::BufferTooLarge {
                buffer: body.data,
                max_packet_size: self.shared.max_packet_size,
            });
        }

        let reliable_index = self.shared.reliable_index.fetch_add(1, Ordering::Relaxed);
        let bytes = body.into_bytes(&reliable_index.to_be_bytes());
        let reliable_index = NonZeroU64::new(reliable_index).expect("Reliable Index has overflowed. Consider reconstructing the state machine earlier to avoid this");

        Ok(ReliablePacket {
            data: bytes,
            index: ReliableIndex(reliable_index),
        })
    }
}

pub(crate) enum HotPacketInner<'a> {
    Borrowed(&'a [u8]),
    Owned(Box<[u8]>),
    Index([u8; 8]),
}

/// A packet of data that the state machine has determined needs to be sent immediately.
///
/// [`OutgoingData`] represents an intent to send data, that the state machine digests.
/// The state machine then produces [`HotPacket`]s at the appropriate time. When the packet
/// has been sent, the state machine should be notified.
pub struct HotPacket<'a> {
    pub(crate) inner: HotPacketInner<'a>,
}

impl<'a> Deref for HotPacket<'a> {
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        match &self.inner {
            HotPacketInner::Borrowed(buf) => buf,
            HotPacketInner::Owned(buf) => buf,
            HotPacketInner::Index(buf) => buf,
        }
    }
}

impl<'a> PartialEq for HotPacket<'a> {
    fn eq(&self, other: &Self) -> bool {
        let self_bytes = match &self.inner {
            HotPacketInner::Borrowed(buf) => *buf,
            HotPacketInner::Owned(buf) => buf,
            HotPacketInner::Index(buf) => buf,
        };
        let other_bytes = match &other.inner {
            HotPacketInner::Borrowed(buf) => *buf,
            HotPacketInner::Owned(buf) => buf,
            HotPacketInner::Index(buf) => buf,
        };
        self_bytes == other_bytes
    }
}

impl<'a> Eq for HotPacket<'a> {}

impl<'a> Debug for HotPacket<'a> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("HotPacket")
            .field("data", &self.deref())
            .finish()
    }
}
