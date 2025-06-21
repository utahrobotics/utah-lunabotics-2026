//! A simple, sans-io, handshakeless, networking protocol that multiplexes unreliable, reliable, and eventually reliable
//! communication over a single, unreliable transport layer. This protocol is connectionless and inherits
//! the ordering guarantees of the transport layer. For example, if the transport layer is UDP, then the
//! protocol will be unordered as well. If the transport layer is something like a WebRTC ordered and unreliable
//! data channel, then this protocol will be ordered as well.
//!
//! # Usage
//! This crate provides just the state machine for the protocol without any I/O. To use it, you must create
//! an event loop for each unique connection and poll the state machine with incoming events. The state machine
//! will then produce a [`RecommendedAction`] that you should take.
//!
//! # Security
//! Since this protocol is handshakeless and connectionless, it is vulnerable to abuse. This protocol is not intended
//! to be used on the open internet in production, but rather in a controlled environment without bad actors. The Utah
//! Student Robotics club uses this protocol to communicate between an operator and a robot in a network with no other
//! clients.

use std::{
    collections::VecDeque,
    num::NonZeroU64,
    sync::{atomic::AtomicU64, Arc},
    time::{Duration, Instant},
    u64,
};

use error::CakapError;
use fxhash::FxHashMap;
use indexmap::IndexSet;
use packet::{
    Action, HotPacket, HotPacketInner, PacketBuilder, ReliableIndex, ReliablePacket,
    UnreliablePacket,
};

pub mod error;
pub mod packet;

#[derive(Debug)]
pub struct Shared {
    reliable_index: AtomicU64,
    max_packet_size: usize,
}

#[derive(Debug)]
struct Retransmit {
    send_at: Instant,
    data: Box<[u8]>,
}

pub struct PeerStateMachine {
    shared: Arc<Shared>,
    retransmission_duration: Duration,
    retransmission_map: FxHashMap<NonZeroU64, Retransmit>,
    retransmission_queue: VecDeque<NonZeroU64>,
    received_set: IndexSet<NonZeroU64>,
    max_received_set_size: usize,
}

impl PeerStateMachine {
    /// Creates a new [`PeerStateMachine`] with the given retransmission duration and maximum received set size.
    ///
    /// The retransmission duration is the amount of time to wait before retransmitting a packet that has not been
    /// acknowledged. The maximum received set size should be proportional to the number of reliable packets sent per second,
    /// and varies based on the unreliability of the transport layer. If you are intending on sending many reliable packets
    /// over a very unreliable transport layer, you should set this to a higher value, which comes at the cost of approximately
    /// 32 bytes per unit. That is, if `max_received_set_size` is 100, then the received set will consume approximately up to 3200 bytes.
    /// Setting this value too low may cause this peer to acknowledge reliable packets that have already been received (thus handling
    /// them twice).
    pub fn new(
        retransmission_duration: Duration,
        max_received_set_size: usize,
        max_packet_size: usize,
    ) -> Self {
        Self {
            retransmission_duration,
            max_received_set_size,
            shared: Arc::new(Shared {
                reliable_index: AtomicU64::new(1),
                max_packet_size,
            }),
            retransmission_map: Default::default(),
            retransmission_queue: Default::default(),
            received_set: Default::default(),
        }
    }

    pub fn send_reconnection_msg<'a>(
        &'a mut self,
        now: Instant,
    ) -> (RecommendedAction<'a, 'static>, ReliableIndex) {
        let index = !(1u64 << 63);
        let data = Box::new(index.to_be_bytes());
        let index = ReliableIndex(NonZeroU64::new(index).unwrap());

        (
            self.poll(
                Event::Action(Action::SendReliable(ReliablePacket { index, data })),
                now,
            ),
            index,
        )
    }

    pub fn get_packet_builder(&self) -> PacketBuilder {
        PacketBuilder {
            shared: self.shared.clone(),
        }
    }

    pub fn is_packet_retransmitting(&self, index: ReliableIndex) -> bool {
        self.retransmission_map.contains_key(&index.0)
    }

    /// Digests the given [`Event`] according to the given [`Instant`] and produces a [`RecommendedAction`] that should be taken.
    ///
    /// Strictly speaking, `now` does not need to be the same [`Instant`] across all calls to `poll`. However, it must
    /// be monotonic across all instances used. Essentially, you can pass a different [`Instant`] to a successive call
    /// to `poll` as it represents a point in the future (you can skip time forward, but not backward).
    pub fn poll<'a, 'b>(&'a mut self, event: Event<'b>, now: Instant) -> RecommendedAction<'a, 'b> {
        match event {
            Event::IncomingData(data) => {
                if data.len() < 8 {
                    return RecommendedAction::HandleError(CakapError::PacketTooSmall);
                }

                let index = u64::from_be_bytes(data[data.len() - 8..].try_into().unwrap());

                if index == !(1 << 63) {
                    // The maximum safe index is 2^63 - 1
                    if data.len() != 8 {
                        return RecommendedAction::HandleError(CakapError::InvalidPacket);
                    }
                    // An empty packet with the max index is a request to clear the received set.
                    // This is important if the peer forgets their reliable index, which could
                    // cause new reliable messages from them to be ignored by us as they would
                    // be considered duplicates.
                    // The max index is the least likely index to be in the `received_set`, so
                    // it is a good choice for this purpose.
                    self.received_set.clear();
                    return RecommendedAction::SendData(HotPacket {
                        inner: HotPacketInner::Index(u64::MAX.to_be_bytes()),
                    });
                } else if let Some(index) = NonZeroU64::new(index) {
                    // A reliable packet from peer
                    let msb = index.get() >> 63;
                    if msb == 0 {
                        let reply_index = index | (1 << 63);

                        // New packet from peer
                        if self.received_set.insert(index) {
                            if self.received_set.len() > self.max_received_set_size {
                                self.received_set.shift_remove_index(0);
                            }

                            return RecommendedAction::HandleDataAndSend {
                                received: &data[0..data.len() - 8],
                                to_send: reply_index.get().to_be_bytes(),
                            };
                        } else {
                            // Duplicate packet from peer, just acknowledge
                            return RecommendedAction::SendData(HotPacket {
                                inner: HotPacketInner::Index(reply_index.get().to_be_bytes()),
                            });
                        }
                    } else {
                        // Acknowledgement from peer
                        let true_index = index.get() & !(1 << 63);
                        let Some(true_index) = NonZeroU64::new(true_index) else {
                            return RecommendedAction::HandleError(CakapError::InvalidPacket);
                        };
                        self.retransmission_map.remove(&true_index);
                    }
                } else {
                    // Unreliable packet from peer
                    return RecommendedAction::HandleData(&data[0..data.len() - 8]);
                }
            }
            Event::Action(action) => match action {
                Action::SendReliable(ReliablePacket { index, data }) => {
                    let index = index.0;
                    let option = self.retransmission_map.insert(
                        index,
                        Retransmit {
                            send_at: now + self.retransmission_duration,
                            data,
                        },
                    );
                    debug_assert!(option.is_none());
                    self.retransmission_queue.push_back(index);

                    return RecommendedAction::SendData(HotPacket {
                        inner: HotPacketInner::Borrowed(
                            &self.retransmission_map.get(&index).unwrap().data,
                        ),
                    });
                }
                Action::CancelReliable(ReliableIndex(index)) => {
                    self.retransmission_map.remove(&index);
                }
                Action::CancelAllReliable => {
                    self.retransmission_map.clear();
                    self.retransmission_queue.clear();
                }
                Action::SendUnreliable(UnreliablePacket { data }) => {
                    return RecommendedAction::SendData(HotPacket {
                        inner: HotPacketInner::Owned(data),
                    })
                }
            },
            Event::NoEvent => {}
        }
        loop {
            let Some(&first_index) = self.retransmission_queue.front() else {
                break RecommendedAction::WaitForData;
            };
            let Some(retransmit) = self.retransmission_map.get_mut(&first_index) else {
                self.retransmission_queue.pop_front();
                continue;
            };
            if retransmit.send_at <= now {
                self.retransmission_queue.pop_front();
                self.retransmission_queue.push_back(first_index);
                retransmit.send_at = now + self.retransmission_duration;
                // To please the borrow checker
                let retransmit = self.retransmission_map.get(&first_index).unwrap();
                break RecommendedAction::SendData(HotPacket {
                    inner: HotPacketInner::Borrowed(&retransmit.data),
                });
            } else {
                break RecommendedAction::WaitForDuration(retransmit.send_at - now);
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum RecommendedAction<'a, 'b> {
    /// Wait indefinitely until data from the peer is received, or there is data to send.
    WaitForData,
    /// Wait at most the given [`Duration`] for data from the peer, or data to be sent.
    ///
    /// If the given duration is 5 seconds, the event loop should still poll the state
    /// machine if there is data to be sent, or data was received from the peer. However,
    /// if 5 seconds have passed and neither event occurs, the state machine should still
    /// be polled anyway.
    WaitForDuration(Duration),
    /// Handle the given error (by logging or otherwise) and poll the state machine again
    /// with `NoEvent`.
    HandleError(CakapError),
    /// Handle the given data from the peer.
    HandleData(&'b [u8]),
    /// Handle `received` from the peer, and send `to_send` to the peer.
    ///
    /// If the given message is not valid for whatever reason, you can choose to not
    /// send `to_send` and *not* poll the state machine with `NoEvent`.
    HandleDataAndSend {
        received: &'b [u8],
        to_send: [u8; 8],
    },
    /// Send the given data to the peer.
    SendData(HotPacket<'a>),
}

impl<'a, 'b> RecommendedAction<'a, 'b> {
    #[cfg(test)]
    fn get_hot_packet(&self) -> &HotPacket<'a> {
        match self {
            Self::SendData(hot_packet) => hot_packet,
            _ => panic!("Expected SendData, got {:?}", self),
        }
    }
}

pub enum Event<'a> {
    /// A whole packet of data, with no padding bytes or otherwise empty space.
    IncomingData(&'a [u8]),
    /// An [`Action`] to perform.
    Action(Action),
    /// No data received, to be sent. Usually used when some duration of time has passed,
    /// or after an error was handled.
    NoEvent,
}

impl<'a> Default for Event<'a> {
    fn default() -> Self {
        Self::NoEvent
    }
}

impl<'a> From<Action> for Event<'a> {
    fn from(value: Action) -> Self {
        Self::Action(value)
    }
}

#[cfg(test)]
mod tests {
    use std::ops::Deref;

    use super::*;

    #[test]
    fn send_unreliable_1() {
        let mut state_machine = PeerStateMachine::new(Duration::from_millis(100), 256, 1400);
        let reliable_builder = state_machine.get_packet_builder();
        let outgoing_data = reliable_builder
            .new_unreliable([217].into_iter().collect())
            .unwrap();
        let event = Event::Action(outgoing_data.into());
        let action = state_machine.poll(event, Instant::now());

        // `state_machine` sends a reliable packet
        assert_eq!(
            action.get_hot_packet().deref(),
            [217, 0, 0, 0, 0, 0, 0, 0, 0]
        );
        // `state_machine` is notified that the packet is sent
        assert_eq!(
            state_machine.poll(Event::NoEvent, Instant::now()),
            RecommendedAction::WaitForData
        );

        let mut other_state_machine = PeerStateMachine::new(Duration::from_millis(100), 256, 1400);
        // `other_state_machine` receives the unreliable packet
        let event = Event::IncomingData(&[217, 0, 0, 0, 0, 0, 0, 0, 0]);
        let action = other_state_machine.poll(event, Instant::now());

        // `other_state_machine` handles the unreliable packet
        assert_eq!(action, RecommendedAction::HandleData(&[217]));
    }

    #[test]
    fn send_reliable_1() {
        let mut state_machine = PeerStateMachine::new(Duration::from_millis(100), 256, 1400);
        let reliable_builder = state_machine.get_packet_builder();
        let outgoing_data = reliable_builder
            .new_reliable([15].into_iter().collect())
            .unwrap();

        assert_eq!(outgoing_data.get_index().0.get(), 1);

        let event = Event::Action(outgoing_data.into());
        let action = state_machine.poll(event, Instant::now());

        // `state_machine` sends a reliable packet
        assert_eq!(
            action.get_hot_packet().deref(),
            [15, 0, 0, 0, 0, 0, 0, 0, 1],
        );
        // `state_machine` is notified that the packet is sent
        let action = state_machine.poll(Event::NoEvent, Instant::now());
        let RecommendedAction::WaitForDuration(duration) = action else {
            panic!("Not WaitForDuration")
        };
        assert!(duration.as_millis() > 98);

        let mut other_state_machine = PeerStateMachine::new(Duration::from_millis(100), 256, 1400);
        // `other_state_machine` receives the reliable packet
        let event = Event::IncomingData(&[15, 0, 0, 0, 0, 0, 0, 0, 1]);
        let action = other_state_machine.poll(event, Instant::now());

        // `other_state_machine` sends an acknowledgement
        let to_send = (1u64 + (1 << 63)).to_be_bytes();
        assert_eq!(
            action,
            RecommendedAction::HandleDataAndSend {
                received: &[15],
                to_send
            }
        );

        // `other_state_machine` is notified that the packet is sent
        assert_eq!(
            other_state_machine.poll(Event::NoEvent, Instant::now()),
            RecommendedAction::WaitForData
        );

        // `state_machine` waits for data
        let action = state_machine.poll(Event::NoEvent, Instant::now());
        let RecommendedAction::WaitForDuration(duration) = action else {
            panic!("Not WaitForDuration")
        };
        assert!(duration.as_millis() > 98);

        // `state_machine` receives the acknowledgement
        let event = Event::IncomingData(&to_send);
        let action = state_machine.poll(event, Instant::now());

        assert_eq!(action, RecommendedAction::WaitForData);
    }

    #[test]
    fn send_reliable_2() {
        let mut state_machine = PeerStateMachine::new(Duration::from_millis(100), 256, 1400);
        let reliable_builder = state_machine.get_packet_builder();
        let outgoing_data = reliable_builder
            .new_reliable([15].into_iter().collect())
            .unwrap();

        assert_eq!(outgoing_data.get_index().0.get(), 1);

        let event = Event::Action(outgoing_data.into());
        let action = state_machine.poll(event, Instant::now());

        // `state_machine` sends a reliable packet
        assert_eq!(
            action.get_hot_packet().deref(),
            [15, 0, 0, 0, 0, 0, 0, 0, 1],
        );
        // `state_machine` is notified that the packet is sent
        let action = state_machine.poll(Event::NoEvent, Instant::now());
        let RecommendedAction::WaitForDuration(duration) = action else {
            panic!("Not WaitForDuration")
        };
        assert!(duration.as_millis() > 98);

        let mut other_state_machine = PeerStateMachine::new(Duration::from_millis(100), 256, 1400);
        // `other_state_machine` receives the reliable packet
        let event = Event::IncomingData(&[15, 0, 0, 0, 0, 0, 0, 0, 1]);
        let action = other_state_machine.poll(event, Instant::now());

        // `other_state_machine` sends an acknowledgement, but the ack is lost
        let to_send = (1u64 + (1 << 63)).to_be_bytes();
        assert_eq!(
            action,
            RecommendedAction::HandleDataAndSend {
                received: &[15],
                to_send
            }
        );

        // `other_state_machine` is notified that the packet is sent
        assert_eq!(
            other_state_machine.poll(Event::NoEvent, Instant::now()),
            RecommendedAction::WaitForData
        );

        // `state_machine` waits for data
        let action = state_machine.poll(Event::NoEvent, Instant::now());
        let RecommendedAction::WaitForDuration(duration) = action else {
            panic!("Not WaitForDuration")
        };
        assert!(duration.as_millis() > 98);

        // 'state_machine' retransmits after some time
        let action =
            state_machine.poll(Event::NoEvent, Instant::now() + Duration::from_millis(100));
        assert_eq!(
            action.get_hot_packet().deref(),
            [15, 0, 0, 0, 0, 0, 0, 0, 1],
        );
        // `state_machine` is notified that the packet is sent
        let action = state_machine.poll(Event::NoEvent, Instant::now());
        let RecommendedAction::WaitForDuration(duration) = action else {
            panic!("Not WaitForDuration")
        };
        assert!(duration.as_millis() > 98);

        // `other_state_machine` receives the data
        let event = Event::IncomingData(&[15, 0, 0, 0, 0, 0, 0, 0, 1]);
        let action = other_state_machine.poll(event, Instant::now());

        // `other_state_machine` sends an acknowledgement without handling the data
        assert_eq!(action.get_hot_packet().deref(), to_send);

        // `state_machine` receives the acknowledgement
        let event = Event::IncomingData(&to_send);
        let action = state_machine.poll(event, Instant::now());

        assert_eq!(action, RecommendedAction::WaitForData);
    }
}
