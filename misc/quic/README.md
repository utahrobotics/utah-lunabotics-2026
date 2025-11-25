# QUIC 
A wrapper for the quinn crate that offers specialized functionality for lunabot <-> lunabase bi directional communication.

## Usage

#### Server and client creation:
```rust
let server = QuicServer::listen(4444, Duration::from_millis(100))?;
let socket_addr = SocketAddr::V4(SocketAddrV4::new("127.0.0.1".parse::<Ipv4Addr>().unwrap(), 4444));
let client = QuicClient::connect(socket_addr);
```

#### Unreliable delivery / Using the keep alive msg to sync state: 
In our codebase, the keep alive msg is always set to reflect the lunabots stage, i.e. Manual, SoftStop, Autonomous.
The lunabase is able to always know what state the lunabot is based on the last seen keep alive.

```rust
// keep alive is sent as an unreliable datagram
server.set_keep_alive_msg(&[0]);

// gets last received keep alive.
client.get_last_keep_alive_msg(); // can later parse the keep alive message into something useful
```

Additionally, the keep alive msg is useful for detecting if the lunabot is still connected:
```rust
server.time_since_last_keep_alive();
```

**The keep alive is NOT AN ECHO, both the server and clients can send their own keep alive packets, as set by the user.** <br>
**IMPORTANT: The keep alive packet must fit into one datagram. (nothing will panic, but an error message will start printing that the keep alive could not be sent)**


#### Reliable delivery
Send over any data that implements bincode encode and decode using the send and recv functions.

