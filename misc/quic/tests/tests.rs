use bincode::{Decode, Encode};
use quic::{QuicClient, QuicServer};
use std::{net::SocketAddr, str::FromStr, thread, time::Duration};

#[derive(Debug, Clone, PartialEq, Encode, Decode)]
struct TestMessage {
    id: u32,
    data: String,
}

#[derive(Debug, Clone, PartialEq, Encode, Decode)]
struct PingMessage {
    timestamp: u64,
}

#[derive(Debug, Clone, PartialEq, Encode, Decode)]
struct LargeMessage {
    id: u32,
    payload: Vec<u8>,
}

const TEST_PORT_BASE: u32 = 9000;
const DEFAULT_KEEP_ALIVE_INTERVAL: Duration = Duration::from_millis(250);

fn get_test_addr(port_offset: u32) -> SocketAddr {
    SocketAddr::from_str(&format!("127.0.0.1:{}", TEST_PORT_BASE + port_offset)).unwrap()
}

#[test]
fn test_basic_send_receive() {
    println!("\n=== TEST: Basic Send/Receive ===");

    let addr = get_test_addr(0);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    // Give server time to start
    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    // Give connection time to establish
    thread::sleep(Duration::from_millis(200));

    println!("Checking connection status...");
    assert!(client.is_connected(), "Client should be connected");

    let test_msg = TestMessage {
        id: 42,
        data: "Hello from client!".to_string(),
    };

    println!("Client sending message: {:?}", test_msg);
    client
        .send(test_msg.clone())
        .expect("Client failed to send");

    println!("Server receiving message...");
    let received = server.recv().expect("Server failed to receive");
    println!("Server received: {:?}", received);

    assert_eq!(received, test_msg, "Messages should match");
    println!("✓ Basic send/receive test passed!");
}

#[test]
fn test_bidirectional_communication() {
    println!("\n=== TEST: Bidirectional Communication ===");

    let addr = get_test_addr(1);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 1, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    // Client -> Server
    let client_msg = TestMessage {
        id: 1,
        data: "Request from client".to_string(),
    };
    println!("Client -> Server: {:?}", client_msg);
    client.send(client_msg.clone()).expect("Client send failed");

    let server_received = server.recv().expect("Server recv failed");
    println!("Server received: {:?}", server_received);
    assert_eq!(server_received, client_msg);

    // Server -> Client
    let server_msg = TestMessage {
        id: 2,
        data: "Response from server".to_string(),
    };
    println!("Server -> Client: {:?}", server_msg);
    server.send(server_msg.clone()).expect("Server send failed");

    let client_received = client.recv().expect("Client recv failed");
    println!("Client received: {:?}", client_received);
    assert_eq!(client_received, server_msg);

    println!("✓ Bidirectional communication test passed!");
}

#[test]
fn test_multiple_messages() {
    println!("\n=== TEST: Multiple Messages ===");

    let addr = get_test_addr(2);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 2, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const NUM_MESSAGES: u32 = 10;

    // Send multiple messages
    for i in 0..NUM_MESSAGES {
        let msg = TestMessage {
            id: i,
            data: format!("Message number {}", i),
        };
        println!("Sending message {}/{}", i + 1, NUM_MESSAGES);
        client.send(msg).expect("Failed to send message");
    }

    // Receive and verify
    for i in 0..NUM_MESSAGES {
        println!("Receiving message {}/{}", i + 1, NUM_MESSAGES);
        let received = server.recv().expect("Failed to receive message");
        assert_eq!(received.id, i, "Message ID should match");
        assert_eq!(received.data, format!("Message number {}", i));
    }

    println!("✓ Multiple messages test passed!");
}

#[test]
fn test_large_message() {
    println!("\n=== TEST: Large Message ===");

    let addr = get_test_addr(3);
    let server = QuicServer::<LargeMessage>::listen(TEST_PORT_BASE + 3, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<LargeMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    // Create a large message (1MB)
    let payload_size = 1024 * 1024;
    let mut payload = vec![0u8; payload_size];
    for (i, byte) in payload.iter_mut().enumerate() {
        *byte = (i % 256) as u8;
    }

    let large_msg = LargeMessage {
        id: 999,
        payload: payload.clone(),
    };

    println!("Sending large message ({} bytes)...", payload_size);
    client
        .send(large_msg.clone())
        .expect("Failed to send large message");

    println!("Receiving large message...");
    let received = server.recv().expect("Failed to receive large message");

    assert_eq!(received.id, 999);
    assert_eq!(received.payload.len(), payload_size);
    assert_eq!(received.payload, payload);

    println!("✓ Large message test passed!");
}

#[test]
fn test_rapid_fire() {
    println!("\n=== TEST: Rapid Fire Messages ===");

    let addr = get_test_addr(4);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 4, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const RAPID_COUNT: u32 = 50;

    println!("Sending {} messages as fast as possible...", RAPID_COUNT);

    // Send rapidly without delays
    for i in 0..RAPID_COUNT {
        let msg = TestMessage {
            id: i,
            data: format!("Rapid #{}", i),
        };
        client.send(msg).expect("Failed to send rapid message");
    }

    println!("Receiving {} messages...", RAPID_COUNT);

    // Receive all
    for i in 0..RAPID_COUNT {
        let received = server.recv().expect("Failed to receive rapid message");
        assert_eq!(received.id, i);
    }

    println!("✓ Rapid fire test passed!");
}

#[test]
fn test_ping_pong() {
    println!("\n=== TEST: Ping Pong ===");

    let addr = get_test_addr(5);
    let server = QuicServer::<PingMessage>::listen(TEST_PORT_BASE + 5, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<PingMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const PING_PONG_ROUNDS: u64 = 5;

    for round in 0..PING_PONG_ROUNDS {
        println!("Round {}/{}", round + 1, PING_PONG_ROUNDS);

        // Client sends ping
        let ping = PingMessage { timestamp: round };
        client.send(ping.clone()).expect("Failed to send ping");

        // Server receives and echoes back
        let received_ping = server.recv().expect("Failed to receive ping");
        assert_eq!(received_ping.timestamp, round);
        server.send(received_ping).expect("Failed to send pong");

        // Client receives pong
        let pong = client.recv().expect("Failed to receive pong");
        assert_eq!(pong.timestamp, round);
    }

    println!("✓ Ping pong test passed!");
}

#[test]
fn test_empty_string_message() {
    println!("\n=== TEST: Empty String Message ===");

    let addr = get_test_addr(6);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 6, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    let empty_msg = TestMessage {
        id: 0,
        data: String::new(),
    };

    client
        .send(empty_msg.clone())
        .expect("Failed to send empty message");
    let received = server.recv().expect("Failed to receive empty message");

    assert_eq!(received, empty_msg);
    println!("✓ Empty string message test passed!");
}

#[test]
fn test_unicode_message() {
    println!("\n=== TEST: Unicode Message ===");

    let addr = get_test_addr(7);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 7, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    let unicode_msg = TestMessage {
        id: 0,
        data: "Hello 世界 🌍 Привет".to_string(),
    };

    client.send(unicode_msg.clone()).expect("Failed to send unicode message");
    let received = server.recv().expect("Failed to receive unicode message");

    assert_eq!(received, unicode_msg);
    println!("✓ Unicode message test passed!");
}



#[test]
fn test_alternating_communication() {
    println!("\n=== TEST: Alternating Communication ===");

    let addr = get_test_addr(8);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 8, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const ROUNDS: u32 = 5;

    for i in 0..ROUNDS {
        // Client talks
        let client_msg = TestMessage {
            id: i * 2,
            data: format!("Client message {}", i),
        };
        client.send(client_msg.clone()).expect("Client send failed");
        let server_recv = server.recv().expect("Server recv failed");
        assert_eq!(server_recv, client_msg);

        // Server talks
        let server_msg = TestMessage {
            id: i * 2 + 1,
            data: format!("Server message {}", i),
        };
        server.send(server_msg.clone()).expect("Server send failed");
        let client_recv = client.recv().expect("Client recv failed");
        assert_eq!(client_recv, server_msg);
    }

    println!("✓ Alternating communication test passed!");
}

#[test]
fn test_stress_test() {
    println!("\n=== TEST: Stress Test (100,000 messages) ===");

    let addr = get_test_addr(9);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 9, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const STRESS_COUNT: u32 = 100_000;
    let start = std::time::Instant::now();

    for i in 0..STRESS_COUNT {
        let msg = TestMessage {
            id: i,
            data: format!(
                "Stress test message #{} with some extra data to make it more realistic",
                i
            ),
        };
        client.send(msg).expect("Failed in stress test send");
        let received = server.recv().expect("Failed in stress test recv");
        assert_eq!(received.id, i);
    }

    let duration = start.elapsed();
    println!("Processed {} messages in {:?}", STRESS_COUNT, duration);
    println!("Average: {:?} per message", duration / STRESS_COUNT);
    println!("✓ Stress test passed!");
}

// Removed: Auto-reconnection tests are no longer relevant with high idle timeout

#[test]
fn test_latency_benchmark() {
    println!("\n=== TEST: Latency Benchmark ===");

    let addr = get_test_addr(10);
    let server = QuicServer::<PingMessage>::listen(TEST_PORT_BASE + 10, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<PingMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const BENCHMARK_COUNT: u32 = 1000;
    println!("Measuring round-trip latency for {} messages...\n", BENCHMARK_COUNT);

    let mut latencies = Vec::with_capacity(BENCHMARK_COUNT as usize);
    
    for i in 0..BENCHMARK_COUNT {
        let start = std::time::Instant::now();
        
        // Client sends message
        let msg = PingMessage { timestamp: i as u64 };
        client.send(msg).expect("Failed to send");
        
        // Server echoes back
        let received = server.recv().expect("Failed to receive");
        server.send(received).expect("Failed to send back");
        
        // Client receives response
        let response = client.recv().expect("Failed to receive response");
        
        let rtt = start.elapsed();
        latencies.push(rtt);
        
        assert_eq!(response.timestamp, i as u64);
        
        if (i + 1) % 100 == 0 {
            println!("Progress: {}/{} messages", i + 1, BENCHMARK_COUNT);
        }
    }

    // Calculate statistics
    let total: Duration = latencies.iter().sum();
    let avg = total / (BENCHMARK_COUNT as u32);
    
    let mut sorted_latencies = latencies.clone();
    sorted_latencies.sort();
    
    let min = sorted_latencies.first().unwrap();
    let max = sorted_latencies.last().unwrap();
    let p50 = sorted_latencies[sorted_latencies.len() / 2];
    let p95 = sorted_latencies[(sorted_latencies.len() * 95) / 100];
    let p99 = sorted_latencies[(sorted_latencies.len() * 99) / 100];

    println!("\n=== Latency Statistics ===");
    println!("Messages:     {}", BENCHMARK_COUNT);
    println!("Average RTT:  {:?}", avg);
    println!("Min RTT:      {:?}", min);
    println!("Max RTT:      {:?}", max);
    println!("P50 (median): {:?}", p50);
    println!("P95:          {:?}", p95);
    println!("P99:          {:?}", p99);
    println!("\n✓ Latency benchmark complete!");
}


// Removed: Multiple reconnection cycles test - no longer relevant with high idle timeout

#[test]
fn test_keep_alive_messages() {
    println!("\n=== TEST: Keep-Alive Messages ===");

    let addr = get_test_addr(13);
    let keep_alive_interval = Duration::from_millis(250);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 13, keep_alive_interval)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    // Configure server keep-alive message
    let custom_keep_alive = b"custom_server_v1";
    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    // Send a regular message first
    println!("--- Phase 1: Initial Communication ---");
    let msg = TestMessage {
        id: 1,
        data: "Test message".to_string(),
    };
    client.send(msg.clone()).expect("Failed to send");
    let received = server.recv().expect("Failed to receive");
    assert_eq!(received, msg);
    println!("✓ Initial message sent successfully");

    // Set the server keep-alive message and wait for exchange to occur
    println!("\n--- Phase 2: Waiting for Keep-Alive ---");
    server.set_keep_alive_msg(custom_keep_alive);
    thread::sleep(Duration::from_millis(1500)); // Wait for at least 2 keep-alive cycles

    // Check that client received keep-alive message from server
    let client_received_ka = client.get_last_keep_alive_msg();
    assert!(client_received_ka.is_some(), "Client should have received keep-alive from server");
    assert_eq!(client_received_ka.unwrap(), custom_keep_alive, "Client should receive server's custom keep-alive message");
    println!("✓ Client received server's keep-alive message");

    // Check that keep-alive was received recently
    let server_time_since = server.time_since_last_keep_alive();
    assert!(server_time_since.is_some(), "Server should have keep-alive timestamp");
    assert!(server_time_since.unwrap() < Duration::from_secs(2), "Keep-alive should be recent");
    println!("✓ Server keep-alive timestamp is recent: {:?}", server_time_since.unwrap());

    let client_time_since = client.time_since_last_keep_alive();
    assert!(client_time_since.is_some(), "Client should have keep-alive timestamp");
    assert!(client_time_since.unwrap() < Duration::from_secs(2), "Keep-alive should be recent");
    println!("✓ Client keep-alive timestamp is recent: {:?}", client_time_since.unwrap());

    // Update client's keep-alive message
    println!("\n--- Phase 3: Update Keep-Alive Message ---");
    let new_keep_alive = b"updated_server_v2";
    server.set_keep_alive_msg(new_keep_alive);
    
    // Wait for updated keep-alive to be sent
    thread::sleep(Duration::from_millis(1500));
    
    let updated_ka = client.get_last_keep_alive_msg();
    assert_eq!(updated_ka.unwrap(), new_keep_alive, "Client should receive updated keep-alive message");
    println!("✓ Client received updated keep-alive message");

    // Verify normal communication still works
    println!("\n--- Phase 4: Verify Normal Communication ---");
    let msg2 = TestMessage {
        id: 2,
        data: "After keep-alive test".to_string(),
    };
    client.send(msg2.clone()).expect("Failed to send");
    let received2 = server.recv().expect("Failed to receive");
    assert_eq!(received2, msg2);
    println!("✓ Normal communication still works");

    println!("\n✅ KEEP-ALIVE MESSAGES TEST PASSED!");
}

#[test]
fn test_connection_health_monitoring() {
    println!("\n=== TEST: Connection Health Monitoring ===");

    let addr = get_test_addr(14);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 14, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    // Check initial connection status
    println!("--- Phase 1: Check Initial Connection Status ---");
    assert!(client.is_connected(), "Client should be connected");
    assert!(server.is_connected(), "Server should be connected");
    println!("✓ Both client and server report connected status");

    // Send message to establish communication
    let msg = TestMessage {
        id: 1,
        data: "Test".to_string(),
    };
    client.send(msg.clone()).expect("Failed to send");
    server.recv().expect("Failed to receive");
    println!("✓ Communication established");

    // Wait for keep-alive to be established
    println!("\n--- Phase 2: Monitor Keep-Alive Health ---");
    thread::sleep(Duration::from_millis(1500));

    // Check that keep-alive is working
    let server_ka_time = server.time_since_last_keep_alive();
    assert!(server_ka_time.is_some(), "Server should have keep-alive timestamp");
    println!("✓ Server received keep-alive: {:?} ago", server_ka_time.unwrap());

    let client_ka_time = client.time_since_last_keep_alive();
    assert!(client_ka_time.is_some(), "Client should have keep-alive timestamp");
    println!("✓ Client received keep-alive: {:?} ago", client_ka_time.unwrap());

    // Verify connection is still healthy
    assert!(client.is_connected(), "Client should still be connected");
    assert!(server.is_connected(), "Server should still be connected");
    println!("✓ Connection health confirmed");

    println!("\n✅ CONNECTION HEALTH MONITORING TEST PASSED!");
}

// Removed: Rapid close and reconnect test - no longer relevant with high idle timeout

// Removed: Bidirectional after auto-reconnect test - no longer relevant with high idle timeout

#[test]
fn test_keep_alive_during_load() {
    println!("\n=== TEST: Keep-Alive During High Load ===");

    let addr = get_test_addr(17);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 17, Duration::from_millis(250))
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    println!("--- Phase 1: Send Messages Under Load ---");
    const LOAD_COUNT: u32 = 100;
    
    // Send messages rapidly while keep-alive is also running
    for i in 0..LOAD_COUNT {
        let msg = TestMessage {
            id: i,
            data: format!("Load message {}", i),
        };
        client.send(msg.clone()).expect(&format!("Failed to send load message {}", i));
        let received = server.recv().expect(&format!("Failed to receive load message {}", i));
        assert_eq!(received, msg);

        // Add small delay to allow keep-alive to interleave
        if i % 10 == 0 {
            thread::sleep(Duration::from_millis(100));
        }
    }
    println!("✓ Sent {} messages successfully", LOAD_COUNT);

    // Check that keep-alive is still working
    println!("\n--- Phase 2: Verify Keep-Alive Still Active ---");
    thread::sleep(Duration::from_millis(1000));

    let server_ka = server.time_since_last_keep_alive();
    assert!(server_ka.is_some(), "Server should have keep-alive");
    assert!(server_ka.unwrap() < Duration::from_secs(2), "Keep-alive should be recent");
    println!("✓ Keep-alive still active: {:?} ago", server_ka.unwrap());

    println!("\n✅ KEEP-ALIVE DURING LOAD TEST PASSED!");
}


#[test]
fn test_connect_blocks() {
    println!("\n=== TEST: connect blocks until it can connect ===");
    let addr = get_test_addr(18);


    let client = QuicClient::<TestMessage>::connect(addr)
       .expect("Failed to create client");

    thread::sleep(Duration::from_millis(1000));

    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 18, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");
    thread::sleep(Duration::from_millis(5000));
    assert!(client.is_connected());
    assert!(server.is_connected());
}

#[test]
fn test_reconnect_and_send_with_close() {
    println!("\n=== TEST: Client Reconnect and Send (with close()) ===");

    let addr = get_test_addr(19);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 19, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    // First connection
    println!("\n--- Phase 1: First Connection ---");
    let client1 = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    let msg1 = TestMessage {
        id: 1,
        data: "First client message".to_string(),
    };
    client1.send(msg1.clone()).expect("Failed to send first message");
    let received1 = server.recv().expect("Failed to receive first message");
    assert_eq!(received1, msg1);
    println!("✓ First message received successfully");

    // Force close the client
    println!("\n--- Phase 2: Close First Client ---");
    {
        let mut shared = client1.shared.lock();
        if let Some(conn) = shared.connection.take() {
            conn.close(0u32.into(), b"test close");
            println!("✓ Client connection closed");
        }
    }
    drop(client1);

    // Wait a moment for server to detect
    thread::sleep(Duration::from_millis(500));

    // Second connection (reconnect)
    println!("\n--- Phase 3: Reconnect with New Client ---");
    let client2 = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create reconnected client");

    thread::sleep(Duration::from_millis(500));

    // Try to send from new client
    println!("\n--- Phase 4: Send from Reconnected Client ---");
    let msg2 = TestMessage {
        id: 2,
        data: "Second client message after reconnect".to_string(),
    };
    
    println!("Sending message from reconnected client...");
    client2.send(msg2.clone()).expect("Failed to send second message");
    
    println!("Server attempting to receive...");
    let received2 = server.recv().expect("Failed to receive second message");
    
    assert_eq!(received2, msg2);
    println!("✓ Second message received successfully after reconnect!");

    // Send a few more to make sure it's stable
    println!("\n--- Phase 5: Multiple Messages After Reconnect ---");
    for i in 3..6 {
        let msg = TestMessage {
            id: i,
            data: format!("Message {} after reconnect", i),
        };
        client2.send(msg.clone()).expect(&format!("Failed to send message {}", i));
        let received = server.recv().expect(&format!("Failed to receive message {}", i));
        assert_eq!(received, msg);
    }
    println!("✓ Multiple messages work after reconnect");

    println!("\n✅ RECONNECT AND SEND TEST PASSED!");
}

#[test]
fn test_reconnect_after_abrupt_disconnect() {
    println!("\n=== TEST: Client Reconnect After Abrupt Disconnect (simulating Ctrl+C) ===");

    let addr = get_test_addr(20);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 20, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    // First connection
    println!("\n--- Phase 1: First Connection ---");
    let client1 = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    let msg1 = TestMessage {
        id: 1,
        data: "First client message".to_string(),
    };
    client1.send(msg1.clone()).expect("Failed to send first message");
    let received1 = server.recv().expect("Failed to receive first message");
    assert_eq!(received1, msg1);
    println!("✓ First message received successfully");

    // Abruptly drop the client WITHOUT calling close (simulates Ctrl+C)
    println!("\n--- Phase 2: Abruptly Drop Client (simulating Ctrl+C) ---");
    drop(client1);
    println!("✓ Client dropped without close frame");

    // Server doesn't know yet that client is gone
    // Wait just a tiny bit (not enough for timeout)
    thread::sleep(Duration::from_millis(100));

    // Second connection (reconnect) happens BEFORE server detects the first is dead
    println!("\n--- Phase 3: Reconnect with New Client (before server detects disconnect) ---");
    let client2 = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create reconnected client");

    thread::sleep(Duration::from_millis(500));

    // Try to send from new client
    println!("\n--- Phase 4: Send from Reconnected Client ---");
    let msg2 = TestMessage {
        id: 2,
        data: "Second client message after abrupt reconnect".to_string(),
    };
    
    println!("Sending message from reconnected client...");
    client2.send(msg2.clone()).expect("Failed to send second message");
    
    println!("Server attempting to receive...");
    let received2 = server.recv().expect("Failed to receive second message");
    
    assert_eq!(received2, msg2);
    println!("✓ Second message received successfully after abrupt reconnect!");

    // Send a few more to make sure it's stable
    println!("\n--- Phase 5: Multiple Messages After Abrupt Reconnect ---");
    for i in 3..6 {
        let msg = TestMessage {
            id: i,
            data: format!("Message {} after abrupt reconnect", i),
        };
        client2.send(msg.clone()).expect(&format!("Failed to send message {}", i));
        let received = server.recv().expect(&format!("Failed to receive message {}", i));
        assert_eq!(received, msg);
    }
    println!("✓ Multiple messages work after abrupt reconnect");

    println!("\n✅ ABRUPT RECONNECT TEST PASSED!");
}
