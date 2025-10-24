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
const DEFAULT_KEEP_ALIVE_MSG: &[u8] = b"test_client";
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

#[test]
fn test_auto_reconnection_after_connection_close() {
    println!("\n=== TEST: Auto-Reconnection After Connection Close ===");

    let addr = get_test_addr(11);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 11, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    // Send initial message to verify connection works
    println!("--- Phase 1: Initial Communication ---");
    let msg1 = TestMessage {
        id: 1,
        data: "Before close".to_string(),
    };
    client.send(msg1.clone()).expect("Failed to send first message");
    let received1 = server.recv().expect("Failed to receive first message");
    assert_eq!(received1, msg1);
    println!("✓ Initial message sent successfully");

    // Force close the client's inner connection
    println!("\n--- Phase 2: Force Close Client Connection ---");
    {
        let mut shared = client.shared.lock();
        if let Some(conn) = shared.connection.take() {
            conn.close(0u32.into(), b"test forced close");
            println!("✓ Client connection closed");
        }
    }

    // Give the keep-alive task time to detect the closure and attempt reconnect
    println!("Waiting for auto-reconnection...");
    thread::sleep(Duration::from_secs(7));

    // Try to send a message - the client should have auto-reconnected
    println!("\n--- Phase 3: Communication After Auto-Reconnect ---");
    let msg2 = TestMessage {
        id: 2,
        data: "After auto-reconnect".to_string(),
    };
    
    client.send(msg2.clone()).expect("Failed to send after reconnect");
    let received2 = server.recv().expect("Failed to receive after reconnect");
    assert_eq!(received2, msg2);
    println!("✓ Message sent successfully after auto-reconnection!");

    // Verify continued communication works
    println!("\n--- Phase 4: Continued Communication ---");
    for i in 3..8 {
        let msg = TestMessage {
            id: i,
            data: format!("Message {}", i),
        };
        client.send(msg.clone()).expect(&format!("Failed to send message {}", i));
        let received = server.recv().expect(&format!("Failed to receive message {}", i));
        assert_eq!(received, msg);
    }
    println!("✓ Sent 5 more messages successfully!");

    println!("\n✅ AUTO-RECONNECTION TEST PASSED!");
}

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


#[test]
fn test_multiple_reconnection_cycles() {
    println!("\n=== TEST: Multiple Reconnection Cycles ===");

    let addr = get_test_addr(12);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 12, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const NUM_CYCLES: u32 = 5;

    for cycle in 0..NUM_CYCLES {
        println!("\n--- Cycle {}/{} ---", cycle + 1, NUM_CYCLES);
        
        // Send message
        let msg = TestMessage {
            id: cycle,
            data: format!("Before close cycle {}", cycle),
        };
        client.send(msg.clone()).expect("Failed to send");
        let received = server.recv().expect("Failed to receive");
        assert_eq!(received, msg);
        println!("✓ Message sent before close");

        // Force close connection
        {
            let mut shared = client.shared.lock();
            if let Some(conn) = shared.connection.take() {
                conn.close(0u32.into(), b"test cycle");
            }
        }
        println!("✓ Connection closed");

        // Wait for auto-reconnection
        thread::sleep(Duration::from_secs(3));

        // Send message after reconnection
        let msg2 = TestMessage {
            id: cycle + 100,
            data: format!("After reconnect cycle {}", cycle),
        };
        client.send(msg2.clone()).expect("Failed to send after reconnect");
        let received2 = server.recv().expect("Failed to receive after reconnect");
        assert_eq!(received2, msg2);
        println!("✓ Message sent after reconnect");
    }

    println!("\n✅ MULTIPLE RECONNECTION CYCLES TEST PASSED!");
    println!("Successfully completed {} reconnection cycles", NUM_CYCLES);
}

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

    // Force close and check status
    println!("\n--- Phase 3: Check Status After Connection Close ---");
    {
        let mut shared = client.shared.lock();
        if let Some(conn) = shared.connection.take() {
            conn.close(0u32.into(), b"test");
        }
    }

    // Connection status should reflect the closure (after a brief moment)
    thread::sleep(Duration::from_millis(100));
    
    // After closure is detected, is_connected should return false
    // (though auto-reconnection may already be in progress)
    println!("Connection closed, waiting for status update...");
    
    // Wait for auto-reconnection
    thread::sleep(Duration::from_secs(7));
    
    // After auto-reconnection, should be connected again
    assert!(client.is_connected(), "Client should be reconnected");
    println!("✓ Client auto-reconnected and reports connected status");

    println!("\n✅ CONNECTION HEALTH MONITORING TEST PASSED!");
}

#[test]
fn test_rapid_close_and_reconnect() {
    println!("\n=== TEST: Rapid Close and Auto-Reconnect ===");

    let addr = get_test_addr(15);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 15, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    const NUM_RAPID_CYCLES: u32 = 3;

    for i in 0..NUM_RAPID_CYCLES {
        println!("\n--- Rapid Cycle {}/{} ---", i + 1, NUM_RAPID_CYCLES);

        // Send message before close
        let msg1 = TestMessage {
            id: i * 2,
            data: format!("Before rapid close {}", i),
        };
        client.send(msg1.clone()).expect("Failed to send before close");
        let recv1 = server.recv().expect("Failed to receive before close");
        assert_eq!(recv1, msg1);
        println!("✓ Message sent before close");

        // Force close
        {
            let mut shared = client.shared.lock();
            if let Some(conn) = shared.connection.take() {
                conn.close(0u32.into(), b"rapid test");
            }
        }
        println!("✓ Connection force closed");

        // Wait for auto-reconnection (shorter wait)
        println!("Waiting for auto-reconnection...");
        thread::sleep(Duration::from_secs(7));

        // Send message after reconnect
        let msg2 = TestMessage {
            id: i * 2 + 1,
            data: format!("After rapid reconnect {}", i),
        };
        client.send(msg2.clone()).expect("Failed to send after reconnect");
        let recv2 = server.recv().expect("Failed to receive after reconnect");
        assert_eq!(recv2, msg2);
        println!("✓ Message sent after auto-reconnect");
    }

    println!("\n✅ RAPID CLOSE AND AUTO-RECONNECT TEST PASSED!");
    println!("Successfully completed {} rapid reconnection cycles", NUM_RAPID_CYCLES);
}

#[test]
fn test_bidirectional_after_auto_reconnect() {
    println!("\n=== TEST: Bidirectional Communication After Auto-Reconnect ===");

    let addr = get_test_addr(16);
    let server = QuicServer::<TestMessage>::listen(TEST_PORT_BASE + 16, DEFAULT_KEEP_ALIVE_INTERVAL)
        .expect("Failed to create server");

    thread::sleep(Duration::from_millis(100));

    let client = QuicClient::<TestMessage>::connect(addr)
        .expect("Failed to create client");

    thread::sleep(Duration::from_millis(200));

    // Initial bidirectional communication
    println!("--- Phase 1: Initial Bidirectional Communication ---");
    let c2s = TestMessage { id: 1, data: "Client to server".to_string() };
    client.send(c2s.clone()).expect("C2S failed");
    let recv_c2s = server.recv().expect("Server recv failed");
    assert_eq!(recv_c2s, c2s);

    let s2c = TestMessage { id: 2, data: "Server to client".to_string() };
    server.send(s2c.clone()).expect("S2C failed");
    let recv_s2c = client.recv().expect("Client recv failed");
    assert_eq!(recv_s2c, s2c);
    println!("✓ Initial bidirectional communication works");

    // Force close client connection
    println!("\n--- Phase 2: Force Close Connection ---");
    {
        let mut shared = client.shared.lock();
        if let Some(conn) = shared.connection.take() {
            conn.close(0u32.into(), b"test");
        }
    }
    println!("✓ Client connection closed");

    // Wait for auto-reconnection
    println!("Waiting for auto-reconnection...");
    thread::sleep(Duration::from_secs(7));

    // Test bidirectional communication after auto-reconnect
    println!("\n--- Phase 3: Bidirectional Communication After Auto-Reconnect ---");
    
    // Client to Server
    let c2s2 = TestMessage { id: 3, data: "Client to server after reconnect".to_string() };
    client.send(c2s2.clone()).expect("C2S after reconnect failed");
    let recv_c2s2 = server.recv().expect("Server recv after reconnect failed");
    assert_eq!(recv_c2s2, c2s2);
    println!("✓ Client to server works after reconnect");

    // Server to Client
    let s2c2 = TestMessage { id: 4, data: "Server to client after reconnect".to_string() };
    server.send(s2c2.clone()).expect("S2C after reconnect failed");
    let recv_s2c2 = client.recv().expect("Client recv after reconnect failed");
    assert_eq!(recv_s2c2, s2c2);
    println!("✓ Server to client works after reconnect");

    // Multiple exchanges
    println!("\n--- Phase 4: Multiple Bidirectional Exchanges ---");
    for i in 0..5 {
        let c_msg = TestMessage { id: 10 + i, data: format!("C->S exchange {}", i) };
        client.send(c_msg.clone()).expect("Exchange C2S failed");
        let recv = server.recv().expect("Exchange S recv failed");
        assert_eq!(recv, c_msg);

        let s_msg = TestMessage { id: 20 + i, data: format!("S->C exchange {}", i) };
        server.send(s_msg.clone()).expect("Exchange S2C failed");
        let recv = client.recv().expect("Exchange C recv failed");
        assert_eq!(recv, s_msg);
    }
    println!("✓ Multiple bidirectional exchanges successful");

    println!("\n✅ BIDIRECTIONAL AFTER AUTO-RECONNECT TEST PASSED!");
}

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
