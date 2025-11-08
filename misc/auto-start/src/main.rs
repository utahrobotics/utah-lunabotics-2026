use std::io::{BufRead, BufReader, Cursor, Write};
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread::spawn;

fn home_page(port: u16) -> tiny_http::Response<Cursor<Vec<u8>>> {
    tiny_http::Response::from_string(format!(
        "
        <!DOCTYPE html>
        <html>
        <head>
            <title>Lunabot Control</title>
        </head>
        <body>
            <h1>Lunabot Control Panel</h1>
            <p>Status: <span id=\"statusText\">Ready</span></p>
            <button id=\"startBtn\" onclick=\"startLunabot()\">Start Lunabot</button>
            <button id=\"killBtn\" onclick=\"killLunabot()\">Kill Lunabot</button>
            <button id=\"clearBtn\" onclick=\"clearLogs()\">Clear Logs</button>
            <hr>
            <pre id=\"output\"></pre>
            <script type=\"text/javascript\">
                var socket = null;
                var output = document.getElementById('output');
                var statusText = document.getElementById('statusText');
                var startBtn = document.getElementById('startBtn');
                var clearBtn = document.getElementById('clearBtn');
                var killBtn = document.getElementById('killBtn');
                
                function setStatus(text) {{
                    statusText.textContent = text;
                }}
                
                function addOutput(text) {{
                    output.textContent += text + '\\n';
                }}
                
                function startLunabot() {{
                    if (socket && socket.readyState === WebSocket.OPEN) {{
                        addOutput('[Warning] Lunabot is already running');
                        return;
                    }}
                    
                    addOutput('[INFO] Starting Lunabot...');
                    setStatus('Connecting...');
                    startBtn.disabled = true;
                    
                    socket = new WebSocket(\"ws://localhost:{}/start\", \"make-prod\");
                    
                    socket.onopen = function() {{
                        console.log('Connected to stream');
                        setStatus('Running');
                        addOutput('[INFO] Connected to Lunabot process');
                    }};
                    
                    socket.onmessage = function(event) {{
                        addOutput(event.data);
                    }};
                    
                    socket.onerror = function(error) {{
                        console.error('WebSocket error:', error);
                        setStatus('Error');
                        addOutput('[ERROR] Connection error');
                        startBtn.disabled = false;
                    }};
                    
                    socket.onclose = function() {{
                        console.log('Connection closed');
                        setStatus('Stopped');
                        addOutput('[INFO] Lunabot process ended');
                        startBtn.disabled = false;
                        socket = null;
                    }};
                }}
                
                function clearLogs() {{
                    clearBtn.disabled = true;
                    addOutput('[INFO] Clearing logs...');
                    
                    fetch('/clear-logs')
                        .then(response => response.text())
                        .then(data => {{
                            addOutput('[INFO] ' + data);
                            clearBtn.disabled = false;
                        }})
                        .catch(error => {{
                            addOutput('[ERROR] Failed to clear logs: ' + error);
                            clearBtn.disabled = false;
                        }});
                }}
                
                function killLunabot() {{
                    killBtn.disabled = true;
                    addOutput('[INFO] Killing Lunabot...');
                    
                    fetch('/kill')
                        .then(response => response.text())
                        .then(data => {{
                            addOutput('[INFO] ' + data);
                            killBtn.disabled = false;
                        }})
                        .catch(error => {{
                            addOutput('[ERROR] Failed to kill Lunabot: ' + error);
                            killBtn.disabled = false;
                        }});
                }}
            </script>
        </body>
        </html>
    ",
        port
    ))
    .with_header(
        "Content-type: text/html"
            .parse::<tiny_http::Header>()
            .unwrap(),
    )
}

fn convert_key(input: &str) -> String {
    use base64::{Engine as _, engine::general_purpose};
    use sha1::Digest;
    use sha1::Sha1;

    let mut input = input.to_string().into_bytes();
    let mut bytes = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
        .to_string()
        .into_bytes();
    input.append(&mut bytes);

    let mut sha1 = Sha1::new();
    sha1.update(&input);

    general_purpose::STANDARD.encode(sha1.finalize())
}

fn encode_websocket_frame(message: &str) -> Vec<u8> {
    let payload = message.as_bytes();
    let payload_len = payload.len();

    let mut frame = Vec::new();

    frame.push(0x81);

    if payload_len < 126 {
        frame.push(payload_len as u8);
    } else if payload_len < 65536 {
        frame.push(126);
        frame.push((payload_len >> 8) as u8);
        frame.push((payload_len & 0xFF) as u8);
    } else {
        frame.push(127);
        for i in (0..8).rev() {
            frame.push(((payload_len >> (i * 8)) & 0xFF) as u8);
        }
    }

    frame.extend_from_slice(payload);

    frame
}

fn main() {
    let server = tiny_http::Server::http("0.0.0.0:8080").unwrap();
    let port = server.server_addr().to_ip().unwrap().port();

    println!("Server started on port {}", port);

    for request in server.incoming_requests() {
        spawn(move || {
            let path = request.url().to_string();

            if path == "/clear-logs" {
                println!("Handling clear-logs request");
                let output = Command::new("make").arg("clear-logs").output();

                let response = match output {
                    Ok(output) if output.status.success() => {
                        tiny_http::Response::from_string("Logs cleared successfully").with_header(
                            "Content-Type: text/plain"
                                .parse::<tiny_http::Header>()
                                .unwrap(),
                        )
                    }
                    Ok(output) => {
                        let error = String::from_utf8_lossy(&output.stderr);
                        tiny_http::Response::from_string(format!("Failed to clear logs: {}", error))
                            .with_status_code(500)
                            .with_header(
                                "Content-Type: text/plain"
                                    .parse::<tiny_http::Header>()
                                    .unwrap(),
                            )
                    }
                    Err(e) => tiny_http::Response::from_string(format!(
                        "Error executing make clear-logs: {}",
                        e
                    ))
                    .with_status_code(500)
                    .with_header(
                        "Content-Type: text/plain"
                            .parse::<tiny_http::Header>()
                            .unwrap(),
                    ),
                };

                request.respond(response).ok();
                return;
            }

            if path == "/kill" {
                println!("Handling kill request");
                let output = Command::new("make").arg("kill").output();

                let response = match output {
                    Ok(output) if output.status.success() => {
                        tiny_http::Response::from_string("Lunabot killed successfully").with_header(
                            "Content-Type: text/plain"
                                .parse::<tiny_http::Header>()
                                .unwrap(),
                        )
                    }
                    Ok(output) => {
                        let error = String::from_utf8_lossy(&output.stderr);
                        tiny_http::Response::from_string(format!(
                            "Failed to kill Lunabot: {}",
                            error
                        ))
                        .with_status_code(500)
                        .with_header(
                            "Content-Type: text/plain"
                                .parse::<tiny_http::Header>()
                                .unwrap(),
                        )
                    }
                    Err(e) => tiny_http::Response::from_string(format!(
                        "Error executing make kill: {}",
                        e
                    ))
                    .with_status_code(500)
                    .with_header(
                        "Content-Type: text/plain"
                            .parse::<tiny_http::Header>()
                            .unwrap(),
                    ),
                };

                request.respond(response).ok();
                return;
            }

            let is_websocket = request
                .headers()
                .iter()
                .find(|h| h.field.equiv(&"Upgrade"))
                .and_then(|hdr| {
                    if hdr.value == "websocket" {
                        Some(hdr)
                    } else {
                        None
                    }
                })
                .is_some();

            if !is_websocket {
                request.respond(home_page(port)).ok();
                return;
            }

            if path != "/start" {
                let response = tiny_http::Response::new_empty(tiny_http::StatusCode(404));
                request.respond(response).ok();
                return;
            }

            let key = match request
                .headers()
                .iter()
                .find(|h| h.field.equiv(&"Sec-WebSocket-Key"))
                .map(|h| h.value.clone())
            {
                None => {
                    let response = tiny_http::Response::new_empty(tiny_http::StatusCode(400));
                    request.respond(response).ok();
                    return;
                }
                Some(k) => k,
            };

            let response = tiny_http::Response::new_empty(tiny_http::StatusCode(101))
                .with_header("Upgrade: websocket".parse::<tiny_http::Header>().unwrap())
                .with_header("Connection: Upgrade".parse::<tiny_http::Header>().unwrap())
                .with_header(
                    "Sec-WebSocket-Protocol: make-prod"
                        .parse::<tiny_http::Header>()
                        .unwrap(),
                )
                .with_header(
                    format!("Sec-WebSocket-Accept: {}", convert_key(key.as_str()))
                        .parse::<tiny_http::Header>()
                        .unwrap(),
                );

            let mut stream = request.upgrade("websocket", response);

            println!("WebSocket connected, starting make prod...");

            let mut child = match Command::new("make")
                .arg("prod")
                .stdout(Stdio::piped())
                .stderr(Stdio::piped())
                .spawn()
            {
                Ok(child) => child,
                Err(e) => {
                    eprintln!("Failed to spawn make prod: {}", e);
                    let msg = format!("Error: Failed to start make prod: {}", e);
                    let frame = encode_websocket_frame(&msg);
                    stream.write_all(&frame).ok();
                    return;
                }
            };

            let stdout = child.stdout.take().unwrap();
            let stderr = child.stderr.take().unwrap();

            let stream_clone = Arc::new(Mutex::new(stream));
            let stream_for_stdout = Arc::clone(&stream_clone);
            let stream_for_stderr = Arc::clone(&stream_clone);

            let stdout_thread = spawn(move || {
                let reader = BufReader::new(stdout);
                for line in reader.lines() {
                    if let Ok(line) = line {
                        let frame = encode_websocket_frame(&line);
                        if let Ok(mut s) = stream_for_stdout.lock() {
                            if s.write_all(&frame).is_err() {
                                break;
                            }
                            s.flush().ok();
                        }
                    }
                }
            });

            let stderr_thread = spawn(move || {
                let reader = BufReader::new(stderr);
                for line in reader.lines() {
                    if let Ok(line) = line {
                        let formatted = format!("[STDERR] {}", line);
                        let frame = encode_websocket_frame(&formatted);
                        if let Ok(mut s) = stream_for_stderr.lock() {
                            if s.write_all(&frame).is_err() {
                                break;
                            }
                            s.flush().ok();
                        }
                    }
                }
            });

            stdout_thread.join().ok();
            stderr_thread.join().ok();

            match child.wait() {
                Ok(status) => {
                    let msg = format!("\n[Process exited with status: {}]", status);
                    let frame = encode_websocket_frame(&msg);
                    if let Ok(mut s) = stream_clone.lock() {
                        s.write_all(&frame).ok();
                    }
                }
                Err(e) => {
                    eprintln!("Error waiting for process: {}", e);
                }
            }

            println!("Make prod completed, closing connection");
        });
    }
}
