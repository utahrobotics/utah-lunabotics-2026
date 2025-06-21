// Helper function to get system info
pub fn get_system_info() -> String {
    format!(
        "   -> Copper Cursive Console Monitor\n\nSystem: {}\nArchitecture: {}\nCPU Count: {}\n\nPress 1-4 to switch between screens\nPress 'q' or ESC to quit",
        std::env::consts::OS,
        std::env::consts::ARCH,
        std::thread::available_parallelism().map(|n| n.get()).unwrap_or(1)
    )
} 