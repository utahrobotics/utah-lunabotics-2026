# Bazel build configuration
BAZEL_BUILD_FLAGS = --sandbox_debug --verbose_failures --spawn_strategy=standalone
RUST_TOOLCHAIN_VERSION = 1.81.0
UNILIDAR_DIR = unilidar_iceoryx_publisher

# Build the unilidar_publisher with Bazel and run the main cargo project
prod:
	cd $(UNILIDAR_DIR) && RULES_RUST_TOOLCHAIN_VERSION=$(RUST_TOOLCHAIN_VERSION) bazel build //:unilidar_publisher $(BAZEL_BUILD_FLAGS)
	cd lunabot-cu && cargo run --release

# Build the unilidar_publisher with Bazel and run the main cargo project in debug mode
debug:
	cd $(UNILIDAR_DIR) && RULES_RUST_TOOLCHAIN_VERSION=$(RUST_TOOLCHAIN_VERSION) bazel build //:unilidar_publisher $(BAZEL_BUILD_FLAGS)
	cd lunabot-cu && cargo run

# Sync Bazel dependencies (useful to run before first build or when dependencies change)
sync:
	cd $(UNILIDAR_DIR) && CARGO_BAZEL_REPIN=1 bazel sync --only=crate_index

# Clean build and sync, then build everything
clean-build: clean sync
	cd $(UNILIDAR_DIR) && RULES_RUST_TOOLCHAIN_VERSION=$(RUST_TOOLCHAIN_VERSION) bazel build //:unilidar_publisher $(BAZEL_BUILD_FLAGS)

# Just build the unilidar_publisher without running anything
build-publisher:
	cd $(UNILIDAR_DIR) && RULES_RUST_TOOLCHAIN_VERSION=$(RUST_TOOLCHAIN_VERSION) bazel build //:unilidar_publisher $(BAZEL_BUILD_FLAGS)

# Run camera discovery
discover-cameras:
	cd camera-discovery && cargo run

# Check the main cargo project
check:
	cd lunabot-cu && cargo check

# Clean Bazel build artifacts
clean:
	cd $(UNILIDAR_DIR) && bazel clean

# Kill all processes
kill:
	killall realsense unilidar_publisher lunabot-ai2

# Help target to show available commands
help:
	@echo "Available targets:"
	@echo "  prod              - Build unilidar_publisher with Bazel and run lunabot-cu in release mode"
	@echo "  debug             - Build unilidar_publisher with Bazel and run lunabot-cu in debug mode"
	@echo "  sync              - Sync Bazel dependencies (run this if you update dependencies)"
	@echo "  clean-build       - Clean, sync, and rebuild everything"
	@echo "  build-publisher   - Just build the unilidar_publisher without running anything"
	@echo "  discover-cameras  - Run camera discovery tool"
	@echo "  check             - Run cargo check on lunabot-cu"
	@echo "  clean             - Clean Bazel build artifacts"
	@echo "  kill              - Kill any lunabot sub processes that may still be running"
	@echo "  help              - Show this help message"

.PHONY: prod debug sync clean-build build-publisher discover-cameras check clean help
