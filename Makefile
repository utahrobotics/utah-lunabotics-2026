prod:
	cd lunabot-cu && cargo run --release

debug:
	cd lunabot-cu && cargo run

discover-cameras:
	cd camera-discovery && cargo run

check:
	cd lunabot-cu && cargo check
