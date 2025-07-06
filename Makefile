prod:
	$(MAKE) -C unilidar_iceoryx_publisher unilidar_publisher
	cd lunabot-cu && cargo run --release

debug:
	$(MAKE) -C unilidar_iceoryx_publisher unilidar_publisher
	cd lunabot-cu && cargo run

discover-cameras:
	cd camera-discovery && cargo run

check:
	cd lunabot-cu && cargo check
