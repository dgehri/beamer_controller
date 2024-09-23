#!/usr/bin/python3

import soco
import threading
import time
from evdev import InputDevice, categorize, ecodes
import time

# Sonos Speaker Room Name
SONOS_ROOM_NAME = "Salon"

# IR Scancodes for Volume Up, Volume Down, and Mute from Apple and LG remotes
VOLUME_UP_CODES = {0x402, 0x87EE000B}
VOLUME_DOWN_CODES = {0x403, 0x87EE000D}
MUTE_CODES = {0x409}

# Initialize Sonos
sonos_device = soco.discovery.by_name(SONOS_ROOM_NAME)
group = sonos_device.group

# Target group volume level (start with the current volume)
target_volume = group.volume
target_muted = group.mute

# Lock to manage volume updates
volume_lock = threading.Lock()

# Device file for the IR receiver (replace with your actual device path)
IR_DEVICE_PATH = "/dev/input/event0"


def handle_volume():
    global target_volume, target_muted

    last_volume = None
    last_muted = None
    last_update_time = time.time()
    delay_seconds = 5  # The delay window for accepting external volume changes

    while True:
        with volume_lock:
            try:
                current_volume = group.volume
                current_muted = group.mute

                if time.time() - last_update_time >= delay_seconds:

                    if last_volume is not None and current_volume != last_volume:
                        target_volume = current_volume
                        print(
                            f"Volume changed outside of script: Target group volume set to {target_volume}"
                        )

                    if last_muted is not None and current_muted != last_muted:
                        target_muted = current_muted
                        print(
                            f"Mute state changed outside of script: Mute state is now {'ON' if target_muted else 'OFF'}"
                        )

                # Handle mute state changes
                if current_muted != target_muted:
                    print(
                        f"Adjusting mute state to {'MUTED' if target_muted else 'UNMUTED'}"
                    )
                    group.mute = target_muted
                    last_update_time = time.time()  # Update the last update timestamp

                # Set the volume if it doesn't match the target
                if target_volume != current_volume and not target_muted:
                    print(f"Adjusting volume from {current_volume} to {target_volume}")
                    group.volume = target_volume
                    last_update_time = time.time()  # Update the last update timestamp

            except Exception as e:
                print(f"Error adjusting volume: {e}")
                time.sleep(10)

        last_volume = current_volume
        last_muted = current_muted

        # Adjust Sonos group volume and mute state every second to avoid rate limits
        time.sleep(1)


def read_ir_input():
    global target_volume, target_muted  # Declare the global variables
    ir_device = InputDevice(IR_DEVICE_PATH)
    print(f"Listening for IR signals on {IR_DEVICE_PATH}")
    
    # Mute debounce timer
    mute_debounce_secs = 0.5
    last_mute_time = time.time()

    for event in ir_device.read_loop():
        if event.type == ecodes.EV_MSC and event.code == ecodes.MSC_SCAN:
            scancode = event.value

            with volume_lock:
                current_volume = target_volume
                if scancode in VOLUME_UP_CODES:
                    target_volume = min(100, current_volume + 1)
                    print(
                        f"Volume up pressed: Target group volume set to {target_volume}"
                    )
                elif scancode in VOLUME_DOWN_CODES:
                    target_volume = max(0, current_volume - 1)
                    print(
                        f"Volume down pressed: Target group volume set to {target_volume}"
                    )
                elif scancode in MUTE_CODES:
                    if time.time() - last_mute_time > mute_debounce_secs:
                        last_mute_time = time.time()
                        target_muted = not target_muted
                        print(
                            f"Mute toggled: Mute state is now {'ON' if target_muted else 'OFF'}"
                        )


if __name__ == "__main__":
    # Start a thread to handle volume changes
    volume_thread = threading.Thread(target=handle_volume, daemon=True)
    volume_thread.start()

    # Read IR input in the main thread
    read_ir_input()
