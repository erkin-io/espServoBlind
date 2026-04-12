# espServoBlind

A highly versatile, native Zigbee firmware for the ESP32-C6 that drives a PWM servo motor. It advertises itself as a standard Window Covering endpoint, making it natively compatible with Zigbee2MQTT and Home Assistant.

Designed originally for a physical camera privacy cover, this codebase is completely agnostic and can be used all applications that relies on a 0-100% position command mapped to a PWM servo pulse.

⚠️ **Notice:** Portions of this codebase and documentation were generated iteratively with the assistance of AI. Please review the codebase and test carefully before full production deployments.

## Features
* **Native Zigbee HA Endpoint:** Exposes a standard Window Covering cluster (0x0102).
* **Status LED:** Indicates network joining/steering by slow blinking, then goes off once joined to eliminate light pollution. Find your device easily by triggering the Identify cluster, which blinks fast the LED.
* **Boot Button:** Short press re-triggers network steering (re-join); long press (≥ 3 s) performs a Zigbee factory reset.
* **Zigbee2MQTT External Converter:** Includes a purpose-built `espServoBlind.js` converter that cleanly maps the 0-100% slider, safely removes the unsupported "Tilt" attribute, and pushes all tuning calibration straight to the Settings tab.
* **On-the-fly Tuning (Custom Cluster `0xFF00`):**
  * **Servo Min/Max (μs):** Dynamically adjust the exact microsecond boundaries of your physical servo over the air.
  * **Move Speed (ms):** Add artificial delays per 1% of movement to softly actuate mechanisms.
  * **Idle PWM Cutoff:** Specify an idle timeout (ms) after which the PWM power disables, preventing cheap servos from "buzzing" or overheating when stationary.
* **Power Resilient:** All dynamic configuration boundaries and network states are backed continuously to Non-Volatile Storage (NVS).
* **OTA Ready:** Core OTA client partition schemes are built-in for future-proofing wireless flashing. -Not tested yet-

## Directory Structure
* `/main/` - Contains the Core C application firmware and CMake definitions.
* `/espServoBlind.js` - The Zigbee2MQTT External Converter.
* `/bin/` - Precompiled, ready-to-flash binaries.
* `/partitions.csv` - Defines the dual-app `ota_0`/`ota_1` layout, plus `zb_storage` FAT space.

## Development Prerequisites
If you wish to build this from source, you must configure your Espressif toolchain to identically match the officially supported Zigbee version boundaries:
1. **ESP-IDF:** Use the exact version advised by the [espressif/esp-zigbee-sdk](https://github.com/espressif/esp-zigbee-sdk) repository (Currently heavily tested against **v5.3.2**).
2. **ESP-Zigbee-SDK:** Currently verified against **v1.6.8**.
3. **Hardware:** Tested on the **ESP32-C6**.

## Flashing
You can flash this firmware by building from source or by using the pre-built binaries.

### Option 1: Building from Source
To easily compile the project, automatically discover your COM port, completely erase the memory (to wipe old stale Zigbee keys/NVS bounds), and flash the device simultaneously, simply run the native ESP-IDF toolchain command:

```bash
idf.py build erase-flash flash
```

### Option 2: Using Pre-built Binaries
If you prefer not to set up the ESP-IDF toolchain, you can flash the pre-compiled binaries located in the `/bin/` directory. You can use the [Espressif Flash Download Tools](https://www.espressif.com/en/support/download/other-tools) or use the `esptool.py` CLI with the following command (replace `(PORT)` with your actual COM port, e.g., `COM3` or `/dev/ttyACM0`):

```bash
esptool.py -p (PORT) -b 460800 --before default_reset --after hard_reset --chip esp32c6 write_flash --flash_mode dio --flash_size detect --flash_freq 80m 0x0 bin/bootloader.bin 0x8000 bin/partition-table.bin 0xd000 bin/ota_data_initial.bin 0x10000 bin/erkinIO_esp32_servo_blind.bin
```

## Zigbee2MQTT Setup
By default, this firmware operates out of the box as a standard Native Window Covering endpoint without any extra configuration.

However, if you want to dial in your precise mechanical tuning limits (Servo Min/Max microseconds, movement speeds, power-saving timeouts), or correctly disable the unusable "Tilt" slider, you should load the provided custom converter:

* Copy the `espServoBlind.js` file into your Zigbee2MQTT `data/` directory.
* Enable the converter in Zigbee2MQTT by editing your `configuration.yaml` for Z2M and adding the converter definition:
   ```yaml
   external_converters:
     - espServoBlind.js
   ```

Or

* You can do this via the UI by navigating to **Settings > Dev Console > External converters** and adding `espServoBlind.js`

Then

* **Restart the Zigbee2MQTT Add-on.**
* Put your ESP32-C6 into pairing mode (it automatically enters Top-Level Commissioning on a fresh boot).
* Once paired, navigate to the device in Z2M. You will see your Position slider and your advanced Servo Calibration limits.

## License
This project is open-sourced under the **Apache License 2.0**, aligns with Espressif's license.
