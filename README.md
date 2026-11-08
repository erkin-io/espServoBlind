# espServoBlind

Zigbee firmware for the ESP32-C6 that moves a hobby servo to a 0–100 % position. It shows up as a standard **Window Covering** in Zigbee2MQTT and Home Assistant, so a slider, Open/Close buttons and automations work out of the box.

It was built to close a physical privacy cover over an indoor camera lens, but it works for anything a servo can push: a small blind, a vent flap, a pointer.

> Parts of this code and documentation were written with the help of AI. Review it and test it on your own hardware before you rely on it.

## Features

- **Standard Zigbee cover** (Window Covering cluster `0x0102`), no hub-specific code needed.
- **Zigbee router**: mains-powered, strengthens your mesh and rejoins its saved network by itself after a power cut or a coordinator restart.
- **Calibration over the air**: servo end points, speed and idle cut-off can be changed from Zigbee2MQTT. Values are stored in flash and sent back, so Zigbee2MQTT always shows what the device really uses.
- **Remembers its position** across restarts and never twitches at boot.
- **Always re-drives**: every command moves the servo, even to the same position, so a cover pushed by hand is put back.
- **Idle cut-off**: the PWM signal stops after the move, so cheap servos don't buzz or heat up.
- **Over-the-air updates** from Zigbee2MQTT, with automatic rollback if a new version fails.
- **Diagnostics**: reset reason, boot count and chip temperature.

## Hardware

| Part | Notes |
|---|---|
| Board | ESP32-C6 SuperMini (others work if you change the pins) |
| Servo signal | GPIO 2 |
| Status LED | GPIO 15 (the RGB LED on GPIO 8 is not used) |
| Button | BOOT button, GPIO 9 |

**Power.** Feed the board and the servo from a steady 5 V supply. A servo starting to move draws a short current spike; if the board resets when the servo moves, add a buffer capacitor near the servo (several hundred µF or more).

> **Don't use a LiPo cell as a buffer** on the board's battery pads. Kept on charge at 4.2 V all the time, next to a warm board, a cell swelled within months in the original build. A supercapacitor (with a series resistor and a diode) or a large electrolytic capacitor is safer.

**Antenna.** The SuperMini's chip antenna is weak. Keep the board away from metal and make sure another Zigbee router is in good range.

## Quick start

### 1. Flash

Prebuilt 1.1.0 binaries are in [`bin/`](bin/). Flash them with [esptool](https://github.com/espressif/esptool) (replace `PORT`, e.g. `COM3` or `/dev/ttyACM0`):

```bash
esptool.py --chip esp32c6 -p PORT -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x0 bin/bootloader.bin 0x8000 bin/partition-table.bin 0xd000 bin/ota_data_initial.bin 0x10000 bin/erkinIO_esp32_servo_blind.bin
```

Or use Espressif's [Flash Download Tool](https://www.espressif.com/en/support/download/other-tools) with the same files and addresses. To build it yourself, see [Building from source](#building-from-source).

### 2. Add the converter to Zigbee2MQTT

The cover works without it, but the converter adds calibration, diagnostics, OTA support, and removes the tilt slider the device doesn't have.

- **In the UI:** Settings › Dev console › External converters, add a file named `espServoBlind.js` with the contents of [`espServoBlind.js`](espServoBlind.js).
- **Or by file:** copy `espServoBlind.js` into Zigbee2MQTT's `data/external_converters/` folder.

Restart Zigbee2MQTT.

### 3. Pair

Turn on **Permit join** in Zigbee2MQTT. A freshly flashed device searches for a network by itself (LED blinking slowly); press the button once to search right away. Once it has joined, the LED turns off.

### 4. Calibrate

On the device's page in Zigbee2MQTT, under **Settings (specific)**:

| Setting | Range | Default | What it does |
|---|---|---|---|
| `servo_min_us` | 500–2500 | 1000 | Pulse width at 0 % |
| `servo_max_us` | 500–2500 | 2000 | Pulse width at 100 % |
| `move_speed_ms` | 0–100 | 20 | Milliseconds per 1 % of travel; 0 = servo's full speed |
| `idle_timeout_ms` | 0–10000 | 1000 | Stop the PWM signal this long after a move; 0 = never |

Changing a setting never moves the servo; the next command uses it. Start with a narrow range and widen it step by step. **Both 0 % and 100 % must be mechanically safe**, because Home Assistant's Open and Close buttons drive straight to them. A servo held against an end stop draws its stall current and gets hot.

## What Zigbee2MQTT shows

| Value | Meaning |
|---|---|
| `position` / `state` | Cover position 0–100 % |
| `identify_action` | Blinks the LED so you can find the device |
| `servo_min_us`, `servo_max_us`, `move_speed_ms`, `idle_timeout_ms` | Calibration, see above |
| `last_reset_reason` | Why it last started: `power_on`, `software`, `brownout`, `crash`, `watchdog`, … |
| `boot_count` | Number of starts since it was flashed |
| `device_temperature` | Chip temperature, every 5 minutes |

## Button and LED

| Button | Action |
|---|---|
| Short press (under 2 s) | **Connect**: not on a network → search now. On a network → announce itself, report its state and check that the coordinator answers. It moves to another network only if the coordinator doesn't answer **and** another network is open for joining. |
| Hold 2–10 s | Ignored, so a long press by accident does nothing |
| Hold 10 s or more, then release | Zigbee factory reset: leaves the network. Calibration and position are kept. |

The button never moves the servo.

| LED | Meaning |
|---|---|
| Off | Joined, all normal |
| Slow blink | Searching for a network |
| Fast blink | Identify, or a Connect check is running |
| One long flash | Connect: the coordinator answered |
| Three slow flashes | Connect: the coordinator didn't answer; the device stays on its network |
| Fast burst | Connect: moving to another open network |
| Solid | Button held long enough for a factory reset; release to reset |

## OTA updates

Version 1.1.0 and later can be updated over the air. A device on 1.0.0 needs one USB flash first, which also installs the rollback-capable bootloader (the bootloader itself can't be updated over the air).

**Send an update from Zigbee2MQTT.** Publish to `zigbee2mqtt/bridge/request/device/ota_update/update`:

```json
{"id": "<device name or IEEE>", "hex": {"data": "<the .ota file as hex>", "file_name": "espServoBlind-<version>.ota"}}
```

Alternatively, put the `.ota` file and the index file made by `tools/make_ota.py` in Zigbee2MQTT's data folder, set `ota: zigbee_ota_override_index_location: espServoBlind-<version>.index.json` in its configuration, restart it, and use **Check for update** and **Update** on the device's OTA tab.

**What to expect**
- The download takes **about an hour** at Zigbee2MQTT's default settings. The cover keeps working meanwhile.
- The device then restarts into the new version and reports back within seconds.
- The new version is kept only after it has been back on the network for one minute. If it crashes or restarts before that, or hasn't joined within 10 minutes, the device goes back to the previous version by itself.
- If the download is interrupted (for example Zigbee2MQTT restarts), nothing changes on the device; just start the update again.

> Images are checked for completeness, checksum, SHA-256 and project name, but they are **not cryptographically signed**. Whoever controls your Zigbee coordinator could install a matching image. Signing would need Secure Boot, which permanently burns keys into the chip.

## Building from source

Requirements:
- **ESP-IDF:** use the exact version recommended by the [espressif/esp-zigbee-sdk](https://github.com/espressif/esp-zigbee-sdk) repository. The Zigbee libraries are only supported on specific ESP-IDF versions; another version may not build or may misbehave. This project is tested with **v5.3.2**.
- **esp-zigbee-lib / esp-zboss-lib:** 1.6.x, tested with **1.6.8**. The component manager fetches them automatically.
- **Hardware:** tested on the ESP32-C6.

```bash
idf.py set-target esp32c6
idf.py build flash monitor
```

`idf.py flash` keeps calibration, position and the Zigbee network. Run `idf.py erase-flash` first only if you want a completely fresh device.

**Releasing a new version**
1. Raise `FW_FILE_VERSION`, `FW_VERSION_STR` and `FW_APP_VERSION` in [`main/esp_zb_customized_server.c`](main/esp_zb_customized_server.c), and `PROJECT_VER` in [`CMakeLists.txt`](CMakeLists.txt). Zigbee2MQTT only offers a file version higher than the installed one.
2. `idf.py build`
3. `python tools/make_ota.py` writes `build/espServoBlind-<version>.ota` (manufacturer `0x131B`, image type `0x5342`) and a matching index file.

## Upgrading from older versions

- **From 1.0.0:** 1.1.0 changed the Zigbee role from end device to router. Flash all four files over USB without erasing (calibration and position stay); this also installs the rollback bootloader, so every later update can go over the air. On first start the firmware clears only its old network data. Turn on Permit join; it rejoins within 5 minutes, or at once if you press the button. Don't remove the device in Zigbee2MQTT first, so names and Home Assistant entities stay. Afterwards click **Interview** and **Reconfigure** on the device.
- **Any version:** update `espServoBlind.js` in Zigbee2MQTT as well.

## Repository layout

| Path | Contents |
|---|---|
| [`main/`](main/) | Firmware source |
| [`espServoBlind.js`](espServoBlind.js) | Zigbee2MQTT external converter |
| [`tools/make_ota.py`](tools/make_ota.py) | Wraps a build into a Zigbee OTA file (no dependencies) |
| [`bin/`](bin/) | Prebuilt 1.1.0 binaries |
| [`partitions.csv`](partitions.csv) | Two app slots for OTA, plus Zigbee storage |

## Changelog

**1.1.0**
- Router instead of end device; rejoins its saved network by itself.
- One-press Connect button; 10 s hold for factory reset.
- Every command re-drives the servo; position is remembered without moving at boot.
- Time-based motion so the speed setting is accurate.
- Sends the saved calibration when it joins and right after every change, so Zigbee2MQTT shows the real values.
- Zigbee OTA updates with automatic rollback: a new version is kept only after a minute on the network.
- Diagnostics: reset reason, boot count and chip temperature.

**1.0.0**
- First release.

## License

[Apache License 2.0](LICENSE), the same as Espressif's SDK.
