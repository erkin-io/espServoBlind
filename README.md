# espServoBlind

Firmware for an ESP32-C6 that drives a hobby servo over Zigbee. It was made to swing a camera away, so that it can't see the room.

## Why

I want a camera at home when nobody is there, and no camera when I know somebody is.

Switching the camera off in software is one way to do that. But no device can be guaranteed, and I can't guarantee myself either: a setting gets changed, a recording stays somewhere. If something isn't needed, I prefer that it isn't possible.

So a servo turns the camera away. This is physical. You can see where the camera points, and you can hear the servo when it moves. Visitors can see it too. I find this more comfortable than a setting on a screen.

The servo controller is its own device, with its own radio. It doesn't depend on the camera, so the two don't fail together by design.

This project does nothing about the other problems of network cameras, like old software or calls to the maker's servers. A camera that is turned away is still online. Keep it isolated on your network as well.

To a Zigbee network the device advertises itself as a blind (a Window Covering). In fact it only drives a servo to a position between 0 and 100 %. A blind was the closest standard device type, and it means that other Zigbee hubs can at least open, close and set a position. I use Zigbee2MQTT and Home Assistant; the extras (calibration, diagnostics, updates) need the Zigbee2MQTT converter in this repository. With another hub, you can make all the settings over USB with the [USB control page](#usb-control-page). This text says "cover", "open" and "closed" because those are the Zigbee words. What the servo moves is up to you.

> Parts of this code and documentation were written with the help of AI. Review it and test it on your own hardware before you rely on it.

## What it does

- **Standard Zigbee cover** (Window Covering cluster `0x0102`), no hub-specific code needed.
- **Zigbee router**: it is powered all the time, so it joins as a router and rejoins its saved network by itself after a power cut or a coordinator restart.
- **Calibration over the air**: servo end points, speed and idle cut-off can be changed from Zigbee2MQTT. Values are stored in flash and sent back, so Zigbee2MQTT always shows what the device really uses.
- **Remembers its position** across restarts and does not move at boot. It waits for a command.
- **Idle cut-off**: the PWM signal stops after the move, so cheap servos don't buzz or heat up.
- **Optional position feedback (1.3.0)**: with a servo that has a feedback wire, the device knows where the cover really is. It reports the real position, notices a cover moved by hand, stops the motor when the cover is blocked, and tells you when the wire is loose. See [Position feedback](#position-feedback-optional).
- **Status and heartbeat (1.3.0)**: a `status` value (`ok`, `moving`, `blocked`, …) and a message every 60 seconds, so Zigbee2MQTT can show the device as offline within about two minutes.
- **USB control page (1.3.0)**: a single HTML file that talks to the board over USB for tests, calibration and diagnostics. See [USB control page](#usb-control-page).
- **Over-the-air updates** from Zigbee2MQTT, with automatic rollback if a new version fails.
- **Diagnostics**: reset reason, boot count and chip temperature.

## Hardware

| Part | Notes |
|---|---|
| Board | ESP32-C6 SuperMini (others work if you change the pins) |
| Servo signal | GPIO 2 |
| Servo feedback (optional) | GPIO 4, analog input |
| Status LED | GPIO 15 (the RGB LED on GPIO 8 is not used) |
| Button | BOOT button, GPIO 9 |

**Power.** A servo draws a short, high current when it starts to move. A supply that is fine on the bench may not handle that peak once the device is installed, for example at the end of a long or thin cable. The sign is a board that resets, or a servo that stops, when a move starts. It took me days to find this. If your supply can't deliver the peak, put an energy buffer near the servo.

> **Don't use a LiPo cell as that buffer.** I did at first, on the board's battery pads. Kept on charge at 4.2 V all the time, next to a warm board, the cell swelled within months. I found it in time; it could have ended in a fire. Supercapacitors (with a series resistor and a diode) do the job now. A large electrolytic capacitor can be enough for a small servo.
## Quick start

### 1. Flash

Prebuilt 1.3.0 binaries are in [`bin/`](bin/). Flash them with [esptool](https://github.com/espressif/esptool) (replace `PORT`, e.g. `COM3` or `/dev/ttyACM0`):

```bash
esptool.py --chip esp32c6 -p PORT -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_freq 80m --flash_size 2MB 0x0 bin/bootloader.bin 0x8000 bin/partition-table.bin 0xd000 bin/ota_data_initial.bin 0x10000 bin/erkinIO_esp32_servo_blind.bin
```

Or use Espressif's [Flash Download Tool](https://www.espressif.com/en/support/download/other-tools) with the same files and addresses. To build it yourself, see [Building from source](#building-from-source).

A device that already runs 1.1.0 or later doesn't need USB: see [OTA updates](#ota-updates).

### 2. Add the converter to Zigbee2MQTT

The cover works without it, but the converter adds calibration, status, diagnostics, OTA support, and removes the tilt slider the device doesn't have.

- **In the UI:** Settings › Dev console › External converters, add a file named `espServoBlind.mjs` with the contents of [`espServoBlind.mjs`](espServoBlind.mjs).
- **Or by file:** copy `espServoBlind.mjs` into Zigbee2MQTT's `data/external_converters/` folder.

If you used the older `espServoBlind.js`, remove it first. Then restart Zigbee2MQTT.

> Zigbee2MQTT 2.11.0 and later loads external converters only when `advanced: enable_external_js: true` is set in its `configuration.yaml`. New installations have it off.

### 3. Pair

Turn on **Permit join** in Zigbee2MQTT. A freshly flashed device searches for a network by itself (LED blinking slowly); press the button once to search right away. Once it has joined, the LED turns off.

### 4. Calibrate

On the device's page in Zigbee2MQTT, on the **Exposes** tab:

| Setting | Range | Default | What it does |
|---|---|---|---|
| `servo_min_us` (Open position) | 500–2500 | 1000 | Pulse width at 0 % |
| `servo_max_us` (Closed position) | 500–2500 | 2000 | Pulse width at 100 % |
| `move_speed_ms` (Speed) | 0–1000 | 20 | Milliseconds per 1 % of travel; 0 = servo's full speed |
| `idle_timeout_ms` (Motor-off delay) | 0, or 500–10000 | 1000 | Stop the PWM signal this long after a move; 0 = never |

Changing a setting never moves the servo; the next command uses it. Start with a narrow range and widen it step by step. **Both 0 % and 100 % must be mechanically safe**, because Home Assistant's Open and Close buttons drive straight to them. A servo held against an end stop draws its stall current and gets hot.

`test_pulse_us` (Test pulse) helps to find the two values: it drives the servo slowly to a pulse width you type and holds it there, for at most 5 minutes. `0` switches the motor off again. The [USB control page](#usb-control-page) does the same with a slider.

## Position feedback (optional)

Some servos have a fourth wire that carries the voltage of the internal position potentiometer (this was developed with a Feetech FS5115M-FB). With that wire connected, the firmware reads where the output shaft really is.

Everything in this chapter is optional. Feedback is **off by default**, and a device without the wire works exactly as before.

**What you get**

- `feedback_position`: the measured position in %, also right after a restart.
- A move starts from where the cover really is, not from where the firmware thinks it is.
- `status: moved_by_hand` when the position changes while the motor is off. The reported position follows.
- `status: blocked` when the cover doesn't reach its position. The motor is switched off instead of pushing until it gets hot.
- `status: no_feedback` when the wire is loose or cut. The firmware tests the wire every 2 seconds while the motor is off, so a floating pin is not mistaken for a moving cover.
- The position is no longer written to flash on every move, because it can be measured.

**Wiring.** Connect the feedback wire straight to GPIO 4. No resistor is needed. The voltage must stay below 3.3 V: the FS5115M-FB gives about 0.2–3.2 V on a 5 V supply. Check your servo with a multimeter before you connect it. Servo ground and board ground must be connected, as for the signal wire.

**Set-up**

1. Switch **Position feedback** to ON.
2. Press **Calibrate**. The servo moves slowly to both ends of its travel and learns the voltage at each end. This takes about 40 seconds, and the result is saved in flash.
3. Set the open and closed positions. Either type `servo_min_us` and `servo_max_us` as before, or switch the motor off, move the cover by hand to the position, and press **Set open position here** or **Set closed position here**.

> **Calibrate moves the servo through its whole range**, until the shaft stops following. If your mechanism can't travel that far, take the servo arm off, or uncouple the cover, before you press it. You only need to calibrate once per servo.

The four `calibration_…` values show what was learned. If they are empty, there is no calibration and the feedback is not used.

The `blocked` detection could not be tested fully on the test hardware: the servo was too strong to stop by hand. Treat it as an extra safety net, not as a replacement for safe end points.

## What Zigbee2MQTT shows

| Value | Meaning |
|---|---|
| `position` / `state` | Cover position 0–100 % (0 % = open) |
| `status` | `ok`, `moving`, `blocked`, `no_feedback`, `moved_by_hand`, `calibrating` or `test` |
| `feedback_position` | Measured position in %. Empty without feedback |
| `feedback_voltage` | Voltage on the feedback wire. Sent once at start, and after that only when it really changes |
| `device_temperature` | Chip temperature, every 5 minutes |
| `last_reset_reason` | Why it last started: `power_on`, `software`, `brownout`, `crash`, `watchdog`, … `storage_erased` means the saved settings were damaged and are back to defaults |
| `boot_count` | Number of starts since it was flashed |
| `servo_min_us`, `servo_max_us`, `move_speed_ms`, `idle_timeout_ms` | Calibration, see above |
| `feedback`, `calibrate`, `set_open_position`, `set_closed_position` | Position feedback, see above |
| `test_pulse_us` | Test pulse, see above |
| `calibration_low_us`, `calibration_low_mv`, `calibration_high_us`, `calibration_high_mv` | Result of **Calibrate** |
| `identify_action` | Blinks the LED so you can find the device |

**Online / offline.** The device sends its status and position every 60 seconds. To see it go offline quickly, give it a short availability timeout in Zigbee2MQTT's `configuration.yaml` (the value is in minutes):

```yaml
devices:
  '<IEEE address of the device>':
    availability:
      timeout: 2
```

## USB control page

[`tools/servo_live.html`](tools/servo_live.html) is a single file that talks to the board over its USB port. Use it to find the end points of a new servo, to watch the feedback voltage while the servo moves, or to see what the device has saved.

It needs no installation and no special firmware: it talks to the normal firmware, 1.3.0 or later. It uses Web Serial, so it needs a **Chromium-based browser on a computer**, such as Brave or Chrome. In Brave you may have to switch Web Serial on first (`brave://flags`, search for "serial"). Firefox and Safari don't have Web Serial. The page loads nothing from the internet and sends nothing anywhere.

**How to use it**

1. Connect the board to the computer with a USB cable.
2. Open `servo_live.html` in the browser (a double click on the file is enough).
3. Click **Connect** and choose the board's serial port.

**What is on it**

- **Feedback, Command, Noise**: the feedback voltage, the pulse being sent, and how much the voltage moved in the last second.
- **Motor ON / STOP**: Motor ON holds the servo where it is now (the centre, if the position is not known) and switches on the slider and the step buttons. STOP switches the motor off at once. Nothing moves until you press Motor ON.
- **Slider, −50 … +50, Speed**: drive the servo to a pulse width between 400 and 2600 µs, at a slow, limited speed.
- **Auto calibrate**: the same as **Calibrate** in Zigbee2MQTT. The result is saved in the device.
- **Chart** of the last 20 seconds, and **Save CSV** for everything since you connected.
- **Device**: everything the device has saved or measured, with a short explanation for each value. It refreshes every 2 seconds; a checkbox switches that off. Buttons switch the feedback on and off, set the open or closed position, and move the cover to a position in %.
- **Settings**: the open and closed pulse, the speed and the motor-off delay, the same four settings as in Zigbee2MQTT. They are saved in the device. **Copy value from slider** puts the pulse of the slider into the box, so you can find a position with Motor ON and then save it. This also works for a servo without feedback, and without Zigbee2MQTT.
- **Backup**: export the settings, feedback on or off, and the calibration to a small JSON file, and import them again later. The file holds nothing about the Zigbee network. The import shows the values and asks before it writes them, and it does not move the servo. A calibration is correct only for the servo it was made with.
- **History**: every change of a value in the Device table, with the time.

The page never shows the Zigbee network key or other secrets; the firmware doesn't send them.

**Good to know**

- The servo needs its normal power supply. The USB port alone may not be able to feed it.
- Only one program can use the serial port. Click **Disconnect** before you flash or open a serial monitor.
- When the page disconnects or the cable is pulled, a test pulse that is still running is switched off.
- Zigbee keeps working while the page is connected.

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

The update file for this release is [`bin/espServoBlind-1.3.0.ota`](bin/espServoBlind-1.3.0.ota), with its index file next to it.

**Install it from Zigbee2MQTT**

1. Copy `espServoBlind-1.3.0.ota` and `espServoBlind-1.3.0.index.json` into Zigbee2MQTT's data folder.
2. Point Zigbee2MQTT at the index file in its `configuration.yaml`, and restart it:
   ```yaml
   ota:
     zigbee_ota_override_index_location: espServoBlind-1.3.0.index.json
   ```
3. On the device's **OTA** tab, click **Check for update**, then **Update**.

Alternatively, publish the file itself to `zigbee2mqtt/bridge/request/device/ota_update/update`:

```json
{"id": "<device name or IEEE>", "hex": {"data": "<the .ota file as hex>", "file_name": "espServoBlind-<version>.ota"}}
```

Update the converter in Zigbee2MQTT as well (see [Quick start](#2-add-the-converter-to-zigbee2mqtt)), or the new values won't show.

**What to expect**
- The download takes **about an hour** at Zigbee2MQTT's default settings. The cover keeps working meanwhile.
- The device then restarts into the new version and reports back within seconds. It doesn't move the servo.
- Settings, calibration and the Zigbee network are kept.
- The new version is kept only after it has been back on the network for one minute **and the coordinator has answered it**. If it crashes or restarts before that, or has no contact within 10 minutes, the device goes back to the previous version by itself.
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

- **From 1.1.0:** update over the air, or flash the four files over USB as in [Quick start](#1-flash). Nothing is erased and nothing has to be set again: all settings are kept, and feedback stays off until you switch it on. Replace `espServoBlind.js` in Zigbee2MQTT with `espServoBlind.mjs`.
- **From 1.0.0:** 1.1.0 changed the Zigbee role from end device to router. Flash all four files over USB without erasing (calibration and position stay); this also installs the rollback bootloader, so every later update can go over the air. On first start the firmware clears only its old network data. Turn on Permit join; it rejoins within 5 minutes, or at once if you press the button. Don't remove the device in Zigbee2MQTT first, so names and Home Assistant entities stay. Afterwards click **Interview** and **Reconfigure** on the device.
- **Any version:** update the converter in Zigbee2MQTT as well. The 1.3.0 converter also works with older firmware; the new values then stay empty.

## Repository layout

| Path | Contents |
|---|---|
| [`main/`](main/) | Firmware source |
| [`espServoBlind.mjs`](espServoBlind.mjs) | Zigbee2MQTT external converter |
| [`tools/servo_live.html`](tools/servo_live.html) | USB control page |
| [`tools/make_ota.py`](tools/make_ota.py) | Wraps a build into a Zigbee OTA file (no dependencies) |
| [`bin/`](bin/) | Prebuilt 1.3.0 binaries and the OTA update file |
| [`partitions.csv`](partitions.csv) | Two app slots for OTA, plus Zigbee storage |

## Changelog

**1.3.0**
- Optional position feedback for servos with a feedback wire (GPIO 4): real position, `moved_by_hand`, `blocked` and `no_feedback` detection, automatic calibration, and "set open / closed position here".
- New `status` value, and a heartbeat every 60 seconds.
- Test pulse: drive the servo to a pulse width to find the end points.
- USB control page, `tools/servo_live.html`, and text commands over the board's USB port. The page can make every setting, and export and import the settings as a file, so Zigbee2MQTT is not needed for set-up.
- At start the device reports all its values by itself, spread over a few seconds.
- A new version from an OTA update is kept only after the coordinator has answered.
- `idle_timeout_ms` below 500 ms is raised to 500 ms (0 still means "hold forever"), so the servo has time to arrive.
- If the saved settings are damaged, they go back to defaults and `last_reset_reason` shows `storage_erased`. The device stays on its network.
- The converter is now `espServoBlind.mjs`, with labels and descriptions for every value. It replaces `espServoBlind.js`.
- Fully compatible with 1.1.0: no setting is lost and nothing has to be set again.

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
