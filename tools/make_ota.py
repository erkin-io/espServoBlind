#!/usr/bin/env python3
"""Wrap the built app image in a standard Zigbee OTA file for Zigbee2MQTT.

Same layout as Espressif's tools/image_builder_tool (header version 0x0100, no optional
fields, one "upgrade image" sub-element), but with no dependencies. The manufacturer
code, image type and file version are read from the firmware source, so the file always
matches what the device accepts.

    python tools/make_ota.py                      # uses build/erkinIO_esp32_servo_blind.bin
    python tools/make_ota.py --app path/to/app.bin --out-dir dist

Writes espServoBlind-<version>.ota and a Zigbee2MQTT OTA index entry next to it.
"""
import argparse
import hashlib
import json
import re
import struct
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
SOURCE = ROOT / 'main' / 'esp_zb_customized_server.c'

OTA_FILE_ID = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_LENGTH = 56
ZIGBEE_STACK_VERSION = 2  # Zigbee PRO
TAG_UPGRADE_IMAGE = 0x0000
ESP_IMAGE_MAGIC = 0xE9


def define(name: str) -> int:
    match = re.search(rf'^#define\s+{name}\s+(0x[0-9A-Fa-f]+|\d+)', SOURCE.read_text(), re.M)
    if not match:
        raise SystemExit(f'{name} not found in {SOURCE}')
    return int(match.group(1), 0)


def build(app: bytes, manufacturer: int, image_type: int, file_version: int, header_string: str) -> bytes:
    element = struct.pack('<HI', TAG_UPGRADE_IMAGE, len(app)) + app
    header = struct.pack(
        '<IHHHHHIH32sI',
        OTA_FILE_ID, OTA_HEADER_VERSION, OTA_HEADER_LENGTH, 0,  # field control: no optional fields
        manufacturer, image_type, file_version, ZIGBEE_STACK_VERSION,
        header_string.encode('ascii')[:32].ljust(32, b'\0'),
        OTA_HEADER_LENGTH + len(element),
    )
    assert len(header) == OTA_HEADER_LENGTH
    return header + element


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--app', type=Path, default=ROOT / 'build' / 'erkinIO_esp32_servo_blind.bin')
    parser.add_argument('--out-dir', type=Path, default=ROOT / 'build')
    args = parser.parse_args()

    app = args.app.read_bytes()
    if not app or app[0] != ESP_IMAGE_MAGIC:
        raise SystemExit(f'{args.app} is not an ESP app image')

    manufacturer = define('ESPRESSIF_MANUFACTURER_CODE')
    image_type = define('OTA_IMAGE_TYPE')
    file_version = define('FW_FILE_VERSION')
    version = '.'.join(str(b) for b in file_version.to_bytes(4, 'big')[:3])
    header_string = f'espServoBlind {version}'

    ota = build(app, manufacturer, image_type, file_version, header_string)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    name = f'espServoBlind-{version}.ota'
    (args.out_dir / name).write_bytes(ota)

    entry = {
        'fileName': name,
        'fileVersion': file_version,
        'fileSize': len(ota),
        'url': name,
        'imageType': image_type,
        'manufacturerCode': manufacturer,
        'sha512': hashlib.sha512(ota).hexdigest(),
        'otaHeaderString': header_string,
        'modelId': 'espServoBlind',
    }
    (args.out_dir / f'espServoBlind-{version}.index.json').write_text(json.dumps([entry], indent=2) + '\n')
    print(f'{name}: {len(ota)} bytes, manufacturer 0x{manufacturer:04X}, image type 0x{image_type:04X}, '
          f'file version 0x{file_version:08X}')


if __name__ == '__main__':
    main()
